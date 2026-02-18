#include "g1_slam/pose_graph.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

namespace g1_slam {

PoseGraph::PoseGraph() = default;

void PoseGraph::addNode(int id, const Eigen::Matrix4d& pose) {
    nodes_[id] = {id, pose};
}

void PoseGraph::addEdge(int from, int to, const Eigen::Matrix4d& measurement,
                         const Eigen::Matrix<double, 6, 6>& information) {
    edges_.push_back({from, to, measurement, information});
}

Eigen::Matrix<double, 6, 1> PoseGraph::poseToVector(const Eigen::Matrix4d& T) {
    Eigen::Matrix<double, 6, 1> v;
    v.head<3>() = T.block<3,1>(0,3);
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    v(3) = std::atan2(R(2,1), R(2,2));  // roll
    v(4) = std::asin(std::clamp(-R(2,0), -1.0, 1.0));  // pitch
    v(5) = std::atan2(R(1,0), R(0,0));  // yaw
    return v;
}

Eigen::Matrix4d PoseGraph::vectorToPose(const Eigen::Matrix<double, 6, 1>& v) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = (Eigen::AngleAxisd(v(5), Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(v(4), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(v(3), Eigen::Vector3d::UnitX())).toRotationMatrix();
    T.block<3,1>(0,3) = v.head<3>();
    return T;
}

Eigen::Matrix<double, 6, 1> PoseGraph::computeError(
    const Eigen::Matrix4d& Ti, const Eigen::Matrix4d& Tj,
    const Eigen::Matrix4d& Zij) {

    Eigen::Matrix4d error_T = Zij.inverse() * Ti.inverse() * Tj;
    return poseToVector(error_T);
}

bool PoseGraph::optimize(int max_iterations) {
    if (nodes_.size() < 2 || edges_.empty()) return false;

    // Collect all node IDs in order
    std::vector<int> ids;
    ids.reserve(nodes_.size());
    for (const auto& [id, node] : nodes_) {
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());

    // Build index mapping: node_id -> index in the state vector
    std::unordered_map<int, int> id_to_idx;
    for (size_t i = 0; i < ids.size(); ++i) {
        id_to_idx[ids[i]] = static_cast<int>(i);
    }

    int n = static_cast<int>(ids.size());
    int state_dim = n * 6;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        // Build linear system H * dx = -b
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(state_dim, state_dim);
        Eigen::VectorXd b_vec = Eigen::VectorXd::Zero(state_dim);
        double total_error = 0.0;

        for (const auto& edge : edges_) {
            auto it_i = id_to_idx.find(edge.from_id);
            auto it_j = id_to_idx.find(edge.to_id);
            if (it_i == id_to_idx.end() || it_j == id_to_idx.end()) continue;

            int idx_i = it_i->second * 6;
            int idx_j = it_j->second * 6;

            const Eigen::Matrix4d& Ti = nodes_[edge.from_id].pose;
            const Eigen::Matrix4d& Tj = nodes_[edge.to_id].pose;

            auto e = computeError(Ti, Tj, edge.measurement);
            total_error += e.transpose() * edge.information * e;

            // Numerical Jacobians (finite differences for robustness)
            const double eps = 1e-6;
            Eigen::Matrix<double, 6, 6> Ji, Jj;

            for (int k = 0; k < 6; ++k) {
                auto vi = poseToVector(Ti);
                vi(k) += eps;
                auto e_plus = computeError(vectorToPose(vi), Tj, edge.measurement);
                vi(k) -= 2.0 * eps;
                auto e_minus = computeError(vectorToPose(vi), Tj, edge.measurement);
                Ji.col(k) = (e_plus - e_minus) / (2.0 * eps);

                auto vj = poseToVector(Tj);
                vj(k) += eps;
                e_plus = computeError(Ti, vectorToPose(vj), edge.measurement);
                vj(k) -= 2.0 * eps;
                e_minus = computeError(Ti, vectorToPose(vj), edge.measurement);
                Jj.col(k) = (e_plus - e_minus) / (2.0 * eps);
            }

            // Accumulate into H and b
            H.block<6,6>(idx_i, idx_i) += Ji.transpose() * edge.information * Ji;
            H.block<6,6>(idx_i, idx_j) += Ji.transpose() * edge.information * Jj;
            H.block<6,6>(idx_j, idx_i) += Jj.transpose() * edge.information * Ji;
            H.block<6,6>(idx_j, idx_j) += Jj.transpose() * edge.information * Jj;

            b_vec.segment<6>(idx_i) += Ji.transpose() * edge.information * e;
            b_vec.segment<6>(idx_j) += Jj.transpose() * edge.information * e;
        }

        // Fix first node (anchor)
        for (int k = 0; k < 6; ++k) {
            H(k, k) += 1e6;
        }

        // Solve
        Eigen::VectorXd dx = H.ldlt().solve(-b_vec);

        // Apply update
        double max_update = 0.0;
        for (size_t i = 0; i < ids.size(); ++i) {
            auto v = poseToVector(nodes_[ids[i]].pose);
            v += dx.segment<6>(i * 6);
            nodes_[ids[i]].pose = vectorToPose(v);
            max_update = std::max(max_update, dx.segment<6>(i * 6).norm());
        }

        if (max_update < 1e-6) break;
    }

    return true;
}

Eigen::Matrix4d PoseGraph::getNodePose(int id) const {
    auto it = nodes_.find(id);
    if (it != nodes_.end()) return it->second.pose;
    return Eigen::Matrix4d::Identity();
}

std::vector<Eigen::Matrix4d> PoseGraph::getAllPoses() const {
    std::vector<std::pair<int, Eigen::Matrix4d>> sorted_poses;
    for (const auto& [id, node] : nodes_) {
        sorted_poses.push_back({id, node.pose});
    }
    std::sort(sorted_poses.begin(), sorted_poses.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    std::vector<Eigen::Matrix4d> result;
    result.reserve(sorted_poses.size());
    for (const auto& [id, pose] : sorted_poses) {
        result.push_back(pose);
    }
    return result;
}

void PoseGraph::clear() {
    nodes_.clear();
    edges_.clear();
}

}  // namespace g1_slam
