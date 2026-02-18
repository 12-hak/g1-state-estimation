#pragma once

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>

namespace g1_slam {

struct PoseNode {
    int id;
    Eigen::Matrix4d pose;
};

struct PoseEdge {
    int from_id;
    int to_id;
    Eigen::Matrix4d measurement;       // Relative transform
    Eigen::Matrix<double, 6, 6> information;
};

class PoseGraph {
public:
    PoseGraph();

    void addNode(int id, const Eigen::Matrix4d& pose);
    void addEdge(int from, int to, const Eigen::Matrix4d& measurement,
                 const Eigen::Matrix<double, 6, 6>& information);

    bool optimize(int max_iterations = 10);

    Eigen::Matrix4d getNodePose(int id) const;
    std::vector<Eigen::Matrix4d> getAllPoses() const;
    size_t numNodes() const { return nodes_.size(); }
    size_t numEdges() const { return edges_.size(); }

    void clear();

private:
    static Eigen::Matrix<double, 6, 1> poseToVector(const Eigen::Matrix4d& T);
    static Eigen::Matrix4d vectorToPose(const Eigen::Matrix<double, 6, 1>& v);
    static Eigen::Matrix<double, 6, 1> computeError(const Eigen::Matrix4d& Ti,
                                                      const Eigen::Matrix4d& Tj,
                                                      const Eigen::Matrix4d& Zij);

    std::unordered_map<int, PoseNode> nodes_;
    std::vector<PoseEdge> edges_;
};

}  // namespace g1_slam
