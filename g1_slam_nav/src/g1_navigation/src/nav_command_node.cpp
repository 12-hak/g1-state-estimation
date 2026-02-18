#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav_msgs/msg/path.hpp>
#include "g1_msgs/msg/navigation_command.hpp"
#include "g1_msgs/msg/waypoint_list.hpp"
#include "g1_msgs/srv/set_waypoints.hpp"

#include <mutex>
#include <vector>

namespace g1_navigation {

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;

class NavCommandNode : public rclcpp::Node {
public:
    explicit NavCommandNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("nav_command_node", options) {

        declare_parameter("map_frame", "map");
        map_frame_ = get_parameter("map_frame").as_string();

        // Action clients for Nav2
        nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");
        wp_client_ = rclcpp_action::create_client<FollowWaypoints>(
            this, "follow_waypoints");

        // Command subscriber (from web interface or other sources)
        cmd_sub_ = create_subscription<g1_msgs::msg::NavigationCommand>(
            "nav/command", 10,
            std::bind(&NavCommandNode::commandCallback, this, std::placeholders::_1));

        // Simple goal subscriber (click-to-navigate from web)
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "nav/goal", 10,
            std::bind(&NavCommandNode::goalCallback, this, std::placeholders::_1));

        // Waypoint list subscriber
        wp_sub_ = create_subscription<g1_msgs::msg::WaypointList>(
            "nav/waypoints", 10,
            std::bind(&NavCommandNode::waypointCallback, this, std::placeholders::_1));

        // Path publisher (for web visualization)
        path_pub_ = create_publisher<nav_msgs::msg::Path>("nav/planned_path", 10);

        // Waypoint service
        wp_srv_ = create_service<g1_msgs::srv::SetWaypoints>(
            "set_waypoints",
            std::bind(&NavCommandNode::setWaypointsCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "Navigation command node ready");
    }

private:
    void commandCallback(const g1_msgs::msg::NavigationCommand::SharedPtr msg) {
        switch (msg->command_type) {
            case g1_msgs::msg::NavigationCommand::STOP:
            case g1_msgs::msg::NavigationCommand::CANCEL:
                cancelNavigation();
                break;
            case g1_msgs::msg::NavigationCommand::GO_TO_POSE:
                navigateToPose(msg->goal);
                break;
            case g1_msgs::msg::NavigationCommand::FOLLOW_WAYPOINTS:
                followWaypoints(msg->waypoints);
                break;
            default:
                RCLCPP_WARN(get_logger(), "Unknown command type: %d", msg->command_type);
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        navigateToPose(*msg);
    }

    void waypointCallback(const g1_msgs::msg::WaypointList::SharedPtr msg) {
        followWaypoints(msg->waypoints);
    }

    void navigateToPose(const geometry_msgs::msg::PoseStamped& goal) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
            return;
        }

        auto nav_goal = NavigateToPose::Goal();
        nav_goal.pose = goal;
        if (nav_goal.pose.header.frame_id.empty()) {
            nav_goal.pose.header.frame_id = map_frame_;
        }

        RCLCPP_INFO(get_logger(), "Navigating to (%.2f, %.2f)",
                    goal.pose.position.x, goal.pose.position.y);

        auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(get_logger(), "Navigation succeeded");
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_WARN(get_logger(), "Navigation aborted");
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_INFO(get_logger(), "Navigation canceled");
                        break;
                    default:
                        RCLCPP_WARN(get_logger(), "Navigation unknown result");
                }
            };

        nav_goal_handle_ = nav_client_->async_send_goal(nav_goal, send_options);
    }

    void followWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints) {
        if (!wp_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_logger(), "FollowWaypoints action server not available");
            return;
        }

        auto wp_goal = FollowWaypoints::Goal();
        wp_goal.poses = waypoints;
        for (auto& pose : wp_goal.poses) {
            if (pose.header.frame_id.empty()) {
                pose.header.frame_id = map_frame_;
            }
        }

        RCLCPP_INFO(get_logger(), "Following %zu waypoints", waypoints.size());

        auto send_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<FollowWaypoints>::WrappedResult& result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "All waypoints reached");
                } else {
                    RCLCPP_WARN(get_logger(), "Waypoint following ended (missed: %zu)",
                                result.result->missed_waypoints.size());
                }
            };

        wp_client_->async_send_goal(wp_goal, send_options);
    }

    void cancelNavigation() {
        RCLCPP_INFO(get_logger(), "Canceling navigation");
        nav_client_->async_cancel_all_goals();
        wp_client_->async_cancel_all_goals();
    }

    void setWaypointsCallback(
        const std::shared_ptr<g1_msgs::srv::SetWaypoints::Request> request,
        std::shared_ptr<g1_msgs::srv::SetWaypoints::Response> response) {

        if (request->waypoints.empty()) {
            response->success = false;
            response->message = "No waypoints provided";
            return;
        }

        followWaypoints(request->waypoints);
        response->success = true;
        response->message = "Waypoint following started with " +
                            std::to_string(request->waypoints.size()) + " waypoints";
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr wp_client_;

    rclcpp::Subscription<g1_msgs::msg::NavigationCommand>::SharedPtr cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<g1_msgs::msg::WaypointList>::SharedPtr wp_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Service<g1_msgs::srv::SetWaypoints>::SharedPtr wp_srv_;

    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr> nav_goal_handle_;

    std::string map_frame_;
};

}  // namespace g1_navigation

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<g1_navigation::NavCommandNode>());
    rclcpp::shutdown();
    return 0;
}
