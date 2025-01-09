#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "reach_target_action.hpp"
void RobotTargetClient::send_goal() {
    auto goal_msg = RobotTarget::Goal();

    goal_msg.target.x = next_target_x_;
    goal_msg.target.y = next_target_y_;
    RCLCPP_INFO_STREAM(this->get_logger(), "Sending goal x[" << goal_msg.target.x << "], y[" << goal_msg.target.y << "]");
    auto goal_options = rclcpp_action::Client<RobotTarget>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(&RobotTargetClient::goal_response_callback, this, std::placeholders::_1);
    goal_options.feedback_callback = std::bind(&RobotTargetClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    goal_options.result_callback = std::bind(&RobotTargetClient::result_callback, this, std::placeholders::_1);

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, goal_options);
}

void RobotTargetClient::goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result...");
    }
}

void RobotTargetClient::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const RobotTarget::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: distance to goal = %f", feedback->distance_to_goal);

    if (feedback->distance_to_goal < 0.04) {
        RCLCPP_INFO(this->get_logger(), "New goal");
        next_target_x_ = 10.0;
        next_target_y_ = 10.0;
    }
}

void RobotTargetClient::result_callback(const GoalHandle::WrappedResult& result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), ">>>>>>>>Goal succeeded: %s", result.result->message.c_str());
    } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(this->get_logger(), ">>>>>>>>Goal was aborted");
    } else {
        RCLCPP_ERROR(this->get_logger(), ">>>>>>>>Unknown result code");
    }
}

void RobotTargetClient::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Process odometry data if needed
    RCLCPP_INFO_STREAM(this->get_logger(), "Current position: x[" << msg->pose.pose.position.x << "], y[" << msg->pose.pose.position.y << "]");
}

void RobotTargetClient::camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Camera1");
    // Include the logic to retrieve part information
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<RobotTargetClient>("robot_target_client");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();  // This will start the execution
    rclcpp::shutdown();
}