#include "geometry_msgs/msg/twist.hpp"
#include "mage_msgs/action/robot_target.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using RobotTarget = mage_msgs::action::RobotTarget;
using GoalHandle = rclcpp_action::ClientGoalHandle<RobotTarget>;

class RobotTargetClient : public rclcpp::Node {
   public:
    explicit RobotTargetClient(const std::string& node_name)
        : Node(node_name), next_target_x_{0.1}, next_target_y_{0.1} {

        // Create a mutually exclusive callback group
        group1_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        

        // Create a mutually exclusive callback group
        rclcpp::SubscriptionOptions sub_option;
        group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_option.callback_group = group2_;

        // Create a callback group for the action client
        action_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // Create action client

        action_client_ = rclcpp_action::create_client<RobotTarget>(this, "reach_target", action_group_);

        // Initialize publisher and subscriber
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&RobotTargetClient::odom_callback, this, std::placeholders::_1), sub_option);

        // Connect to action server
        timer_ = this->create_wall_timer(1s, std::bind(&RobotTargetClient::send_goal, this), group1_);

        camera1_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "/mage/camera1/image", rclcpp::SensorDataQoS(),
            std::bind(&RobotTargetClient::camera1_callback, this, std::placeholders::_1), sub_option);
    }

   private:
    void send_goal();
    void goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future);
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const RobotTarget::Feedback> feedback);
    void result_callback(const GoalHandle::WrappedResult& result);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

   private:
    rclcpp::CallbackGroup::SharedPtr action_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Client<RobotTarget>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // callback groups
    rclcpp::CallbackGroup::SharedPtr group1_;
    rclcpp::CallbackGroup::SharedPtr group2_;
    double next_target_x_;
    double next_target_y_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera1_sub_;

};