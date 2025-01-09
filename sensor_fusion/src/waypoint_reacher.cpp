#include "waypoint_reacher.hpp"

// Callback function for receiving a target waypoint
void Turtlebot3::WaypointReacher::reacher_callback(const geometry_msgs::msg::Pose2D target_waypoint) {
    // Set goal coordinates and orientation from the received waypoint
    goal_x_ = target_waypoint.x;
    goal_y_ = target_waypoint.y;
    goal_theta_ = target_waypoint.theta;
    RCLCPP_INFO(this->get_logger(), output_yellow("Received target").c_str());
}

// Callback function to process odometry data
void Turtlebot3::WaypointReacher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract current position of the robot from the odometry message
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Extract current orientation (yaw) using a quaternion-to-RPY conversion
    tf2::Quaternion quaternion;
    tf2::fromMsg(msg->pose.pose.orientation, quaternion); // Convert message orientation to a quaternion
    tf2::Matrix3x3(quaternion).getRPY(roll_, pitch_, current_theta_); // Extract roll, pitch, and yaw

    // Run the control loop to navigate toward the waypoint
    control_loop();
}

// Main control loop for navigating to the waypoint
void Turtlebot3::WaypointReacher::control_loop() {
    // Calculate the distance error to the goal
    double distance_error = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    // Calculate the desired angle to the goal
    double desired_angle = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
    // Calculate the angular error relative to the current orientation
    double angle_error = normalize_angle(desired_angle - current_theta_);
    // Calculate the final orientation error at the goal
    double final_orientation_error = normalize_angle(goal_theta_ - current_theta_);
    
    // Phase 1: Navigate to goal position
    if (distance_error > epsilon_) {
        // Calculating linear and angular velocities using proportional gains(with max vel limits)        
        double linear_velocity = std::clamp(kp_linear_ * distance_error, -max_linear_velocity_, max_linear_velocity_);
        double angular_velocity = std::clamp(kp_angular_ * angle_error, -max_angular_velocity_, max_angular_velocity_);

        std::ostringstream distance_msg;
        distance_msg << std::fixed << std::setprecision(2)
                    << "Distance remaining to reach target position: " << distance_error << " meters";
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), 
            *clock_,            // Using RCL System Time
            2500,               // milliseconds
            distance_msg.str().c_str()
        );

        // Publish velocity commands
        publish_velocity(linear_velocity, angular_velocity);
    }

    // Phase 2: Adjust the final orientation once the goal position is reached
    else if (std::abs(final_orientation_error) > epsilon_theta_) {
        // Stop linear motion and adjust angular orientation
        double angular_velocity = kp_angular_ * final_orientation_error;

        std::ostringstream angle_msg;
        angle_msg << std::fixed << std::setprecision(2)
                << "Angle remaining to reach target orientation: " << final_orientation_error << " rads";
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), 
            *clock_,            // Using RCL System Time
            1000,               // milliseconds
            angle_msg.str().c_str()
        );

        publish_velocity(0.0, angular_velocity); // Only angular velocity is applied
    }
    
    // Phase 3: Goal is reached with the correct orientation
    else {
        publish_velocity(0.0, 0.0);
        publish_readiness();
    }
}

// Function to publish velocity commands to the robot
void Turtlebot3::WaypointReacher::publish_velocity(double linear, double angular) {
    // Create a Twist message for velocity commands
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear;   // Set linear velocity
    msg.angular.z = angular; // Set angular velocity

    // Publish the velocity commands to the /cmd_vel topic
    velocity_publisher_->publish(msg);
}

// Function to normalize an angle to the range [-pi, pi]
double Turtlebot3::WaypointReacher::normalize_angle(double angle) {
    // Adjust the angle if it exceeds pi
    while (angle > M_PI) angle -= 2.0 * M_PI;
    // Adjust the angle if it is below -pi
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Function to publish the readiness of the robot (whether the robot is ready for the next waypoint)
void Turtlebot3::WaypointReacher::publish_readiness() {
    auto msg = std_msgs::msg::Bool();
    msg.set__data(true);
    readiness_publisher_->publish(msg);
}