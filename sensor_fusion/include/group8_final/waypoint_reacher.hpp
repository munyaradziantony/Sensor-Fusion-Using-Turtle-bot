/**
 * @file waypoint_reacher.hpp
 * @brief Defines the WaypointReacher class for controlling the Turtlebot3 robot using a P-Controller.
 * 
 * The WaypointReacher class calculates linear and angular velocity commands to move the robot to specified waypoints.
 * It subscribes to odometry and waypoint topics, and publishes velocity commands and readiness signals.
 * 
 * @version 0.1
 * @date 2024-12-03
 * 
 * @authors
 *    - Varad Nerlekar (nerlekar@umd.edu)
 *    - Antony Munyaradzi (mantony2@umd.edu)
 *    - Siddhant Deshmukh (iamsid@umd.edu)
 * 
 * @copyright Copyright (c) 2024
 */

#pragma once

#include <iostream>
#include <memory>
#include <cmath>
#include <sstream>
#include <utility> // For std::clamp
#include <iomanip> // for std::setprecision
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "color_utils.hpp"

namespace Turtlebot3 {
    /**
     * @class WaypointReacher
     * @brief A ROS 2 node for controlling Turtlebot3 movement to specified waypoints using a P-Controller.
     * 
     * This class calculates velocity commands based on proportional control logic and guides the robot
     * to a target position and orientation. It communicates with other nodes via topics to receive waypoints,
     * odometry data, and to publish control commands and readiness signals.
     */
    class WaypointReacher : public rclcpp::Node {
        private:

            /**
             * @brief RCLCPP::Clock instance for log throttling
             */
            rclcpp::Clock::SharedPtr clock_;

            /**
             * @brief Subscriber for receiving waypoint data.
             */
            rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr waypoint_subscriber_;

            /**
             * @brief Subscriber for receiving odometry data.
             */
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

            /**
             * @brief Publisher for readiness signals.
             */
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr readiness_publisher_;

            /**
             * @brief Publisher for velocity commands.
             *
             * Publishes `geometry_msgs::msg::Twist` messages to control the robot's linear
             * and angular velocities.
             */
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

            /**
             * @brief Callback function for processing received waypoints.
             *
             * This function is triggered whenever a waypoint message is received.
             * 
             * @param target_waypoint The target waypoint to reach.
             */
            void reacher_callback(const geometry_msgs::msg::Pose2D target_waypoint);

            /**
             * @brief Callback function to process odometry messages.
             *
             * This function extracts the current position and orientation of the robot.
             *
             * @param msg Shared pointer to the incoming odometry message.
             */
            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

            /**
             * @brief Executes the control logic for navigating to the waypoint.
             *
             * Calculates errors and computes linear and angular velocity commands to 
             * guide the robot toward the target waypoint.
             */
            void control_loop();

            /**
             * @brief Publishes velocity commands to the robot.
             *
             * Sends the calculated linear and angular velocities to the robot.
             *
             * @param linear Linear velocity (m/s).
             * @param angular Angular velocity (rad/s).
             */
            void publish_velocity(double linear, double angular);

            /**
             * @brief Normalizes an angle to the range [-π, π].
             *
             * Ensures that the input angle is within a valid range to prevent wraparound issues.
             *
             * @param angle Input angle in radians.
             * @return Normalized angle in radians within the range [-π, π].
             */
            double normalize_angle(double angle);

            /**
             * @brief Publishes bool to indicate if the robot has reached the target waypoint and is ready to receive a new target.
             * 
             * Sends true when the robot has reached the target waypoint and is ready to receive a new target.
            */
            void publish_readiness();

            /**
             * @brief Parameters for the target waypoint.
             *
             * These parameters define the desired position, orientation, and control tolerances.
             */
            double goal_x_;                     /**< Desired x-coordinate of the goal. */
            double goal_y_;                     /**< Desired y-coordinate of the goal. */
            double goal_theta_{-1.57};           /**< Desired orientation (yaw) at the goal in radians. */
            double kp_linear_{0.3};             /**< Proportional gain for the distance controller. */
            double kp_angular_{1.0};            /**< Proportional gain for the angular controller. */
            double epsilon_{0.1};               /**< Positional tolerance for reaching the goal. */
            double epsilon_theta_{0.017};       /**< Angular tolerance for achieving the desired orientation. */
            double max_linear_velocity_{0.26};  /**< Maximum linear velocity allowed. */
            double max_angular_velocity_{1.00}; /**< Maximum angular velocity allowed. */

            /**
             * @brief Current state of the robot.
             *
             * Stores the robot's current position, orientation, and other state-related information.
             */
            double current_x_;     /**< Current x-coordinate of the robot. */
            double current_y_;     /**< Current y-coordinate of the robot. */
            double current_theta_; /**< Current orientation (yaw) of the robot in radians. */
            double roll_;          /**< Current roll angle (not actively used). */
            double pitch_;         /**< Current pitch angle (not actively used). */

        public:
            /**
             * @brief Constructs a new WaypointReacher object.
             *
             * Initializes the ROS node and sets up publishers and subscribers for waypoints,
             * odometry, and velocity control.
             *
             * @param node_name The name of the ROS node.
             */
            WaypointReacher(const std::string &node_name) : rclcpp::Node(node_name) {
                RCLCPP_INFO_ONCE(this->get_logger(), output_green("Started Waypoint Reacher Node").c_str());

                // Using simulation time to avoid performance degradation due to real time factor.
                this->set_parameter(rclcpp::Parameter("use_sim_time", true));

                // Create a rclcpp::Clock instance for log throttling
                clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

                // Create a publisher for sending velocity commands to the "/cmd_vel" topic.
                velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

                // Create a subscription to the "/bot_waypoint" topic to receive waypoint messages.
                waypoint_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
                    "/bot_waypoint", 10, 
                    std::bind(&WaypointReacher::reacher_callback, this, std::placeholders::_1)
                );

                // Create a publisher for sending readiness messages to the "/next_waypoint" topic.
                readiness_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/next_waypoint", 10);

                // Create a subscription to the "/odom" topic to receive odometry data.
                odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "/odom", 10,
                    std::bind(&WaypointReacher::odom_callback, this, std::placeholders::_1)
                );
            }
    }; // class WaypointReacher
} // namespace Turtlebot3