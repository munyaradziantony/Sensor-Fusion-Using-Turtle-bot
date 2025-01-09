/**
 * @file waypoint_publisher.hpp
 * @brief Defines the WaypointPublisher class for managing and publishing waypoints for a Turtlebot3 robot.
 *
 * The WaypointPublisher class handles the reception of waypoints from external sources,
 * maintains a queue of waypoints, and publishes them to a robot for navigation. It also
 * subscribes to readiness signals to determine when the next waypoint should be published.
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
#include <queue>
#include <sstream>
#include <iomanip> // for std::setprecision
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mage_msgs/msg/part_pose.hpp"
#include "color_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace Turtlebot3
{
    /**
     * @class WaypointPublisher
     * @brief Publishes waypoints to move the Turtlebot3 robot to specified locations.
     *
     * This class maintains a queue of waypoints and publishes them sequentially upon
     * receiving readiness signals. It also receives waypoints from an external source
     * such as the part pose finder and prepares them for publication.
     */
    class WaypointPublisher : public rclcpp::Node
    {
    private:
        /**
         * @brief Subscriber for receiving waypoints from the part pose finder.
         *
         * This subscriber listens to the `waypoint_from_part_finder` topic and
         * adds new waypoints to the `waypoint_list_` queue.
         */
        rclcpp::Subscription<mage_msgs::msg::PartPose>::SharedPtr waypoint_from_part_finder_;

        /**
         * @brief Queue of waypoints to be published.
         *
         * This queue maintains the list of waypoints received from the part pose
         * finder and ensures they are published sequentially.
         */
        std::queue<mage_msgs::msg::PartPose> waypoint_list_;

        /**
         * @brief Publisher for broadcasting waypoints to the robot.
         *
         * Publishes waypoints in the form of `geometry_msgs::msg::Pose2D` messages
         * on the `bot_waypoint` topic.
         */
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr waypoint_publisher_;

        /**
         * @brief Subscriber for receiving readiness signals.
         *
         * Listens to the `next_waypoint` topic to determine when the robot is ready
         * to receive the next waypoint.
         */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr readiness_subscriber_;

        /**
         * @brief Current index of the waypoint being published.
         *
         * Tracks the index of the waypoint currently being processed and published.
         * This is useful for debugging and monitoring waypoint progress.
         */
        unsigned short index_{0};

        /**
         * @brief Flag to check readiness is logged or not.
         *
         */
        bool readiness_logged_{false};

        /**
         * @brief Flag indicating if the robot is waiting to reach the home position.
         */
        bool waiting_for_home{false};

        /**
         * @brief Marks the end of running of the task
         *
         */
        bool done_with_project{false};

        /**
         * @brief Timer to periodically check if the robot has reached the home position.
         */
        rclcpp::TimerBase::SharedPtr home_check_timer_;

        /**
         * @brief Flag indicating if the robot has reached the home position.
         */
        bool reached_home{false};

        /**
         * @brief Callback function triggered upon receiving a readiness signal.
         *
         * @param msg The message containing the readiness status.
         */
        void readiness_callback(const std_msgs::msg::Bool::SharedPtr msg);

        /**
         * @brief Publishes the next waypoint in the list.
         *
         * This function retrieves the next waypoint from the `waypoint_list_` queue,
         * publishes it to the robot, and logs the action for debugging purposes.
         */
        void publish_waypoint();

        /**
         * @brief Callback function for receiving waypoints from the part pose finder.
         *
         * This function listens to the `waypoint_from_part_finder` topic and adds
         * received waypoints to the `waypoint_list_` queue for future publication.
         *
         * @param msg The message containing the new waypoint coordinates.
         */
        void waypoint_from_part_finder_callback(const mage_msgs::msg::PartPose::SharedPtr msg);

        /**
         * @brief Variable to store whether waypoint list was initiated
         *
         */
        bool waypoint_list_inititated_{false};

    public:
        /**
         * @brief Constructs a new WaypointPublisher object.
         *
         * Initializes the node, sets up publishers and subscribers, and logs the
         * startup message.
         *
         * @param node_name The name of the ROS node.
         */
        WaypointPublisher(const std::string &node_name) : rclcpp::Node(node_name)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), output_green("Started Waypoint Publisher Node").c_str());

            // Create a publisher for BotWaypoint messages on the "bot_waypoint" topic
            waypoint_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("bot_waypoint", 10);

            // Create a subscription to the "next_waypoint" topic
            readiness_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
                "next_waypoint",
                10,
                std::bind(&WaypointPublisher::readiness_callback, this, std::placeholders::_1));

            // Create a subscription to the "waypoint_from_part_finder" topic
            waypoint_from_part_finder_ = this->create_subscription<mage_msgs::msg::PartPose>(
                "waypoint_from_part_finder",
                10,
                std::bind(&WaypointPublisher::waypoint_from_part_finder_callback, this, std::placeholders::_1));
        }

        /**
         * @brief Maps the part color to a string.
         *
         * @param color The color value.
         * @return The corresponding color string.
         */
        static std::string get_color_from_part(const uint8_t &color);

        /**
         * @brief Maps the part type to a string.
         *
         * @param type The type value.
         * @return The corresponding type string.
         */
        static std::string get_type_from_part(const uint8_t &type);

    }; // class WaypointPublisher
} // namespace Turtlebot3
