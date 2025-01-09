/**
 * @file part_pose_finder.hpp
 * @author 
 *    - Varad Nerlekar (nerlekar@umd.edu)
 *    - Antony Munyaradzi (mantony2@umd.edu)
 *    - Siddhant Deshmukh (iamsid@umd.edu)
 * @brief Defines the PartPoseFinder class for detecting and storing part poses in the environment using multiple cameras.
 * 
 * This class processes data from multiple advanced logical cameras, computes the global poses of parts, 
 * and publishes waypoints for further processing or navigation. It handles part identification, deduplication, 
 * and efficient pose storage.
 * 
 * @version 0.1
 * @date 2024-12-03
 * @copyright Copyright (c) 2024
 */

#pragma once

#include <memory>
#include <set>
#include <vector>
#include <unordered_map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "color_utils.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "mage_msgs/msg/part.hpp"
#include "mage_msgs/msg/parts.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/convert.h>
#include <geometry_msgs/msg/pose.hpp>
#include <functional> // For std::less
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "waypoint_publisher.hpp"

/**
 * @struct PartComparator
 * @brief Custom comparator for sorting parts by their color and type.
 */
struct PartComparator {
     /**
     * @brief Compares two parts based on their color and type.
     * @param lhs Left-hand side part.
     * @param rhs Right-hand side part.
     * @return True if lhs numeric representaion is less than rhs numeric representaion, false otherwise.
     */
    bool operator()(const mage_msgs::msg::Part& lhs, const mage_msgs::msg::Part& rhs) const {
        if (lhs.color != rhs.color) {
            return lhs.color < rhs.color;
        }
        return lhs.type < rhs.type;
    }
};

namespace Turtlebot3 {
    /**
     * @class PartPoseFinder
     * @brief A ROS 2 node that processes part pose data from multiple cameras.
     * 
     * The PartPoseFinder node subscribes to multiple logical camera topics to detect parts in the environment,
     * computes their global poses, and publishes waypoints for navigation. It also ensures deduplication of parts
     * and maintains an efficient data structure for storing poses.
     */
    class PartPoseFinder : public rclcpp::Node {
        public:
            /**
             * @brief Constructor for the PartPoseFinder class.
             * @param node_name The name of the node.
             */
            PartPoseFinder(const std::string &node_name) 
                : rclcpp::Node(node_name), 
                tf_buffer_(this->get_clock()), // Initialize tf_buffer with the node's clock
                tf_listener_(tf_buffer_) { // Initialize tf_listener with the tf_buffer
                RCLCPP_INFO_ONCE(this->get_logger(), output_green("Started PartPoseFinder Node").c_str());

                // Initialize camera processed flags
                std::fill(std::begin(cameras_processed), std::end(cameras_processed), false);
                parts_processed = false;
                list_logged = false;

                /**
                 * @brief Initializes subscribers and publishers for the PartPoseFinder node.
                 * 
                 * This block sets up:
                 * 1. Subscribers for receiving data from eight advanced logical cameras.
                 * 2. A subscriber for the parts list, which contains metadata about detected parts.
                 * 3. A publisher for sending waypoints derived from the detected parts' global poses.
                 */

                // Create subscribers for each camera topic

                /**
                 * @brief Subscriber for Camera 1 to process advanced logical camera image messages.
                 * @topic /mage/camera1/image
                 */ 
                camera1_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                    "/mage/camera1/image", rclcpp::SensorDataQoS(),
                    std::bind(&PartPoseFinder::camera1_callback, this, std::placeholders::_1)
                );

                /**
                 * @brief Subscriber for Camera 2 to process advanced logical camera image messages.
                 * @topic /mage/camera2/image
                 */ 
                camera2_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                    "/mage/camera2/image", rclcpp::SensorDataQoS(),
                    std::bind(&PartPoseFinder::camera2_callback, this, std::placeholders::_1)
                );

                 /**
                 * @brief Subscriber for Camera 3 to process advanced logical camera image messages.
                 * @topic /mage/camera3/image
                 */
                camera3_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                    "/mage/camera3/image", rclcpp::SensorDataQoS(),
                    std::bind(&PartPoseFinder::camera3_callback, this, std::placeholders::_1)
                );

                 /**
                 * @brief Subscriber for Camera 4 to process advanced logical camera image messages.
                 * @topic /mage/camera4/image
                 */
                camera4_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                    "/mage/camera4/image", rclcpp::SensorDataQoS(),
                    std::bind(&PartPoseFinder::camera4_callback, this, std::placeholders::_1)
                );

                 /**
                 * @brief Subscriber for Camera 5 to process advanced logical camera image messages.
                 * @topic /mage/camera5/image
                 */
                camera5_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                    "/mage/camera5/image", rclcpp::SensorDataQoS(),
                    std::bind(&PartPoseFinder::camera5_callback, this, std::placeholders::_1)
                );

                 /**
                 * @brief Subscriber for Camera 6 to process advanced logical camera image messages.
                 * @topic /mage/camera6/image
                 */
                camera6_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                    "/mage/camera6/image", rclcpp::SensorDataQoS(),
                    std::bind(&PartPoseFinder::camera6_callback, this, std::placeholders::_1)
                );

                 /**
                 * @brief Subscriber for Camera 7 to process advanced logical camera image messages.
                 * @topic /mage/camera7/image
                 */
                camera7_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                    "/mage/camera7/image", rclcpp::SensorDataQoS(),
                    std::bind(&PartPoseFinder::camera7_callback, this, std::placeholders::_1)
                );

                 /**
                 * @brief Subscriber for Camera 8 to process advanced logical camera image messages.
                 * @topic /mage/camera8/image
                 */
                camera8_sub_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                    "/mage/camera8/image", rclcpp::SensorDataQoS(),
                    std::bind(&PartPoseFinder::camera8_callback, this, std::placeholders::_1)
                );

                // Tuning QOS
                rclcpp::QoS qos(rclcpp::KeepLast(15));
                qos.reliable();

                /**
                 * @brief Publisher for sending waypoints based on detected parts' global poses.
                 * @topic /waypoint_from_part_finder
                 */
                waypoint_publish_from_finder_ = this->create_publisher<mage_msgs::msg::PartPose>("/waypoint_from_part_finder", qos);

                // Create a subscriber for parts list
                /**
                 * @brief Subscriber for the parts list containing metadata about detected parts.
                 * @topic /parts
                 * @QoS KeepLast(10)
                 */
                parts_sub_ = this->create_subscription<mage_msgs::msg::Parts>(
                    "/parts", rclcpp::QoS(10), // Specify a QoS profile
                    std::bind(&PartPoseFinder::parts_callback, this, std::placeholders::_1)
                );
            }

        private:
            // Variables to track the state of processing
            bool cameras_processed[8];  ///< Flags to indicate if each camera's data has been processed.
            bool parts_processed;       ///< Flag to indicate if parts list has been processed.
            bool list_logged;           ///< Flag to indicate if the list of detected parts has been logged.
            std::vector<mage_msgs::msg::PartPose> global_poses_; // List to store global poses

            // ROS 2 subscribers for camera and parts data
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera1_sub_;///< Subscriber for Camera 1.
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera2_sub_;///< Subscriber for Camera 2.
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera3_sub_;///< Subscriber for Camera 3.
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera4_sub_;///< Subscriber for Camera 4.
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera5_sub_;///< Subscriber for Camera 5.
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera6_sub_;///< Subscriber for Camera 6.
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera7_sub_;///< Subscriber for Camera 7.
            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera8_sub_;///< Subscriber for Camera 8.
            
            // ROS 2 publisher for waypoints
            rclcpp::Subscription<mage_msgs::msg::Parts>::SharedPtr parts_sub_;///< Publisher for waypoints.

            // Data structures for storing part information
            std::set<mage_msgs::msg::Part, PartComparator> parts_from_parts_topic; // Set to store unique parts

            rclcpp::Publisher<mage_msgs::msg::PartPose>::SharedPtr waypoint_publish_from_finder_;///<publisher to publish waypoints_from_part_finder on topic 

            // A list of all the part poses
            std::vector<mage_msgs::msg::Part> parts_from_camera;///< A list of all the part poses
            std::unordered_map<std::string, geometry_msgs::msg::Pose> part_poses_;   ///< HashMap for storing part poses

            tf2_ros::Buffer tf_buffer_; ///< Buffer for TF transforms
            tf2_ros::TransformListener tf_listener_; ///< Listener for TF transforms

            // Callback methods
            /**
             * @brief Processes data from Camera 1.
             * @param msg Message containing camera data.
             */   
            void camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

            /**
             * @brief Processes data from Camera 2.
             * @param msg Message containing camera data.
             */
            void camera2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

            /**
             * @brief Processes data from Camera 3.
             * @param msg Message containing camera data.
             */
            void camera3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

            /**
             * @brief Processes data from Camera 4.
             * @param msg Message containing camera data.
             */
            void camera4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

            /**
             * @brief Processes data from Camera 5.
             * @param msg Message containing camera data.
             */
            void camera5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

            /**
             * @brief Processes data from Camera 6.
             * @param msg Message containing camera data.
             */
            void camera6_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

            /**
             * @brief Processes data from Camera 7.
             * @param msg Message containing camera data.
             */
            void camera7_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

            /**
             * @brief Processes data from Camera 8.
             * @param msg Message containing camera data.
             */
            void camera8_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
            
            /**
             * @brief Processes the parts list message.
             * @param msg Message containing the list of parts.
             */
            void parts_callback(const mage_msgs::msg::Parts::SharedPtr msg);

            // Methods for processing and logging data
            /**
             * @brief Processes and computes global poses of detected parts.
             * @param msg Message containing camera data.
             * @param camera_name Name of the camera.
             */
            void process_part_poses(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg, const std::string &camera_name);

            /**
             * @brief Converts a local pose to a global pose using the sensor's pose.
             * @param local_pose Local pose of part relative to the sensor.
             * @param camera_name name of the camera in the global frame.
             * @return Pose of the part in the global frame.
             */
            geometry_msgs::msg::Pose convert_to_global_pose(const geometry_msgs::msg::Pose &local_pose, const std::string &camera_name);

            // Method to get the frame ID based on the camera name
            std::string get_frame_id(const std::string &camera_name) {
                if (camera_name == "camera1") return "camera1_frame";
                if (camera_name == "camera2") return "camera2_frame";
                if (camera_name == "camera3") return "camera3_frame";
                if (camera_name == "camera4") return "camera4_frame";
                if (camera_name == "camera5") return "camera5_frame";
                if (camera_name == "camera6") return "camera6_frame";
                if (camera_name == "camera7") return "camera7_frame";
                if (camera_name == "camera8") return "camera8_frame";
                return ""; // Return an empty string if the camera name is invalid
            }

            /**
             * @brief Logs all detected parts and their poses.
             */
            void log_all_parts_detected();

    }; // class PartPoseFinder
} // namespace Turtlebot3
