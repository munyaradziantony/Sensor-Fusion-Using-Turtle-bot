#include "part_pose_finder.hpp"

void Turtlebot3::PartPoseFinder::camera1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (cameras_processed[0] || !parts_processed) return;  // Ignore if already processed
    
    process_part_poses(msg, "camera1");
    cameras_processed[0] = true;  // Mark as processed
}

void Turtlebot3::PartPoseFinder::camera2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (cameras_processed[1] || !parts_processed) return;  // Ignore if already processed
    process_part_poses(msg, "camera2");
    cameras_processed[1] = true;  // Mark as processed
}

void Turtlebot3::PartPoseFinder::camera3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (cameras_processed[2] || !parts_processed) return;  // Ignore if already processed
    process_part_poses(msg, "camera3");
    cameras_processed[2] = true;  // Mark as processed
}

void Turtlebot3::PartPoseFinder::camera4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (cameras_processed[3] || !parts_processed) return;  // Ignore if already processed
    process_part_poses(msg, "camera4");
    cameras_processed[3] = true;  // Mark as processed
}

void Turtlebot3::PartPoseFinder::camera5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (cameras_processed[4] || !parts_processed) return;  // Ignore if already processed
    process_part_poses(msg, "camera5");
    cameras_processed[4] = true;  // Mark as processed
}

void Turtlebot3::PartPoseFinder::camera6_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (cameras_processed[5] || !parts_processed) return;  // Ignore if already processed
    process_part_poses(msg, "camera6");
    cameras_processed[5] = true;  // Mark as processed
}

void Turtlebot3::PartPoseFinder::camera7_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (cameras_processed[6] || !parts_processed) return;  // Ignore if already processed
    process_part_poses(msg, "camera7");
    cameras_processed[6] = true;  // Mark as processed
}

void Turtlebot3::PartPoseFinder::camera8_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if (cameras_processed[7] || !parts_processed) return;  // Ignore if already processed
    process_part_poses(msg, "camera8");
    cameras_processed[7] = true;  // Mark as processed
}

void Turtlebot3::PartPoseFinder::process_part_poses(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg, const std::string &camera_name) {
    // Log the received data of camera
    RCLCPP_INFO(this->get_logger(), output_green("Received image from " + camera_name).c_str());

    auto sensor_pose = msg->sensor_pose;

    // Log the sensor pose
    RCLCPP_INFO(this->get_logger(), "Sensor Pose: position(x=%.2f, y=%.2f, z=%.2f) orientation(x=%.2f, y=%.2f, z=%.2f, w=%.2f)", 
        sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z,
        sensor_pose.orientation.x, sensor_pose.orientation.y, sensor_pose.orientation.z, sensor_pose.orientation.w);

    // Initialize a count for parts detected by this camera
    size_t parts_detected_count = 0;

    // Iterate over the part_poses array in the message
    for (const auto &part_pose : msg->part_poses) {
        // Log the detected part information
        RCLCPP_INFO(this->get_logger(), "Detected part: color=%d, type=%d", part_pose.part.color, part_pose.part.type);
        RCLCPP_INFO(this->get_logger(), "Pose: (x=%.2f, y=%.2f, z=%.2f)", part_pose.pose.position.x, part_pose.pose.position.y, part_pose.pose.position.z);
        
        // Convert local pose to global pose using TF transforms
        geometry_msgs::msg::Pose global_pose = convert_to_global_pose(part_pose.pose, camera_name);
        
        // Log the global pose
        RCLCPP_INFO(this->get_logger(), "Global Pose: (x =%.2f, y=%.2f, z=%.2f)", global_pose.position.x, global_pose.position.y, global_pose.position.z);
        
        // Create a unique key for the part using color and type
        std::string key = std::to_string(part_pose.part.color) + "_" + std::to_string(part_pose.part.type);
        
        // Check if the part exists in parts_from_parts_topic
        auto it = parts_from_parts_topic.find(part_pose.part);
        
        // If the part exists in parts_from_parts_topic, store the global pose
        if (it != parts_from_parts_topic.end()) {
            // Create a PartPose message directly instead of using string keys
            mage_msgs::msg::PartPose msg_part;
            msg_part.pose = global_pose;
            msg_part.part = part_pose.part;
            
            // Store the PartPose in global_poses_ vector
            global_poses_.push_back(msg_part);

            // Store the part in parts_from_camera
            parts_from_camera.push_back(part_pose.part);

            // Increment the count of parts detected
            parts_detected_count++;
        }
    }

    // Log the total number of parts detected
    RCLCPP_INFO(this->get_logger(), output_yellow("Total parts detected in '" + camera_name + "': " + std::to_string(parts_detected_count)).c_str());
}

geometry_msgs::msg::Pose Turtlebot3::PartPoseFinder::convert_to_global_pose(const geometry_msgs::msg::Pose &local_pose, const std::string &camera_name) {
    geometry_msgs::msg::Pose global_pose;

    // Use TF to transform the local pose to the global frame
    geometry_msgs::msg::TransformStamped transform;
    try {
        // Use the identified frame_id for the sensor based on the camera name
        std::string sensor_frame_id = get_frame_id(camera_name);
        transform = tf_buffer_.lookupTransform("world", sensor_frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform pose: %s", ex.what());
        return global_pose; // Return an empty pose if transformation fails
    }

    // Transform the local position
    tf2::doTransform(local_pose.position, global_pose.position, transform);
    
    // Transform the local orientation
    tf2::Quaternion local_orientation;
    tf2::fromMsg(local_pose.orientation, local_orientation);
    
    // Convert the transform's rotation from geometry_msgs to tf2
    tf2::Quaternion transform_rotation;
    tf2::fromMsg(transform.transform.rotation, transform_rotation);
    
    // Combine the sensor's orientation with the local orientation
    tf2::Quaternion global_orientation = transform_rotation * local_orientation; // Quaternion multiplication
    global_pose.orientation = tf2::toMsg(global_orientation);

    return global_pose;
}

void Turtlebot3::PartPoseFinder::parts_callback(const mage_msgs::msg::Parts::SharedPtr msg) {
    // Check if the parts have already been processed
    if (parts_processed) {
        // Check if all cameras have processed
        if (std::all_of(std::begin(cameras_processed), std::end(cameras_processed), [](bool processed) { return processed; })) {
            if (!list_logged) {
                // Log the total number of parts detected in the environment
                RCLCPP_INFO(this->get_logger(), "Total parts detected in the environment: %zu", parts_from_camera.size());
                log_all_parts_detected();
                list_logged = true;

                // Log newly added parts
                for (const auto &part : parts_from_parts_topic) {
                    RCLCPP_INFO(this->get_logger(), output_cyan("Newly added part: color=" + std::to_string(part.color) + ", type=" + std::to_string(part.type)).c_str());
                }
            }
        }
        return;  // Ignore if already processed
    }

    // Check if the message contains any parts
    if (!msg->parts.empty()) {
        // Store all parts in the set
        for (const auto &part : msg->parts) {
            // Check for duplicate parts.
            auto result = parts_from_parts_topic.insert(part); // Store the part in the set
            if (result.second) { // If the unique insertion was successful
                // Log the stored part information
                RCLCPP_INFO(this->get_logger(), "Stored part: color=%d, type=%d", part.color, part.type);
            } else {
                // Log that the part was a duplicate
                RCLCPP_INFO(this->get_logger(), "Duplicate part ignored: color=%d, type=%d", part.color, part.type);
            }
        }

        // Log the number of unique parts in the parts topic
        RCLCPP_INFO(this->get_logger(), output_yellow("Unique parts topic list: " + std::to_string(parts_from_parts_topic.size())).c_str());
        
        // Set the flag to indicate that parts have been processed
        parts_processed = true;
    }
}

// Call this method after processing all camera callbacks
void Turtlebot3::PartPoseFinder::log_all_parts_detected() {
    // Sort the global poses based on the part's color and type.
    std::sort(global_poses_.begin(), global_poses_.end(),
              [](const mage_msgs::msg::PartPose& a, const mage_msgs::msg::PartPose& b) {
                  // Sort the part by color and then type
                  if (a.part.color == b.part.color) {
                      return a.part.type < b.part.type;
                  }
                  return a.part.color < b.part.color;
              });

    // Loop through each part in parts_from_parts_topic.
    for (const auto& part_from_topic : parts_from_parts_topic) {
        // Check if the part exists in global_poses_
        bool part_found = false;
        for (const auto& part_pose : global_poses_) {
            // Compare the parts color and type 
            if (part_from_topic.color == part_pose.part.color && part_from_topic.type == part_pose.part.type) {
                part_found = true;
                break;
            }
        }

        if (!part_found) {
            // Log that the part will be skipped if it's not found in global_poses_
            std::string color_str = WaypointPublisher::get_color_from_part(part_from_topic.color);  // Use static function
            std::string type_str = WaypointPublisher::get_type_from_part(part_from_topic.type);     // Use static function

            RCLCPP_INFO(this->get_logger(), "Skipping part: Color: %s, Type: %s. Part not found in global poses.",
                        color_str.c_str(), type_str.c_str());
        }
    }

    // Loop through each pose in global_poses_ and publish the information
    for (const auto& part_pose : global_poses_) {
        // Publish the message
        waypoint_publish_from_finder_->publish(part_pose);
    }

    mage_msgs::msg::PartPose home_pose;
    home_pose.part.color = 99;  // 
    home_pose.part.type = 99;
    home_pose.pose.position.x = 0.0;
    home_pose.pose.position.y = 0.0;

    waypoint_publish_from_finder_->publish(home_pose);
}

