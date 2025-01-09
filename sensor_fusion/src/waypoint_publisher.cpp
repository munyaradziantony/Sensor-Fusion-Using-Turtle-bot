#include "waypoint_publisher.hpp"

// Callback function for the readiness signal
void Turtlebot3::WaypointPublisher::readiness_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    // Check if the readiness signal is true
    if (msg->data == true) {
        if (waypoint_list_inititated_) {
            if (!readiness_logged_) {
                // Skip the first log and set the flag to true
                readiness_logged_ = true;
            } else {
                // Log only if the flag is true
                RCLCPP_INFO(this->get_logger(), output_white("Published readiness. Robot is ready for the next waypoint.").c_str());
                RCLCPP_INFO(this->get_logger(), output_green("Goal reached with correct orientation!").c_str());
            }
        }
        if (!waypoint_list_.empty()) {
            Turtlebot3::WaypointPublisher::publish_waypoint();
        }
        else{
            if (waypoint_list_inititated_) // If waypoint list was initiated and waypoint list is empty shutdown
            {
                RCLCPP_INFO(this->get_logger(), output_red("End of waypoints. Shutting down...").c_str());
                rclcpp::shutdown();
            }            
        }
    }
}

void Turtlebot3::WaypointPublisher::publish_waypoint() {
    std::ostringstream oss;

    oss << "Size of list: " << waypoint_list_.size();
    RCLCPP_INFO(this->get_logger(), output_white(oss.str()).c_str());

    // Get the current waypoint from the list
    geometry_msgs::msg::Pose2D new_waypoint;
    mage_msgs::msg::PartPose current_waypoint = waypoint_list_.front();

    new_waypoint.x = current_waypoint.pose.position.x;
    new_waypoint.y = current_waypoint.pose.position.y;
    new_waypoint.theta = 0.0;

    // Build and log the message
    oss.str("");
    oss << std::fixed << std::setprecision(2);
    oss << "Published waypoint (x: " << new_waypoint.x
        << ", y: " << new_waypoint.y
        << ", theta: " << new_waypoint.theta << ")";

    // Log the color and type of the part being moved to
    if (current_waypoint.part.type == 99) // Check if type is 99 means if it has to go to home position
    {
      RCLCPP_INFO(this->get_logger(),
                output_yellow("Going to home position " ).c_str());  
    }
    else
    {
      RCLCPP_INFO(this->get_logger(),
                output_yellow("Going to " + get_color_from_part(current_waypoint.part.color)
                              + " " + get_type_from_part(current_waypoint.part.type)).c_str());  
    }
   
    RCLCPP_INFO(this->get_logger(), output_yellow(oss.str()).c_str());

    // Publish the new waypoint and remove it from the list
    waypoint_publisher_->publish(new_waypoint);
    waypoint_list_.pop(); // Remove waypoint after publishing
}

void Turtlebot3::WaypointPublisher::waypoint_from_part_finder_callback(const mage_msgs::msg::PartPose::SharedPtr msg) {   
    RCLCPP_INFO(this->get_logger(), 
    "Waypoint Received from Part Finder. (x=%.2f, y=%.2f)", msg->pose.position.x, msg->pose.position.y);
    waypoint_list_.push(*msg); 
    waypoint_list_inititated_ = true;  // Set flag to true
}

// Function to map part color to a string
std::string Turtlebot3::WaypointPublisher::get_color_from_part(const uint8_t& color) {
    switch (color) {
        case mage_msgs::msg::Part::RED:
            return "RED";      // RED
        case mage_msgs::msg::Part::GREEN:
            return "GREEN";    // GREEN
        case mage_msgs::msg::Part::BLUE:
            return "BLUE";     // BLUE
        case mage_msgs::msg::Part::ORANGE:
            return "ORANGE";   // ORANGE
        case mage_msgs::msg::Part::PURPLE:
            return "PURPLE";   // PURPLE
        default:
            return "N/A";      // Default color if not found
    }
}

// Function to map part type to a string
std::string Turtlebot3::WaypointPublisher::get_type_from_part(const uint8_t& type) {
    switch (type) {
        case mage_msgs::msg::Part::BATTERY:
            return "BATTERY";  // BATTERY (10)
        case mage_msgs::msg::Part::PUMP:
            return "PUMP";     // PUMP (11)
        case mage_msgs::msg::Part::SENSOR:
            return "SENSOR";   // SENSOR (12)
        case mage_msgs::msg::Part::REGULATOR:
            return "REGULATOR"; // REGULATOR (13)
        default:
            return "SENSOR";    // Default type if not found (return "SENSOR")
    }
}