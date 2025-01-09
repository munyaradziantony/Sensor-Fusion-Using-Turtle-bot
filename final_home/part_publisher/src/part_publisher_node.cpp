
#include <mage_msgs/msg/part.hpp>
#include <mage_msgs/msg/parts.hpp>
#include <part_publisher_node.hpp>
#include <rclcpp/rclcpp.hpp>

void PartPublisher::publish_parts() {
    parts_msg_.parts.clear();
    // red battery
    auto part = mage_msgs::msg::Part();
    // part.color = mage_msgs::msg::Part::RED;
    // part.type = mage_msgs::msg::Part::BATTERY;
    // parts_msg_.parts.push_back(part);

    // red regulator
    part.color = mage_msgs::msg::Part::RED;
    part.type = mage_msgs::msg::Part::REGULATOR;
    parts_msg_.parts.push_back(part);

    // // blue battery
    // part.color = mage_msgs::msg::Part::BLUE;
    // part.type = mage_msgs::msg::Part::BATTERY;
    // parts_msg_.parts.push_back(part);

    // // green pump
    // part.color = mage_msgs::msg::Part::GREEN;
    // part.type = mage_msgs::msg::Part::PUMP;
    // parts_msg_.parts.push_back(part);

    part_publisher_->publish(parts_msg_);
}