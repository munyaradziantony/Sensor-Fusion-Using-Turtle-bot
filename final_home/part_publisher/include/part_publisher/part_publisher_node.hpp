#include <mage_msgs/msg/parts.hpp>
#include <rclcpp/rclcpp.hpp>

class PartPublisher : public rclcpp::Node {
   public:
    PartPublisher(std::string node_name)
        : rclcpp::Node(node_name) {
        // Initialize the publisher object
        part_publisher_ = this->create_publisher<mage_msgs::msg::Parts>("parts", 10);
        // Initialize the message to be published
        parts_msg_ = mage_msgs::msg::Parts();
        // Initialize the timer
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PartPublisher::publish_parts, this));
    }

   private:
    void publish_parts();
    rclcpp::Publisher<mage_msgs::msg::Parts>::SharedPtr part_publisher_;
    mage_msgs::msg::Parts parts_msg_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};