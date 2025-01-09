#include <part_publisher_node.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PartPublisher>("part_publisher"));
    rclcpp::shutdown();
}