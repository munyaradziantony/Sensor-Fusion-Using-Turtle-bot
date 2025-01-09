#include "waypoint_publisher.hpp"
#include "waypoint_reacher.hpp"
#include "part_pose_finder.hpp"

int main(int argc, char **argv) {
    // Initialize ROS 2 client library
    rclcpp::init(argc, argv);

    // Create instances of the WaypointPublisher and WaypointReacher nodes
    auto waypoint_publisher_node = std::make_shared<Turtlebot3::WaypointPublisher>("waypoint_publisher");
    auto waypoint_reacher_node = std::make_shared<Turtlebot3::WaypointReacher>("waypoint_reacher");
    auto part_pose_finder_node = std::make_shared<Turtlebot3::PartPoseFinder>("part_pose_finder");

    // Create a multi-threaded executor to run both nodes concurrently
    rclcpp::executors::MultiThreadedExecutor executor;

    // Add both nodes to the executor
    executor.add_node(waypoint_reacher_node);
    executor.add_node(waypoint_publisher_node);
    executor.add_node(part_pose_finder_node);

    // Spin the executor to process incoming callbacks for both nodes
    executor.spin();

    // Shutdown ROS 2 client library
    rclcpp::shutdown();
    return 0;
}