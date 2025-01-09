#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SpawnEntity
from final_project.spawn_params import SpawnParams
from final_project.spawn_params import PartSpawnParams


class PartSpawner(Node):
    def __init__(self):
        super().__init__("part_spawner")

        # Create service client to spawn objects into gazebo
        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")

    def spawn_part(self, name, type, color, xyz, rpy):
        params = PartSpawnParams(name, type, color, xyz=xyz, rpy=rpy)

        self.spawn_entity(params)

    def spawn_entity(self, params: SpawnParams) -> bool:
        self.spawn_client.wait_for_service()

        self.get_logger().info(f"Spawning: {params.name}")

        req = SpawnEntity.Request()

        req.name = params.name
        req.xml = params.xml
        req.initial_pose = params.initial_pose
        req.robot_namespace = params.robot_namespace
        req.reference_frame = params.reference_frame

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success


if __name__ == "__main__":
    rclpy.init()

    part_spawner = PartSpawner()
    part_spawner.spawn_part(
        "blue_battery_1",
        "battery",
        "blue",
        [0.831388, 4.142605, 1.5],
        [0, 0, 0.77],
    )

    part_spawner.spawn_part(
        "red_regulator_1",
        "regulator",
        "red",
        [0.254164, 4.737885, 1.5],
        [0, 0, 0.77],
    )

    part_spawner.spawn_part(
        "green_battery_1", "battery", "green", [5.0, 5.0, 1.5], [0, 0, 1.57]
    )

    part_spawner.spawn_part(
        "red_battery_1", "battery", "red", [5.0, 0.0, 1.5], [0, 0, -1.57]
    )
    part_spawner.spawn_part(
        "orange_battery_1",
        "battery",
        "orange",
        [5.0, -5.0, 1.0],
        [0, 0, 0],
    )
    part_spawner.spawn_part(
        "green_pump_2",
        "pump",
        "green",
        [3.806237, -5.0, 1.444065],
        [0, 0, 0],
    )
    part_spawner.spawn_part(
        "purple_battery_1",
        "battery",
        "purple",
        [0.392852, -4.231281, 1.0],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.spawn_part(
        "red_pump_1",
        "pump",
        "red",
        [0.0, -5.0, 1.0],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.spawn_part(
        "orange_regulator_1",
        "regulator",
        "orange",
        [-0.526753, -5.526753, 1.0],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.spawn_part(
        "blue_battery_2",
        "battery",
        "blue",
        [-4.753633, -5.891300, 1.171726],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.spawn_part(
        "blue_battery_3",
        "battery",
        "blue",
        [-4.438945, -4.839310, 1.171726],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.spawn_part(
        "red_battery_2",
        "battery",
        "red",
        [-4.628356, -0.371644, 1.0],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.spawn_part(
        "green_pump_1",
        "pump",
        "green",
        [-5.285342, 0.285342, 1.0],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.spawn_part(
        "red_battery_3",
        "battery",
        "red",
        [-4.642777, 5.357223, 1],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.spawn_part(
        "purple_regulator_1",
        "regulator",
        "purple",
        [-5.420137, 4.685093, 1],
        [0, 0.0, -0.78539816339],
    )

    part_spawner.destroy_node()
    rclpy.shutdown()
