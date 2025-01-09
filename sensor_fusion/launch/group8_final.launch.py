import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "rviz", default_value="false", description="Whether to start RViz"
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="Use simulation time"
        ),
    ]
    
    # Set the environment variable for TURTLEBOT3_MODEL
    set_turtlebot_model = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL', value='waffle'
    )
    
    # Path to the final_project package
    final_project_share = FindPackageShare(package="final_project").find("final_project")

    # Include the original launch file from final_project
    include_final_project_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(final_project_share, 'launch', 'final_project.launch.py')
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Add additional nodes from your sensor_fusion package (main.cpp node)
    # Specify the executable name generated by the build system (replace "main_node" with your actual executable name)
    main_cpp_node = Node(
        package="sensor_fusion",         # Package where main.cpp is located
        executable="waypoint_main",     # Executable name generated from main.cpp
        name="main_node",               # Name of the node
        output="screen",                # Output to screen
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # Create a TimerAction to delay the launch of the main_cpp_node by 10 seconds
    delayed_main_cpp_node = TimerAction(
        period=10.0,  # Delay in seconds
        actions=[main_cpp_node]  # Action to execute after the delay
    )

    # Return a LaunchDescription containing all nodes
    return LaunchDescription(
        declared_arguments + [
            set_turtlebot_model,           # Set the environment variable
            include_final_project_launch,  # Include the final_project launch
            delayed_main_cpp_node,         # Add the delayed main.cpp node
        ]
    )
