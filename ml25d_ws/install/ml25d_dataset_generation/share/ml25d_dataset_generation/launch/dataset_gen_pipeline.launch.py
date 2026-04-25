from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_dir = LaunchConfiguration("config_dir")
    output_dir = LaunchConfiguration("output_dir")
    backend = LaunchConfiguration("backend")
    num_samples = LaunchConfiguration("num_samples")
    seed = LaunchConfiguration("seed")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_dir",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ml25d_dataset_generation"), "config"]
                ),
            ),
            DeclareLaunchArgument("output_dir", default_value="data/generated"),
            DeclareLaunchArgument("backend", default_value="surrogate"),
            DeclareLaunchArgument("num_samples", default_value="100"),
            DeclareLaunchArgument("seed", default_value="42"),
            Node(
                package="ml25d_dataset_generation",
                executable="sim_bridge_node",
                name="sim_bridge_node",
                output="screen",
            ),
            Node(
                package="ml25d_dataset_generation",
                executable="generate_dataset.py",
                name="dataset_generator",
                output="screen",
                arguments=[
                    "--config-dir",
                    config_dir,
                    "--output-dir",
                    output_dir,
                    "--backend",
                    backend,
                    "--num-samples",
                    num_samples,
                    "--seed",
                    seed,
                ],
            ),
        ]
    )
