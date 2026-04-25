from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gz_args = LaunchConfiguration("gz_args")
    pkg_share = FindPackageShare("ml25d_dataset_generation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_args",
                default_value=["-r ", PathJoinSubstitution([pkg_share, "worlds", "ml25d_empty.sdf"])],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
                    )
                ),
                launch_arguments={"gz_args": gz_args}.items(),
            ),
        ]
    )
