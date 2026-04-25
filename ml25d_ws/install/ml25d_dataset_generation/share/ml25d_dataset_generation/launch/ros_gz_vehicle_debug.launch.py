from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": "-r /opt/ros/jazzy/share/ros_gz_sim_demos/worlds/vehicle.sdf"
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/vehicle/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/vehicle/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/world/demo/set_pose@ros_gz_interfaces/srv/SetEntityPose",
        ],
        output="screen",
    )

    return LaunchDescription([gz_sim, bridge])
