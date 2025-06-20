import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


###############################################################################
# Helper functions
###############################################################################


def _load_file(package_name: str, relative_path: str) -> str | None:
    """Return the contents of a file installed in a ROS 2 package."""
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, relative_path)
    try:
        with open(abs_path, "r") as f:
            return f.read()
    except (EnvironmentError, FileNotFoundError):
        return None


def _load_yaml(package_name: str, relative_path: str) -> dict | None:
    """Return YAML as a Python dict from a file inside a ROS 2 package."""
    pkg_path = get_package_share_directory(package_name)
    abs_path = os.path.join(pkg_path, relative_path)
    try:
        with open(abs_path, "r") as f:
            return yaml.safe_load(f)
    except (EnvironmentError, FileNotFoundError):
        return None


###############################################################################
# Launch description
###############################################################################


def generate_launch_description() -> LaunchDescription:
    # ----------------------------------------------------------------------------
    #  MoveIt configuration
    # ----------------------------------------------------------------------------
    pkg_rover = get_package_share_directory("rove_config")
    xacro_path = os.path.join(pkg_rover, "config", "rove_full_low_res.urdf.xacro")
    srdf_path = os.path.join(pkg_rover, "config", "rove_full_low_res.srdf")
    kinematics_path = os.path.join(pkg_rover, "config", "kinematics.yaml")

    moveit_config = (
        MoveItConfigsBuilder("rove_full_low_res", package_name="rove_config")
        .robot_description(file_path=xacro_path)
        .to_moveit_configs()
    )

    # ----------------------------------------------------------------------------
    #  Parameter blocks
    # ----------------------------------------------------------------------------
    servo_yaml = _load_yaml("rove_config", "config/rove_servo_parameters.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    rviz_config = os.path.join(pkg_rover, "config", "moveit.rviz")
    rviz_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
    ]
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=rviz_parameters,
    )

    joint_limits_yaml = _load_yaml("rove_config", "config/joint_limits.yaml")
    joint_limits_params = {"robot_description_planning": joint_limits_yaml}

    # The real / simulated joint‑state topic
    joint_state_topic = "/kinova_arm/joint_states"  # change if needed

    # ----------------------------------------------------------------------------
    #  RViz
    # ----------------------------------------------------------------------------
    

    # ----------------------------------------------------------------------------
    #  Servo node (stand‑alone process)
    # ----------------------------------------------------------------------------
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            joint_limits_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        remappings=[("/joint_states", joint_state_topic)],
        output="screen",
    )

    # ----------------------------------------------------------------------------
    #  Robot‑state publisher
    # ----------------------------------------------------------------------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    # ----------------------------------------------------------------------------
    #  Arm trajectory controller (example stub)
    # ----------------------------------------------------------------------------
    traj_ctrl_node = Node(
        package="arm_trajectory_controller",
        executable="arm_trajectory_controller_node",
        name="arm_trajectory_controller_node",
        output="screen",
    )

    # ----------------------------------------------------------------------------
    #  Optional composable container for future nodes
    # ----------------------------------------------------------------------------
    container = ComposableNodeContainer(
        name="rove_servo_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::JoyToServoPub",
                name="joy_to_servo",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
    )

    # Return the aggregated LaunchDescription
    return LaunchDescription(
        [
            rviz_node,
            rsp_node,
            traj_ctrl_node,
            servo_node,
            container,
        ]
    )
