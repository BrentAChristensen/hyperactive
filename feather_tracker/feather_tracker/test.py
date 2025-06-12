from threading import Thread  # Unused import; consider removing if not needed
import rclpy
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ar4 as robot

# Initialize ROS 2 Python client library
rclpy.init()

# Create node for this example
node = Node("ex_pose_goal")

node.declare_parameter("position", [0.45, 0.0, 0.3])
node.declare_parameter("quat_xyzw", [0.0, 0.707, 0.0, 0.707])
node.declare_parameter("cartesian", True)

moveit2 = MoveIt2(
    node=node,
    joint_names=robot.joint_names(),
    base_link_name="base_link",
    end_effector_name="ee_link",
    group_name="ar_manipulator",
)

moveit2.execute = True  # Critical!

moveit2.cartesian_avoid_collisions = cartesian_avoid_collisions
moveit2.cartesian_jump_threshold = cartesian_jump_threshold

position = node.get_parameter("position").get_parameter_value().double_array_value
quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
# target_pose = [0.4, 0.0, 0.2]
# target_quat = [0.0, 0.0, 0.0, 1.0]  # No rotation

moveit2.move_to_pose(position,quat_xyzw,)

moveit2.wait_until_executed()
