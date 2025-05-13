#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# moveit_py modules (namespaces may differ depending on the version).
from moveit_py.core import RobotInterface
from moveit_py.planning import PlanParameters, MoveItPy
from moveit_py.robot import PlanningComponent

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        # Initialize MoveItPy main interface
        self.moveit_py = MoveItPy(node_name=self.get_name())
        
        # Get a RobotInterface (abstract interface to the entire robot)
        self.robot_interface = self.moveit_py.get_robot_interface()
        
        # Create a PlanningComponent for the robot arm
        self.arm = self.moveit_py.get_planning_component("ar_manipulator")
        
    def plan_and_move(self):
        # Example: Plan to a predefined joint configuration or pose
        # (In a real scenario, you'd set your target angles or pose below)
        
        # 1) Define Joint Goal (replace with actual values for your robot)
        joint_goal = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        self.arm.set_joint_goal(joint_goal)

        # 2) Plan
        plan_result = self.arm.plan()
        if plan_result:
            self.get_logger().info("Plan succeeded, executing...")
            # 3) Execute
            self.arm.execute(plan_result.trajectory)
        else:
            self.get_logger().error("Plan failed!")

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    node.plan_and_move()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#
