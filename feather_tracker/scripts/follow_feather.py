#!/usr/bin/env python3
"""
A script to follow an peacock feather with a robot arm using MoveItPy.
"""

# generic ros libraries
import rclpy
from rclpy.node import Node
from rclpy.time import Time

# moveit python library
import tf2_ros
import transforms3d
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from moveit.planning import MoveItPy
#from ros2_aruco_interfaces.msg import ArucoMarkers
from yolo_inference import YOLOInferenceNode
from tf2_geometry_msgs import do_transform_pose


class FeatherFollower(Node):

    def __init__(self, moveit: MoveItPy):
        super().__init__("feather_follower")
        self.logger = self.get_logger()

        self.moveit = moveit
        self.arm = self.moveit.get_planning_component("ar_manipulator")

        # ID of the aruco marker mounted on the robot
        self.marker_id = self.declare_parameter(
            "marker_id", 1).get_parameter_value().integer_value


        
        self.subscription = self.create_subscription(YOLOInferenceNode,
                                                     "/detections",
                                                     self.handle_detections,
                                                     1)
        self.pose_pub = self.create_publisher(PoseStamped, "/feather_pose",
                                              1)

        self.target_pose_pub = self.create_publisher(
            PoseStamped, "/follow_feather_target_pose", 1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._prev_marker_pose = None

    def handle_detections(self, msg: YOLOInferenceNode):
        feather_pos = None
        for i, marker_id in enumerate(msg.marker_ids):
            if marker_id == self.marker_id:
                feather_pos = msg.poses[i]
                break

        if feather_pos is None:
            return

        # only start following if the marker pose has changed by at least 2cm
        if self._prev_marker_pose is not None:
            if ((feather_pos.position.x -
                 self._prev_marker_pose.position.x)**2 +
                (feather_pos.position.y -
                 self._prev_marker_pose.position.y)**2 +
                (feather_pos.position.z -
                 self._prev_marker_pose.position.z)**2 > 0.02**2):
                self._prev_marker_pose = feather_pos
                return

        self._prev_marker_pose = feather_pos

        # get pose in robot base frame
        try:
            transformed_pose = self._transform_pose(feather_pos,
                                                    "camera_color_optical_frame",
                                                    "base_link")
        except tf2_ros.LookupException as e:
            self.logger.error(f"Error transforming pose: {e}")
            return

        self.logger.info(f"Following feather at pose: {transformed_pose}")

        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose = transformed_pose

        # set plan start state to current state
        self.arm.set_start_state_to_current_state()

        # set pose goal with PoseStamped message
        self.arm.set_goal_state(pose_stamped_msg=pose_goal,
                                pose_link="ee_link")

        # plan to goal
        self._plan_and_execute()

    def _transform_pose(self, pose: Pose, source_frame,
                        target_frame: str) -> Pose:
        # Get the transform from source frame to target frame
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                    Time())
        # Transform the pose
        transformed_pose = do_transform_pose(pose, transform)
        # publish pose
        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = target_frame
        stamped_pose.pose = transformed_pose
        self.pose_pub.publish(stamped_pose)

        pose.position.z += 0.05
        transformed_pose = do_transform_pose(pose, transform)

        stamped_pose = PoseStamped()
        stamped_pose.header.frame_id = target_frame
        stamped_pose.pose = transformed_pose
        self.target_pose_pub.publish(stamped_pose)
        return transformed_pose

    def _plan_and_execute(self):
        """Helper function to plan and execute a motion."""
        # plan to goal
        self.logger.info("Planning trajectory")
        plan_result = self.arm.plan()

        # execute the plan
        if plan_result:
            self.logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
        else:
            self.logger.error("Planning failed")


def main():
    rclpy.init()
    moveit = MoveItPy(node_name="moveit_py")
    node = FeatherFollower(moveit)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
