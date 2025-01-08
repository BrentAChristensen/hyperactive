#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from yolo_msgs.msg import DetectionArray
import math

class DetectionToPoseNode(Node):
    def __init__(self, target_class_name, detection_topic):
        super().__init__('detection_to_pose_node')

        # Store the target class name
        self.target_class_name = target_class_name
        
        # Create a subscription with the provided topic
        self.detection_subscriber = self.create_subscription(
            DetectionArray,
            detection_topic,
            self.detection_callback,
            10
        )

        # Publisher for mapped poses
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/mapped_pose',
            10
        )
        self.previous_position = None
        self.change_threshold = 0.02

    def detection_callback(self, detection_array_msg):
        for detection in detection_array_msg.detections:
            if detection.class_name == self.target_class_name:  # Use the class name argument
                pose_msg = PoseStamped()
                
                # Extracting position from bbox3d center
                pose_msg.pose.position.x = detection.bbox3d.center.position.x
                pose_msg.pose.position.y = detection.bbox3d.center.position.y
                pose_msg.pose.position.z = detection.bbox3d.center.position.z
                
                # Extracting orientation from bbox3d center
                pose_msg.pose.orientation.x = detection.bbox3d.center.orientation.x
                pose_msg.pose.orientation.y = detection.bbox3d.center.orientation.y
                pose_msg.pose.orientation.z = detection.bbox3d.center.orientation.z
                pose_msg.pose.orientation.w = detection.bbox3d.center.orientation.w
                
                # Frame ID from the detection
                pose_msg.header.frame_id = detection.bbox3d.frame_id
                
                # Set the current time stamp
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                
                current_position = (
                    pose_msg.pose.position.x,
                    pose_msg.pose.position.y,
                    pose_msg.pose.position.z
                )
                
                if self.previous_position is None or self.has_significant_change(current_position):
                    # Publish the mapped PoseStamped message
                    self.pose_publisher.publish(pose_msg)
                    self.previous_position = current_position
                    # Create a detailed string representation of the pose_msg
                    pose_msg_str = (
                        f"PoseStamped:\n"
                        f"  Frame ID: {pose_msg.header.frame_id}\n"
                        f"  Timestamp: {pose_msg.header.stamp}\n"
                        f"  Position: (x: {pose_msg.pose.position.x}, "
                        f"y: {pose_msg.pose.position.y}, "
                        f"z: {pose_msg.pose.position.z})\n"
                        f"  Orientation: (x: {pose_msg.pose.orientation.x}, "
                        f"y: {pose_msg.pose.orientation.y}, "
                        f"z: {pose_msg.pose.orientation.z}, "
                        f"w: {pose_msg.pose.orientation.w})"
                    )

                    # Log the detailed pose message
                    self.get_logger().info(pose_msg_str)
                
    def has_significant_change(self, current_position):
        if self.previous_position is None:
            return True
        
        delta_x = current_position[0] - self.previous_position[0]
        delta_y = current_position[1] - self.previous_position[1]
        delta_z = current_position[2] - self.previous_position[2]
        
        # Calculate Euclidean distance
        distance = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        #self.get_logger().info(f'Distance computed: {distance}')
        return distance > self.change_threshold

def main(args=None):
    rclpy.init(args=args)

    # Define the class name and topic as needed
    target_class_name = "bare peacock quill"
    detection_topic = '/yolo/detections_3d'

    node = DetectionToPoseNode(target_class_name, detection_topic)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
