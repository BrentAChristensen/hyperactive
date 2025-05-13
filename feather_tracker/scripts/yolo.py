#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
from ultralytics.utils.plotting import colors
from collections import defaultdict
import tf_transformations


class RealSenseListener(Node):
    def __init__(self):
        super().__init__('realsense_listener')

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Initialize YOLO model
        self.model = YOLO("/home/brent/ar4_ws/data/peacockv2/peacock/runs/detect/train6/weights/best.pt")
        self.names = self.model.model.names

        # Track history for each ID
        self.track_history = defaultdict(lambda: [])

        # Subscription to the color image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Subscription to the depth image topic
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # Publisher for YOLO annotated images
        self.image_publisher = self.create_publisher(
            Image,
            'yolo_annotated_image',
            10
        )

        # Transform broadcaster for "bare peacock quill"
        self.tf_broadcaster = TransformBroadcaster(self)

        # Storage for the latest depth image
        self.latest_depth_frame = None

        # Camera intrinsic parameters - Example values, replace with actual camera intrinsics
        self.fx = 618.148  # Focal length in x axis
        self.fy = 614.788  # Focal length in y axis
        self.cx = 323.961  # Optical center x
        self.cy = 237.724  # Optical center y

    def depth_callback(self, msg: Image):
        try:
            # Convert ROS Image message to OpenCV image (16UC1 format)
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")

    def image_callback(self, msg: Image):
        if self.latest_depth_frame is None:
            self.get_logger().warning("No depth image received yet.")
            return

        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.track(frame, persist=True, verbose=False)
        boxes = results[0].boxes.xyxy.cpu()

        if results[0].boxes.id is not None:
            # Extract prediction results
            clss = results[0].boxes.cls.cpu().tolist()
            track_ids = results[0].boxes.id.int().cpu().tolist()
            confs = results[0].boxes.conf.float().cpu().tolist()

            # Annotate frame
            annotator = Annotator(frame, line_width=2)

            for box, cls, track_id in zip(boxes, clss, track_ids):
                annotator.box_label(box, color=colors(int(cls), True), label=self.names[int(cls)])
                
                # Track handling
                track = self.track_history[track_id]
                track.append((int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)))
                if len(track) > 30:
                    track.pop(0)

                # Plot tracks
                points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
                cv2.circle(frame, track[-1], 7, colors(int(cls), True), -1)
                cv2.polylines(frame, [points], isClosed=False, color=colors(int(cls), True), thickness=2)

                # Publish TransformStamped if class is "bare peacock quill"
                if self.names[int(cls)] == "bare peacock quill":
                    x_center = float((box[0] + box[2]) / 2)
                    y_center = float((box[1] + box[3]) / 2)
                    
                    # Retrieve Z coordinate using depth image
                    z_coord_mm = self.get_depth_value(int(x_center), int(y_center))

                    # Convert to real-world coordinates
                    if z_coord_mm > 0:
                        real_world_coord = self.calculate_real_world_coordinates(x_center, y_center, z_coord_mm)
                        
                        # Create TransformStamped message
                        transform_msg = TransformStamped()
                        transform_msg.header.stamp = self.get_clock().now().to_msg()
                        transform_msg.header.frame_id = "base_link"  # Replace with your frame id
                        transform_msg.child_frame_id = f"quill_{track_id}"  # Unique child frame for each detected quill

                        transform_msg.transform.translation.x = real_world_coord[0]
                        transform_msg.transform.translation.y = real_world_coord[1]
                        transform_msg.transform.translation.z = real_world_coord[2]

                        # Identity quaternion representing no rotation
                        q = tf_transformations.quaternion_from_euler(0, 0, 0)
                        transform_msg.transform.rotation.x = q[0]
                        transform_msg.transform.rotation.y = q[1]
                        transform_msg.transform.rotation.z = q[2]
                        transform_msg.transform.rotation.w = q[3]
                        print (transform_msg)
                        # Broadcast the transform
                        self.tf_broadcaster.sendTransform(transform_msg)

        # Display the annotated frame
        cv2.imshow('RealSense', frame)
        cv2.waitKey(1)

        # Convert OpenCV image back to ROS Image message and publish
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(annotated_msg)

    def get_depth_value(self, x, y):
        # Ensure valid access within depth image bounds
        if self.latest_depth_frame is not None:
            if 0 <= x < self.latest_depth_frame.shape[1] and 0 <= y < self.latest_depth_frame.shape[0]:
                return float(self.latest_depth_frame[y, x])  # Depth in millimeters
        return 0.0

    def calculate_real_world_coordinates(self, x, y, z_mm):
        """
        Convert image coordinates (x, y) and depth z in millimeters to 
        real-world coordinates (X, Y, Z) in meters.
        """
        X = (x - self.cx) * z_mm / self.fx
        Y = (y - self.cy) * z_mm / self.fy
        Z = z_mm * 0.001  # Convert mm to meters
        return (X, Y, Z)

def main(args=None):
    rclpy.init(args=args)

    realsense_listener = RealSenseListener()

    print("Processing images from RealSense...")

    try:
        rclpy.spin(realsense_listener)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
