#!/usr/bin/env python3

import os
import cv2
import sys
import numpy as np
import argparse
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3, TransformStamped
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# Add TF2 imports
import tf2_ros
import geometry_msgs.msg
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import math

from transforms3d.quaternions import mat2quat

from ultralytics import YOLO
from config_reader import ConfigurationReader, load_detector_configuration
from custom_interfaces.msg import HookStatus
from std_msgs.msg import Header

class LadleDetectorNode(Node):
    def __init__(self, config_name="west"):
        super().__init__('ladle_detector_node')


        # Load configuration
        try:
            self.config_reader = ConfigurationReader("config.json")
            self.config = self.config_reader.get_configuration(config_name)
            self.config_reader.print_configuration_summary(config_name)
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration '{config_name}': {e}")
            rclpy.shutdown()
            sys.exit(1)

        self.weights_path_hook = self.config["weights_path_hook"]
        self.weights_path_ladle = self.config["weights_path_ladle"]
        # TF frame IDs
        self.camera_frame_id = self.config["camera_frame_id"]
        self.ladle_frame_id = self.config["ladle_frame_id"]
        self.hook_frame_id = self.config["hook_frame_id"]

        self.video_path = self.config["video_path"]

        self.model_points_ladle = self.config["model_points_ladle"]
        self.model_points_hook = self.config["model_points_hook"]

        self.R_combined = self.config["R_combined_ladle"]
        self.R_combined_hook = self.config["R_combined_hook"]

        self.hook_point = self.config["hook_point"]
        self.ladle_point = self.config["ladle_point"]

        # Inference parameters
        self.conf_threshold = self.config["conf_threshold"]
        self.iou_threshold = self.config["iou_threshold"]

        self.x_offset = self.config["x_offset"]
        self.y_offset = self.config["y_offset"]
        # parameters 
        self.focal_length = 1.0
        self.target_size = (512, 512)

        self.show_display = True

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Create TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Load YOLO model
        try:
            self.model_ladle = YOLO(self.weights_path_ladle)
            self.get_logger().info(f"YOLO model loaded from: {self.weights_path_ladle}")
            self.model_hook = YOLO(self.weights_path_hook)
            self.get_logger().info(f"YOLO model loaded from: {self.weights_path_hook}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            rclpy.shutdown()
            sys.exit(1)

        # Check if video path is provided
        if not self.video_path:
            self.get_logger().error("No video path provided. Use: ros2 run <package> ladle_detector_node --ros-args -p video_path:=<path>")
            rclpy.shutdown()
            sys.exit(1)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create timer for video processing
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video: {self.video_path}")
            rclpy.shutdown()
            sys.exit(1)

        # Get video properties
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.get_logger().info(f"Video: {self.width}x{self.height} at {self.fps} fps")

        timer_period = 1.0 / self.fps if self.fps > 0 else 0.05  # Default to 30fps
        self.timer = self.create_timer(timer_period, self.process_frame)

        self.frame_count = 0

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.output_filename = "output_inference.mp4"
        self.video_writer = cv2.VideoWriter(
            self.output_filename,
            fourcc,
            self.fps,
            (self.target_size[0], self.target_size[1])
        )

        if self.fps <= 0:
            self.fps = 30.0
        self.frame_delay = int(1000.0 / self.fps)  # in milliseconds

        self.prev_rvec_ladle = None
        self.prev_tvec_ladle = None

        self.prev_rvec_hook = None
        self.prev_tvec_hook = None

        self.publisher = self.create_publisher(HookStatus, '/is_good', 10)

        self.get_logger().info("Ladle detector node initialized")
            
    def center_crop_resize(self, img, target_size, x_offset=0.0, y_offset=0.0):
        """Center crop to target aspect ratio with offset, then resize"""
        h, w = img.shape[:2]
        target_ratio = target_size[1] / target_size[0]
        current_ratio = h / w
        
        if current_ratio > target_ratio:
            # Crop height
            new_h = int(w * target_ratio)
            center_y = h // 2 + int(y_offset * (h - new_h) / 2)
            start_y = max(0, min(center_y - new_h//2, h - new_h))
            img_cropped = img[start_y:start_y + new_h, :]
        else:
            # Crop width
            new_w = int(h / target_ratio)
            center_x = w // 2 + int(x_offset * (w - new_w) / 2)
            start_x = max(0, min(center_x - new_w//2, w - new_w))
            img_cropped = img[:, start_x:start_x + new_w]
        
        # Resize to target
        return cv2.resize(img_cropped, target_size)
    
    def Rx(self, A):
        """Rotation matrix around X axis by angle A (in radians)"""
        return np.array([[1,             0,              0],
                        [0, math.cos(A), -math.sin(A)],
                        [0, math.sin(A),  math.cos(A)]])

    def Ry(self, A):
        """Rotation matrix around Y axis by angle A (in radians)"""
        return np.array([[ math.cos(A), 0, math.sin(A)],
                        [           0, 1,           0],
                        [-math.sin(A), 0, math.cos(A)]])

    def Rz(self,A):
        """Rotation matrix around Z axis by angle A (in radians)"""
        return np.array([[math.cos(A), -math.sin(A), 0],
                        [math.sin(A),  math.cos(A), 0],
                        [         0,            0, 1]])
    
    def make_transform(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3,  3] = tvec.flatten()
        return T


    def draw_pose_keypoints(self, img, keypoints, skeleton=None):
        """Draw pose keypoints and skeleton on image"""
        if keypoints is None or len(keypoints) == 0:
            return img
        
        # Keypoints format: [x, y, confidence]
        for i, kp in enumerate(keypoints):
            if len(kp) >= 3:
                x, y, conf = kp[0], kp[1], kp[2]
                if conf > 0.5:  # Only draw confident keypoints
                    cv2.circle(img, (int(x), int(y)), 2, (0, 255, 0), -1)
                    # Add keypoint index
                    cv2.putText(img, str(i), (int(x) + 8, int(y) - 8), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
            
        return img
    
    def validate_keypoints(self, keypoints_2d, model_points_3d=None):

                # Filter confident keypoints and get corresponding 3D points
        valid_2d_points = []
        valid_3d_points = []
        
        for i, kp in enumerate(keypoints_2d):
            if len(kp) >= 3 and kp[2] > 0.5 and i < len(model_points_3d):
                valid_2d_points.append([kp[0], kp[1]])
                valid_3d_points.append(model_points_3d[i])
        
        if len(valid_2d_points) < 4:  # Need at least 4 points for PnP
            return None, None
        
        valid_2d_points = np.array(valid_2d_points, dtype=np.float32)
        valid_3d_points = np.array(valid_3d_points, dtype=np.float32)

        return valid_2d_points, valid_3d_points
    
    def run_inference_ladle(self, frame, annotated_frame, cameraMatrix=None, mdists=None, timestamp=None):
        """Run YOLO pose inference on frame"""
        try:
            # Run inference
            results = self.model_ladle(frame, conf=self.conf_threshold, iou=self.iou_threshold)
            rvec_ladle = None
            p_ladle_cam = None
            rvec_ladle_rotated = None

            for result in results:
                
                # Simple check: if no keypoints or keypoints.conf is None, skip this result
                if (not hasattr(result, 'keypoints') or 
                    result.keypoints is None or 
                    not hasattr(result.keypoints, 'conf') or 
                    result.keypoints.conf is None or
                    not hasattr(result.keypoints, 'xy') or 
                    result.keypoints.xy is None):
                    continue
                    
                # Draw pose keypoints if available
                keypoints = result.keypoints.xy.cpu().numpy()  # Get x,y coordinates
                confidences = result.keypoints.conf.cpu().numpy()
                        
                for i, kp_set in enumerate(keypoints):
                    # Combine coordinates with confidences if available
                    if confidences is not None and i < len(confidences):
                        kp_with_conf = np.column_stack([kp_set, confidences[i]])
                    else:
                        kp_with_conf = np.column_stack([kp_set, np.ones(len(kp_set))])
                    
                    # Draw keypoints
                    # annotated_frame = self.draw_pose_keypoints(annotated_frame, kp_with_conf)

                    # Validate keypoints for PnP
                    valid_2d_points, valid_3d_points = self.validate_keypoints(kp_with_conf, self.model_points_ladle)
                    
                    if valid_2d_points is None:
                        continue

                    if self.prev_rvec_ladle is None:
                        success, rvec_ladle, tvec_ladle = cv2.solvePnP(
                            valid_3d_points,
                            valid_2d_points,
                            cameraMatrix,
                            mdists,
                            flags=cv2.SOLVEPNP_ITERATIVE
                        )
                    else:
                        success, rvec_ladle, tvec_ladle = cv2.solvePnP(
                            valid_3d_points,
                            valid_2d_points,
                            cameraMatrix,
                            mdists,
                            rvec=self.prev_rvec_ladle,
                            tvec=self.prev_tvec_ladle,
                            useExtrinsicGuess=True,
                            flags=cv2.SOLVEPNP_ITERATIVE
                        )

                    if success:
                        a_ladle = 0.4
                        if self.prev_rvec_ladle is not None:
                            rvec_ladle = a_ladle * self.prev_rvec_ladle + (1 - a_ladle) * rvec_ladle
                            tvec_ladle = a_ladle * self.prev_tvec_ladle + (1 - a_ladle) * tvec_ladle

                        # save for next frame
                        self.prev_rvec_ladle, self.prev_tvec_ladle = rvec_ladle, tvec_ladle

                        # rotation matrix and translation vector
                        rmat_ladle, _ = cv2.Rodrigues(rvec_ladle)
                        rmat_ladle_rotated = rmat_ladle @ self.R_combined
                        rvec_ladle_rotated, _ = cv2.Rodrigues(rmat_ladle_rotated)
                        
                        # Create a 3D point for the ladle and apply rotation correction
                        point_3D_ladle_original = self.model_points_ladle[self.ladle_point].reshape(3, 1)
                        
                        # Transform the point using the rotation correction matrix
                        point_3D_ladle_corrected = (np.linalg.inv(self.R_combined) @ point_3D_ladle_original).reshape(3, 1)
                        tvec_ladle_corrected = tvec_ladle.reshape(3, 1)

                        # Calculate ladle position in camera coordinates with corrected rotation
                        p_ladle_cam = rmat_ladle_rotated @ point_3D_ladle_corrected + tvec_ladle_corrected

                        # For visualization axes
                        point_3D_ladle_vis = point_3D_ladle_corrected.reshape(1, 3)
                        axis_length = 1.0
                        x_axis = point_3D_ladle_vis + np.array([[axis_length, 0, 0]], dtype=np.float64)
                        y_axis = point_3D_ladle_vis + np.array([[0, axis_length, 0]], dtype=np.float64)
                        z_axis = point_3D_ladle_vis + np.array([[0, 0, axis_length]], dtype=np.float64)

                        axis_points = np.vstack([point_3D_ladle_vis, x_axis, y_axis, z_axis])

                        # Project using the corrected rotation and corrected translation
                        axis_points_2D, jacobian = cv2.projectPoints(axis_points, rvec_ladle_rotated, tvec_ladle_corrected.flatten(), cameraMatrix, mdists)
                            
                        # Draw the axes
                        p0 = (int(axis_points_2D[0, 0, 0]), int(axis_points_2D[0, 0, 1]))
                        p1 = (int(axis_points_2D[1, 0, 0]), int(axis_points_2D[1, 0, 1]))
                        p2 = (int(axis_points_2D[2, 0, 0]), int(axis_points_2D[2, 0, 1]))
                        p3 = (int(axis_points_2D[3, 0, 0]), int(axis_points_2D[3, 0, 1]))

                        # Draw the three orientation lines with different colors
                        # cv2.line(annotated_frame, p0, p1, (0, 0, 255), thickness=2, lineType=cv2.LINE_AA)
                        # cv2.line(annotated_frame, p0, p2, (0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
                        # cv2.line(annotated_frame, p0, p3, (255, 0, 0), thickness=2, lineType=cv2.LINE_AA)

            return annotated_frame, results, rvec_ladle_rotated, p_ladle_cam
            
        except Exception as e:
            self.get_logger().error(f"Inference failed --- ladle: {e}")
            return frame, None, None, None
        
    def run_inference_hook(self, frame, annotated_frame, cameraMatrix=None, mdists=None, timestamp=None):
        """Run YOLO pose inference on frame"""
        try:
            # Run inference
            results = self.model_hook(frame, conf=self.conf_threshold, iou=self.iou_threshold)
            rvec_hook = None
            p_hook_cam = None
            rvec_hook_rotated = None
            detected_classes = []  # Store detected classes
            class_names = ['false', 'true']  # Your class names

            
            for result in results:
                # ====== HANDLE BOUNDING BOX DETECTION (INDEPENDENT FROM KEYPOINTS) ======
                if hasattr(result, 'boxes') and result.boxes is not None:
                    boxes = result.boxes
                    if len(boxes.cls) > 0:
                        classes = boxes.cls.cpu().numpy().astype(int)  # Get class indices
                        confidences_det = boxes.conf.cpu().numpy()     # Detection confidences
                        
                        # Store class information for each detection
                        for box_idx, (cls_idx, conf) in enumerate(zip(classes, confidences_det)):
                            class_name = class_names[cls_idx] if cls_idx < len(class_names) else f"class_{cls_idx}"
                            detected_classes.append({
                                'class_id': int(cls_idx),
                                'class_name': class_name,
                                'confidence': float(conf)
                            })
                            
                            # Log the detection
                            self.get_logger().info(f"Detected: {class_name} (confidence: {conf:.3f})")
                            
                            # Add text annotation on frame for each box
                            if box_idx < len(boxes.xyxy):
                                x1, y1, x2, y2 = boxes.xyxy[box_idx].cpu().numpy().astype(int)
                                label = f"{class_name}: {conf:.2f}"
                                
                                # Draw bounding box
                                color = (0, 255, 0) if cls_idx == 1 else (0, 0, 255)  # Green for 'true', Red for 'false'
                                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                                
                                # Draw label
                                cv2.putText(annotated_frame, label, (x1, y1-10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # ====== HANDLE KEYPOINT DETECTION (SEPARATE FROM BOXES) ======
                # Check if keypoints are available for pose estimation
                has_valid_keypoints = (hasattr(result, 'keypoints') and 
                                    result.keypoints is not None and 
                                    hasattr(result.keypoints, 'conf') and 
                                    result.keypoints.conf is not None and
                                    hasattr(result.keypoints, 'xy') and 
                                    result.keypoints.xy is not None)
                
                if has_valid_keypoints:
                        
                    # Draw pose keypoints if available
                    keypoints = result.keypoints.xy.cpu().numpy()  # Get x,y coordinates
                    confidences = result.keypoints.conf.cpu().numpy()
                        
                    for i, kp_set in enumerate(keypoints):
                        # Combine coordinates with confidences if available
                        if confidences is not None and i < len(confidences):
                            kp_with_conf = np.column_stack([kp_set, confidences[i]])
                        else:
                            kp_with_conf = np.column_stack([kp_set, np.ones(len(kp_set))])
                        
                        # Draw keypoints
                        annotated_frame = self.draw_pose_keypoints(annotated_frame, kp_with_conf)

                        valid_2d_points, valid_3d_points = self.validate_keypoints(kp_with_conf, self.model_points_hook)

                        if valid_2d_points is None:
                            continue

                        if self.prev_rvec_hook is None:
                            success, rvec_hook, tvec_hook = cv2.solvePnP(
                                valid_3d_points,
                                valid_2d_points,
                                cameraMatrix,
                                mdists,
                                flags=cv2.SOLVEPNP_ITERATIVE
                            )
                        else:
                            success, rvec_hook, tvec_hook = cv2.solvePnP(
                                valid_3d_points,
                                valid_2d_points,
                                cameraMatrix,
                                mdists,
                                rvec=self.prev_rvec_hook,
                                tvec=self.prev_tvec_hook,
                                useExtrinsicGuess=True,
                                flags=cv2.SOLVEPNP_ITERATIVE
                            )

                        if success:
                            a_hook = 0.6
                            if self.prev_rvec_hook is not None:
                                rvec_hook = a_hook * self.prev_rvec_hook + (1 - a_hook) * rvec_hook
                                tvec_hook = a_hook * self.prev_tvec_hook + (1 - a_hook) * tvec_hook

                            # save for next frame
                            self.prev_rvec_hook, self.prev_tvec_hook = rvec_hook, tvec_hook

                            # rotation matrix and translation vector
                            rmat_hook, _ = cv2.Rodrigues(rvec_hook)
                            rmat_hook_rotated = rmat_hook @ self.R_combined_hook
                            rvec_hook_rotated, _ = cv2.Rodrigues(rmat_hook_rotated)

                            # Create a 3D point for the hook but rotated by the rotation matrix
                            point_3D_hook_original = self.model_points_hook[self.hook_point].reshape(3, 1)

                            # Transform the point using the rotation matrix
                            point_3D_hook_corrected = (np.linalg.inv(self.R_combined_hook) @ point_3D_hook_original).reshape(3, 1)
                            tvec_hook_corrected = tvec_hook.reshape(3, 1)

                            p_hook_cam = rmat_hook_rotated @ point_3D_hook_corrected + tvec_hook_corrected

                            point_3D_hook_vis = point_3D_hook_corrected.reshape(1, 3)
                            axis_length = 1.0
                            x_axis = point_3D_hook_vis + np.array([[axis_length, 0, 0]], dtype=np.float64)
                            y_axis = point_3D_hook_vis + np.array([[0, axis_length, 0]], dtype=np.float64)
                            z_axis = point_3D_hook_vis + np.array([[0, 0, axis_length]], dtype=np.float64)

                            axis_points = np.vstack([point_3D_hook_vis, x_axis, y_axis, z_axis])

                            axis_points_2D, jacobian = cv2.projectPoints(axis_points, rvec_hook_rotated, tvec_hook_corrected.flatten(), cameraMatrix, mdists)

                            # Draw the axes
                            p0 = (int(axis_points_2D[0, 0, 0]), int(axis_points_2D[0, 0, 1]))
                            p1 = (int(axis_points_2D[1, 0, 0]), int(axis_points_2D[1, 0, 1]))
                            p2 = (int(axis_points_2D[2, 0, 0]), int(axis_points_2D[2, 0, 1]))
                            p3 = (int(axis_points_2D[3, 0, 0]), int(axis_points_2D[3, 0, 1]))

                            # Draw the three orientation lines with different colors
                            cv2.line(annotated_frame, p0, p1, (0, 0, 255), thickness=2, lineType=cv2.LINE_AA)
                            cv2.line(annotated_frame, p0, p2, (0, 255, 0), thickness=2, lineType=cv2.LINE_AA)
                            cv2.line(annotated_frame, p0, p3, (255, 0, 0), thickness=2, lineType=cv2.LINE_AA)



                # ====== PUBLISH HOOK STATUS (BASED ON BOX DETECTIONS) ======
                hook_status = HookStatus()
                hook_status.header = Header()
                hook_status.header.stamp = self.get_clock().now().to_msg()
                hook_status.header.frame_id = self.hook_frame_id
                
                # Check if any detection is class 1 ('true') - hook is properly engaged
                hook_status.is_hooked = any(d['class_name'] == 'true' for d in detected_classes)
                self.publisher.publish(hook_status)

                return annotated_frame, results, rvec_hook_rotated, p_hook_cam
            
        except Exception as e:
            self.get_logger().error(f"Inference failed ---hook : {e}")
            return frame, None, None, None
            
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video reached")
            self.timer.cancel()
            self.cap.release()
            if self.show_display:
                cv2.destroyAllWindows()
            return
        
        current_time = self.get_clock().now()

        self.frame_count += 1

        frame = self.center_crop_resize(frame, self.target_size, self.x_offset, self.y_offset)

        # Camera matrix setup
        height, width, channel = frame.shape
        print(f"Frame size: {height}x{width}")


        focalLength = self.focal_length * width
        cameraMatrix = self.cameraMatrix(focalLength, (height / 2, width / 2))
        mdists = np.zeros((4, 1), dtype=np.float64)

        annotated_frame = frame.copy()

        # Pass current timestamp to inference functions
        annotated_frame, results, rvec_ladle, tvec_ladle = self.run_inference_ladle(
            frame, annotated_frame, cameraMatrix, mdists, current_time
        )

        annotated_frame, results_hook, rvec_hook, tvec_hook = self.run_inference_hook(
            frame, annotated_frame, cameraMatrix, mdists, current_time
        )

        if rvec_ladle is not None and rvec_hook is not None and tvec_ladle is not None and tvec_hook is not None:
            T_ladle = self.make_transform(rvec_ladle, tvec_ladle)
            T_hook  = self.make_transform(rvec_hook,  tvec_hook)

            T_l2h = np.linalg.inv(T_ladle) @ T_hook

            rel_t = T_l2h[:3, 3]
            R     = T_l2h[:3, :3]
            
            w, x, y, z = mat2quat(R)

            rel_tf = TransformStamped()
            rel_tf.header.stamp = self.get_clock().now().to_msg()
            rel_tf.header.frame_id    = self.ladle_frame_id
            rel_tf.child_frame_id     = self.hook_frame_id
            rel_tf.transform.translation.x = float(rel_t[0])
            rel_tf.transform.translation.y = float(rel_t[1])
            rel_tf.transform.translation.z = float(rel_t[2])
            rel_tf.transform.rotation.x    = x
            rel_tf.transform.rotation.y    = y
            rel_tf.transform.rotation.z    = z
            rel_tf.transform.rotation.w    = w

            self.tf_broadcaster.sendTransform(rel_tf)

            camera_tf = TransformStamped()
            camera_tf.header.stamp = current_time.to_msg()
            camera_tf.header.frame_id = "world"
            camera_tf.child_frame_id = self.ladle_frame_id
            camera_tf.transform.translation.x = 0.0
            camera_tf.transform.translation.y = 0.0
            camera_tf.transform.translation.z = 0.0
            camera_tf.transform.rotation.x = 0.0
            camera_tf.transform.rotation.y = 0.0
            camera_tf.transform.rotation.z = 0.0
            camera_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(camera_tf)

        # Display the frame if enabled
        if self.show_display:

            output_image = cv2.resize(
                annotated_frame,
                dsize=None,
                fx=1.5,
                fy=1.5,
                interpolation=cv2.INTER_AREA
            )

            self.video_writer.write(annotated_frame)

            cv2.imshow("Ladle Detection", output_image)
            key = cv2.waitKey(self.frame_delay) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                self.get_logger().info("User interrupted playback")
                self.timer.cancel()
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()


    def cameraMatrix(self, fl, center):
        """
        Create camera matrix for pose estimation.
        """
        mat = np.zeros((3, 3), dtype=np.float64)
        mat[0, 0] = fl
        mat[1, 1] = fl
        mat[0, 2] = center[1]
        mat[1, 2] = center[0]
        mat[2, 2] = 1
        return mat


def main(args=None):
    rclpy.init(args=args)
    # Parse command line arguments for configuration
    parser = argparse.ArgumentParser(description='Ladle Detector Node')
    parser.add_argument('--config', '-c', 
                        default='west',
                        help='Configuration name (e.g., west, east)')
    
    # Parse known args to avoid conflicts with ROS parameters
    parsed_args, unknown = parser.parse_known_args()
    
    # Create node with specified configuration
    node = LadleDetectorNode(config_name=parsed_args.config)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
