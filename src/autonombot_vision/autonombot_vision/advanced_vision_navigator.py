#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import struct
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
from threading import Lock


def _as_float(val, default: float) -> float:
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


class AdvancedVisionNavigator(Node):
    """
    Advanced Computer Vision Navigation System

    Features:
    - Multi-scale YOLO detection with confidence weighting
    - Dynamic obstacle classification and priority scoring
    - Temporal tracking and prediction
    - 3D spatial mapping with depth estimation
    - Intelligent obstacle filtering for navigation
    - Real-time performance monitoring
    - Visual debugging and analytics

    NPU extension (add / tune without rewriting core vision math):
    - Parameters: npu_active_topic, external_npu_detections_topic (plus yolo_model_path, conf_thr, …)
    - Subscriptions: Bool on npu_active_topic; vision_msgs/Detection2DArray on external_npu_detections_topic
    - Handlers: _npu_active_cb, _detections_2d_cb — publish path matches image_callback tail (local YOLO)
    """

    def __init__(self):
        super().__init__('advanced_vision_navigator')

        self.declare_parameter('yolo_model_path', 'yolov8n.pt')
        self.declare_parameter('conf_thr', 0.25)
        self.declare_parameter('nms_thr', 0.45)
        self.declare_parameter('use_cpu_yolo', True)
        self.declare_parameter('npu_active_topic', '/npu_vision_active')
        self.declare_parameter('external_npu_detections_topic', '/vision/detections_2d')

        self.conf_thr = _as_float(self.get_parameter('conf_thr').value, 0.25)
        self.nms_thr = _as_float(self.get_parameter('nms_thr').value, 0.45)
        use_cpu_yolo = self.get_parameter('use_cpu_yolo').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.model = None
        if use_cpu_yolo:
            try:
                from ultralytics import YOLO

                path = (
                    self.get_parameter('yolo_model_path').get_parameter_value().string_value
                    or 'yolov8n.pt'
                )
                self.model = YOLO(path)
                self.get_logger().info(
                    '🚀 Advanced YOLO Vision Navigator - CPU model loaded successfully'
                )
            except Exception as e:
                self.get_logger().error(f'❌ Failed to load CPU YOLO: {e}')
        else:
            self.get_logger().info(
                'NPU YOLO mode: CPU inference disabled; waiting for external detections'
            )
        self.detection_lock = Lock()

        # Performance monitoring
        self.frame_count = 0
        self.start_time = time.time()
        self.last_fps_report = time.time()

        # Obstacle tracking and memory
        self.obstacle_history = {}
        self.max_history_length = 10
        self.obstacle_id_counter = 0

        # Transform handling
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- NPU integration (separate process / topic; same downstream pipeline) ---
        self._npu_claims_vision = False
        self._last_cv_image = None
        self._last_img_header = None

        npu_topic = (
            self.get_parameter('npu_active_topic').get_parameter_value().string_value.strip()
            or '/npu_vision_active'
        )
        npu_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, npu_topic, self._npu_active_cb, npu_qos)
        ext_det = (
            self.get_parameter('external_npu_detections_topic')
            .get_parameter_value()
            .string_value.strip()
            or '/vision/detections_2d'
        )
        self.create_subscription(
            Detection2DArray,
            ext_det,
            self._detections_2d_cb,
            qos_profile_sensor_data,
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )

        # Publishers
        self.obstacle_pub = self.create_publisher(PointCloud2, '/vision_obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/vision_markers', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/vision_debug', qos_profile_sensor_data
        )

        # Advanced obstacle classification with navigation priorities
        self.obstacle_categories = {
            'critical_static': {
                'classes': [
                    'car', 'truck', 'bus', 'motorcycle', 'bicycle', 'stop sign', 'bench',
                ],
                'priority': 10,
                'min_confidence': 0.4,
                'obstacle_height': 1.5,
                'color': [1.0, 0.0, 0.0, 0.8],
            },
            'critical_dynamic': {
                'classes': ['person', 'dog', 'cat', 'bird'],
                'priority': 9,
                'min_confidence': 0.3,
                'obstacle_height': 1.8,
                'color': [1.0, 0.5, 0.0, 0.8],
            },
            'furniture': {
                'classes': [
                    'chair',
                    'couch',
                    'sofa',
                    'bed',
                    'dining table',
                    'diningtable',
                    'toilet',
                    'refrigerator',
                    'tv',
                ],
                'priority': 7,
                'min_confidence': 0.5,
                'obstacle_height': 1.2,
                'color': [0.0, 0.0, 1.0, 0.6],
            },
            'small_objects': {
                'classes': [
                    'bottle', 'cup', 'bowl', 'book', 'laptop', 'cell phone', 'backpack',
                ],
                'priority': 5,
                'min_confidence': 0.6,
                'obstacle_height': 0.3,
                'color': [0.0, 1.0, 0.0, 0.4],
            },
            'plants_decor': {
                'classes': ['potted plant', 'vase', 'clock'],
                'priority': 4,
                'min_confidence': 0.5,
                'obstacle_height': 0.8,
                'color': [0.5, 0.0, 1.0, 0.5],
            },
        }

        # Camera parameters (can be made configurable)
        self.camera_height = 0.2
        self.camera_fov_h = 62.2
        self.image_width = 640
        self.image_height = 480

        # Navigation-specific parameters
        self.max_detection_range = 5.0
        self.min_obstacle_size = 0.1

        self.get_logger().info(
            '🎯 Advanced Vision Navigator initialized — Ready for intelligent obstacle detection!'
        )

    # --- NPU callbacks (optional path; upstream code can ignore this block) ---

    def _npu_active_cb(self, msg: Bool):
        self._npu_claims_vision = bool(msg.data)

    @staticmethod
    def _detection2d_array_to_rows(arr: Detection2DArray):
        """Convert vision_msgs detections to the same (bbox, class_name, conf) rows as YOLO."""
        rows = []
        for det in arr.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            class_name = hyp.class_id
            confidence = float(hyp.score)
            bb = det.bbox
            cx = bb.center.position.x
            cy = bb.center.position.y
            sx = bb.size_x
            sy = bb.size_y
            x1 = cx - sx / 2.0
            y1 = cy - sy / 2.0
            x2 = cx + sx / 2.0
            y2 = cy + sy / 2.0
            bbox = np.array([x1, y1, x2, y2], dtype=np.float32)
            rows.append((bbox, class_name, confidence))
        return rows

    def _detections_2d_cb(self, msg: Detection2DArray):
        if not self._npu_claims_vision:
            return
        if self._last_cv_image is None:
            return
        try:
            det_rows = self._detection2d_array_to_rows(msg)
            cv_image = self._last_cv_image
            img_header = self._last_img_header if self._last_img_header is not None else msg.header
            with self.detection_lock:
                self.frame_count += 1
                detections = []
                obstacles = []
                markers = MarkerArray()
                for bbox, class_name, confidence in det_rows:
                    category, priority, obs_height, color = self.classify_obstacle(
                        class_name, confidence
                    )
                    if category and priority > 0:
                        detections.append(
                            (bbox, class_name, confidence, category, priority, color)
                        )
                        x, y, z, depth = self.estimate_depth_and_position(bbox, cv_image.shape)
                        if depth <= self.max_detection_range:
                            obstacles.append([x, z, obs_height, priority])
                            marker = Marker()
                            marker.header.frame_id = 'base_link'
                            marker.header.stamp = img_header.stamp
                            marker.ns = 'vision_obstacles'
                            marker.id = len(markers.markers)
                            marker.type = Marker.CYLINDER
                            marker.action = Marker.ADD
                            marker.pose.position.x = x
                            marker.pose.position.y = z
                            marker.pose.position.z = obs_height / 2
                            marker.pose.orientation.w = 1.0
                            marker.scale.x = 0.3
                            marker.scale.y = 0.3
                            marker.scale.z = obs_height
                            marker.color.r = color[0]
                            marker.color.g = color[1]
                            marker.color.b = color[2]
                            marker.color.a = color[3]
                            marker.lifetime.sec = 1
                            markers.markers.append(marker)
                if obstacles:
                    pointcloud = self.create_pointcloud(obstacles, img_header)
                    self.obstacle_pub.publish(pointcloud)
                if markers.markers:
                    self.marker_pub.publish(markers)
                debug_image = self.create_debug_image(cv_image, detections)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                debug_msg.header = img_header
                self.debug_image_pub.publish(debug_msg)
                current_time = time.time()
                if current_time - self.last_fps_report > 5.0:
                    fps = self.frame_count / max(1, current_time - self.start_time)
                    self.get_logger().info(
                        f'🎯 Vision Performance: {fps:.1f} FPS | Frame {self.frame_count} | '
                        f'Obstacles: {len(obstacles)} | '
                        f'Categories: {len(set(d[3] for d in detections))} [ext_npu]'
                    )
                    self.last_fps_report = current_time
        except Exception as e:
            self.get_logger().error(f'External NPU detections error: {e}')

    # --- Core helpers (upstream-style; publish loops also inlined in image_callback / _detections_2d_cb) ---

    def classify_obstacle(self, class_name, confidence):
        """Classify detected object and return navigation priority."""
        for category, config in self.obstacle_categories.items():
            if class_name in config['classes']:
                if confidence >= config['min_confidence']:
                    return (
                        category,
                        config['priority'],
                        config['obstacle_height'],
                        config['color'],
                    )
        return None, 0, 0.0, [0.5, 0.5, 0.5, 0.3]

    def estimate_depth_and_position(self, bbox, image_shape):
        """Advanced depth estimation using object size and position."""
        x1, y1, x2, y2 = bbox
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        bbox_width = x2 - x1
        bbox_height = y2 - y1
        relative_size = (bbox_width * bbox_height) / (image_shape[1] * image_shape[0])
        depth = max(0.5, min(5.0, 2.0 / (relative_size * 10 + 0.1)))
        angle_per_pixel = self.camera_fov_h / image_shape[1]
        horizontal_angle = (center_x - image_shape[1] / 2) * angle_per_pixel * np.pi / 180
        x = depth * np.sin(horizontal_angle)
        z = depth * np.cos(horizontal_angle)
        y = -self.camera_height
        return x, y, z, depth

    def create_debug_image(self, cv_image, detections):
        """Create annotated debug image."""
        debug_image = cv_image.copy()
        for detection in detections:
            bbox, class_name, confidence, category, priority, color = detection
            x1, y1, x2, y2 = bbox
            cv_color = (int(color[2] * 255), int(color[1] * 255), int(color[0] * 255))
            cv2.rectangle(
                debug_image, (int(x1), int(y1)), (int(x2), int(y2)), cv_color, 2
            )
            label = f'{class_name} ({confidence:.2f}) P{priority}'
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            cv2.rectangle(
                debug_image,
                (int(x1), int(y1) - label_size[1] - 10),
                (int(x1) + label_size[0], int(y1)),
                cv_color,
                -1,
            )
            cv2.putText(
                debug_image,
                label,
                (int(x1), int(y1) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
        fps = self.frame_count / max(1, time.time() - self.start_time)
        info_text = f'FPS: {fps:.1f} | Detections: {len(detections)}'
        cv2.putText(
            debug_image,
            info_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        return debug_image

    def create_pointcloud(self, obstacles, header):
        """Create PointCloud2 message from obstacle list."""
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        points = []
        for obstacle in obstacles:
            x, y, z, intensity = obstacle
            points.append(struct.pack('ffff', float(x), float(y), float(z), float(intensity)))
        pointcloud = PointCloud2()
        pointcloud.header = header
        pointcloud.header.frame_id = 'base_link'
        pointcloud.width = len(obstacles)
        pointcloud.height = 1
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 16
        pointcloud.row_step = pointcloud.point_step * pointcloud.width
        pointcloud.data = b''.join(points)
        pointcloud.is_dense = True
        return pointcloud

    def image_callback(self, msg):
        """Upstream-style: image -> YOLO -> classify / obstacles / markers / debug (in callback)."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self._last_cv_image = cv_image
            self._last_img_header = msg.header
        except Exception:
            return

        if self.model is None or self._npu_claims_vision:
            return

        try:
            results = self.model(
                cv_image, verbose=False, conf=self.conf_thr, iou=self.nms_thr
            )
            det_rows = []
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                for box in boxes:
                    class_id = int(box.cls[0])
                    confidence = float(box.conf[0])
                    class_name = self.model.names[class_id]
                    bbox = box.xyxy[0].cpu().numpy()
                    det_rows.append((bbox, class_name, confidence))

            img_header = msg.header
            with self.detection_lock:
                self.frame_count += 1
                detections = []
                obstacles = []
                markers = MarkerArray()
                for bbox, class_name, confidence in det_rows:
                    category, priority, obs_height, color = self.classify_obstacle(
                        class_name, confidence
                    )
                    if category and priority > 0:
                        detections.append(
                            (bbox, class_name, confidence, category, priority, color)
                        )
                        x, y, z, depth = self.estimate_depth_and_position(bbox, cv_image.shape)
                        if depth <= self.max_detection_range:
                            obstacles.append([x, z, obs_height, priority])
                            marker = Marker()
                            marker.header.frame_id = 'base_link'
                            marker.header.stamp = img_header.stamp
                            marker.ns = 'vision_obstacles'
                            marker.id = len(markers.markers)
                            marker.type = Marker.CYLINDER
                            marker.action = Marker.ADD
                            marker.pose.position.x = x
                            marker.pose.position.y = z
                            marker.pose.position.z = obs_height / 2
                            marker.pose.orientation.w = 1.0
                            marker.scale.x = 0.3
                            marker.scale.y = 0.3
                            marker.scale.z = obs_height
                            marker.color.r = color[0]
                            marker.color.g = color[1]
                            marker.color.b = color[2]
                            marker.color.a = color[3]
                            marker.lifetime.sec = 1
                            markers.markers.append(marker)
                if obstacles:
                    pointcloud = self.create_pointcloud(obstacles, img_header)
                    self.obstacle_pub.publish(pointcloud)
                if markers.markers:
                    self.marker_pub.publish(markers)
                debug_image = self.create_debug_image(cv_image, detections)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                debug_msg.header = img_header
                self.debug_image_pub.publish(debug_msg)
                current_time = time.time()
                if current_time - self.last_fps_report > 5.0:
                    fps = self.frame_count / max(1, current_time - self.start_time)
                    self.get_logger().info(
                        f'🎯 Vision Performance: {fps:.1f} FPS | Frame {self.frame_count} | '
                        f'Obstacles: {len(obstacles)} | '
                        f'Categories: {len(set(d[3] for d in detections))}'
                    )
                    self.last_fps_report = current_time
        except Exception as e:
            self.get_logger().error(f'❌ Vision processing error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AdvancedVisionNavigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
