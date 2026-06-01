#!/usr/bin/env python3
"""
Remote Forwarder for Isaac Sim -> Contact-GraspNet Communication

This script runs on the remote machine (192.168.100.12) with ROS2 environment.
It bridges Isaac Sim ROS2 topics to ZMQ sockets for communication with the local machine.

Architecture:
1. Subscribe to Isaac Sim ROS2 topics (depth, rgb, camera_info, segmentation)
2. Pack data and send via ZMQ PUB socket to local machine
3. Receive grasp results via ZMQ SUB socket from local machine
4. Publish grasp results to Isaac Sim ROS2 topics for execution

Usage:
    python3 remote_forwarder.py --sensor-port 5555 --grasp-port 5556

Requirements (on remote machine):
    - ROS2 (Humble/Jazzy)
    - Isaac Sim with ROS2 bridge enabled
    - pip install pyzmq msgpack msgpack-numpy
"""

import argparse
import json
import threading
import time
import numpy as np
import zmq
import msgpack
import msgpack_numpy as m

# Patch msgpack for numpy support
m.patch()

# ROS2 imports - these will only work on the remote machine with ROS2 installed
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import Bool, Float32, String
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("Warning: ROS2 not available. This script must run on a machine with ROS2 installed.")


class IsaacSimForwarder(Node if ROS2_AVAILABLE else object):
    """
    ROS2 node that forwards data between Isaac Sim and Contact-GraspNet

    Subscribes to Isaac Sim sensor topics and publishes grasp results
    """

    def __init__(self,
                 sensor_port: int = 5555,
                 grasp_port: int = 5556,
                 local_ip: str = "0.0.0.0",
                 depth_topic: str = '/piper/end_cam/depth_image',
                 rgb_topic: str = '/piper/end_cam/color_image',
                 camera_info_topic: str = '',
                 end_pose_topic: str = '/piper/end_pose',
                 execution_busy_topic: str = '/piper/grasp_execution_busy',
                 execution_status_topic: str = '/piper/grasp_execution_status',
                 segmap_topic: str = '',
                 enable_segmap: bool = False):  # Bind to all interfaces
        if ROS2_AVAILABLE:
            super().__init__('isaac_sim_forwarder')

        self.sensor_port = sensor_port
        self.grasp_port = grasp_port
        self.local_ip = local_ip
        self.depth_topic = depth_topic
        self.rgb_topic = rgb_topic
        self.camera_info_topic = camera_info_topic
        self.end_pose_topic = end_pose_topic
        self.execution_busy_topic = execution_busy_topic
        self.execution_status_topic = execution_status_topic
        self.segmap_topic = segmap_topic
        self.enable_segmap = enable_segmap

        # Initialize CV bridge for image conversion
        if ROS2_AVAILABLE:
            self.cv_bridge = CvBridge()

        # Current sensor data cache
        self._depth_data = None
        self._rgb_data = None
        self._camera_info = None
        self._segmap_data = None
        self._end_pose = None
        self._end_pose_frame = ''
        self._execution_busy = False
        self._execution_status = 'idle'
        self._data_lock = threading.Lock()
        self._frame_id = 0
        self._timestamp = 0.0

        # Initialize ZMQ
        self._running = True  # Set running flag before starting threads
        self._init_zmq()

        # Initialize ROS2 subscriptions and publishers
        if ROS2_AVAILABLE:
            self._init_ros2()

        # Start sender thread
        self._sender_thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._sender_thread.start()

        print(f"IsaacSimForwarder initialized:")
        print(f"  - ZMQ Sensor Publisher: tcp://{local_ip}:{sensor_port}")
        print(f"  - ZMQ Grasp Subscriber: tcp://{local_ip}:{grasp_port}")

    def _init_zmq(self):
        """Initialize ZMQ sockets"""
        self.zmq_context = zmq.Context()

        # PUB socket for sending sensor data
        self.sensor_pub_socket = self.zmq_context.socket(zmq.PUB)
        self.sensor_pub_socket.bind(f"tcp://{self.local_ip}:{self.sensor_port}")

        # SUB socket for receiving grasp results
        self.grasp_sub_socket = self.zmq_context.socket(zmq.SUB)
        self.grasp_sub_socket.setsockopt(zmq.SUBSCRIBE, b"grasp_result")
        self.grasp_sub_socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        self.grasp_sub_socket.bind(f"tcp://{self.local_ip}:{self.grasp_port}")

        # Thread for receiving grasp results
        self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
        self._receiver_thread.start()

    def _init_ros2(self):
        """Initialize ROS2 subscriptions and publishers"""
        # QoS profile for Isaac Sim (best effort for real-time)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to Isaac Sim topics (Piper robot camera topics)
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            qos
        )

        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self._rgb_callback,
            qos
        )

        self.camera_info_sub = None
        if self.camera_info_topic:
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self._camera_info_callback,
                qos
            )

        self.end_pose_sub = None
        if self.end_pose_topic:
            self.end_pose_sub = self.create_subscription(
                PoseStamped,
                self.end_pose_topic,
                self._end_pose_callback,
                qos
            )

        self.execution_busy_sub = None
        if self.execution_busy_topic:
            self.execution_busy_sub = self.create_subscription(
                Bool,
                self.execution_busy_topic,
                self._execution_busy_callback,
                qos
            )

        self.execution_status_sub = None
        if self.execution_status_topic:
            self.execution_status_sub = self.create_subscription(
                String,
                self.execution_status_topic,
                self._execution_status_callback,
                qos
            )

        # Default end_cam intrinsics from the current Isaac Sim setup.
        # 640x480, focal_length 1.0mm -> fx=fy≈200 px, cx=320, cy=240.
        self._default_K = np.array([
            [200.0, 0.0, 320.0],
            [0.0, 200.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self._camera_info = self._default_K
        print(f"Using end_cam intrinsics (640x480, fx=fy≈200): {self._default_K}")

        self.segmap_sub = None
        if self.enable_segmap and self.segmap_topic:
            self.segmap_sub = self.create_subscription(
                Image,
                self.segmap_topic,
                self._segmap_callback,
                qos
            )
            print(f"Segmentation topic enabled: {self.segmap_topic}")
        elif self.enable_segmap:
            print("Warning: --enable-segmap set but no --segmap-topic provided; segmap forwarding disabled")

        # Publisher for grasp results (to Piper robot)
        self.grasp_pose_pub = self.create_publisher(
            PoseArray,
            '/piper/grasp_poses',
            10
        )

        self.best_grasp_pub = self.create_publisher(
            PoseStamped,
            '/piper/target_grasp',
            10
        )

        self.pregrasp_pub = self.create_publisher(
            PoseStamped,
            '/piper/pregrasp_pose',
            10
        )

        self.retreat_pub = self.create_publisher(
            PoseStamped,
            '/piper/postgrasp_pose',
            10
        )

        self.gripper_width_pub = self.create_publisher(
            Float32,
            '/piper/gripper_opening',
            10
        )

        self.execute_grasp_pub = self.create_publisher(
            Bool,
            '/piper/execute_grasp',
            10
        )

        self.execution_json_pub = self.create_publisher(
            String,
            '/piper/grasp_execution',
            10
        )

        print("ROS2 subscriptions and publishers initialized")
        print(f"  Depth topic: {self.depth_topic}")
        print(f"  RGB topic: {self.rgb_topic}")
        if self.camera_info_topic:
            print(f"  Camera info topic: {self.camera_info_topic}")
        if self.end_pose_topic:
            print(f"  End pose topic: {self.end_pose_topic}")
        if self.execution_busy_topic:
            print(f"  Execution busy topic: {self.execution_busy_topic}")
        if self.execution_status_topic:
            print(f"  Execution status topic: {self.execution_status_topic}")

    def _matrix_to_pose_stamped(self, pose_matrix, frame_id: str) -> PoseStamped:
        """Convert a 4x4 transform matrix into a PoseStamped."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose = self._matrix_to_pose(pose_matrix)
        return pose_msg

    def _matrix_to_pose(self, pose_matrix) -> Pose:
        """Convert a 4x4 transform matrix into a Pose."""
        from scipy.spatial.transform import Rotation as R

        pose = Pose()
        pose.position.x = float(pose_matrix[0, 3])
        pose.position.y = float(pose_matrix[1, 3])
        pose.position.z = float(pose_matrix[2, 3])

        rot = R.from_matrix(np.asarray(pose_matrix)[:3, :3])
        quat = rot.as_quat()
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def _publish_execution_command(self, result_dict: dict):
        """Publish executable grasp data for a downstream simulator/controller."""
        execution = result_dict.get('execution')
        if not execution:
            return

        frame_id = execution.get('frame', 'camera')
        pose = np.asarray(execution['pose'])
        pregrasp_pose = np.asarray(execution['pregrasp_pose'])
        retreat_pose = np.asarray(execution['retreat_pose'])

        self.best_grasp_pub.publish(self._matrix_to_pose_stamped(pose, frame_id))
        self.pregrasp_pub.publish(self._matrix_to_pose_stamped(pregrasp_pose, frame_id))
        self.retreat_pub.publish(self._matrix_to_pose_stamped(retreat_pose, frame_id))
        self.gripper_width_pub.publish(Float32(data=float(execution.get('gripper_opening', 0.0))))
        self.execute_grasp_pub.publish(Bool(data=True))

        execution_json = {
            'frame_id': int(result_dict.get('frame_id', 0)),
            'status': result_dict.get('status', 'success'),
            'message': result_dict.get('message', ''),
            'score': float(execution.get('score', 0.0)),
            'segment_id': int(execution.get('segment_id', -1)),
            'frame': frame_id,
            'gripper_opening': float(execution.get('gripper_opening', 0.0)),
            'pregrasp_offset': float(execution.get('pregrasp_offset', 0.0)),
            'retreat_offset': float(execution.get('retreat_offset', 0.0)),
            'contact_point': np.asarray(execution.get('contact_point', np.zeros(3))).tolist(),
            'approach_vector': np.asarray(execution.get('approach_vector', np.zeros(3))).tolist(),
            'pose': pose.tolist(),
            'pregrasp_pose': pregrasp_pose.tolist(),
            'retreat_pose': retreat_pose.tolist(),
        }
        self.execution_json_pub.publish(String(data=json.dumps(execution_json)))

        self.get_logger().info(
            f"Published execution command for frame {execution_json['frame_id']} with score {execution_json['score']:.3f}"
        )

    def _depth_callback(self, msg: Image):
        """Callback for depth image"""
        try:
            # Convert ROS Image to numpy
            depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Convert to meters if needed (Isaac Sim usually outputs in meters)
            if msg.encoding == '32FC1':
                depth = depth.astype(np.float32)
            elif msg.encoding == '16UC1':
                depth = depth.astype(np.float32) / 1000.0  # mm to m

            with self._data_lock:
                self._depth_data = depth
                self._timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

    def _rgb_callback(self, msg: Image):
        """Callback for RGB image"""
        try:
            rgb = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            with self._data_lock:
                self._rgb_data = rgb
        except Exception as e:
            self.get_logger().error(f"RGB callback error: {e}")

    def _camera_info_callback(self, msg: CameraInfo):
        """Callback for camera info (intrinsics)"""
        try:
            K = np.array(msg.k).reshape(3, 3)
            with self._data_lock:
                self._camera_info = K
        except Exception as e:
            self.get_logger().error(f"Camera info callback error: {e}")

    def _segmap_callback(self, msg: Image):
        """Callback for segmentation map"""
        try:
            segmap = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self._data_lock:
                self._segmap_data = segmap.astype(np.int32)
        except Exception as e:
            self.get_logger().error(f"Segmap callback error: {e}")

    def _end_pose_callback(self, msg: PoseStamped):
        """Callback for current end-effector pose."""
        try:
            pose_matrix = np.eye(4, dtype=np.float32)
            pose_matrix[0, 3] = float(msg.pose.position.x)
            pose_matrix[1, 3] = float(msg.pose.position.y)
            pose_matrix[2, 3] = float(msg.pose.position.z)

            quat = np.array([
                float(msg.pose.orientation.x),
                float(msg.pose.orientation.y),
                float(msg.pose.orientation.z),
                float(msg.pose.orientation.w),
            ], dtype=np.float32)
            quat_norm = np.linalg.norm(quat)
            if quat_norm > 1e-8:
                quat /= quat_norm
                x, y, z, w = quat
                pose_matrix[:3, :3] = np.array([
                    [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                    [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                    [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
                ], dtype=np.float32)

            with self._data_lock:
                self._end_pose = pose_matrix
                self._end_pose_frame = msg.header.frame_id or 'world'
        except Exception as e:
            self.get_logger().error(f"End pose callback error: {e}")

    def _execution_busy_callback(self, msg: Bool):
        """Callback for executor busy state."""
        with self._data_lock:
            self._execution_busy = bool(msg.data)

    def _execution_status_callback(self, msg: String):
        """Callback for executor status string."""
        with self._data_lock:
            self._execution_status = str(msg.data)

    def _sender_loop(self):
        """Background loop to send sensor data via ZMQ"""
        while self._running:
            time.sleep(0.05)  # 20 Hz update rate

            with self._data_lock:
                if self._depth_data is None or self._camera_info is None:
                    continue

                # Pack sensor data
                data_dict = {
                    'depth': self._depth_data,
                    'rgb': self._rgb_data,
                    'K': self._camera_info,
                    'segmap': self._segmap_data,
                    'timestamp': self._timestamp,
                    'frame_id': self._frame_id,
                    'end_pose': self._end_pose,
                    'end_pose_frame': self._end_pose_frame,
                    'execution_busy': self._execution_busy,
                    'execution_status': self._execution_status,
                }

                self._frame_id += 1

            # Send via ZMQ
            topic = b"sensor_data"
            timestamp = str(time.time()).encode('utf-8')
            payload = msgpack.packb(data_dict, use_bin_type=True)

            self.sensor_pub_socket.send_multipart([topic, timestamp, payload])

    def _receiver_loop(self):
        """Background loop to receive grasp results via ZMQ"""
        while self._running:
            try:
                parts = self.grasp_sub_socket.recv_multipart()
                if len(parts) >= 2:
                    topic = parts[0].decode('utf-8')
                    payload = parts[-1]

                    if topic == "grasp_result":
                        self._process_grasp_result(payload)

            except zmq.error.Again:
                # Timeout, continue
                continue
            except Exception as e:
                print(f"Receiver error: {e}")

    def _process_grasp_result(self, payload: bytes):
        """Process received grasp result and publish to ROS2"""
        if not ROS2_AVAILABLE:
            return

        try:
            result_dict = msgpack.unpackb(payload, raw=False, strict_map_key=False)
            execution = result_dict.get('execution')

            # Extract best grasp if available
            grasp_poses = result_dict.get('grasp_poses', {})
            scores = result_dict.get('scores', {})

            if not grasp_poses:
                self.get_logger().info("No grasps received")
                return

            # Find best grasp across all segments
            best_score = -1
            best_pose = None

            for seg_id, poses in grasp_poses.items():
                seg_scores = scores.get(seg_id, [])
                if len(seg_scores) > 0:
                    max_idx = np.argmax(seg_scores)
                    if seg_scores[max_idx] > best_score:
                        best_score = seg_scores[max_idx]
                        best_pose = poses[max_idx]

            if best_pose is not None and not execution:
                self.best_grasp_pub.publish(self._matrix_to_pose_stamped(best_pose, "camera"))
                self.get_logger().info(f"Published best grasp with score {best_score:.3f}")

            # Publish all grasps
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = "camera"

            for seg_id, poses in grasp_poses.items():
                for pose_matrix in poses:
                    pose_array.poses.append(self._matrix_to_pose(pose_matrix))

            self.grasp_pose_pub.publish(pose_array)
            self.get_logger().info(f"Published {len(pose_array.poses)} grasps")

            if execution:
                self._publish_execution_command(result_dict)

        except Exception as e:
            self.get_logger().error(f"Grasp result processing error: {e}")

    def stop(self):
        """Stop the forwarder"""
        self._running = False
        self.sensor_pub_socket.close()
        self.grasp_sub_socket.close()
        self.zmq_context.term()
        if ROS2_AVAILABLE:
            self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Isaac Sim to Contact-GraspNet Forwarder')
    parser.add_argument('--sensor-port', type=int, default=5555,
                        help='Port for publishing sensor data')
    parser.add_argument('--grasp-port', type=int, default=5556,
                        help='Port for receiving grasp results')
    parser.add_argument('--local-ip', type=str, default='0.0.0.0',
                        help='IP to bind ZMQ sockets (default: all interfaces)')
    parser.add_argument('--depth-topic', type=str, default='/piper/end_cam/depth_image',
                        help='ROS2 depth image topic to forward')
    parser.add_argument('--rgb-topic', type=str, default='/piper/end_cam/color_image',
                        help='ROS2 RGB image topic to forward')
    parser.add_argument('--camera-info-topic', type=str, default='',
                        help='Optional ROS2 camera_info topic; if omitted, built-in intrinsics are used')
    parser.add_argument('--end-pose-topic', type=str, default='/piper/end_pose',
                        help='ROS2 PoseStamped topic carrying the current end-effector pose')
    parser.add_argument('--execution-busy-topic', type=str, default='/piper/grasp_execution_busy',
                        help='ROS2 Bool topic carrying whether the remote executor is busy')
    parser.add_argument('--execution-status-topic', type=str, default='/piper/grasp_execution_status',
                        help='ROS2 String topic carrying the remote executor state')
    parser.add_argument('--enable-segmap', action='store_true',
                        help='Forward an integer instance segmentation map alongside depth/RGB')
    parser.add_argument('--segmap-topic', type=str, default='',
                        help='ROS2 segmentation topic to forward when --enable-segmap is set')
    args = parser.parse_args()

    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 is not available. Please run this script on a machine with ROS2 installed.")
        return

    # Initialize ROS2
    rclpy.init()

    # Create forwarder node
    forwarder = IsaacSimForwarder(
        sensor_port=args.sensor_port,
        grasp_port=args.grasp_port,
        local_ip=args.local_ip,
        depth_topic=args.depth_topic,
        rgb_topic=args.rgb_topic,
        camera_info_topic=args.camera_info_topic,
        end_pose_topic=args.end_pose_topic,
        execution_busy_topic=args.execution_busy_topic,
        execution_status_topic=args.execution_status_topic,
        segmap_topic=args.segmap_topic,
        enable_segmap=args.enable_segmap,
    )

    print("Forwarder running. Press Ctrl+C to stop.")

    try:
        rclpy.spin(forwarder)
    except KeyboardInterrupt:
        pass
    finally:
        forwarder.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
