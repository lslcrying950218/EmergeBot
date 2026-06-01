"""Local client for communicating with Isaac Sim remote forwarder."""
import math
import queue
import threading
import time
from typing import Dict, Optional

import numpy as np
import zmq

try:
    from .data_types import ExecutionCommand, GraspResult, SensorData
except ImportError:
    from contact_graspnet_pytorch.comm.data_types import ExecutionCommand, GraspResult, SensorData


class IsaacSimClient:
    """
    Client for communicating with Isaac Sim via remote forwarder

    Communication architecture:
    - Remote (192.168.100.12): Isaac Sim -> ROS2 -> Forwarder -> ZMQ
    - Local: ZMQ -> Contact-GraspNet -> ZMQ -> Forwarder -> ROS2 -> Isaac Sim

    Uses two ZMQ sockets:
    1. SUB socket: receives sensor data (depth, rgb, K, segmap)
    2. PUB socket: sends grasp results back
    """

    _DEFAULT_CAMERA_TRANSLATION_GRIPPER = np.array([-0.0763, 0.0039, 0.035], dtype=np.float32)
    _DEFAULT_CAMERA_QUATERNION_GRIPPER = np.array([-0.120, 0.124, -0.697, 0.696], dtype=np.float32)
    # D435i: hand-eye calibration was done in optical frame
    # Jetson's /piper/end_pose is optical frame pose in world
    #   verified: end_pose +Z points forward-down (= optical +Z = shooting direction)
    #   NOT camera_link (+Z=up) - D435i camera_link = optical frame
    # ContactGraspNet outputs in optical frame
    # So no transformation needed: identity
    _OPTICAL_TO_CAMERA_MATRIX = np.eye(3, dtype=np.float32)

    def __init__(self,
                 remote_ip: str = "192.168.100.12",
                 sensor_port: int = 5555,
                 grasp_port: int = 5556,
                 timeout_ms: int = 5000):
        """
        Initialize the client

        Args:
            remote_ip: IP address of the remote forwarder
            sensor_port: Port for receiving sensor data (SUB)
            grasp_port: Port for sending grasp results (PUB)
            timeout_ms: Timeout for receiving data in milliseconds
        """
        self.remote_ip = remote_ip
        self.sensor_port = sensor_port
        self.grasp_port = grasp_port
        self.timeout_ms = timeout_ms

        self.context = zmq.Context()

        # Subscriber for sensor data
        self.sensor_socket = self.context.socket(zmq.SUB)
        self.sensor_socket.setsockopt(zmq.SUBSCRIBE, b"sensor_data")
        self.sensor_socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self.sensor_socket.connect(f"tcp://{remote_ip}:{sensor_port}")

        # Publisher for grasp results
        self.grasp_socket = self.context.socket(zmq.PUB)
        self.grasp_socket.connect(f"tcp://{remote_ip}:{grasp_port}")

        # Thread for continuous data reception
        self._receive_thread = None
        self._running = False

        # Queue for received sensor data
        self._data_queue = queue.Queue(maxsize=10)

        # Latest data cache
        self._latest_data: Optional[SensorData] = None
        self._data_lock = threading.Lock()

        print(f"IsaacSimClient initialized:")
        print(f"  - Sensor subscriber: tcp://{remote_ip}:{sensor_port}")
        print(f"  - Grasp publisher: tcp://{remote_ip}:{grasp_port}")

    @staticmethod
    def _normalize_pose_array(poses) -> np.ndarray:
        poses = np.asarray(poses)
        if poses.size == 0:
            return poses.reshape(0, 4, 4)
        if poses.ndim == 2:
            return poses[np.newaxis, ...]
        return poses

    @staticmethod
    def _normalize_vector_array(values, width: int) -> np.ndarray:
        values = np.asarray(values)
        if values.size == 0:
            return values.reshape(0, width)
        if values.ndim == 1:
            return values[np.newaxis, ...]
        return values

    @staticmethod
    def _normalize_scalar_array(values) -> np.ndarray:
        values = np.asarray(values)
        return np.atleast_1d(values)

    @staticmethod
    def _normalize_workspace_bounds(workspace_bounds) -> Optional[np.ndarray]:
        """Normalize workspace bounds into a (3, 2) array."""
        if workspace_bounds is None:
            return None

        bounds = np.asarray(workspace_bounds, dtype=np.float32)
        if bounds.shape != (3, 2):
            raise ValueError('workspace_bounds must have shape (3, 2)')
        return bounds

    @staticmethod
    def _normalize_vector3(values, name: str) -> np.ndarray:
        """Normalize a 3D vector into shape (3,)."""
        vector = np.asarray(values, dtype=np.float32)
        if vector.shape != (3,):
            raise ValueError(f'{name} must have shape (3,)')
        return vector

    @staticmethod
    def _normalize_matrix4(values, name: str) -> np.ndarray:
        """Normalize a homogeneous transform into shape (4, 4)."""
        matrix = np.asarray(values, dtype=np.float32)
        if matrix.shape != (4, 4):
            raise ValueError(f'{name} must have shape (4, 4)')
        return matrix

    @staticmethod
    def _quaternion_to_matrix(quaternion) -> np.ndarray:
        """Convert a quaternion [x, y, z, w] into a 3x3 rotation matrix."""
        quat = np.asarray(quaternion, dtype=np.float32)
        if quat.shape != (4,):
            raise ValueError('quaternion must have shape (4,)')
        norm = np.linalg.norm(quat)
        if norm < 1e-8:
            return np.eye(3, dtype=np.float32)
        x, y, z, w = quat / norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float32)

    @staticmethod
    def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
        """Convert a 3x3 rotation matrix into a quaternion [x, y, z, w]."""
        from scipy.spatial.transform import Rotation as R
        r = R.from_matrix(matrix)
        return r.as_quat().astype(np.float32)

    @staticmethod
    def _rotation_matrix_z(yaw_deg: float) -> np.ndarray:
        """Build a 3x3 rotation matrix for a yaw angle in degrees."""
        yaw_rad = math.radians(float(yaw_deg))
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        return np.array([
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)

    def _transform_pose_to_planning_frame(self,
                                          pose_input: np.ndarray,
                                          input_frame: str,
                                          end_pose_world: np.ndarray,
                                          planning_frame: str = 'world',
                                          reference_frame: str = 'world',
                                          camera_translation_gripper=None,
                                          camera_quaternion_gripper=None,
                                          end_pose_is_camera_pose: bool = False) -> np.ndarray:
        """Mirror the remote executor's optical/camera/gripper/world transform locally."""
        pose_input = self._normalize_matrix4(pose_input, 'pose_input')
        end_pose_world = self._normalize_matrix4(end_pose_world, 'end_pose_world')

        if input_frame in (planning_frame, reference_frame):
            return pose_input.copy()

        camera_translation = self._normalize_vector3(
            camera_translation_gripper if camera_translation_gripper is not None else self._DEFAULT_CAMERA_TRANSLATION_GRIPPER,
            'camera_translation_gripper',
        )
        camera_quaternion = np.asarray(
            camera_quaternion_gripper if camera_quaternion_gripper is not None else self._DEFAULT_CAMERA_QUATERNION_GRIPPER,
            dtype=np.float32,
        )

        # Build gripper -> camera from the fixed extrinsics, then invert when
        # we need camera/optical poses expressed in the gripper frame.
        R_gripper_to_cam = self._quaternion_to_matrix(camera_quaternion)
        t_gripper_to_cam = np.eye(4, dtype=np.float32)
        t_gripper_to_cam[:3, :3] = R_gripper_to_cam
        t_gripper_to_cam[:3, 3] = camera_translation

        # Optical -> camera transform: pure rotation (det=+1)
        t_optical_to_cam = np.eye(4, dtype=np.float32)
        t_optical_to_cam[:3, :3] = self._OPTICAL_TO_CAMERA_MATRIX

        if input_frame in ('camera_optical_frame', 'end_cam_optical_frame', 'optical'):
            pose_in_cam = t_optical_to_cam @ pose_input
        else:
            pose_in_cam = pose_input

        if end_pose_is_camera_pose:
            return end_pose_world @ pose_in_cam

        pose_in_gripper = np.linalg.inv(t_gripper_to_cam) @ pose_in_cam
        return end_pose_world @ pose_in_gripper

    def _pose_in_base_workspace(self,
                                planning_pose: np.ndarray,
                                base_position_world,
                                base_yaw_deg: float,
                                base_workspace_bounds) -> bool:
        """Check whether a planning-frame pose lies inside the configured base workspace."""
        planning_pose = self._normalize_matrix4(planning_pose, 'planning_pose')
        base_position = self._normalize_vector3(base_position_world, 'base_position_world')
        bounds = self._normalize_workspace_bounds(base_workspace_bounds)

        base_rotation = self._rotation_matrix_z(base_yaw_deg)
        rel_world = planning_pose[:3, 3] - base_position
        rel_base = base_rotation.T @ rel_world
        return bool(np.all(rel_base >= bounds[:, 0]) and np.all(rel_base <= bounds[:, 1]))

    def _execution_in_base_workspace(self,
                                     execution: ExecutionCommand,
                                     end_pose_world: np.ndarray,
                                     planning_frame: str = 'world',
                                     reference_frame: str = 'world',
                                     base_position_world=None,
                                     base_yaw_deg: float = 0.0,
                                     base_workspace_bounds=None,
                                     camera_translation_gripper=None,
                                     camera_quaternion_gripper=None,
                                     end_pose_is_camera_pose: bool = False) -> bool:
        """Check grasp/pregrasp/retreat against the base-frame workspace."""
        if base_workspace_bounds is None:
            return True

        for pose in (execution.pose, execution.pregrasp_pose, execution.retreat_pose):
            planning_pose = self._transform_pose_to_planning_frame(
                pose,
                execution.frame,
                end_pose_world=end_pose_world,
                planning_frame=planning_frame,
                reference_frame=reference_frame,
                camera_translation_gripper=camera_translation_gripper,
                camera_quaternion_gripper=camera_quaternion_gripper,
                end_pose_is_camera_pose=end_pose_is_camera_pose,
            )
            if not self._pose_in_base_workspace(
                planning_pose,
                base_position_world=base_position_world,
                base_yaw_deg=base_yaw_deg,
                base_workspace_bounds=base_workspace_bounds,
            ):
                return False

        return True

    def _iter_grasp_candidates(self,
                               pred_grasps_cam: Dict[int, np.ndarray],
                               scores: Dict[int, np.ndarray],
                               contact_pts: Dict[int, np.ndarray],
                               gripper_openings: Dict[int, np.ndarray],
                               preferred_seg_id: Optional[int] = None):
        """Yield normalized grasp candidates across segments."""
        seg_ids = [preferred_seg_id] if preferred_seg_id is not None else list(scores.keys())
        if preferred_seg_id is not None and preferred_seg_id not in scores:
            if -1 in scores:
                seg_ids = [-1]
            elif len(scores) == 1:
                seg_ids = list(scores.keys())
            else:
                seg_ids = []

        for seg_id in seg_ids:
            if seg_id not in scores:
                continue

            seg_poses = self._normalize_pose_array(pred_grasps_cam[seg_id])
            seg_scores = self._normalize_scalar_array(scores[seg_id])
            seg_contacts = self._normalize_vector_array(contact_pts[seg_id], 3)
            seg_openings = self._normalize_scalar_array(gripper_openings[seg_id])

            count = min(len(seg_poses), len(seg_scores), len(seg_contacts), len(seg_openings))
            for idx in range(count):
                yield {
                    'pose': seg_poses[idx],
                    'score': float(seg_scores[idx]),
                    'contact_point': seg_contacts[idx],
                    'gripper_opening': float(seg_openings[idx]),
                    'segment_id': seg_id,
                }

    @staticmethod
    def _candidate_in_workspace(candidate: Dict, workspace_bounds: Optional[np.ndarray]) -> bool:
        """Check whether the grasp position lies inside the configured workspace."""
        if workspace_bounds is None:
            return True

        grasp_pos = np.asarray(candidate['pose'])[:3, 3]
        mins = workspace_bounds[:, 0]
        maxs = workspace_bounds[:, 1]
        return bool(np.all(grasp_pos >= mins) and np.all(grasp_pos <= maxs))

    @staticmethod
    def _pose_in_workspace(pose: np.ndarray, workspace_bounds: Optional[np.ndarray]) -> bool:
        """Check whether a full pose translation lies inside the configured workspace."""
        if workspace_bounds is None:
            return True

        pose = np.asarray(pose, dtype=np.float32)
        pos = pose[:3, 3]
        mins = workspace_bounds[:, 0]
        maxs = workspace_bounds[:, 1]
        return bool(np.all(pos >= mins) and np.all(pos <= maxs))

    def connect(self):
        """Start the background receiver thread"""
        if self._running:
            return

        self._running = True
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()
        print("Connected and listening for sensor data...")

    def disconnect(self):
        """Stop the receiver thread and close sockets"""
        self._running = False
        if self._receive_thread:
            self._receive_thread.join(timeout=2)

        self.sensor_socket.close()
        self.grasp_socket.close()
        self.context.term()
        print("Disconnected from Isaac Sim")

    def _receive_loop(self):
        """Background loop for receiving sensor data"""
        while self._running:
            try:
                # Receive multipart message: [topic, timestamp, payload]
                parts = self.sensor_socket.recv_multipart()
                if len(parts) >= 2:
                    topic = parts[0].decode('utf-8')
                    payload = parts[-1]

                    if topic == "sensor_data":
                        sensor_data = SensorData.from_bytes(payload)
                        with self._data_lock:
                            self._latest_data = sensor_data

                        # Put in queue (non-blocking, drop old if full)
                        try:
                            self._data_queue.put_nowait(sensor_data)
                        except queue.Full:
                            # Drop oldest and put new
                            try:
                                self._data_queue.get_nowait()
                                self._data_queue.put_nowait(sensor_data)
                            except queue.Empty:
                                pass

            except zmq.error.Again:
                # Timeout, continue loop
                continue
            except Exception as e:
                if self._running:
                    print(f"Receive error: {e}")

    def get_latest_data(self) -> Optional[SensorData]:
        """Get the latest sensor data (non-blocking)"""
        with self._data_lock:
            return self._latest_data

    def wait_for_data(self, timeout_s: float = 5.0) -> Optional[SensorData]:
        """Wait for new sensor data with timeout"""
        try:
            return self._data_queue.get(timeout=timeout_s)
        except queue.Empty:
            return None

    def send_grasp_result(self, result: GraspResult):
        """
        Send grasp prediction result back to Isaac Sim

        Args:
            result: GraspResult containing predicted grasps
        """
        # Send multipart: [topic, timestamp, payload]
        topic = b"grasp_result"
        timestamp = str(time.time()).encode('utf-8')
        payload = result.to_bytes()

        self.grasp_socket.send_multipart([topic, timestamp, payload])
        print(f"Sent grasp result for frame {result.frame_id}: {result.status}")

    def send_grasp_from_prediction(self,
                                   pred_grasps_cam: Dict[int, np.ndarray],
                                   scores: Dict[int, np.ndarray],
                                   contact_pts: Dict[int, np.ndarray],
                                   gripper_openings: Dict[int, np.ndarray],
                                   frame_id: int = 0,
                                   status: str = 'success',
                                   message: str = '',
                                   execution: Optional[ExecutionCommand] = None):
        """
        Convenience method to send grasp result from Contact-GraspNet prediction

        Args:
            pred_grasps_cam: Dict of Nx4x4 grasp poses per segment
            scores: Dict of N confidence scores per segment
            contact_pts: Dict of Nx3 contact points per segment
            gripper_openings: Dict of N gripper widths per segment
            frame_id: Frame ID this result corresponds to
            status: Status string
            message: Debug message
        """
        result = GraspResult(
            timestamp=time.time(),
            frame_id=frame_id,
            status=status,
            message=message,
            execution=execution,
        )

        for seg_id in pred_grasps_cam.keys():
            result.add_segment(
                seg_id,
                pred_grasps_cam[seg_id],
                scores[seg_id],
                contact_pts[seg_id],
                gripper_openings[seg_id]
            )

        self.send_grasp_result(result)

    def get_best_grasp_for_execution(self,
                                      pred_grasps_cam: Dict[int, np.ndarray],
                                      scores: Dict[int, np.ndarray],
                                      contact_pts: Dict[int, np.ndarray],
                                      gripper_openings: Dict[int, np.ndarray],
                                      min_score: float = float('-inf'),
                                      workspace_bounds=None,
                                      top_k: Optional[int] = None,
                                      preferred_seg_id: Optional[int] = None) -> Optional[Dict]:
        """
        Get the best grasp across all segments for execution

        Returns:
            Dict with 'pose', 'score', 'contact_point', 'gripper_opening'
            or None if no valid grasps
        """
        workspace_bounds = self._normalize_workspace_bounds(workspace_bounds)
        candidates = [
            candidate for candidate in self._iter_grasp_candidates(
                pred_grasps_cam,
                scores,
                contact_pts,
                gripper_openings,
                preferred_seg_id=preferred_seg_id,
            )
            if candidate['score'] >= min_score and self._candidate_in_workspace(candidate, workspace_bounds)
        ]

        if not candidates:
            return None

        candidates.sort(key=lambda candidate: candidate['score'], reverse=True)
        if top_k is not None:
            candidates = candidates[:max(0, top_k)]

        return candidates[0] if candidates else None

    def build_execution_command(self,
                                grasp_pose: np.ndarray,
                                score: float,
                                contact_point: np.ndarray,
                                gripper_opening: float,
                                segment_id: int = -1,
                                frame: str = 'camera_optical_frame',
                                pregrasp_offset: float = 0.10,
                                retreat_offset: float = 0.10,
                                max_pregrasp_offset: Optional[float] = None,
                                max_retreat_offset: Optional[float] = None) -> ExecutionCommand:
        """
        Build a simple executable grasp command.

        The pregrasp and retreat poses move opposite to the gripper approach axis.
        """
        grasp_pose = np.asarray(grasp_pose, dtype=np.float32)
        contact_point = np.asarray(contact_point, dtype=np.float32)

        approach_vector = grasp_pose[:3, 2].astype(np.float32)
        norm = np.linalg.norm(approach_vector)
        if norm < 1e-8:
            raise ValueError('Invalid grasp pose: zero approach vector')
        approach_vector /= norm

        effective_pregrasp_offset = float(pregrasp_offset)
        if max_pregrasp_offset is not None:
            effective_pregrasp_offset = min(effective_pregrasp_offset, float(max_pregrasp_offset))
        pregrasp_pose = grasp_pose.copy()
        pregrasp_pose[:3, 3] -= approach_vector * effective_pregrasp_offset

        effective_retreat_offset = float(retreat_offset)
        if max_retreat_offset is not None:
            effective_retreat_offset = min(effective_retreat_offset, float(max_retreat_offset))
        retreat_pose = grasp_pose.copy()
        retreat_pose[:3, 3] -= approach_vector * effective_retreat_offset

        return ExecutionCommand(
            pose=grasp_pose,
            pregrasp_pose=pregrasp_pose,
            retreat_pose=retreat_pose,
            contact_point=contact_point,
            approach_vector=approach_vector,
            gripper_opening=float(gripper_opening),
            score=float(score),
            segment_id=segment_id,
            frame=frame,
            pregrasp_offset=float(effective_pregrasp_offset),
            retreat_offset=float(effective_retreat_offset),
        )

    def build_execution_command_from_prediction(self,
                                                pred_grasps_cam: Dict[int, np.ndarray],
                                                scores: Dict[int, np.ndarray],
                                                contact_pts: Dict[int, np.ndarray],
                                                gripper_openings: Dict[int, np.ndarray],
                                                min_score: float = 0.0,
                                                frame: str = 'camera_optical_frame',
                                                pregrasp_offset: float = 0.10,
                                                retreat_offset: float = 0.10,
                                                max_pregrasp_offset: Optional[float] = None,
                                                max_retreat_offset: Optional[float] = None,
                                                workspace_bounds=None,
                                                top_k: Optional[int] = None,
                                                preferred_seg_id: Optional[int] = None,
                                                end_pose_world=None,
                                                planning_frame: str = 'world',
                                                reference_frame: str = 'world',
                                                base_position_world=None,
                                                base_yaw_deg: float = 0.0,
                                                base_workspace_bounds=None,
                                                camera_translation_gripper=None,
                                                camera_quaternion_gripper=None,
                                                end_pose_is_camera_pose: bool = False) -> Optional[ExecutionCommand]:
        """Select the best grasp and convert it into an executable command."""
        workspace_bounds = self._normalize_workspace_bounds(workspace_bounds)
        base_workspace_bounds = self._normalize_workspace_bounds(base_workspace_bounds)
        candidates = [
            candidate for candidate in self._iter_grasp_candidates(
                pred_grasps_cam,
                scores,
                contact_pts,
                gripper_openings,
                preferred_seg_id=preferred_seg_id,
            )
            if candidate['score'] >= min_score and self._candidate_in_workspace(candidate, workspace_bounds)
        ]

        if not candidates:
            return None

        candidates.sort(key=lambda candidate: candidate['score'], reverse=True)
        if top_k is not None:
            candidates = candidates[:max(0, top_k)]

        reachability_check_enabled = base_workspace_bounds is not None and base_position_world is not None
        if reachability_check_enabled:
            end_pose_world = self._normalize_matrix4(end_pose_world, 'end_pose_world')

        for candidate in candidates:
            execution = self.build_execution_command(
                grasp_pose=candidate['pose'],
                score=candidate['score'],
                contact_point=candidate['contact_point'],
                gripper_opening=candidate['gripper_opening'],
                segment_id=candidate['segment_id'],
                frame=frame,
                pregrasp_offset=pregrasp_offset,
                retreat_offset=retreat_offset,
                max_pregrasp_offset=max_pregrasp_offset,
                max_retreat_offset=max_retreat_offset,
            )
            if (
                self._pose_in_workspace(execution.pose, workspace_bounds)
                and self._pose_in_workspace(execution.pregrasp_pose, workspace_bounds)
                and self._pose_in_workspace(execution.retreat_pose, workspace_bounds)
                and (
                    not reachability_check_enabled
                    or self._execution_in_base_workspace(
                        execution,
                        end_pose_world=end_pose_world,
                        planning_frame=planning_frame,
                        reference_frame=reference_frame,
                        base_position_world=base_position_world,
                        base_yaw_deg=base_yaw_deg,
                        base_workspace_bounds=base_workspace_bounds,
                        camera_translation_gripper=camera_translation_gripper,
                        camera_quaternion_gripper=camera_quaternion_gripper,
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                    )
                )
            ):
                return execution

        return None


class IsaacSimGraspPipeline:
    """
    Complete pipeline: receive data -> predict grasps -> send results

    Integrates Contact-GraspNet with Isaac Sim communication
    """

    def __init__(self,
                 grasp_estimator,
                 remote_ip: str = "192.168.100.12",
                 sensor_port: int = 5555,
                 grasp_port: int = 5556,
                 z_range: list = [0.2, 1.8],
                 local_regions: bool = True,
                 filter_grasps: bool = True,
                 forward_passes: int = 1):
        """
        Initialize the pipeline

        Args:
            grasp_estimator: GraspEstimator instance from Contact-GraspNet
            remote_ip: Remote forwarder IP
            sensor_port: Port for sensor data
            grasp_port: Port for grasp results
            z_range: Z distance range for point cloud filtering
            local_regions: Whether to extract local regions
            filter_grasps: Whether to filter grasps by segment
            forward_passes: Number of forward passes
        """
        self.grasp_estimator = grasp_estimator
        self.z_range = z_range
        self.local_regions = local_regions
        self.filter_grasps = filter_grasps
        self.forward_passes = forward_passes

        self.client = IsaacSimClient(
            remote_ip=remote_ip,
            sensor_port=sensor_port,
            grasp_port=grasp_port
        )

        self._running = False
        self._pipeline_thread = None

    def start(self):
        """Start the continuous grasp prediction pipeline"""
        self.client.connect()
        self._running = True
        self._pipeline_thread = threading.Thread(target=self._pipeline_loop, daemon=True)
        self._pipeline_thread.start()
        print("Grasp pipeline started")

    def stop(self):
        """Stop the pipeline"""
        self._running = False
        if self._pipeline_thread:
            self._pipeline_thread.join(timeout=3)
        self.client.disconnect()
        print("Grasp pipeline stopped")

    def _pipeline_loop(self):
        """Background loop for continuous grasp prediction"""
        while self._running:
            # Wait for sensor data
            sensor_data = self.client.wait_for_data(timeout_s=2.0)

            if sensor_data is None:
                continue

            try:
                # Process sensor data
                print(f"Processing frame {sensor_data.frame_id}")

                # Extract point clouds
                pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(
                    sensor_data.depth,
                    sensor_data.K,
                    segmap=sensor_data.segmap,
                    rgb=sensor_data.rgb,
                    z_range=self.z_range
                )

                # Predict grasps
                pred_grasps_cam, scores, contact_pts, gripper_openings = \
                    self.grasp_estimator.predict_scene_grasps(
                        pc_full,
                        pc_segments=pc_segments,
                        local_regions=self.local_regions,
                        filter_grasps=self.filter_grasps,
                        forward_passes=self.forward_passes
                    )

                # Send results back
                self.client.send_grasp_from_prediction(
                    pred_grasps_cam, scores, contact_pts, gripper_openings,
                    frame_id=sensor_data.frame_id,
                    status='success',
                    message=f'Generated {sum(len(s) for s in scores.values())} grasps'
                )

                print(f"Sent {sum(len(s) for s in scores.values())} grasps for frame {sensor_data.frame_id}")

            except Exception as e:
                # Send error result
                result = GraspResult(
                    frame_id=sensor_data.frame_id if sensor_data else 0,
                    status='error',
                    message=str(e)
                )
                self.client.send_grasp_result(result)
                print(f"Error processing frame: {e}")

    def process_single_frame(self, timeout_s: float = 5.0) -> Optional[Dict]:
        """
        Process a single frame and return the best grasp

        Returns:
            Dict with prediction results or None if no data received
        """
        sensor_data = self.client.wait_for_data(timeout_s=timeout_s)

        if sensor_data is None:
            print("No sensor data received")
            return None

        # Extract point clouds
        pc_full, pc_segments, pc_colors = self.grasp_estimator.extract_point_clouds(
            sensor_data.depth,
            sensor_data.K,
            segmap=sensor_data.segmap,
            rgb=sensor_data.rgb,
            z_range=self.z_range
        )

        # Predict grasps
        pred_grasps_cam, scores, contact_pts, gripper_openings = \
            self.grasp_estimator.predict_scene_grasps(
                pc_full,
                pc_segments=pc_segments,
                local_regions=self.local_regions,
                filter_grasps=self.filter_grasps,
                forward_passes=self.forward_passes
            )

        # Get best grasp
        best_grasp = self.client.get_best_grasp_for_execution(
            pred_grasps_cam, scores, contact_pts, gripper_openings
        )

        return {
            'sensor_data': sensor_data,
            'pc_full': pc_full,
            'pc_colors': pc_colors,
            'pred_grasps_cam': pred_grasps_cam,
            'scores': scores,
            'contact_pts': contact_pts,
            'gripper_openings': gripper_openings,
            'best_grasp': best_grasp
        }
