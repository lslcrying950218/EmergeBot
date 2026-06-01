#!/usr/bin/env python3
"""
Grasp Executor for Piper Robot using MoveIt motion planning.

This version fixes three issues from the ad-hoc remote script:
1. Do not run the full grasp sequence inside a subscription callback.
2. Do not busy-wait on service futures without letting the executor spin.
3. Subscribe to gripper end-effector pose (/end_pose_stamped) for hand-eye transform.

Coordinate Transform Architecture:
- Subscribes to /end_pose_stamped (gripper/link6 pose in world frame)
- When input_frame='camera_optical_frame', applies hand-eye calibration to transform to world
- When input_frame='world', uses the pose directly (PGX already did the transform)

Usage:
    source /opt/ros/humble/setup.bash
    source /root/piper_ros_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 tools/grasp_executor_motion.py --move-group arm
"""

import argparse
import json
import random
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from shape_msgs.msg import SolidPrimitive

MOVEIT_SUCCESS = getattr(MoveItErrorCodes, 'SUCCESS', 1)
MOVEIT_ROBOT_STATE_INCOMPLETE = getattr(MoveItErrorCodes, 'ROBOT_STATE_INCOMPLETE', -31)


def parse_vector3(value: str, default):
    if not value:
        return np.asarray(default, dtype=np.float32)
    return np.asarray(json.loads(value), dtype=np.float32)


def parse_workspace_bounds(value: str, default):
    if not value:
        return np.asarray(default, dtype=np.float32)
    bounds = np.asarray(json.loads(value), dtype=np.float32)
    if bounds.shape != (3, 2):
        raise ValueError('workspace bounds must have shape [[xmin,xmax],[ymin,ymax],[zmin,zmax]]')
    return bounds


class GraspExecutorMotion(Node):
    """Execute grasp commands using IK + MoveIt joint-space planning."""

    def __init__(self,
                 move_group: str = 'arm',
                 planning_frame: str = 'world',
                 reference_frame: str = 'world',
                 gripper_open_width: float = 0.08,
                 force_close_width: float = None,
                 close_width_margin: float = 0.0,
                 gripper_settle_time: float = 0.5,
                 post_grasp_lift_distance: float = 0.0,
                 post_grasp_lift_hold_time: float = 0.0,
                 base_position_world=None,
                 base_yaw_deg: float = 180.0,
                 base_workspace_bounds=None,
                 max_pregrasp_distance: float = 0.06,
                 max_retreat_distance: float = 0.08,
                 align_poses: bool = False,
                 ik_attempts: int = 2,
                 ik_request_timeout_sec: float = 1.0,
                 ik_wait_timeout_sec: float = 1.2,
                 max_ik_pose_variants: int = 2,
                 enable_pose_fallback: bool = False,
                 verbose_debug: bool = False,
                 return_home: bool = True,
                 return_home_on_failure: bool = True,
                 grasp_offset: list = None,
                 gripper_depth: float = 0.161):
        super().__init__('grasp_executor_motion')

        self._execution_lock = threading.Lock()
        self._executing = False

        self._arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self._gripper_joint = 'joint7'
        # Gripper depth: distance from Link6 (flange) to gripper center (end-effector)
        # Link6 -> 35mm -> camera optical center -> 76mm -> gripper center
        # Total = 35 + 76 = 111mm, add 50mm = 161mm
        # Contact-GraspNet outputs pose at gripper center, MoveIt IK solves for Link6
        # Need to move backward along approach direction to get Link6 position
        self._gripper_depth = float(gripper_depth)

        self._planning_frame = str(planning_frame)
        self._reference_frame = str(reference_frame)
        self._reference_to_planning = np.eye(4, dtype=np.float32)

        self._move_group = str(move_group)
        self._gripper_open_width = float(gripper_open_width)
        self._force_close_width = None if force_close_width is None else float(force_close_width)
        self._close_width_margin = float(close_width_margin)
        self._gripper_settle_time = float(gripper_settle_time)
        self._post_grasp_lift_distance = float(post_grasp_lift_distance)
        self._post_grasp_lift_hold_time = float(post_grasp_lift_hold_time)
        self._base_position_world = np.asarray(
            base_position_world if base_position_world is not None else [-50.6, -27.7, 0.85],
            dtype=np.float32,
        )
        self._base_rotation_world = R.from_euler('z', float(base_yaw_deg), degrees=True)
        self._base_workspace_bounds = np.asarray(
            base_workspace_bounds if base_workspace_bounds is not None else [[-0.15, 0.35], [-0.35, 0.35], [-0.10, 0.45]],
            dtype=np.float32,
        )
        self._max_pregrasp_distance = float(max_pregrasp_distance)
        self._max_retreat_distance = float(max_retreat_distance)
        self._align_poses = bool(align_poses)
        self._ik_attempts = max(1, int(ik_attempts))
        self._ik_request_timeout_sec = float(ik_request_timeout_sec)
        self._ik_wait_timeout_sec = float(ik_wait_timeout_sec)
        self._max_ik_pose_variants = max(1, int(max_ik_pose_variants))
        self._enable_pose_fallback = bool(enable_pose_fallback)
        self._verbose_debug = bool(verbose_debug)
        self._return_home = bool(return_home)
        self._return_home_on_failure = bool(return_home_on_failure)
        # 抓取位置偏移校正 [x, y, z]，用于补偿手眼标定误差
        self._grasp_offset = np.asarray(
            grasp_offset if grasp_offset is not None else [0.0, 0.0, 0.0],
            dtype=np.float32,
        )

        self._current_joints = {}
        self._home_joints = None
        self._stored_pose = None
        self._end_pose_source = None
        self._end_pose_frame = None
        self._pending_grasp = None

        self._service_cb_group = ReentrantCallbackGroup()

        self.create_subscription(JointState, '/piper/joint_states', self._joint_cb, 10)
        # 订阅机械臂末端位姿 (link6)，用于手眼变换和 IK 解算
        # 注意: 这是机械臂末端位姿，不是相机位姿
        self.create_subscription(PoseStamped, '/end_pose_stamped', self._end_pose_cb, 10)
        self.create_subscription(String, '/piper/grasp_execution', self._grasp_execution_cb, 10)
        self.create_subscription(Bool, '/piper/execute_grasp', self._execute_cb, 10)
        self.create_subscription(PoseStamped, '/piper/target_grasp', self._target_cb, 10)

        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._service_cb_group
        )
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path', callback_group=self._service_cb_group
        )
        self.joint_pub = self.create_publisher(JointState, '/joint_ctrl_single', 10)
        self.execution_busy_pub = self.create_publisher(Bool, '/piper/grasp_execution_busy', 10)
        self.execution_status_pub = self.create_publisher(String, '/piper/grasp_execution_status', 10)

        # MoveGroup action client (更稳定)
        from rclpy.action import ActionClient
        from moveit_msgs.action import MoveGroup
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # Planning scene client (清空碰撞场景)
        from moveit_msgs.srv import ApplyPlanningScene
        self._scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self._service_cb_group
        )

        self.get_logger().info('GraspExecutorMotion initialized')
        self.get_logger().info(f'  Move group: {self._move_group}')
        self.get_logger().info(f'  Planning frame: {self._planning_frame}')
        self.get_logger().info(f'  Reference frame: {self._reference_frame}')
        self.get_logger().info(f'  Base position (world): {np.round(self._base_position_world, 4).tolist()}')
        self.get_logger().info(f'  Base workspace bounds: {self._base_workspace_bounds.tolist()}')
        self.get_logger().info(f'  IK attempts: {self._ik_attempts}')
        self.get_logger().info(f'  IK wait timeout: {self._ik_wait_timeout_sec}')
        self.get_logger().info(f'  Return home: {self._return_home} (on failure: {self._return_home_on_failure})')
        if np.any(self._grasp_offset != 0):
            self.get_logger().info(f'  Grasp offset: {np.round(self._grasp_offset, 4).tolist()}')
        self._publish_execution_state(False, 'idle')

    def _publish_execution_state(self, busy: bool, status: str):
        """Publish executor busy/state information for upstream throttling."""
        self.execution_busy_pub.publish(Bool(data=bool(busy)))
        self.execution_status_pub.publish(String(data=str(status)))

    def _joint_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._current_joints[name] = msg.position[i]
        self._maybe_capture_home_joints()

    def _maybe_capture_home_joints(self):
        if self._home_joints is not None:
            return
        if not all(name in self._current_joints for name in self._arm_joints):
            return
        self._home_joints = [float(self._current_joints[name]) for name in self._arm_joints]
        self.get_logger().info(
            f'Captured home joints: {[f"{joint:.3f}" for joint in self._home_joints]}'
        )

    def _end_pose_cb(self, msg: PoseStamped):
        self._end_pose_source = np.eye(4, dtype=np.float32)
        self._end_pose_source[0, 3] = msg.pose.position.x
        self._end_pose_source[1, 3] = msg.pose.position.y
        self._end_pose_source[2, 3] = msg.pose.position.z
        quat = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        self._end_pose_source[:3, :3] = R.from_quat(quat).as_matrix()
        self._end_pose_frame = msg.header.frame_id or self._reference_frame
        # 只在首次或每10秒打印一次日志
        if not hasattr(self, '_last_end_pose_log_time'):
            self._last_end_pose_log_time = 0
        now_time = time.time()
        if now_time - self._last_end_pose_log_time > 10.0:
            self._last_end_pose_log_time = now_time
            self.get_logger().info(
                f'[end_pose] pos=[{msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}]'
            )

    def _target_cb(self, msg: PoseStamped):
        self._stored_pose = msg
        self.get_logger().info('Stored target grasp pose')

    def _execute_cb(self, msg: Bool):
        if not msg.data or self._stored_pose is None:
            return
        pose_msg = self._stored_pose
        self._stored_pose = None
        self._start_execution_thread(self._execute_single_pose, pose_msg)

    def _grasp_execution_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().error(f'Execution JSON parse error: {exc}')
            return

        frame_id = data.get('frame_id', 0)
        score = data.get('score', 0.0)
        input_frame = data.get('frame', 'camera_optical_frame')

        grasp = np.asarray(data.get('pose', []), dtype=np.float32)
        pregrasp = np.asarray(data.get('pregrasp_pose', []), dtype=np.float32)
        retreat = np.asarray(data.get('retreat_pose', []), dtype=np.float32)
        gripper_width = float(data.get('gripper_opening', 0.08))

        if grasp.shape != (4, 4):
            self.get_logger().error(f'Invalid grasp pose shape: {grasp.shape}')
            return

        with self._execution_lock:
            if self._executing:
                self._publish_execution_state(True, 'busy_drop')
                self.get_logger().warn(
                    f'Execution already in progress, dropping grasp frame={frame_id}'
                )
                return
            self._pending_grasp = (pregrasp, grasp, retreat, gripper_width, input_frame, frame_id, score)
            self.get_logger().info(
                f'Stored grasp: frame={frame_id}, score={score:.3f}, input_frame={input_frame}'
            )
            self._executing = True

        self._publish_execution_state(True, f'queued:{frame_id}')
        self._start_execution_with_pending()

    def _start_execution_thread(self, target, *args):
        with self._execution_lock:
            if self._executing:
                self.get_logger().warn('Execution already in progress, ignoring new command')
                return
            self._executing = True

        self._publish_execution_state(True, 'busy')
        thread = threading.Thread(target=self._run_execution, args=(target, args), daemon=True)
        thread.start()

    def _start_execution_with_pending(self):
        thread = threading.Thread(target=self._run_pending_execution, daemon=True)
        thread.start()

    def _run_pending_execution(self):
        while True:
            with self._execution_lock:
                if self._pending_grasp is None:
                    self._executing = False
                    self._publish_execution_state(False, 'idle')
                    return
                pregrasp, grasp, retreat, gripper_width, input_frame, frame_id, score = self._pending_grasp
                self._pending_grasp = None

            self._publish_execution_state(True, f'executing:{frame_id}')
            self.get_logger().info(
                f'Executing grasp: frame={frame_id}, score={score:.3f}, input_frame={input_frame}'
            )
            try:
                self._execute_sequence(pregrasp, grasp, retreat, gripper_width, input_frame)
            except Exception as exc:
                self._publish_execution_state(True, f'error:{frame_id}')
                self.get_logger().error(f'Execution error: {exc}')

    def _run_execution(self, target, args):
        try:
            target(*args)
        except Exception as exc:
            self._publish_execution_state(True, 'error')
            self.get_logger().error(f'Execution error: {exc}')
        finally:
            with self._execution_lock:
                self._executing = False
            self._publish_execution_state(False, 'idle')

    @staticmethod
    def _matrix_to_pose(matrix: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = float(matrix[0, 3])
        pose.position.y = float(matrix[1, 3])
        pose.position.z = float(matrix[2, 3])
        quat = R.from_matrix(matrix[:3, :3]).as_quat()
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def _wait_for_future(self, future, timeout_sec: float, poll_sec: float = 0.02):
        """Wait for a future to complete, using a separate spinning context."""
        # 创建一个独立的 node 和 executor 来处理这个 future
        # 这避免了与主 executor 的冲突
        import threading

        done_event = threading.Event()

        def done_callback(fut):
            done_event.set()

        future.add_done_callback(done_callback)

        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if done_event.is_set():
                return True
            time.sleep(poll_sec)

        return future.done()

    def _convert_pose_to_planning_frame(self, pose_matrix: np.ndarray, source_frame: str) -> np.ndarray:
        if source_frame == self._planning_frame:
            return pose_matrix
        if source_frame == self._reference_frame:
            return self._reference_to_planning @ pose_matrix
        raise ValueError(
            f'Unsupported source frame {source_frame!r}; expected '
            f'{self._planning_frame!r} or {self._reference_frame!r}'
        )

    def _get_end_pose_in_planning_frame(self):
        if self._end_pose_source is None:
            return None
        source_frame = self._end_pose_frame or self._reference_frame
        result = self._convert_pose_to_planning_frame(self._end_pose_source, source_frame)
        if self._verbose_debug:
            self.get_logger().info(
                f'[end_pose] frame={source_frame}, pos={np.round(result[:3, 3], 4).tolist()}, '
                f'quat={np.round(R.from_matrix(result[:3, :3]).as_quat(), 4).tolist()}'
            )
        return result

    def _transform_pose_to_planning(self, pose_input: np.ndarray, input_frame: str = 'camera_optical_frame'):
        if input_frame == self._planning_frame:
            return pose_input
        if input_frame == self._reference_frame:
            return self._convert_pose_to_planning_frame(pose_input, self._reference_frame)

        end_pose_planning = self._get_end_pose_in_planning_frame()
        if end_pose_planning is None:
            raise RuntimeError('No /piper/end_pose received yet')

        # piper_ros/Isaac Sim 当前配置：
        #   gripper_base -> end_cam: xyz=[-0.045, 0.0, 0.065], rpy=[0, -90deg, 0]
        q_gripper_to_cam = [-0.120, 0.124, -0.697, 0.696]  # Real hand-eye calibration
        T_gripper_to_cam = np.eye(4, dtype=np.float32)
        T_gripper_to_cam[:3, :3] = R.from_quat(q_gripper_to_cam).as_matrix()
        T_gripper_to_cam[:3, 3] = [-0.0763, 0.0039, 0.0350]  # Real hand-eye calibration
        T_cam_to_gripper = np.linalg.inv(T_gripper_to_cam)

        # 感知端输出的是 camera_optical_frame，需要先转回 end_cam。
        q_cam_to_optical = [-0.5, 0.5, -0.5, 0.5]
        T_cam_to_optical = np.eye(4, dtype=np.float32)
        T_cam_to_optical[:3, :3] = R.from_quat(q_cam_to_optical).as_matrix()

        if input_frame in ('camera_optical_frame', 'end_cam_optical_frame', 'optical'):
            pose_in_cam = np.linalg.inv(T_cam_to_optical) @ pose_input
        elif input_frame in ('camera', 'end_cam', 'camera_link'):
            pose_in_cam = pose_input
        else:
            raise ValueError(f'Unsupported input frame: {input_frame}')

        pose_in_gripper = T_cam_to_gripper @ pose_in_cam
        result = end_pose_planning @ pose_in_gripper

        return result

    def _position_in_base_frame(self, world_position: np.ndarray) -> np.ndarray:
        world_position = np.asarray(world_position, dtype=np.float32)
        return self._base_rotation_world.inv().apply(world_position - self._base_position_world)

    def _pose_in_base_workspace(self, pose_matrix: np.ndarray) -> bool:
        pos_base = self._position_in_base_frame(pose_matrix[:3, 3])
        mins = self._base_workspace_bounds[:, 0]
        maxs = self._base_workspace_bounds[:, 1]
        return bool(np.all(pos_base >= mins) and np.all(pos_base <= maxs))

    def _limit_offset_pose(self, anchor_pose: np.ndarray, pose_matrix: np.ndarray, max_distance: float) -> np.ndarray:
        if pose_matrix is None:
            return None
        limited = np.asarray(pose_matrix, dtype=np.float32).copy()
        delta = limited[:3, 3] - np.asarray(anchor_pose, dtype=np.float32)[:3, 3]
        dist = float(np.linalg.norm(delta))
        if dist <= max_distance or dist < 1e-8:
            return limited
        limited[:3, 3] = np.asarray(anchor_pose, dtype=np.float32)[:3, 3] + (delta / dist) * float(max_distance)
        return limited

    def _align_pose_to_direction(self, pose_matrix: np.ndarray, source_position: np.ndarray, target_position: np.ndarray) -> np.ndarray:
        aligned = np.asarray(pose_matrix, dtype=np.float32).copy()
        direction = np.asarray(target_position, dtype=np.float32) - np.asarray(source_position, dtype=np.float32)
        norm = float(np.linalg.norm(direction))
        if norm < 1e-8:
            return aligned
        z_axis = direction / norm
        ref_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        if abs(float(np.dot(z_axis, ref_up))) > 0.95:
            ref_up = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        x_axis = np.cross(ref_up, z_axis)
        x_norm = float(np.linalg.norm(x_axis))
        if x_norm < 1e-8:
            return aligned
        x_axis /= x_norm
        y_axis = np.cross(z_axis, x_axis)
        y_norm = float(np.linalg.norm(y_axis))
        if y_norm < 1e-8:
            return aligned
        y_axis /= y_norm
        aligned[:3, :3] = np.stack([x_axis, y_axis, z_axis], axis=1)
        return aligned

    def _with_orientation(self, pose_matrix: np.ndarray, orientation_matrix: np.ndarray) -> np.ndarray:
        pose = np.asarray(pose_matrix, dtype=np.float32).copy()
        pose[:3, :3] = np.asarray(orientation_matrix, dtype=np.float32)
        return pose

    def _fixed_downward_orientation(self) -> np.ndarray:
        """生成固定的向下抓取姿态（末端执行器垂直向下）。"""
        # Piper 机械臂的向下姿态：Z轴向下，X轴向前
        # 这种姿态适合从上方接近抓取
        return np.array([
            [1.0, 0.0, 0.0],   # X: 向前
            [0.0, 0.0, -1.0],  # Y: 向后
            [0.0, 1.0, 0.0],   # Z: 向下（接近方向）
        ], dtype=np.float32)

    def _generate_ik_pose_variants(self, pose_matrix: np.ndarray):
        target_pose = np.asarray(pose_matrix, dtype=np.float32)
        variants = []
        current_pose = self._get_end_pose_in_planning_frame()

        # 首选：固定向下姿态（适合桌面抓取，减少不必要的旋转）
        fixed_downward = self._with_orientation(target_pose, self._fixed_downward_orientation())
        variants.append(('fixed_downward', fixed_downward))

        if current_pose is not None:
            variants.append((
                'current_orientation',
                self._with_orientation(target_pose, current_pose[:3, :3]),
            ))

            current_to_target = self._align_pose_to_direction(
                target_pose,
                current_pose[:3, 3],
                target_pose[:3, 3],
            )
            variants.append(('approach_aligned', current_to_target))
        variants.append(('target', target_pose))

        seen = []
        deduped = []
        for label, variant in variants:
            key = np.round(variant[:3, :3], 4).tobytes() + np.round(variant[:3, 3], 4).tobytes()
            if key in seen:
                continue
            seen.append(key)
            deduped.append((label, variant))
        return deduped[:self._max_ik_pose_variants]

    def _send_arm(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Include current gripper position to prevent gripper from closing
        # when arm commands are sent
        gripper_pos = self._current_joints.get(self._gripper_joint, self._gripper_open_width)
        msg.name = self._arm_joints + [self._gripper_joint]
        msg.position = list(positions) + [float(gripper_pos)]
        self.joint_pub.publish(msg)

    def _send_gripper(self, width: float):
        width = max(0.0, min(float(width), 0.08))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Include current arm positions to maintain continuity
        arm_positions = [float(self._current_joints.get(name, 0.0)) for name in self._arm_joints]
        msg.name = self._arm_joints + [self._gripper_joint]
        msg.position = arm_positions + [width]
        self.joint_pub.publish(msg)
        self.get_logger().info(f'Gripper: {width:.3f}')

    def _resolve_close_width(self, predicted_width: float) -> float:
        predicted_width = float(predicted_width)
        if self._force_close_width is not None:
            commanded = self._force_close_width
            strategy = 'forced'
        else:
            commanded = predicted_width - self._close_width_margin
            strategy = 'predicted-minus-margin'
        commanded = max(0.0, min(commanded, 0.08))
        self.get_logger().info(
            f'Close width strategy={strategy}, predicted={predicted_width:.3f}, '
            f'margin={self._close_width_margin:.3f}, commanded={commanded:.3f}'
        )
        return commanded

    def _build_joint_state_seed(self, arm_seed=None):
        if self._current_joints:
            name_to_pos = {name: float(pos) for name, pos in self._current_joints.items()}
        else:
            name_to_pos = {}

        if arm_seed is not None:
            for idx, joint_name in enumerate(self._arm_joints):
                name_to_pos[joint_name] = float(arm_seed[idx])

        if not name_to_pos:
            return [], []
        return list(name_to_pos.keys()), [name_to_pos[name] for name in name_to_pos]

    def _random_arm_seed(self):
        joint_limits = [
            (-2.618, 2.618),
            (0.0, 3.14),
            (-2.967, 0.0),
            (-1.745, 1.745),
            (-1.22, 1.22),
            (-2.0944, 2.0944),
        ]
        return [random.uniform(low, high) for low, high in joint_limits]

    def _clear_planning_scene(self):
        """清空规划场景中的碰撞物体，避免误碰撞检测。"""
        # 跳过清空规划场景，因为这可能导致阻塞
        # 如果需要清空，应该在 MoveIt 启动时配置空场景
        pass

    def _solve_ik(self, pose_matrix: np.ndarray, attempts: int = None):
        """使用 IK 服务求解逆运动学，然后用关节约束规划轨迹。"""
        attempts = self._ik_attempts if attempts is None else max(1, int(attempts))
        if self.ik_client.wait_for_service(timeout_sec=1.0):
            from moveit_msgs.srv import GetPositionIK

            seeds_to_try = []
            if self._current_joints:
                seeds_to_try.append([self._current_joints.get(name, 0.0) for name in self._arm_joints])
            else:
                seeds_to_try.append([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            for _ in range(max(0, attempts - 1)):
                seeds_to_try.append(self._random_arm_seed())

            pose_variants = self._generate_ik_pose_variants(pose_matrix)
            for pose_label, pose_variant in pose_variants:
                pos = pose_variant[:3, 3]
                quat = R.from_matrix(pose_variant[:3, :3]).as_quat()
                self.get_logger().info(
                    f'IK request [{pose_label}] - pos ({self._planning_frame}): '
                    f'{np.round(pos, 4).tolist()}, quat: {np.round(quat, 4).tolist()}'
                )

                for seed_idx, arm_seed in enumerate(seeds_to_try):
                    seed_names, seed_positions = self._build_joint_state_seed(arm_seed)
                    try_without_seed = bool(seed_names)
                    for variant_idx, use_seed in enumerate((True, False) if try_without_seed else (True,)):
                        ik_request = GetPositionIK.Request()
                        ik_request.ik_request.group_name = self._move_group
                        ik_request.ik_request.pose_stamped.header.frame_id = self._planning_frame
                        ik_request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
                        ik_request.ik_request.pose_stamped.pose = self._matrix_to_pose(pose_variant)
                        ik_request.ik_request.timeout.sec = int(max(1.0, self._ik_request_timeout_sec))
                        ik_request.ik_request.timeout.nanosec = int(
                            max(0.0, self._ik_request_timeout_sec - int(self._ik_request_timeout_sec)) * 1e9
                        )
                        ik_request.ik_request.avoid_collisions = False

                        if use_seed and seed_names:
                            ik_request.ik_request.robot_state.joint_state.name = seed_names
                            ik_request.ik_request.robot_state.joint_state.position = seed_positions

                        future = self.ik_client.call_async(ik_request)
                        if not self._wait_for_future(future, timeout_sec=self._ik_wait_timeout_sec):
                            label = 'fallback ' if not use_seed and variant_idx > 0 else ''
                            if self._verbose_debug:
                                self.get_logger().warn(
                                    f'IK [{pose_label}] {label}attempt {seed_idx + 1} timed out'
                                )
                            continue

                        try:
                            result = future.result()
                        except Exception as exc:
                            self.get_logger().warn(f'IK service exception: {exc}')
                            continue

                        if result.error_code.val == MOVEIT_SUCCESS:
                            solved_joints = result.solution.joint_state
                            name_to_pos = dict(zip(solved_joints.name, solved_joints.position))
                            target_joints = [name_to_pos.get(name, 0.0) for name in self._arm_joints]
                            self.get_logger().info(f'IK solved with pose variant {pose_label}')
                            return self._plan_to_joints(target_joints)

                        if use_seed and result.error_code.val == MOVEIT_ROBOT_STATE_INCOMPLETE:
                            if self._verbose_debug:
                                self.get_logger().warn(
                                    f'IK [{pose_label}] attempt {seed_idx + 1} returned ROBOT_STATE_INCOMPLETE; '
                                    'retrying without explicit robot_state'
                                )
                            continue

                        if self._verbose_debug:
                            self.get_logger().warn(
                                f'IK [{pose_label}] attempt {seed_idx + 1} failed: error_code={result.error_code.val}'
                            )
                        break

        if self._enable_pose_fallback:
            self.get_logger().warn('All IK attempts failed, trying pose constraint planning')
            return self._plan_with_pose_constraint(pose_matrix, relaxed_orientation=True)
        self.get_logger().warn('All IK attempts failed')
        return None

    def _plan_with_pose_constraint(self, pose_matrix: np.ndarray, relaxed_orientation: bool = False):
        """使用位姿约束进行规划（回退方法）。"""
        if not self._move_group_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().error('Pose-constraint fallback unavailable: move_action server not available')
            return None

        from moveit_msgs.action import MoveGroup
        from geometry_msgs.msg import Transform

        pos = pose_matrix[:3, 3]
        quat = R.from_matrix(pose_matrix[:3, :3]).as_quat()

        goal = MoveGroup.Goal()
        goal.request.group_name = self._move_group
        goal.request.allowed_planning_time = 30.0
        goal.request.num_planning_attempts = 10

        if self._current_joints:
            goal.request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            goal.request.start_state.joint_state.name = list(self._current_joints.keys())
            goal.request.start_state.joint_state.position = list(self._current_joints.values())

        # 位置约束 - 使用较大的球体范围
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = self._planning_frame
        pos_constraint.link_name = 'link6'
        pos_constraint.constraint_region.primitives.append(SolidPrimitive())
        pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        # 放宽位置约束范围
        pos_tolerance = 0.05 if relaxed_orientation else 0.02
        pos_constraint.constraint_region.primitives[0].dimensions = [pos_tolerance]
        pos_constraint.constraint_region.primitive_poses.append(Pose())
        pos_constraint.constraint_region.primitive_poses[0].position.x = float(pos[0])
        pos_constraint.constraint_region.primitive_poses[0].position.y = float(pos[1])
        pos_constraint.constraint_region.primitive_poses[0].position.z = float(pos[2])
        pos_constraint.weight = 1.0

        # 朝向约束
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = self._planning_frame
        ori_constraint.link_name = 'link6'
        ori_constraint.orientation.x = float(quat[0])
        ori_constraint.orientation.y = float(quat[1])
        ori_constraint.orientation.z = float(quat[2])
        ori_constraint.orientation.w = float(quat[3])
        # 放宽朝向约束范围
        if relaxed_orientation:
            ori_constraint.absolute_x_axis_tolerance = 0.5
            ori_constraint.absolute_y_axis_tolerance = 0.5
            ori_constraint.absolute_z_axis_tolerance = 0.5
            ori_constraint.weight = 0.1
        else:
            ori_constraint.absolute_x_axis_tolerance = 0.3
            ori_constraint.absolute_y_axis_tolerance = 0.3
            ori_constraint.absolute_z_axis_tolerance = 0.3
            ori_constraint.weight = 0.3

        constraints = Constraints()
        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [ori_constraint]
        goal.request.goal_constraints = [constraints]

        self.get_logger().info(
            f'Pose constraint planning: pos_tolerance={pos_tolerance}, '
            f'ori_tolerance={ori_constraint.absolute_x_axis_tolerance}, '
            f'target=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}]'
        )

        future = self._move_group_client.send_goal_async(goal)

        if not self._wait_for_future(future, timeout_sec=10.0):
            self.get_logger().error('Pose planning: goal acceptance timed out')
            return None

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Pose planning: goal exception: {exc}')
            return None

        if not goal_handle.accepted:
            self.get_logger().error('Pose planning: goal rejected')
            return None

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, timeout_sec=35.0):
            self.get_logger().error('Pose planning: result timed out')
            return None

        try:
            result = result_future.result()
        except Exception as exc:
            self.get_logger().error(f'Pose planning: result exception: {exc}')
            return None

        if result.result.error_code.val != MOVEIT_SUCCESS:
            self.get_logger().error(f'Pose planning failed: error_code={result.result.error_code.val}')
            return None

        traj = result.result.planned_trajectory.joint_trajectory
        if not traj.points:
            self.get_logger().error('Pose planning: no trajectory points')
            return None

        self.get_logger().info(f'Pose plan OK: {len(traj.points)} points')
        return traj

    def _plan_to_joints(self, target_joints):
        """使用 /plan_kinematic_path 服务规划到目标关节位置。"""
        if not self.plan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Plan service unavailable')
            return None

        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = self._move_group
        req.motion_plan_request.allowed_planning_time = 10.0
        req.motion_plan_request.num_planning_attempts = 10

        if self._current_joints:
            req.motion_plan_request.start_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            req.motion_plan_request.start_state.joint_state.name = list(self._current_joints.keys())
            req.motion_plan_request.start_state.joint_state.position = list(self._current_joints.values())

        self.get_logger().info(f'Planning to joints: {[f"{p:.3f}" for p in target_joints]}')

        constraints = Constraints()
        for idx, joint_name in enumerate(self._arm_joints):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(target_joints[idx])
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.motion_plan_request.goal_constraints.append(constraints)

        future = self.plan_client.call_async(req)
        if not self._wait_for_future(future, timeout_sec=10.0):
            self.get_logger().error('Planning: service result timed out')
            return None

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'Planning: service exception: {exc}')
            return None

        if result.motion_plan_response.error_code.val != MOVEIT_SUCCESS:
            self.get_logger().error(
                f'Planning failed: error_code={result.motion_plan_response.error_code.val}'
            )
            return None

        trajectory = result.motion_plan_response.trajectory.joint_trajectory
        self.get_logger().info(f'Plan OK: {len(trajectory.points)} points')
        return trajectory

    def _execute_trajectory(self, trajectory):
        points = trajectory.points
        if not points:
            return False

        for idx, point in enumerate(points):
            self._send_arm(point.positions)
            if idx >= len(points) - 1:
                continue
            now_t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            next_t = (
                points[idx + 1].time_from_start.sec +
                points[idx + 1].time_from_start.nanosec * 1e-9
            )
            dt = max(0.03, min(next_t - now_t, 0.5))
            time.sleep(dt)

        time.sleep(0.5)
        return True

    def _return_to_home(self, reason: str = 'complete') -> bool:
        if not self._return_home:
            return True
        self._maybe_capture_home_joints()
        if self._home_joints is None:
            self.get_logger().warn('Home return skipped: no home joint state captured yet')
            return False

        self.get_logger().info(f'Step 7: Return home ({reason})')
        trajectory = self._plan_to_joints(self._home_joints)
        if trajectory is None:
            self.get_logger().error('Return-home planning failed')
            return False
        self._execute_trajectory(trajectory)
        self.get_logger().info('Return-home complete')
        return True

    def _abort_sequence(self, failure_code: str, message: str):
        self._publish_execution_state(True, failure_code)
        self.get_logger().error(message)
        if self._return_home and self._return_home_on_failure:
            self._return_to_home(reason=failure_code.replace('failed:', ''))

    @staticmethod
    def _lift_pose(pose_matrix: np.ndarray, dz: float):
        lifted = pose_matrix.copy()
        lifted[2, 3] += float(dz)
        return lifted

    def _execute_sequence(self, pregrasp, grasp, retreat, gripper_width, input_frame):
        if self._verbose_debug:
            self.get_logger().info(f'=== INPUT DEBUG ===')
            self.get_logger().info(f'Input grasp pose (in {input_frame}):')
            self.get_logger().info(f'  translation: {np.round(grasp[:3, 3], 4).tolist()}')
            self.get_logger().info(f'  rotation:\n{np.round(grasp[:3, :3], 4)}')
            if isinstance(pregrasp, np.ndarray) and pregrasp.shape == (4, 4):
                self.get_logger().info(f'Input pregrasp pose (in {input_frame}):')
                self.get_logger().info(f'  translation: {np.round(pregrasp[:3, 3], 4).tolist()}')
                offset = pregrasp[:3, 3] - grasp[:3, 3]
                self.get_logger().info(f'  pregrasp offset from grasp: {np.round(offset, 4).tolist()}')
            self.get_logger().info(f'=== END INPUT DEBUG ===')

        # ===== DEBUG: 转换前后的坐标 =====
        self.get_logger().info(f'=== COORD TRANSFORM DEBUG ===')
        self.get_logger().info(f'  input_frame: {input_frame}')
        self.get_logger().info(f'  input grasp pos: [{grasp[0,3]:.4f}, {grasp[1,3]:.4f}, {grasp[2,3]:.4f}]')

        current_pose_planning = self._get_end_pose_in_planning_frame()
        if current_pose_planning is not None:
            self.get_logger().info(f'  current end_pose pos: [{current_pose_planning[0,3]:.4f}, {current_pose_planning[1,3]:.4f}, {current_pose_planning[2,3]:.4f}]')

        grasp_planning = self._transform_pose_to_planning(grasp, input_frame)

        # Gripper depth correction:
        # Contact-GraspNet grasp pose origin = gripper center (between fingers)
        # MoveIt IK target = link6 (robot flange)
        # link6 is BEHIND gripper center (opposite to approach direction)
        # So we move backward along approach to get link6 position from gripper center
        if self._gripper_depth > 0:
            approach = grasp_planning[:3, 2].copy()
            approach_norm = float(np.linalg.norm(approach))
            if approach_norm > 1e-6:
                approach = approach / approach_norm
                original_pos = grasp_planning[:3, 3].copy()
                grasp_planning[:3, 3] -= approach * self._gripper_depth
                self.get_logger().info(
                    f'[GRIPPER DEPTH] -{self._gripper_depth*1000:.0f}mm along approach: '
                    f'{np.round(original_pos, 4).tolist()} -> {np.round(grasp_planning[:3,3], 4).tolist()}'
                )

        # 应用手眼标定误差偏移校正
        grasp_planning[:3, 3] += self._grasp_offset
        grasp_pos = grasp_planning[:3, 3]
        self.get_logger().info(f'  transformed grasp pos (world): [{grasp_pos[0]:.4f}, {grasp_pos[1]:.4f}, {grasp_pos[2]:.4f}]')
        if np.any(self._grasp_offset != 0):
            self.get_logger().info(f'  applied grasp offset: {np.round(self._grasp_offset, 4).tolist()}')
        self.get_logger().info(f'=== END COORD TRANSFORM DEBUG ===')
        self.get_logger().info(f'Grasp planning: {np.round(grasp_pos, 3).tolist()}')

        pregrasp_planning = None
        if isinstance(pregrasp, np.ndarray) and pregrasp.shape == (4, 4):
            pregrasp_planning = self._transform_pose_to_planning(pregrasp, input_frame)
            # Apply gripper depth correction to pregrasp as well (backward along approach)
            if self._gripper_depth > 0:
                pregrasp_approach = pregrasp_planning[:3, 2].copy()
                pregrasp_approach_norm = float(np.linalg.norm(pregrasp_approach))
                if pregrasp_approach_norm > 1e-6:
                    pregrasp_approach = pregrasp_approach / pregrasp_approach_norm
                    pregrasp_planning[:3, 3] -= pregrasp_approach * self._gripper_depth
            pregrasp_planning = self._limit_offset_pose(
                grasp_planning,
                pregrasp_planning,
                self._max_pregrasp_distance,
            )

        retreat_planning = None
        if isinstance(retreat, np.ndarray) and retreat.shape == (4, 4):
            retreat_planning = self._transform_pose_to_planning(retreat, input_frame)
            # Apply gripper depth correction to retreat as well (backward along approach)
            if self._gripper_depth > 0:
                retreat_approach = retreat_planning[:3, 2].copy()
                retreat_approach_norm = float(np.linalg.norm(retreat_approach))
                if retreat_approach_norm > 1e-6:
                    retreat_approach = retreat_approach / retreat_approach_norm
                    retreat_planning[:3, 3] -= retreat_approach * self._gripper_depth
            retreat_planning = self._limit_offset_pose(
                grasp_planning,
                retreat_planning,
                self._max_retreat_distance,
            )

        if self._align_poses:
            motion_start = current_pose_planning[:3, 3] if current_pose_planning is not None else grasp_planning[:3, 3]
            if pregrasp_planning is not None:
                pregrasp_planning = self._align_pose_to_direction(
                    pregrasp_planning,
                    motion_start,
                    pregrasp_planning[:3, 3],
                )
                grasp_planning = self._align_pose_to_direction(
                    grasp_planning,
                    pregrasp_planning[:3, 3],
                    grasp_planning[:3, 3],
                )
            else:
                grasp_planning = self._align_pose_to_direction(
                    grasp_planning,
                    motion_start,
                    grasp_planning[:3, 3],
                )
            if retreat_planning is not None:
                retreat_planning = self._align_pose_to_direction(
                    retreat_planning,
                    grasp_planning[:3, 3],
                    retreat_planning[:3, 3],
                )

        for label, pose_matrix in (
            ('pregrasp', pregrasp_planning),
            ('grasp', grasp_planning),
            ('retreat', retreat_planning),
        ):
            if pose_matrix is None:
                continue
            pos_base = self._position_in_base_frame(pose_matrix[:3, 3])
            self.get_logger().info(f'{label} in base: {np.round(pos_base, 4).tolist()}')
            if not self._pose_in_base_workspace(pose_matrix):
                self.get_logger().error(
                    f'{label} pose outside base workspace: {np.round(pos_base, 4).tolist()}'
                )
                self._abort_sequence(
                    f'failed:{label}_workspace',
                    f'{label} pose outside base workspace: {np.round(pos_base, 4).tolist()}'
                )
                return

        self.get_logger().info('Step 1: Open gripper')
        self._send_gripper(self._gripper_open_width)
        # Update tracked gripper state so subsequent _send_arm calls keep it open
        self._current_joints[self._gripper_joint] = float(self._gripper_open_width)
        time.sleep(0.3)

        if pregrasp_planning is not None:
            self.get_logger().info('Step 2: Move to pregrasp')
            trajectory = self._solve_ik(pregrasp_planning)
            if trajectory is None:
                self._abort_sequence('failed:pregrasp', 'Pregrasp planning failed')
                return
            self._execute_trajectory(trajectory)
            time.sleep(0.5)

        self.get_logger().info('Step 3: Move to grasp')
        trajectory = self._solve_ik(grasp_planning)
        if trajectory is None:
            self._abort_sequence('failed:grasp', 'Grasp planning failed')
            return
        self._execute_trajectory(trajectory)
        time.sleep(0.5)

        close_width = self._resolve_close_width(gripper_width)
        self.get_logger().info('Step 4: Close gripper')
        self._send_gripper(close_width)
        # Update tracked gripper state
        self._current_joints[self._gripper_joint] = float(close_width)
        time.sleep(0.5)

        if self._post_grasp_lift_distance > 0.0:
            lift_pose = self._lift_pose(grasp_planning, self._post_grasp_lift_distance)
            self.get_logger().info('Step 5: Lift')
            trajectory = self._solve_ik(lift_pose)
            if trajectory is None:
                self._abort_sequence('failed:lift', 'Lift planning failed')
                return
            self._execute_trajectory(trajectory)
            time.sleep(self._post_grasp_lift_hold_time)

        if retreat_planning is not None:
            self.get_logger().info('Step 6: Retreat')
            trajectory = self._solve_ik(retreat_planning)
            if trajectory is None:
                self._abort_sequence('failed:retreat', 'Retreat planning failed')
                return
            self._execute_trajectory(trajectory)

        if self._return_home:
            self._publish_execution_state(True, 'returning_home')
            self._return_to_home(reason='complete')

        self._publish_execution_state(True, 'complete')
        self.get_logger().info('Grasp sequence complete')

    def _execute_single_pose(self, pose_msg: PoseStamped):
        matrix = np.eye(4, dtype=np.float32)
        matrix[0, 3] = pose_msg.pose.position.x
        matrix[1, 3] = pose_msg.pose.position.y
        matrix[2, 3] = pose_msg.pose.position.z
        quat = [
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ]
        matrix[:3, :3] = R.from_quat(quat).as_matrix()
        input_frame = pose_msg.header.frame_id or self._planning_frame
        planning_pose = self._transform_pose_to_planning(matrix, input_frame)
        trajectory = self._solve_ik(planning_pose)
        if trajectory is not None:
            self._execute_trajectory(trajectory)


def main():
    parser = argparse.ArgumentParser(description='Piper grasp executor using MoveIt planning')
    parser.add_argument('--move-group', type=str, default='arm')
    parser.add_argument('--planning-frame', type=str, default='world')
    parser.add_argument('--reference-frame', type=str, default='world')
    parser.add_argument('--gripper-open-width', type=float, default=0.08)
    parser.add_argument('--force-close-width', type=float, default=None)
    parser.add_argument('--close-width-margin', type=float, default=0.0)
    parser.add_argument('--gripper-settle-time', type=float, default=0.5)
    parser.add_argument('--post-grasp-lift-distance', type=float, default=0.0)
    parser.add_argument('--post-grasp-lift-hold-time', type=float, default=0.0)
    parser.add_argument('--base-position-world', type=str, default='[-50.6,-27.7,0.85]')
    parser.add_argument('--base-yaw-deg', type=float, default=180.0)
    parser.add_argument('--base-workspace-bounds', type=str, default='[[-0.15,0.35],[-0.35,0.35],[-0.10,0.45]]')
    parser.add_argument('--max-pregrasp-distance', type=float, default=0.06)
    parser.add_argument('--max-retreat-distance', type=float, default=0.08)
    parser.add_argument('--disable-align-poses', action='store_true')
    parser.add_argument('--ik-attempts', type=int, default=2)
    parser.add_argument('--ik-request-timeout-sec', type=float, default=1.0)
    parser.add_argument('--ik-wait-timeout-sec', type=float, default=1.2)
    parser.add_argument('--max-ik-pose-variants', type=int, default=2)
    parser.add_argument('--enable-pose-fallback', action='store_true')
    parser.add_argument('--verbose-debug', action='store_true')
    parser.add_argument('--disable-return-home', action='store_true')
    parser.add_argument('--disable-return-home-on-failure', action='store_true')
    parser.add_argument('--grasp-offset', type=str, default='[0.0,0.0,0.0]',
                        help='Position offset [x,y,z] to correct hand-eye calibration errors (meters)')
    parser.add_argument('--gripper-depth', type=float, default=0.111,
                        help='Distance from Link6 (flange) to gripper center (meters). Link6->camera=35mm, camera->gripper=76mm, total=111mm')
    args = parser.parse_args()

    rclpy.init()
    node = GraspExecutorMotion(
        move_group=args.move_group,
        planning_frame=args.planning_frame,
        reference_frame=args.reference_frame,
        gripper_open_width=args.gripper_open_width,
        force_close_width=args.force_close_width,
        close_width_margin=args.close_width_margin,
        gripper_settle_time=args.gripper_settle_time,
        post_grasp_lift_distance=args.post_grasp_lift_distance,
        post_grasp_lift_hold_time=args.post_grasp_lift_hold_time,
        base_position_world=parse_vector3(args.base_position_world, [-50.6, -27.7, 0.85]),
        base_yaw_deg=args.base_yaw_deg,
        base_workspace_bounds=parse_workspace_bounds(
            args.base_workspace_bounds,
            [[-0.15, 0.35], [-0.35, 0.35], [-0.10, 0.45]],
        ),
        max_pregrasp_distance=args.max_pregrasp_distance,
        max_retreat_distance=args.max_retreat_distance,
        align_poses=not args.disable_align_poses,
        ik_attempts=args.ik_attempts,
        ik_request_timeout_sec=args.ik_request_timeout_sec,
        ik_wait_timeout_sec=args.ik_wait_timeout_sec,
        max_ik_pose_variants=args.max_ik_pose_variants,
        enable_pose_fallback=args.enable_pose_fallback,
        verbose_debug=args.verbose_debug,
        return_home=not args.disable_return_home,
        return_home_on_failure=not args.disable_return_home_on_failure,
        grasp_offset=parse_vector3(args.grasp_offset, [0.0, 0.0, 0.0]),
        gripper_depth=args.gripper_depth,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    print('GraspExecutorMotion running. Ctrl+C to stop.')
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Context may already be shut down


if __name__ == '__main__':
    main()
