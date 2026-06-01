#!/usr/bin/env python3
"""
Grasp Executor for Piper Robot (Simplified Version)

Uses direct joint command publishing instead of action clients for reliability.

Usage (system Python, not conda):
    source /opt/ros/humble/setup.bash
    source /home/jjl/git_projects/piper_ros/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 grasp_executor_simple.py
"""

import argparse
import json
import time
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes


class GraspExecutorSimple(Node):
    """
    Simple grasp executor using IK service and direct joint commands.
    """

    def __init__(self,
                 workspace_bounds=None,
                 move_group: str = 'piper_arm',
                 gripper_open_width: float = 0.08,
                 force_close_width: float = None,
                 close_width_margin: float = 0.0,
                 gripper_settle_time: float = 0.5,
                 post_grasp_lift_distance: float = 0.0,
                 post_grasp_lift_hold_time: float = 0.0):
        super().__init__('grasp_executor_simple')

        # Execution lock
        self._execution_lock = threading.Lock()
        self._executing = False

        # Joint names
        self._arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self._gripper_joint = 'joint7'

        # Current joint states
        self._current_joints = {}
        self._end_pose_source = None  # Current end effector pose in the frame published by /piper/end_pose
        self._end_pose_frame = None
        self._joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Planning frame used by MoveIt, and the source frame used by Isaac.
        self._planning_frame = 'world'
        self._reference_frame = 'world'
        # /piper/end_pose is published directly in the MoveIt planning frame (`world`).
        self._planning_frame_in_reference = np.eye(4, dtype=np.float32)
        self._reference_to_planning = np.linalg.inv(self._planning_frame_in_reference)
        self._auto_align_orientation = True
        self._workspace_bounds = self._parse_workspace_bounds(workspace_bounds)
        self._move_group = str(move_group)
        self._gripper_open_width = float(gripper_open_width)
        self._force_close_width = None if force_close_width is None else float(force_close_width)
        self._close_width_margin = float(close_width_margin)
        self._gripper_settle_time = float(gripper_settle_time)
        self._post_grasp_lift_distance = float(post_grasp_lift_distance)
        self._post_grasp_lift_hold_time = float(post_grasp_lift_hold_time)

        # Subscribe to joint states (from piper, forwarded by bridge)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/piper/joint_states',  # Isaac Sim publishes here, bridge forwards to /joint_states
            self._joint_state_callback,
            10
        )

        # Subscribe to end effector pose (for coordinate transform)
        self.end_pose_sub = self.create_subscription(
            PoseStamped,
            '/piper/end_pose',
            self._end_pose_callback,
            10
        )

        # Subscribe to grasp execution command
        self.grasp_exec_sub = self.create_subscription(
            String,
            '/piper/grasp_execution',
            self._grasp_execution_callback,
            10
        )

        # Subscribe to execute trigger
        self.execute_sub = self.create_subscription(
            Bool,
            '/piper/execute_grasp',
            self._execute_callback,
            10
        )

        # Subscribe to target grasp pose
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/piper/target_grasp',
            self._target_callback,
            10
        )

        # IK service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Joint command publishers
        self.arm_pub = self.create_publisher(JointState, '/piper/joint_ctrl', 10)

        # Stored poses
        self._stored_pose = None

        self.get_logger().info('GraspExecutorSimple initialized')
        self.get_logger().info('  Subscribed: /piper/grasp_execution')
        self.get_logger().info('  IK service: /compute_ik')
        self.get_logger().info('  Publishes: /piper/joint_ctrl')
        self.get_logger().info(f'  Move group: {self._move_group}')
        if self._workspace_bounds is not None:
            self.get_logger().info(
                f'  Workspace bounds ({self._planning_frame}): {self._workspace_bounds.tolist()}'
            )
        self.get_logger().info(f'  Gripper open width: {self._gripper_open_width:.3f}')
        if self._force_close_width is not None:
            self.get_logger().info(f'  Forced grasp close width: {self._force_close_width:.3f}')
        self.get_logger().info(f'  Close width margin: {self._close_width_margin:.3f}')
        self.get_logger().info(f'  Gripper settle time: {self._gripper_settle_time:.2f}s')
        self.get_logger().info(f'  Post-grasp lift distance: {self._post_grasp_lift_distance:.3f}')
        self.get_logger().info(f'  Post-grasp lift hold time: {self._post_grasp_lift_hold_time:.2f}s')

    @staticmethod
    def _parse_workspace_bounds(bounds):
        """Parse workspace bounds into a (3, 2) array."""
        if bounds in (None, ''):
            return None
        if isinstance(bounds, str):
            bounds = eval(bounds)
        bounds = np.asarray(bounds, dtype=np.float32)
        if bounds.shape != (3, 2):
            raise ValueError(
                'workspace bounds must have shape [[xmin,xmax],[ymin,ymax],[zmin,zmax]]'
            )
        return bounds

    def _pose_within_workspace(self, pose_matrix: np.ndarray) -> bool:
        """Check whether a planning-frame pose lies inside the configured workspace box."""
        if self._workspace_bounds is None:
            return True
        pos = pose_matrix[:3, 3]
        mins = self._workspace_bounds[:, 0]
        maxs = self._workspace_bounds[:, 1]
        return bool(np.all(pos >= mins) and np.all(pos <= maxs))

    def _require_pose_in_workspace(self, label: str, pose_matrix: np.ndarray):
        """Raise with a clear message if the target lies outside the configured workspace."""
        if self._pose_within_workspace(pose_matrix):
            return

        pos = np.round(pose_matrix[:3, 3], 3).tolist()
        mins = np.round(self._workspace_bounds[:, 0], 3).tolist()
        maxs = np.round(self._workspace_bounds[:, 1], 3).tolist()
        raise ValueError(
            f'{label} outside workspace in {self._planning_frame}: pos={pos}, mins={mins}, maxs={maxs}'
        )

    def _resolve_close_width(self, predicted_width: float) -> float:
        """Resolve the final gripper close command from prediction and CLI overrides."""
        predicted_width = float(predicted_width)
        if self._force_close_width is not None:
            commanded_width = self._force_close_width
            strategy = 'forced'
        else:
            commanded_width = predicted_width - self._close_width_margin
            strategy = 'predicted-minus-margin'

        commanded_width = max(0.0, min(commanded_width, 0.08))
        self.get_logger().info(
            f'Gripper close strategy={strategy}, predicted={predicted_width:.3f}, '
            f'margin={self._close_width_margin:.3f}, commanded={commanded_width:.3f}'
        )
        return commanded_width

    @staticmethod
    def _offset_pose_along_planning_z(pose_matrix: np.ndarray, delta_z: float) -> np.ndarray:
        """Return a copy of the pose translated along planning-frame +Z."""
        offset_pose = pose_matrix.copy()
        offset_pose[2, 3] += float(delta_z)
        return offset_pose

    def _joint_state_callback(self, msg: JointState):
        """Store current joint states."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._current_joints[name] = msg.position[i]

    def _end_pose_callback(self, msg: PoseStamped):
        """Store current end effector pose in the source frame published by Isaac."""
        self._end_pose_source = np.eye(4)
        self._end_pose_source[0, 3] = msg.pose.position.x
        self._end_pose_source[1, 3] = msg.pose.position.y
        self._end_pose_source[2, 3] = msg.pose.position.z
        quat = [msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w]
        self._end_pose_source[:3, :3] = R.from_quat(quat).as_matrix()
        self._end_pose_frame = msg.header.frame_id or self._reference_frame

    def _target_callback(self, msg: PoseStamped):
        """Store target grasp pose."""
        self._stored_pose = msg
        self.get_logger().info('Stored target grasp pose')

    def _execute_callback(self, msg: Bool):
        """Execute stored pose on trigger."""
        if msg.data and self._stored_pose:
            self._execute_single_pose(self._stored_pose)
            self._stored_pose = None

    def _grasp_execution_callback(self, msg: String):
        """Process grasp execution JSON."""
        try:
            data = json.loads(msg.data)
            frame_id = data.get('frame_id', 0)
            score = data.get('score', 0)
            input_frame = data.get('frame', 'camera_optical_frame')
            self.get_logger().info(
                f'Received grasp command: frame={frame_id}, score={score:.3f}, input_frame={input_frame}'
            )

            # Extract poses
            grasp = np.array(data.get('pose', []))
            pregrasp = np.array(data.get('pregrasp_pose', []))
            retreat = np.array(data.get('retreat_pose', []))
            gripper_width = data.get('gripper_opening', 0.08)

            if grasp.shape == (4, 4):
                self._execute_sequence(pregrasp, grasp, retreat, gripper_width, input_frame=input_frame)
            else:
                self.get_logger().error(f'Invalid pose shape: {grasp.shape}')

        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')

    def _matrix_to_pose(self, matrix: np.ndarray) -> Pose:
        """Convert 4x4 matrix to Pose."""
        pose = Pose()
        pose.position.x = float(matrix[0, 3])
        pose.position.y = float(matrix[1, 3])
        pose.position.z = float(matrix[2, 3])
        rot = R.from_matrix(matrix[:3, :3])
        quat = rot.as_quat()
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def _log_pose_debug(self, label: str, pose_matrix: np.ndarray):
        """Log pose position, quaternion, and local axes for transform debugging."""
        rot = R.from_matrix(pose_matrix[:3, :3])
        quat = rot.as_quat()
        x_axis = pose_matrix[:3, 0]
        y_axis = pose_matrix[:3, 1]
        z_axis = pose_matrix[:3, 2]
        self.get_logger().info(
            f'{label}: pos={np.round(pose_matrix[:3, 3], 3).tolist()}, '
            f'quat={np.round(quat, 3).tolist()}'
        )
        self.get_logger().info(
            f'{label}: axes x={np.round(x_axis, 3).tolist()}, '
            f'y={np.round(y_axis, 3).tolist()}, z={np.round(z_axis, 3).tolist()}'
        )

    def _get_current_arm_joints(self):
        """Return the latest arm joint state in controller order, or None if incomplete."""
        if not self._current_joints:
            return None

        if not all(name in self._current_joints for name in self._arm_joints):
            return None

        return [self._current_joints[name] for name in self._arm_joints]

    def _build_seed_joint_state(self, seed_joints=None):
        """
        Build a complete joint-state seed for MoveIt if possible.

        MoveIt can reject IK requests with ROBOT_STATE_INCOMPLETE if we only send a
        partial robot state. Prefer the latest full joint_state snapshot and only
        overwrite the arm joints with the desired seed values.
        """
        if seed_joints is not None:
            arm_seed_positions = [float(v) for v in seed_joints]
        elif self._current_joints:
            arm_seed_positions = [float(self._current_joints.get(n, 0.0)) for n in self._arm_joints]
        else:
            arm_seed_positions = [0.0] * len(self._arm_joints)

        if self._current_joints:
            name_to_pos = {name: float(pos) for name, pos in self._current_joints.items()}
            for idx, joint_name in enumerate(self._arm_joints):
                name_to_pos[joint_name] = arm_seed_positions[idx]

            joint_names = list(name_to_pos.keys())
            joint_positions = [name_to_pos[name] for name in joint_names]
            seed_source = 'current_full_state'
        else:
            joint_names = list(self._arm_joints)
            joint_positions = arm_seed_positions
            seed_source = 'arm_only_fallback'

        return joint_names, joint_positions, arm_seed_positions, seed_source

    def _call_ik(self, request: GetPositionIK.Request, timeout_sec: float = 5.0):
        """Call the MoveIt IK service and wait for completion on the current node."""
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error('IK timeout')
            return None

        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f'IK call exception: {e}')
            return None

    def _wait_for_arm_target(self, target_positions, timeout_s: float = 4.0, tol: float = 0.05):
        """Wait until /joint_states is close to the commanded arm target."""
        start = time.time()
        while time.time() - start < timeout_s:
            current = self._get_current_arm_joints()
            if current is not None:
                error = np.max(np.abs(np.asarray(current) - np.asarray(target_positions)))
                if error <= tol:
                    self.get_logger().info(f'Arm reached target within tolerance {tol:.3f}')
                    return current
            time.sleep(0.05)

        current = self._get_current_arm_joints()
        if current is not None:
            error = np.max(np.abs(np.asarray(current) - np.asarray(target_positions)))
            self.get_logger().warn(
                f'Arm did not fully reach target within {timeout_s:.1f}s, max error={error:.3f}'
            )
        else:
            self.get_logger().warn('Arm target wait timed out before receiving complete joint state')
        return current

    def _convert_pose_to_planning_frame(self,
                                        pose_matrix: np.ndarray,
                                        source_frame: str) -> np.ndarray:
        """Convert a pose from the source frame into MoveIt's planning frame."""
        if source_frame == self._planning_frame:
            return pose_matrix

        if source_frame == self._reference_frame:
            return self._reference_to_planning @ pose_matrix

        if source_frame in ('base_link', 'base'):
            return pose_matrix

        raise ValueError(
            f'Unsupported source frame {source_frame!r}; expected {self._planning_frame!r} or {self._reference_frame!r}'
        )

    def _get_end_pose_in_planning_frame(self) -> np.ndarray:
        """Return the latest /piper/end_pose transformed into MoveIt's planning frame."""
        if self._end_pose_source is None:
            return None
        source_frame = self._end_pose_frame or self._reference_frame
        return self._convert_pose_to_planning_frame(self._end_pose_source, source_frame)

    def _align_pose_orientation(self,
                                origin_pose: np.ndarray,
                                target_pose: np.ndarray) -> np.ndarray:
        """
        Rebuild target orientation so the tool Z axis points toward the target position.

        This relaxes the raw vision pose while keeping a stable roll close to the current tool frame.
        """
        aligned_pose = target_pose.copy()
        delta = target_pose[:3, 3] - origin_pose[:3, 3]
        distance = np.linalg.norm(delta)
        if distance < 1e-6:
            return aligned_pose

        z_axis = delta / distance

        # Prefer preserving current tool roll by projecting the current X axis.
        x_seed = origin_pose[:3, 0]
        x_axis = x_seed - np.dot(x_seed, z_axis) * z_axis
        if np.linalg.norm(x_axis) < 1e-6:
            up_seed = np.array([0.0, 0.0, 1.0], dtype=np.float32)
            x_axis = np.cross(up_seed, z_axis)
        if np.linalg.norm(x_axis) < 1e-6:
            alt_seed = np.array([1.0, 0.0, 0.0], dtype=np.float32)
            x_axis = np.cross(alt_seed, z_axis)

        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        aligned_pose[:3, 0] = x_axis
        aligned_pose[:3, 1] = y_axis
        aligned_pose[:3, 2] = z_axis
        return aligned_pose

    def _transform_pose_to_world(self,
                                 pose_input: np.ndarray,
                                 input_frame: str = 'camera_optical_frame') -> np.ndarray:
        """
        Transform pose from a supported input frame to the planning/world frame.

        TF tree: gripper_base -> end_cam -> end_cam_optical_frame
        """
        # Get current end effector pose (gripper_base in planning/world frame)
        end_pose_world = self._get_end_pose_in_planning_frame()
        if end_pose_world is None:
            self.get_logger().warn('No end pose available')
            end_pose_world = np.eye(4)

        if input_frame == 'world':
            return pose_input

        # TF transforms (parent->child)
        q_gripper_to_cam = [0, -0.707, 0, 0.707]  # [x, y, z, w]
        rot_gripper_to_cam = R.from_quat(q_gripper_to_cam).as_matrix()
        T_gripper_to_cam = np.eye(4)
        T_gripper_to_cam[:3, :3] = rot_gripper_to_cam
        T_gripper_to_cam[:3, 3] = [-0.045, 0.0, 0.065]

        # T_cam_to_optical
        q_cam_to_optical = [-0.5, 0.5, -0.5, 0.5]  # [x, y, z, w]
        rot_cam_to_optical = R.from_quat(q_cam_to_optical).as_matrix()
        T_cam_to_optical = np.eye(4)
        T_cam_to_optical[:3, :3] = rot_cam_to_optical

        T_cam_to_gripper = np.linalg.inv(T_gripper_to_cam)

        if input_frame in ('camera_optical_frame', 'end_cam_optical_frame', 'optical'):
            pose_in_cam = np.linalg.inv(T_cam_to_optical) @ pose_input
        elif input_frame in ('camera', 'end_cam', 'camera_link'):
            pose_in_cam = pose_input
        else:
            raise ValueError(f'Unsupported input frame: {input_frame}')

        pose_in_gripper = T_cam_to_gripper @ pose_in_cam

        # Final transform: gripper -> world
        pose_world = end_pose_world @ pose_in_gripper

        return pose_world

    def _execute_sequence(self, pregrasp, grasp, retreat, gripper_width, input_frame='camera_optical_frame'):
        """Execute grasp sequence."""
        with self._execution_lock:
            if self._executing:
                self.get_logger().warn('Already executing')
                return
            self._executing = True

        try:
            # Log current end pose
            current_end_pose = self._get_end_pose_in_planning_frame()
            if self._end_pose_source is not None:
                self.get_logger().info(
                    f'Current end pose ({self._end_pose_frame or "unknown"}): pos={self._end_pose_source[:3, 3]}'
                )
                self._log_pose_debug('Current end pose source', self._end_pose_source)
            if current_end_pose is not None:
                self.get_logger().info(
                    f'Current end pose ({self._planning_frame}): pos={current_end_pose[:3, 3]}'
                )
                self._log_pose_debug('Current end pose planning', current_end_pose)
            else:
                self.get_logger().warn('No end pose received yet!')

            # Transform poses from input frame to world frame
            grasp_world = self._transform_pose_to_world(grasp, input_frame=input_frame)
            self.get_logger().info(f'Grasp in world: pos={grasp_world[:3, 3]}')
            self._log_pose_debug('Target grasp world', grasp_world)

            grasp_planning = self._convert_pose_to_planning_frame(grasp_world, self._planning_frame)
            self.get_logger().info(f'Grasp in planning frame ({self._planning_frame}): pos={grasp_planning[:3, 3]}')
            self._log_pose_debug('Target grasp planning', grasp_planning)

            if current_end_pose is not None:
                delta_world = grasp_world[:3, 3] - current_end_pose[:3, 3]
                delta_in_ee = current_end_pose[:3, :3].T @ delta_world
                self.get_logger().info(
                    f'Delta current->target (world): {np.round(delta_world, 3).tolist()}'
                )
                self.get_logger().info(
                    f'Delta current->target (ee local): {np.round(delta_in_ee, 3).tolist()}'
                )

            pregrasp_world = None
            pregrasp_planning = None
            if pregrasp.shape == (4, 4):
                pregrasp_world = self._transform_pose_to_world(pregrasp, input_frame=input_frame)
                self.get_logger().info(f'Pregrasp in world: pos={pregrasp_world[:3, 3]}')
                pregrasp_planning = self._convert_pose_to_planning_frame(pregrasp_world, self._planning_frame)

            retreat_world = None
            retreat_planning = None
            if retreat.shape == (4, 4):
                retreat_world = self._transform_pose_to_world(retreat, input_frame=input_frame)
                self.get_logger().info(f'Retreat in world: pos={retreat_world[:3, 3]}')
                retreat_planning = self._convert_pose_to_planning_frame(retreat_world, self._planning_frame)

            if self._auto_align_orientation:
                if current_end_pose is None:
                    raise ValueError('Cannot align grasp orientation before receiving /piper/end_pose')
                current_planning = current_end_pose
                if pregrasp_planning is not None:
                    pregrasp_planning = self._align_pose_orientation(current_planning, pregrasp_planning)
                    self._log_pose_debug('Aligned pregrasp planning', pregrasp_planning)

                origin_for_grasp = pregrasp_planning if pregrasp_planning is not None else current_planning
                grasp_planning = self._align_pose_orientation(origin_for_grasp, grasp_planning)
                self._log_pose_debug('Aligned grasp planning', grasp_planning)

                if retreat_planning is not None:
                    retreat_planning = self._align_pose_orientation(grasp_planning, retreat_planning)
                    self._log_pose_debug('Aligned retreat planning', retreat_planning)

            if pregrasp_planning is not None:
                self._require_pose_in_workspace('Pregrasp', pregrasp_planning)
            self._require_pose_in_workspace('Grasp', grasp_planning)
            if retreat_planning is not None:
                self._require_pose_in_workspace('Retreat', retreat_planning)

            # Step 1: Open gripper
            self.get_logger().info(f'Step 1: Opening gripper to {self._gripper_open_width:.3f}')
            self._send_gripper(self._gripper_open_width)
            time.sleep(self._gripper_settle_time)

            # Step 2: Move to pregrasp
            pregrasp_joints = None
            if pregrasp_planning is not None:
                self.get_logger().info('Step 2: Moving to pregrasp')
                pregrasp_joints = self._move_to_pose(pregrasp_planning, frame_id=self._planning_frame)

            # Step 3: Move to grasp using the latest reached joint state as seed.
            self.get_logger().info('Step 3: Moving to grasp')
            grasp_joints = self._move_to_pose(grasp_planning, seed_joints=pregrasp_joints, frame_id=self._planning_frame)

            # Step 4: Close gripper
            close_width = self._resolve_close_width(gripper_width)
            self.get_logger().info(f'Step 4: Closing gripper to {close_width:.3f}')
            self._send_gripper(close_width)
            time.sleep(self._gripper_settle_time)

            # Step 5: Optional diagnostic lift straight up in planning frame.
            post_close_joints = grasp_joints
            if self._post_grasp_lift_distance > 0.0:
                lift_pose = self._offset_pose_along_planning_z(
                    grasp_planning,
                    self._post_grasp_lift_distance,
                )
                self._require_pose_in_workspace('Post-grasp lift', lift_pose)
                self.get_logger().info(
                    f'Step 5: Lifting object test by {self._post_grasp_lift_distance:.3f} m in {self._planning_frame} +Z'
                )
                self._log_pose_debug('Post-grasp lift planning', lift_pose)
                post_close_joints = self._move_to_pose(
                    lift_pose,
                    seed_joints=grasp_joints,
                    frame_id=self._planning_frame,
                )
                if self._post_grasp_lift_hold_time > 0.0:
                    self.get_logger().info(
                        f'Holding lifted pose for {self._post_grasp_lift_hold_time:.2f}s'
                    )
                    time.sleep(self._post_grasp_lift_hold_time)

            # Step 6: Retreat using the latest reached joint state as seed.
            if retreat_planning is not None:
                self.get_logger().info('Step 6: Moving to retreat')
                self._move_to_pose(
                    retreat_planning,
                    seed_joints=post_close_joints,
                    frame_id=self._planning_frame,
                )

            self.get_logger().info('Grasp execution complete!')

        except Exception as e:
            self.get_logger().error(f'Sequence error: {e}')

        finally:
            self._executing = False

    def _execute_single_pose(self, pose_msg: PoseStamped):
        """Execute single pose."""
        with self._execution_lock:
            if self._executing:
                return
            self._executing = True

        try:
            matrix = np.eye(4)
            matrix[0, 3] = pose_msg.pose.position.x
            matrix[1, 3] = pose_msg.pose.position.y
            matrix[2, 3] = pose_msg.pose.position.z
            quat = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                    pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
            matrix[:3, :3] = R.from_quat(quat).as_matrix()
            self._move_to_pose(matrix)
        finally:
            self._executing = False

    def _move_to_pose(self, pose_matrix: np.ndarray, seed_joints=None, frame_id=None):
        """Move to pose using IK.

        Args:
            pose_matrix: 4x4 transformation matrix in MoveIt planning frame
            seed_joints: Optional seed joint positions for IK solver
            frame_id: frame name for the IK target pose
        """
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('IK service unavailable')
            return

        request = GetPositionIK.Request()
        request.ik_request.group_name = self._move_group
        request.ik_request.timeout.sec = 3

        # Target pose in MoveIt's planning frame.
        request.ik_request.pose_stamped.header.frame_id = frame_id or self._planning_frame
        request.ik_request.pose_stamped.pose = self._matrix_to_pose(pose_matrix)

        joint_names, joint_positions, arm_seed_positions, seed_source = self._build_seed_joint_state(seed_joints)
        request.ik_request.robot_state.joint_state.name = joint_names
        request.ik_request.robot_state.joint_state.position = joint_positions

        # Important: disable collision checking when state is incomplete
        request.ik_request.avoid_collisions = False

        self.get_logger().info(
            f'Calling IK in frame {request.ik_request.pose_stamped.header.frame_id}: '
            f'pos=[{pose_matrix[0,3]:.3f}, {pose_matrix[1,3]:.3f}, {pose_matrix[2,3]:.3f}]'
        )
        self.get_logger().info(
            f'Seed source: {seed_source}, joint_state_count={len(joint_names)}'
        )
        self.get_logger().info(f'Seed arm joints: {[f"{p:.3f}" for p in arm_seed_positions]}')
        response = self._call_ik(request, timeout_sec=5.0)
        if response is None:
            return None

        if response.error_code.val == MoveItErrorCodes.ROBOT_STATE_INCOMPLETE:
            self.get_logger().warn(
                'IK returned ROBOT_STATE_INCOMPLETE with explicit robot_state; retrying without robot_state seed'
            )
            fallback_request = GetPositionIK.Request()
            fallback_request.ik_request.group_name = self._move_group
            fallback_request.ik_request.timeout.sec = request.ik_request.timeout.sec
            fallback_request.ik_request.pose_stamped = request.ik_request.pose_stamped
            fallback_request.ik_request.avoid_collisions = request.ik_request.avoid_collisions
            response = self._call_ik(fallback_request, timeout_sec=5.0)
            if response is None:
                return None

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'IK failed: error_code={response.error_code.val}')
            return None

        # Extract joint positions
        js = response.solution.joint_state
        name_to_pos = dict(zip(js.name, js.position))
        arm_joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_positions = [name_to_pos.get(n, 0.0) for n in arm_joints]
        self.get_logger().info(f'IK solution: {[f"{p:.3f}" for p in joint_positions]}')

        # Send arm command
        self._send_arm(joint_positions)
        reached_joints = self._wait_for_arm_target(joint_positions)
        return reached_joints if reached_joints is not None else joint_positions

    def _send_arm(self, positions: list):
        """Send arm joint command."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._arm_joints
        msg.position = positions
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Arm command sent: {[f"{p:.3f}" for p in positions]}')

    def _send_gripper(self, width: float):
        """Send gripper command."""
        width = max(0.0, min(width, 0.08))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self._gripper_joint, 'joint8']
        msg.position = [width, -width]  # joint8 is mirror of joint7
        self.arm_pub.publish(msg)
        self.get_logger().info(f'Gripper command sent: width={width:.3f}')


def main():
    parser = argparse.ArgumentParser(description='Simplified Piper grasp executor')
    parser.add_argument(
        '--workspace-bounds',
        type=str,
        default='',
        help='Planning-frame workspace bounds as [[xmin,xmax],[ymin,ymax],[zmin,zmax]]',
    )
    parser.add_argument(
        '--move-group',
        type=str,
        default='arm',
        help='MoveIt planning group used for IK requests',
    )
    parser.add_argument(
        '--gripper-open-width',
        type=float,
        default=0.08,
        help='Commanded gripper width before the approach begins',
    )
    parser.add_argument(
        '--force-close-width',
        type=float,
        default=None,
        help='If set, ignore predicted grasp width and always close to this width',
    )
    parser.add_argument(
        '--close-width-margin',
        type=float,
        default=0.0,
        help='Subtract this margin from the predicted grasp width before closing',
    )
    parser.add_argument(
        '--gripper-settle-time',
        type=float,
        default=0.5,
        help='Seconds to wait after open/close commands before continuing',
    )
    parser.add_argument(
        '--post-grasp-lift-distance',
        type=float,
        default=0.0,
        help='If > 0, lift straight up by this distance after closing before retreating',
    )
    parser.add_argument(
        '--post-grasp-lift-hold-time',
        type=float,
        default=0.0,
        help='Seconds to hold at the diagnostic post-grasp lift pose',
    )
    args = parser.parse_args()

    rclpy.init()
    node = GraspExecutorSimple(
        workspace_bounds=args.workspace_bounds,
        move_group=args.move_group,
        gripper_open_width=args.gripper_open_width,
        force_close_width=args.force_close_width,
        close_width_margin=args.close_width_margin,
        gripper_settle_time=args.gripper_settle_time,
        post_grasp_lift_distance=args.post_grasp_lift_distance,
        post_grasp_lift_hold_time=args.post_grasp_lift_hold_time,
    )
    print('GraspExecutorSimple running. Press Ctrl+C to stop.')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
