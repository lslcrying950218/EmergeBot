"""Data types for Isaac Sim communication."""
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import msgpack
import msgpack_numpy as m
import numpy as np


# Enable numpy support in msgpack
m.patch()


@dataclass
class SensorData:
    """
    Sensor data from Isaac Sim

    Contains depth map, RGB image, camera intrinsics, and optionally segmentation
    and end-effector pose metadata from the remote robot.
    """
    depth: np.ndarray  # HxW depth map in meters
    rgb: Optional[np.ndarray] = None  # HxWx3 RGB image
    K: np.ndarray = None  # 3x3 camera intrinsics matrix
    segmap: Optional[np.ndarray] = None  # HxW segmentation map
    timestamp: float = 0.0  # timestamp in seconds
    frame_id: int = 0  # frame sequence number
    end_pose: Optional[np.ndarray] = None  # 4x4 end-effector pose matrix
    end_pose_frame: str = ''  # frame_id for end_pose
    execution_busy: bool = False  # remote executor currently processing a grasp
    execution_status: str = ''  # human-readable remote execution state

    def to_bytes(self) -> bytes:
        """Serialize to bytes using msgpack"""
        data_dict = {
            'depth': self.depth,
            'rgb': self.rgb,
            'K': self.K,
            'segmap': self.segmap,
            'timestamp': self.timestamp,
            'frame_id': self.frame_id,
            'end_pose': self.end_pose,
            'end_pose_frame': self.end_pose_frame,
            'execution_busy': self.execution_busy,
            'execution_status': self.execution_status,
        }
        return msgpack.packb(data_dict, use_bin_type=True)

    @classmethod
    def from_bytes(cls, data_bytes: bytes) -> 'SensorData':
        """Deserialize from bytes"""
        data_dict = msgpack.unpackb(data_bytes, raw=False)
        return cls(
            depth=data_dict['depth'],
            rgb=data_dict.get('rgb'),
            K=data_dict.get('K'),
            segmap=data_dict.get('segmap'),
            timestamp=data_dict.get('timestamp', 0.0),
            frame_id=data_dict.get('frame_id', 0),
            end_pose=data_dict.get('end_pose'),
            end_pose_frame=data_dict.get('end_pose_frame', ''),
            execution_busy=bool(data_dict.get('execution_busy', False)),
            execution_status=data_dict.get('execution_status', ''),
        )


@dataclass
class ExecutionCommand:
    """Executable grasp command derived from the best predicted grasp."""
    pose: np.ndarray  # 4x4 grasp pose in `frame`
    pregrasp_pose: np.ndarray  # 4x4 approach pose in `frame`
    retreat_pose: np.ndarray  # 4x4 post-grasp retreat pose in `frame`
    contact_point: np.ndarray  # 3D contact point in `frame`
    approach_vector: np.ndarray  # unit vector in `frame`
    gripper_opening: float
    score: float
    segment_id: int = -1
    frame: str = "camera"
    pregrasp_offset: float = 0.0
    retreat_offset: float = 0.0

    def to_dict(self) -> Dict:
        """Serialize to a msgpack-friendly dictionary."""
        return {
            'pose': self.pose,
            'pregrasp_pose': self.pregrasp_pose,
            'retreat_pose': self.retreat_pose,
            'contact_point': self.contact_point,
            'approach_vector': self.approach_vector,
            'gripper_opening': float(self.gripper_opening),
            'score': float(self.score),
            'segment_id': int(self.segment_id),
            'frame': self.frame,
            'pregrasp_offset': float(self.pregrasp_offset),
            'retreat_offset': float(self.retreat_offset),
        }

    @classmethod
    def from_dict(cls, data_dict: Optional[Dict]) -> Optional['ExecutionCommand']:
        """Deserialize from dictionary."""
        if not data_dict:
            return None

        return cls(
            pose=np.asarray(data_dict['pose']),
            pregrasp_pose=np.asarray(data_dict['pregrasp_pose']),
            retreat_pose=np.asarray(data_dict['retreat_pose']),
            contact_point=np.asarray(data_dict['contact_point']),
            approach_vector=np.asarray(data_dict['approach_vector']),
            gripper_opening=float(data_dict.get('gripper_opening', 0.0)),
            score=float(data_dict.get('score', 0.0)),
            segment_id=int(data_dict.get('segment_id', -1)),
            frame=data_dict.get('frame', 'camera'),
            pregrasp_offset=float(data_dict.get('pregrasp_offset', 0.0)),
            retreat_offset=float(data_dict.get('retreat_offset', 0.0)),
        )


@dataclass
class GraspResult:
    """
    Grasp prediction result to send back to Isaac Sim

    Contains grasp poses, confidence scores, contact points, and gripper openings
    """
    # Grasp poses as 4x4 transformation matrices (Nx4x4)
    grasp_poses: Dict[int, np.ndarray] = field(default_factory=dict)
    # Confidence scores for each grasp (N)
    scores: Dict[int, np.ndarray] = field(default_factory=dict)
    # Contact points (Nx3)
    contact_points: Dict[int, np.ndarray] = field(default_factory=dict)
    # Gripper opening widths (N)
    gripper_openings: Dict[int, np.ndarray] = field(default_factory=dict)
    # Timestamp when the prediction was made
    timestamp: float = 0.0
    # Frame ID this result corresponds to
    frame_id: int = 0
    # Status: 'success', 'no_grasp', 'error'
    status: str = 'success'
    # Message for debugging
    message: str = ''
    # Optional executable command for the best grasp
    execution: Optional[ExecutionCommand] = None

    def to_bytes(self) -> bytes:
        """Serialize to bytes using msgpack"""
        data_dict = {
            'grasp_poses': self.grasp_poses,
            'scores': self.scores,
            'contact_points': self.contact_points,
            'gripper_openings': self.gripper_openings,
            'timestamp': self.timestamp,
            'frame_id': self.frame_id,
            'status': self.status,
            'message': self.message,
            'execution': self.execution.to_dict() if self.execution else None,
        }
        return msgpack.packb(data_dict, use_bin_type=True)

    @classmethod
    def from_bytes(cls, data_bytes: bytes) -> 'GraspResult':
        """Deserialize from bytes"""
        data_dict = msgpack.unpackb(data_bytes, raw=False)
        return cls(
            grasp_poses=data_dict.get('grasp_poses', {}),
            scores=data_dict.get('scores', {}),
            contact_points=data_dict.get('contact_points', {}),
            gripper_openings=data_dict.get('gripper_openings', {}),
            timestamp=data_dict.get('timestamp', 0.0),
            frame_id=data_dict.get('frame_id', 0),
            status=data_dict.get('status', 'success'),
            message=data_dict.get('message', ''),
            execution=ExecutionCommand.from_dict(data_dict.get('execution')),
        )

    def add_segment(self, seg_id: int, poses: np.ndarray, scores: np.ndarray,
                    contact_pts: np.ndarray, openings: np.ndarray):
        """Add grasps for a specific segment"""
        self.grasp_poses[seg_id] = poses
        self.scores[seg_id] = scores
        self.contact_points[seg_id] = contact_pts
        self.gripper_openings[seg_id] = openings

    def get_best_grasp(self, seg_id: int = -1) -> Optional[Dict]:
        """Get the best grasp (highest score) for a segment"""
        if seg_id not in self.scores or len(self.scores[seg_id]) == 0:
            return None

        best_idx = np.argmax(self.scores[seg_id])
        return {
            'pose': self.grasp_poses[seg_id][best_idx],
            'score': self.scores[seg_id][best_idx],
            'contact_point': self.contact_points[seg_id][best_idx],
            'gripper_opening': self.gripper_openings[seg_id][best_idx]
        }

    def get_top_k_grasps(self, k: int = 5, seg_id: int = -1) -> List[Dict]:
        """Get top k grasps for a segment"""
        if seg_id not in self.scores or len(self.scores[seg_id]) == 0:
            return []

        top_indices = np.argsort(self.scores[seg_id])[::-1][:k]
        return [
            {
                'pose': self.grasp_poses[seg_id][idx],
                'score': self.scores[seg_id][idx],
                'contact_point': self.contact_points[seg_id][idx],
                'gripper_opening': self.gripper_openings[seg_id][idx]
            }
            for idx in top_indices
        ]
