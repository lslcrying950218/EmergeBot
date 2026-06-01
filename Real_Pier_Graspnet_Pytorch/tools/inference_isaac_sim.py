#!/usr/bin/env python3
"""
Contact-GraspNet Isaac Sim Interface

This script demonstrates the complete pipeline:
1. Connect to remote forwarder
2. Receive sensor data (depth + RGB + camera info)
3. Predict grasps using Contact-GraspNet
4. Send results back to Isaac Sim for execution
5. Visualize grasp results (optional)

Usage:
    # Single frame inference with visualization
    python3 inference_isaac_sim.py --remote-ip 192.168.100.12 --visualize

    # Continuous inference
    python3 inference_isaac_sim.py --remote-ip 192.168.100.12 --continuous

    # No visualization (faster)
    python3 inference_isaac_sim.py --remote-ip 192.168.100.12 --no-visualize
"""

import argparse
import math
import os
import time
import warnings
import numpy as np

import torch
import open3d as o3d

from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch import config_utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO
try:
    from piper_local_client import ExecutionCommand, IsaacSimClient
except ImportError:
    from contact_graspnet_pytorch.comm import ExecutionCommand, IsaacSimClient
from contact_graspnet_pytorch.segmentation import build_segmenter
from contact_graspnet_pytorch.target_selection import build_target_selector

DEFAULT_SELF_EXCLUSION_RECTS = [
    [0.38, 0.74, 0.62, 1.00],  # gripper foreground at bottom center
    [0.72, 0.55, 1.00, 1.00],  # robot body / support at right foreground
]
DEFAULT_SELF_EXCLUSION_OVERLAP_RATIO = 0.15
DEFAULT_SEGMENT_COMPLETION_MIN_AREA = 300
DEFAULT_SEGMENT_COMPLETION_DEPTH_DELTA_M = 0.035
DEFAULT_SEGMENT_COMPLETION_VERTICAL_GAP_PX = 28
DEFAULT_SEGMENT_COMPLETION_HORIZONTAL_GAP_PX = 20
DEFAULT_SEGMENT_COMPLETION_OVERLAP_RATIO = 0.55
DEFAULT_SEGMENT_COMPLETION_MAX_AREA_RATIO = 3.0
DEFAULT_SEGMENT_COMPLETION_MAX_SPAN_RATIO = 2.0
DEFAULT_SEGMENT_COMPLETION_MAX_UNION_BBOX_FRACTION = 0.10
DEFAULT_EXECUTION_RERANK_TOP_K = 32
DEFAULT_TARGET_REFINEMENT_SCORE_MARGIN = 0.02
DEFAULT_TARGET_REFINEMENT_MAX_CENTROID_SHIFT_PX = 42.0
DEFAULT_TARGET_REFINEMENT_MAX_DEPTH_DELTA_M = 0.06
DEFAULT_TARGET_REFINEMENT_MAX_AREA_RATIO = 2.5
DEFAULT_TARGET_REFINEMENT_MIN_IOU = 0.10
DEFAULT_TARGET_REFINEMENT_MIN_AXIS_OVERLAP = 0.45
DEFAULT_TARGET_REFINEMENT_MIN_SELECTION_MARGIN = 0.008
DEFAULT_TARGET_REFINEMENT_MIN_QUERY_ADVANTAGE = 0.000
DEFAULT_TARGET_REFINEMENT_MIN_AGREEMENT = 0.22
DEFAULT_EXECUTION_TEMPORAL_POSITION_DELTA_M = 0.02
DEFAULT_EXECUTION_TEMPORAL_ANGLE_DELTA_DEG = 12.0
DEFAULT_EXECUTION_TEMPORAL_FIT_MARGIN = 0.03
DEFAULT_EXECUTION_TEMPORAL_MAX_POSITION_DELTA_M = 0.08
DEFAULT_EXECUTION_TEMPORAL_MAX_ANGLE_DELTA_DEG = 25.0
DEFAULT_EXECUTION_CACHE_REPLACE_FIT_MARGIN = 0.04
DEFAULT_PLANNING_APPROACH_VERTICAL_BLEND = 0.75
DEFAULT_PLANNING_APPROACH_MAX_LATERAL_RATIO = 0.30
DEFAULT_PLANNING_APPROACH_MIN_VERTICAL_COMPONENT = 0.85
DEFAULT_GRIPPER_DEPTH_M = 0.1034
DEFAULT_EXECUTION_MIN_LINE_FRACTION = 0.02
DEFAULT_EXECUTION_MIN_FINGER_SPAN_PX = 2.0
DEFAULT_EXECUTION_MIN_ROW_SPAN_RATIO = 0.04
DEFAULT_EXECUTION_MIN_SPAN_RATIO_FOR_NARROW_OPENING = 0.18
DEFAULT_EXECUTION_MIN_CENTER_T = 0.0
DEFAULT_EXECUTION_MAX_CENTER_T = 1.0
DEFAULT_EXECUTION_MIN_TARGET_Y_FRACTION = 0.0
DEFAULT_EXECUTION_MAX_APPROACH_Z = -0.55
DEFAULT_EXECUTION_CACHED_MAX_APPROACH_Z = -0.45
DEFAULT_EXECUTION_MIN_PREGRASP_BACKOFF_M = 0.003
DEFAULT_EXECUTION_GLOBAL_FALLBACK_MIN_SPAN_PRIORITY = 0.30
DEFAULT_EXECUTION_GLOBAL_FALLBACK_MIN_SPAN_IMPROVEMENT = 0.03
DEFAULT_EXECUTION_GLOBAL_FALLBACK_MIN_FIT_IMPROVEMENT = 0.01
DEFAULT_DEPTH_PREPROCESS_ENABLED = True
DEFAULT_DEPTH_HOLE_FILL_ITERS = 2
DEFAULT_DEPTH_HOLE_FILL_MIN_VALID_NEIGHBORS = 5
DEFAULT_DEPTH_HOLE_FILL_MAX_DELTA_M = 0.04
DEFAULT_DEPTH_OUTLIER_THRESHOLD_M = 0.03
DEFAULT_DEPTH_OUTLIER_RELATIVE_THRESHOLD = 0.04
DEFAULT_TARGET_DEPTH_CLEANING_ENABLED = True
DEFAULT_TARGET_DEPTH_CORE_ERODE_ITERS = 2
DEFAULT_TARGET_DEPTH_MIN_CORE_PIXELS = 64
DEFAULT_TARGET_DEPTH_SEED_PERCENTILE = 15.0
DEFAULT_TARGET_DEPTH_FRONT_TOLERANCE_M = 0.04
DEFAULT_TARGET_DEPTH_BACK_TOLERANCE_M = 0.12
DEFAULT_TARGET_DEPTH_MIN_COMPONENT_PIXELS = 96
DEFAULT_TARGET_DEPTH_COMPONENT_DILATE_ITERS = 1


def parse_workspace_bounds(bounds_str: str):
    """Parse workspace bounds string into a (3, 2) array."""
    if not bounds_str:
        return None

    bounds = np.asarray(eval(bounds_str), dtype=np.float32)
    if bounds.shape != (3, 2):
        raise ValueError('workspace bounds must have shape [[xmin,xmax],[ymin,ymax],[zmin,zmax]]')
    return bounds


def parse_vector3(vector_str: str):
    """Parse a 3-vector string into a float32 numpy array."""
    if not vector_str:
        return None

    vector = np.asarray(eval(vector_str), dtype=np.float32)
    if vector.shape != (3,):
        raise ValueError('vector must have shape [x,y,z]')
    return vector


def parse_quaternion(quat_str: str):
    """Parse a quaternion string into a float32 numpy array."""
    if not quat_str:
        return None

    quaternion = np.asarray(eval(quat_str), dtype=np.float32)
    if quaternion.shape != (4,):
        raise ValueError('quaternion must have shape [x,y,z,w]')
    return quaternion


def normalize_prediction_dict(predictions):
    """Ensure predictions are stored as {segment_id: ndarray}."""
    if isinstance(predictions, dict):
        return predictions
    return {-1: predictions}


def count_predicted_grasps(scores_dict):
    """Count total grasps across segments."""
    total = 0
    for seg_scores in scores_dict.values():
        total += len(np.atleast_1d(seg_scores))
    return total


def get_segment_point_count(pc_segments, segment_id):
    """Return the point count for one selected segment id."""
    if segment_id is None or segment_id == 0:
        return None
    if segment_id not in pc_segments:
        return 0
    return int(np.asarray(pc_segments[segment_id]).shape[0])


def resolve_preferred_prediction_segment(scores_dict, preferred_segment_id):
    """Only keep a preferred segment id if the prediction result actually contains that key."""
    if preferred_segment_id is None:
        return None
    if preferred_segment_id in scores_dict:
        return preferred_segment_id
    return None


def build_normalized_rect_mask(image_hw, normalized_rects):
    """Build a boolean mask from normalized [x0,y0,x1,y1] rectangles."""
    if not normalized_rects:
        return None
    height, width = int(image_hw[0]), int(image_hw[1])
    mask = np.zeros((height, width), dtype=bool)
    for rect in normalized_rects:
        x0, y0, x1, y1 = [float(v) for v in rect]
        px0 = max(0, min(width, int(np.floor(x0 * width))))
        py0 = max(0, min(height, int(np.floor(y0 * height))))
        px1 = max(0, min(width, int(np.ceil(x1 * width))))
        py1 = max(0, min(height, int(np.ceil(y1 * height))))
        if px1 <= px0 or py1 <= py0:
            continue
        mask[py0:py1, px0:px1] = True
    return mask


def compute_excluded_segment_ids(segmap, exclusion_mask, overlap_ratio_threshold=0.15):
    """Exclude segments that substantially overlap a known self-mask region."""
    if segmap is None or exclusion_mask is None:
        return set()
    segmap = np.asarray(segmap, dtype=np.int32)
    exclusion_mask = np.asarray(exclusion_mask, dtype=bool)
    excluded = set()
    for seg_id in np.unique(segmap):
        seg_id = int(seg_id)
        if seg_id <= 0:
            continue
        seg_mask = segmap == seg_id
        seg_area = int(seg_mask.sum())
        if seg_area <= 0:
            continue
        overlap = int(np.logical_and(seg_mask, exclusion_mask).sum())
        if (overlap / float(seg_area)) >= float(overlap_ratio_threshold):
            excluded.add(seg_id)
    return excluded


def compute_candidate_segment_ids(segmap, excluded_segment_ids):
    """Return foreground segment ids excluding known self-mask hits."""
    if segmap is None:
        return None
    return [
        int(seg_id) for seg_id in np.unique(np.asarray(segmap, dtype=np.int32))
        if int(seg_id) > 0 and int(seg_id) not in set(excluded_segment_ids or ())
    ]


def _compute_segment_geometry(segmap, seg_id, depth):
    """Compute area/bbox/median-depth stats for one segment id."""
    segmap = np.asarray(segmap, dtype=np.int32)
    mask = np.asarray(segmap == int(seg_id), dtype=bool)
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return None

    depth_vals = np.asarray(depth, dtype=np.float32)[mask]
    depth_vals = depth_vals[np.isfinite(depth_vals) & (depth_vals > 0)]
    median_depth = float(np.median(depth_vals)) if depth_vals.size > 0 else None
    x0, x1 = int(xs.min()), int(xs.max()) + 1
    y0, y1 = int(ys.min()), int(ys.max()) + 1
    return {
        'segment_id': int(seg_id),
        'area': int(mask.sum()),
        'bbox_xyxy': np.asarray([x0, y0, x1, y1], dtype=np.int32),
        'median_depth': median_depth,
        'image_area': int(segmap.shape[0] * segmap.shape[1]),
    }


def _segments_should_merge(stats_a, stats_b):
    """Return whether two segments likely belong to one fragmented instance."""
    if stats_a is None or stats_b is None:
        return False
    if stats_a['median_depth'] is None or stats_b['median_depth'] is None:
        return False
    if abs(float(stats_a['median_depth']) - float(stats_b['median_depth'])) > DEFAULT_SEGMENT_COMPLETION_DEPTH_DELTA_M:
        return False

    ax0, ay0, ax1, ay1 = [int(v) for v in stats_a['bbox_xyxy']]
    bx0, by0, bx1, by1 = [int(v) for v in stats_b['bbox_xyxy']]
    aw, ah = max(1, ax1 - ax0), max(1, ay1 - ay0)
    bw, bh = max(1, bx1 - bx0), max(1, by1 - by0)
    area_a = max(1, int(stats_a['area']))
    area_b = max(1, int(stats_b['area']))

    area_ratio = max(area_a / float(area_b), area_b / float(area_a))
    if area_ratio > DEFAULT_SEGMENT_COMPLETION_MAX_AREA_RATIO:
        return False

    x_overlap = max(0, min(ax1, bx1) - max(ax0, bx0))
    y_overlap = max(0, min(ay1, by1) - max(ay0, by0))
    x_overlap_ratio = x_overlap / float(max(1, min(aw, bw)))
    y_overlap_ratio = y_overlap / float(max(1, min(ah, bh)))

    vertical_gap = max(0, max(ay0, by0) - min(ay1, by1))
    horizontal_gap = max(0, max(ax0, bx0) - min(ax1, bx1))

    union_x0, union_y0 = min(ax0, bx0), min(ay0, by0)
    union_x1, union_y1 = max(ax1, bx1), max(ay1, by1)
    union_area = max(1, union_x1 - union_x0) * max(1, union_y1 - union_y0)
    image_area = max(1, int(stats_a.get('image_area', 1)))
    if (union_area / float(image_area)) > DEFAULT_SEGMENT_COMPLETION_MAX_UNION_BBOX_FRACTION:
        return False

    vertical_fragment_match = (
        x_overlap_ratio >= DEFAULT_SEGMENT_COMPLETION_OVERLAP_RATIO
        and vertical_gap <= DEFAULT_SEGMENT_COMPLETION_VERTICAL_GAP_PX
        and max(aw / float(bw), bw / float(aw)) <= DEFAULT_SEGMENT_COMPLETION_MAX_SPAN_RATIO
    )
    horizontal_fragment_match = (
        y_overlap_ratio >= DEFAULT_SEGMENT_COMPLETION_OVERLAP_RATIO
        and horizontal_gap <= DEFAULT_SEGMENT_COMPLETION_HORIZONTAL_GAP_PX
        and max(ah / float(bh), bh / float(ah)) <= DEFAULT_SEGMENT_COMPLETION_MAX_SPAN_RATIO
    )
    return bool(vertical_fragment_match or horizontal_fragment_match)


def complete_fragmented_segments(segmap, depth):
    """
    Merge obviously fragmented neighboring segments into more complete instances.

    This is category-agnostic: it only uses spatial continuity and depth
    consistency, so it can help when SAM splits one object into upper/lower or
    left/right pieces.
    """
    if segmap is None:
        return None

    segmap = np.asarray(segmap, dtype=np.int32)
    segment_ids = [int(seg_id) for seg_id in np.unique(segmap) if int(seg_id) > 0]
    if len(segment_ids) < 2:
        return segmap

    stats_by_id = {}
    for seg_id in segment_ids:
        stats = _compute_segment_geometry(segmap, seg_id, depth)
        if stats is None or stats['area'] < DEFAULT_SEGMENT_COMPLETION_MIN_AREA:
            continue
        stats_by_id[seg_id] = stats

    if len(stats_by_id) < 2:
        return segmap

    parent = {seg_id: seg_id for seg_id in stats_by_id.keys()}

    def find(seg_id):
        root = seg_id
        while parent[root] != root:
            root = parent[root]
        while parent[seg_id] != seg_id:
            next_seg = parent[seg_id]
            parent[seg_id] = root
            seg_id = next_seg
        return root

    def union(seg_a, seg_b):
        root_a = find(seg_a)
        root_b = find(seg_b)
        if root_a == root_b:
            return
        if root_a < root_b:
            parent[root_b] = root_a
        else:
            parent[root_a] = root_b

    stats_items = list(stats_by_id.items())
    for idx, (seg_a, stats_a) in enumerate(stats_items):
        for seg_b, stats_b in stats_items[idx + 1:]:
            if _segments_should_merge(stats_a, stats_b):
                union(seg_a, seg_b)

    merged_segmap = segmap.copy()
    changed = False
    for seg_id in stats_by_id.keys():
        root_id = find(seg_id)
        if root_id != seg_id:
            merged_segmap[segmap == seg_id] = root_id
            changed = True

    return merged_segmap if changed else segmap


def _bbox_overlap_ratios(bbox_a, bbox_b):
    """Return x/y overlap ratios and IoU for two xyxy boxes."""
    ax0, ay0, ax1, ay1 = [int(v) for v in bbox_a]
    bx0, by0, bx1, by1 = [int(v) for v in bbox_b]
    inter_x0 = max(ax0, bx0)
    inter_y0 = max(ay0, by0)
    inter_x1 = min(ax1, bx1)
    inter_y1 = min(ay1, by1)
    inter_w = max(0, inter_x1 - inter_x0)
    inter_h = max(0, inter_y1 - inter_y0)
    inter_area = inter_w * inter_h
    area_a = max(1, (ax1 - ax0) * (ay1 - ay0))
    area_b = max(1, (bx1 - bx0) * (by1 - by0))
    union_area = max(1, area_a + area_b - inter_area)
    x_overlap_ratio = inter_w / float(max(1, min(ax1 - ax0, bx1 - bx0)))
    y_overlap_ratio = inter_h / float(max(1, min(ay1 - ay0, by1 - by0)))
    iou = inter_area / float(union_area)
    return float(x_overlap_ratio), float(y_overlap_ratio), float(iou)


def collect_instance_companion_segments(segmap, depth, seed_segment_id, candidate_segment_ids=None):
    """
    Collect neighboring segments that likely belong to the same physical instance.

    This specifically helps when SAM splits one object into a textured patch and
    a larger body region. The grouping stays conservative: only nearby segments
    with similar depth and strong box overlap are merged into the selected seed.
    """
    if segmap is None or seed_segment_id is None or int(seed_segment_id) <= 0:
        return [int(seed_segment_id)] if seed_segment_id is not None else []

    segmap = np.asarray(segmap, dtype=np.int32)
    seed_segment_id = int(seed_segment_id)
    seed_stats = _compute_segment_geometry(segmap, seed_segment_id, depth)
    if seed_stats is None:
        return [seed_segment_id]
    if seed_stats['median_depth'] is None:
        return [seed_segment_id]

    if candidate_segment_ids is None:
        candidate_ids = [int(seg_id) for seg_id in np.unique(segmap) if int(seg_id) > 0]
    else:
        candidate_ids = [int(seg_id) for seg_id in candidate_segment_ids if int(seg_id) > 0]

    cluster = {seed_segment_id}
    for seg_id in candidate_ids:
        if seg_id == seed_segment_id:
            continue
        stats = _compute_segment_geometry(segmap, seg_id, depth)
        if stats is None:
            continue
        if stats['median_depth'] is None:
            continue
        depth_delta = abs(float(stats['median_depth']) - float(seed_stats['median_depth']))
        if depth_delta > 0.05:
            continue

        x_overlap_ratio, y_overlap_ratio, iou = _bbox_overlap_ratios(
            seed_stats['bbox_xyxy'],
            stats['bbox_xyxy'],
        )
        area_ratio = max(
            float(seed_stats['area']) / float(max(1, stats['area'])),
            float(stats['area']) / float(max(1, seed_stats['area'])),
        )

        strongly_nested = (
            x_overlap_ratio >= 0.75 and y_overlap_ratio >= 0.45 and area_ratio <= 8.0
        )
        overlapping_body = (
            iou >= 0.10 and x_overlap_ratio >= 0.45 and y_overlap_ratio >= 0.45 and area_ratio <= 6.0
        )
        if strongly_nested or overlapping_body:
            cluster.add(seg_id)

    return sorted(cluster)


def merge_selected_instance_segments(segmap, seed_segment_id, companion_segment_ids):
    """Merge one selected instance cluster back into the seed segment id."""
    if segmap is None or seed_segment_id is None:
        return segmap
    segmap = np.asarray(segmap, dtype=np.int32)
    merged = segmap.copy()
    seed_segment_id = int(seed_segment_id)
    for seg_id in companion_segment_ids:
        seg_id = int(seg_id)
        if seg_id > 0 and seg_id != seed_segment_id:
            merged[segmap == seg_id] = seed_segment_id
    return merged


def _segment_centroid_pose(segment_points):
    """Build a simple 4x4 pose located at the centroid of one segment point cloud."""
    segment_points = np.asarray(segment_points, dtype=np.float32)
    if segment_points.size == 0:
        return None
    centroid = np.median(segment_points[:, :3], axis=0).astype(np.float32)
    pose = np.eye(4, dtype=np.float32)
    pose[:3, 3] = centroid
    return pose


def filter_semantically_consistent_candidates(scores_by_segment,
                                              best_segment_id,
                                              score_margin=DEFAULT_TARGET_REFINEMENT_SCORE_MARGIN,
                                              segmap=None,
                                              depth=None):
    """Keep only high-scoring candidates that still look like the same instance."""
    if not scores_by_segment:
        return {}
    scores_by_segment = {
        int(seg_id): float(score)
        for seg_id, score in scores_by_segment.items()
    }
    best_segment_id = int(best_segment_id)
    if best_segment_id not in scores_by_segment:
        return scores_by_segment

    best_score = float(scores_by_segment[best_segment_id])
    min_allowed_score = best_score - float(score_margin)
    filtered = {
        seg_id: score
        for seg_id, score in scores_by_segment.items()
        if float(score) >= min_allowed_score
    }
    if not filtered:
        return {best_segment_id: best_score}
    if segmap is None or depth is None:
        return filtered

    best_signature = compute_target_signature(segmap, best_segment_id, depth)
    best_stats = _compute_segment_geometry(segmap, best_segment_id, depth)
    if best_signature is None or best_stats is None:
        return {best_segment_id: best_score}

    same_instance = {}
    best_bbox = best_stats['bbox_xyxy']
    best_area = max(1, int(best_stats['area']))

    for seg_id, score in filtered.items():
        if seg_id == best_segment_id:
            same_instance[seg_id] = score
            continue

        candidate_signature = compute_target_signature(segmap, seg_id, depth)
        candidate_stats = _compute_segment_geometry(segmap, seg_id, depth)
        if candidate_signature is None or candidate_stats is None:
            continue
        if not target_signatures_match(
            best_signature,
            candidate_signature,
            centroid_tol_px=DEFAULT_TARGET_REFINEMENT_MAX_CENTROID_SHIFT_PX,
            depth_tol_m=DEFAULT_TARGET_REFINEMENT_MAX_DEPTH_DELTA_M,
            max_area_ratio_change=DEFAULT_TARGET_REFINEMENT_MAX_AREA_RATIO,
        ):
            continue

        x_overlap_ratio, y_overlap_ratio, iou = _bbox_overlap_ratios(
            best_bbox,
            candidate_stats['bbox_xyxy'],
        )
        area_ratio = max(
            best_area / float(max(1, int(candidate_stats['area']))),
            float(max(1, int(candidate_stats['area']))) / float(best_area),
        )
        if iou < DEFAULT_TARGET_REFINEMENT_MIN_IOU:
            if (
                x_overlap_ratio < DEFAULT_TARGET_REFINEMENT_MIN_AXIS_OVERLAP
                or y_overlap_ratio < DEFAULT_TARGET_REFINEMENT_MIN_AXIS_OVERLAP
            ):
                continue
        if area_ratio > DEFAULT_TARGET_REFINEMENT_MAX_AREA_RATIO:
            continue
        same_instance[seg_id] = score

    return same_instance or {best_segment_id: best_score}


def choose_reachability_preferred_target_segment(client,
                                                 sensor_data,
                                                 target_selection,
                                                 pc_segments,
                                                 segmap_aligned,
                                                 target_min_points=0,
                                                 execution_reference_frame='world',
                                                 execution_base_position_world=None,
                                                 execution_base_yaw_deg=0.0,
                                                 execution_base_workspace_bounds=None,
                                                 execution_camera_translation_gripper=None,
                                                 execution_camera_quaternion_gripper=None,
                                                 end_pose_is_camera_pose: bool = False):
    """Refine text-guided target selection using local reachability and segment geometry."""
    if target_selection is None or not target_selection.scores_by_segment:
        return None
    if sensor_data is None or sensor_data.end_pose is None:
        return None

    candidate_scores = filter_semantically_consistent_candidates(
        target_selection.scores_by_segment,
        target_selection.segment_id,
        segmap=segmap_aligned,
        depth=sensor_data.depth,
    )
    candidates = []
    signatures = collect_segment_signatures(segmap_aligned, sensor_data.depth)
    reachability_enabled = (
        execution_base_workspace_bounds is not None
        and execution_base_position_world is not None
    )

    for seg_id, selector_score in candidate_scores.items():
        seg_id = int(seg_id)
        segment_points = pc_segments.get(seg_id)
        if segment_points is None:
            continue
        segment_points = np.asarray(segment_points, dtype=np.float32)
        point_count = int(segment_points.shape[0])
        if point_count < int(target_min_points):
            continue

        centroid_pose = _segment_centroid_pose(segment_points)
        if centroid_pose is None:
            continue

        if reachability_enabled:
            planning_pose = client._transform_pose_to_planning_frame(
                centroid_pose,
                'camera_optical_frame',
                end_pose_world=sensor_data.end_pose,
                planning_frame=execution_reference_frame,
                reference_frame=sensor_data.end_pose_frame or execution_reference_frame,
                camera_translation_gripper=execution_camera_translation_gripper,
                camera_quaternion_gripper=execution_camera_quaternion_gripper,
                end_pose_is_camera_pose=end_pose_is_camera_pose,
            )
            reachable = client._pose_in_base_workspace(
                planning_pose,
                base_position_world=execution_base_position_world,
                base_yaw_deg=execution_base_yaw_deg,
                base_workspace_bounds=execution_base_workspace_bounds,
            )
        else:
            reachable = True

        signature = signatures.get(seg_id) or {}
        median_depth = signature.get('median_depth')
        if median_depth is None:
            median_depth = float(np.median(segment_points[:, 2]))

        candidates.append({
            'segment_id': seg_id,
            'selector_score': float(selector_score),
            'point_count': point_count,
            'median_depth': float(median_depth),
            'reachable': bool(reachable),
        })

    if not candidates:
        return None

    candidates.sort(
        key=lambda item: (
            1 if item['reachable'] else 0,
            item['selector_score'],
            item['point_count'],
            -item['median_depth'],
        ),
        reverse=True,
    )
    return candidates[0]


def should_refine_target_with_reachability(use_segmap, explicit_segmap_id, target_query):
    """Only refine text-guided target selection when predicting segmented objects."""
    return bool(use_segmap and not explicit_segmap_id and target_query)


def get_target_selection_debug_info(target_selector):
    """Return diagnostics for the last text-guided target selection attempt."""
    if target_selector is None or not hasattr(target_selector, 'get_last_selection_debug'):
        return {}
    info = target_selector.get_last_selection_debug()
    return info if isinstance(info, dict) else {}


def print_target_selection_debug_info(debug_info, prefix='  '):
    """Print one compact summary for the latest target-selection attempt."""
    if not debug_info:
        return
    reason = str(debug_info.get('reason', 'unknown'))
    best_segment_id = debug_info.get('best_segment_id')
    if best_segment_id is None:
        print(f"{prefix}Target selection diagnostics: reason={reason}")
        return
    print(
        f"{prefix}Target selection diagnostics: "
        f"reason={reason}, seg={int(best_segment_id)}, "
        f"score={float(debug_info.get('best_score', 0.0)):.3f}, "
        f"masked={float(debug_info.get('masked_score', 0.0)):.3f}, "
        f"context={float(debug_info.get('context_score', 0.0)):.3f}, "
        f"distractor={float(debug_info.get('distractor_score', 0.0)):.3f}, "
        f"agreement={float(debug_info.get('agreement_score', 0.0)):.3f}, "
        f"advantage={float(debug_info.get('query_advantage', 0.0)):.3f}, "
        f"masked_adv={float(debug_info.get('masked_query_advantage', 0.0)):.3f}, "
        f"context_bias={float(debug_info.get('context_bias', 0.0)):.3f}, "
        f"margin={float(debug_info.get('selection_margin', 0.0)):.3f}, "
        f"raw_margin={float(debug_info.get('raw_selection_margin', 0.0)):.3f}, "
        f"competing_seg={debug_info.get('competing_segment_id')}"
    )


def target_selection_is_semantically_confident(target_selection):
    """Gate target refinement on semantic confidence, not just grasp quality."""
    if target_selection is None:
        return False
    if float(getattr(target_selection, 'selection_margin', 0.0)) < DEFAULT_TARGET_REFINEMENT_MIN_SELECTION_MARGIN:
        return False
    if float(getattr(target_selection, 'query_advantage', 0.0)) < DEFAULT_TARGET_REFINEMENT_MIN_QUERY_ADVANTAGE:
        return False
    if float(getattr(target_selection, 'agreement_score', 0.0)) < DEFAULT_TARGET_REFINEMENT_MIN_AGREEMENT:
        return False
    return True


def choose_execution_preferred_target_segment(client,
                                              sensor_data,
                                              target_selection,
                                              pc_segments,
                                              segmap_aligned,
                                              pred_grasps_dict,
                                              scores_dict,
                                              contact_pts_dict,
                                              gripper_openings_dict,
                                              min_grasp_score=0.0,
                                              execution_frame='camera_optical_frame',
                                              pregrasp_offset=0.10,
                                              retreat_offset=0.10,
                                              max_pregrasp_offset=None,
                                              max_retreat_offset=None,
                                              target_min_points=0,
                                              execution_top_k=None,
                                              execution_reference_frame='world',
                                              execution_base_position_world=None,
                                              execution_base_yaw_deg=0.0,
                                              execution_base_workspace_bounds=None,
                                              execution_camera_translation_gripper=None,
                                              execution_camera_quaternion_gripper=None,
                                              end_pose_is_camera_pose: bool = False):
    """Choose the target segment that has the best locally executable grasp candidate."""
    if target_selection is None or not target_selection.scores_by_segment:
        return None
    if sensor_data is None or sensor_data.end_pose is None:
        return None

    candidate_scores = filter_semantically_consistent_candidates(
        target_selection.scores_by_segment,
        target_selection.segment_id,
        segmap=segmap_aligned,
        depth=sensor_data.depth,
    )
    signatures = collect_segment_signatures(segmap_aligned, sensor_data.depth)
    candidates = []

    for seg_id, selector_score in candidate_scores.items():
        seg_id = int(seg_id)
        segment_points = pc_segments.get(seg_id)
        if segment_points is None:
            continue
        segment_points = np.asarray(segment_points, dtype=np.float32)
        point_count = int(segment_points.shape[0])
        if point_count < int(target_min_points):
            continue

        target_mask = np.asarray(segmap_aligned == seg_id, dtype=bool)
        execution, execution_meta = select_best_execution_candidate(
            client,
            pred_grasps_dict,
            scores_dict,
            contact_pts_dict,
            gripper_openings_dict,
            min_score=min_grasp_score,
            frame=execution_frame,
            pregrasp_offset=pregrasp_offset,
            retreat_offset=retreat_offset,
            max_pregrasp_offset=max_pregrasp_offset,
            max_retreat_offset=max_retreat_offset,
            workspace_bounds=None,
            top_k=execution_top_k,
            preferred_seg_id=seg_id,
            end_pose_world=sensor_data.end_pose,
            planning_frame=execution_reference_frame,
            reference_frame=sensor_data.end_pose_frame or execution_reference_frame,
            base_position_world=execution_base_position_world,
            base_yaw_deg=execution_base_yaw_deg,
            base_workspace_bounds=execution_base_workspace_bounds,
            camera_translation_gripper=execution_camera_translation_gripper,
            camera_quaternion_gripper=execution_camera_quaternion_gripper,
            sensor_data=sensor_data,
            target_mask=target_mask,
            target_segment_points=segment_points,
            end_pose_is_camera_pose=end_pose_is_camera_pose,
        )

        signature = signatures.get(seg_id) or {}
        median_depth = signature.get('median_depth')
        if median_depth is None:
            median_depth = float(np.median(segment_points[:, 2]))

        candidates.append({
            'segment_id': seg_id,
            'selector_score': float(selector_score),
            'point_count': point_count,
            'median_depth': float(median_depth),
            'executable': execution is not None,
            'execution_score': None if execution is None else float(execution.score),
            'fit_score': 0.0 if execution_meta is None else float(execution_meta.get('fit_score', 0.0)),
        })

    if not candidates:
        return None

    executable_candidates = [item for item in candidates if item['executable']]
    if not executable_candidates:
        return None

    executable_candidates.sort(
        key=lambda item: (
            item['fit_score'],
            item['execution_score'],
            item['selector_score'],
            item['point_count'],
            -item['median_depth'],
        ),
        reverse=True,
    )
    return executable_candidates[0]


def compute_contact_to_segment_distance(pc_segments, segment_id, contact_point):
    """Return nearest-neighbor distance from one contact point to a target segment point cloud."""
    if segment_id is None or segment_id == 0:
        return None
    if segment_id not in pc_segments:
        return None

    segment_points = np.asarray(pc_segments[segment_id], dtype=np.float32)
    if segment_points.size == 0:
        return None

    contact_point = np.asarray(contact_point, dtype=np.float32).reshape(1, 3)
    dists = np.linalg.norm(segment_points[:, :3] - contact_point, axis=1)
    if dists.size == 0:
        return None
    return float(np.min(dists))


def compute_contact_to_points_distance(segment_points, contact_point):
    """Return nearest-neighbor distance from one contact point to a point cloud array."""
    if segment_points is None:
        return None
    segment_points = np.asarray(segment_points, dtype=np.float32)
    if segment_points.size == 0:
        return None
    contact_point = np.asarray(contact_point, dtype=np.float32).reshape(1, 3)
    dists = np.linalg.norm(segment_points[:, :3] - contact_point, axis=1)
    if dists.size == 0:
        return None
    return float(np.min(dists))


def planning_pose_to_base_position(client,
                                   planning_pose,
                                   base_position_world,
                                   base_yaw_deg):
    """Convert one planning/world pose to base-frame xyz for logging."""
    planning_pose = client._normalize_matrix4(planning_pose, 'planning_pose')
    base_position = client._normalize_vector3(base_position_world, 'base_position_world')
    base_rotation = client._rotation_matrix_z(base_yaw_deg)
    rel_world = planning_pose[:3, 3] - base_position
    rel_base = base_rotation.T @ rel_world
    return np.asarray(rel_base, dtype=np.float32)


def get_camera_to_optical_rotation(client):
    """Return the Camera->Optical rotation matrix from whichever client constant is available."""
    if hasattr(client, '_DEFAULT_CAMERA_TO_OPTICAL_QUATERNION'):
        return client._quaternion_to_matrix(client._DEFAULT_CAMERA_TO_OPTICAL_QUATERNION)
    if hasattr(client, '_OPTICAL_TO_CAMERA_MATRIX'):
        optical_to_camera = np.asarray(client._OPTICAL_TO_CAMERA_MATRIX, dtype=np.float32)
        return np.linalg.inv(optical_to_camera).astype(np.float32)
    raise AttributeError('IsaacSimClient is missing camera/optical transform constants')


def describe_pose_coordinate_chain(client,
                                   pose_optical,
                                   end_pose_world,
                                   base_position_world,
                                   base_yaw_deg,
                                   camera_translation_gripper,
                                   camera_quaternion_gripper,
                                   end_pose_is_camera_pose: bool = False):
    """Return one pose translation expressed across optical/camera/gripper/world/base.

    Args:
        end_pose_is_camera_pose: If True, end_pose_world is camera pose in world frame,
                                 not gripper pose. Skip gripper->camera transform.
    """
    pose_optical = client._normalize_matrix4(pose_optical, 'pose_optical')
    end_pose_world = client._normalize_matrix4(end_pose_world, 'end_pose_world')

    t_cam_to_optical = np.eye(4, dtype=np.float32)
    t_cam_to_optical[:3, :3] = get_camera_to_optical_rotation(client)

    pose_camera = np.linalg.inv(t_cam_to_optical) @ pose_optical

    if end_pose_is_camera_pose:
        # end_pose is camera pose, direct transform
        pose_world = end_pose_world @ pose_camera
        # For debug output, gripper = camera when end_pose is camera pose
        pose_gripper = pose_camera.copy()
    else:
        # Traditional: end_pose is gripper pose
        camera_translation = client._normalize_vector3(
            camera_translation_gripper if camera_translation_gripper is not None else client._DEFAULT_CAMERA_TRANSLATION_GRIPPER,
            'camera_translation_gripper',
        )
        camera_quaternion = np.asarray(
            camera_quaternion_gripper if camera_quaternion_gripper is not None else client._DEFAULT_CAMERA_QUATERNION_GRIPPER,
            dtype=np.float32,
        )
        t_gripper_to_cam = np.eye(4, dtype=np.float32)
        t_gripper_to_cam[:3, :3] = client._quaternion_to_matrix(camera_quaternion)
        t_gripper_to_cam[:3, 3] = camera_translation
        pose_gripper = np.linalg.inv(t_gripper_to_cam) @ pose_camera
        pose_world = end_pose_world @ pose_gripper

    pose_base_xyz = planning_pose_to_base_position(
        client,
        pose_world,
        base_position_world=base_position_world,
        base_yaw_deg=base_yaw_deg,
    )
    return {
        'optical': np.asarray(pose_optical[:3, 3], dtype=np.float32),
        'camera': np.asarray(pose_camera[:3, 3], dtype=np.float32),
        'gripper': np.asarray(pose_gripper[:3, 3], dtype=np.float32),
        'world': np.asarray(pose_world[:3, 3], dtype=np.float32),
        'base': np.asarray(pose_base_xyz, dtype=np.float32),
    }


def log_base_workspace_debug(client,
                             candidates,
                             end_pose_world,
                             planning_frame,
                             reference_frame,
                             base_position_world,
                             base_yaw_deg,
                             camera_translation_gripper,
                             camera_quaternion_gripper,
                             pregrasp_offset,
                             retreat_offset,
                             max_pregrasp_offset,
                             max_retreat_offset,
                             frame,
                             max_items=3,
                             end_pose_is_camera_pose: bool = False):
    """Print base-frame grasp/pregrasp/retreat positions for the top few candidates."""
    if not candidates or end_pose_world is None or base_position_world is None:
        return
    print("  Base workspace debug (top candidates):")
    for idx, candidate in enumerate(candidates[:max(1, int(max_items))], start=1):
        try:
            execution = client.build_execution_command(
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
            grasp_chain = describe_pose_coordinate_chain(
                client,
                execution.pose,
                end_pose_world=end_pose_world,
                base_position_world=base_position_world,
                base_yaw_deg=base_yaw_deg,
                camera_translation_gripper=camera_translation_gripper,
                camera_quaternion_gripper=camera_quaternion_gripper,
                end_pose_is_camera_pose=end_pose_is_camera_pose,
            )
            pregrasp_chain = describe_pose_coordinate_chain(
                client,
                execution.pregrasp_pose,
                end_pose_world=end_pose_world,
                base_position_world=base_position_world,
                base_yaw_deg=base_yaw_deg,
                camera_translation_gripper=camera_translation_gripper,
                camera_quaternion_gripper=camera_quaternion_gripper,
                end_pose_is_camera_pose=end_pose_is_camera_pose,
            )
            retreat_chain = describe_pose_coordinate_chain(
                client,
                execution.retreat_pose,
                end_pose_world=end_pose_world,
                base_position_world=base_position_world,
                base_yaw_deg=base_yaw_deg,
                camera_translation_gripper=camera_translation_gripper,
                camera_quaternion_gripper=camera_quaternion_gripper,
                end_pose_is_camera_pose=end_pose_is_camera_pose,
            )
            print(
                f"    #{idx} score={float(candidate['score']):.3f}: "
                f"grasp_base={np.round(grasp_chain['base'], 4).tolist()}, "
                f"pregrasp_base={np.round(pregrasp_chain['base'], 4).tolist()}, "
                f"retreat_base={np.round(retreat_chain['base'], 4).tolist()}"
            )
            print(
                f"      grasp chain: optical={np.round(grasp_chain['optical'], 4).tolist()}, "
                f"camera={np.round(grasp_chain['camera'], 4).tolist()}, "
                f"gripper={np.round(grasp_chain['gripper'], 4).tolist()}, "
                f"world={np.round(grasp_chain['world'], 4).tolist()}, "
                f"base={np.round(grasp_chain['base'], 4).tolist()}"
            )
            print(
                f"      pregrasp chain: optical={np.round(pregrasp_chain['optical'], 4).tolist()}, "
                f"camera={np.round(pregrasp_chain['camera'], 4).tolist()}, "
                f"gripper={np.round(pregrasp_chain['gripper'], 4).tolist()}, "
                f"world={np.round(pregrasp_chain['world'], 4).tolist()}, "
                f"base={np.round(pregrasp_chain['base'], 4).tolist()}"
            )
            print(
                f"      retreat chain: optical={np.round(retreat_chain['optical'], 4).tolist()}, "
                f"camera={np.round(retreat_chain['camera'], 4).tolist()}, "
                f"gripper={np.round(retreat_chain['gripper'], 4).tolist()}, "
                f"world={np.round(retreat_chain['world'], 4).tolist()}, "
                f"base={np.round(retreat_chain['base'], 4).tolist()}"
            )
        except Exception as exc:
            print(f"    #{idx} base-debug failed: {exc}")


def compute_mask_boundary_points(mask):
    """Return boundary pixel coordinates (x, y) for a boolean mask."""
    mask = np.asarray(mask, dtype=bool)
    if mask.size == 0 or not np.any(mask):
        return np.empty((0, 2), dtype=np.int32)
    edge = mask.copy()
    edge[1:, :] &= mask[:-1, :]
    edge[:-1, :] &= mask[1:, :]
    edge[:, 1:] &= mask[:, :-1]
    edge[:, :-1] &= mask[:, 1:]
    boundary = mask & (~edge)
    ys, xs = np.nonzero(boundary)
    if ys.size == 0:
        ys, xs = np.nonzero(mask)
    return np.stack([xs, ys], axis=1).astype(np.int32)


def sample_mask_fraction_along_line(mask, start_xy, end_xy, num_samples=25):
    """Sample how much of a 2D line segment lies inside a binary mask."""
    if start_xy is None or end_xy is None:
        return 0.0
    mask = np.asarray(mask, dtype=bool)
    x0, y0 = [float(v) for v in start_xy]
    x1, y1 = [float(v) for v in end_xy]
    xs = np.linspace(x0, x1, max(2, int(num_samples)))
    ys = np.linspace(y0, y1, max(2, int(num_samples)))
    xi = np.clip(np.round(xs).astype(np.int32), 0, mask.shape[1] - 1)
    yi = np.clip(np.round(ys).astype(np.int32), 0, mask.shape[0] - 1)
    return float(np.mean(mask[yi, xi]))


def compute_center_clearance_score(center_uv, boundary_points, target_area):
    """Prefer gripper centers that sit deeper inside the target mask."""
    if center_uv is None or boundary_points is None or len(boundary_points) == 0:
        return 0.0
    center_xy = np.asarray(center_uv, dtype=np.float32).reshape(1, 2)
    boundary_xy = np.asarray(boundary_points, dtype=np.float32)
    min_dist = float(np.min(np.linalg.norm(boundary_xy - center_xy, axis=1)))
    scale = max(4.0, math.sqrt(max(1.0, float(target_area))) * 0.25)
    return float(np.clip(min_dist / scale, 0.0, 1.0))


def compute_mask_centroid_score(center_uv, target_mask):
    """Prefer image-space gripper centers that stay near the target mask centroid."""
    if center_uv is None or target_mask is None:
        return 0.0
    target_mask = np.asarray(target_mask, dtype=bool)
    ys, xs = np.nonzero(target_mask)
    if ys.size == 0:
        return 0.0
    centroid_xy = np.array([float(xs.mean()), float(ys.mean())], dtype=np.float32)
    center_xy = np.asarray(center_uv, dtype=np.float32)
    bbox_w = max(1.0, float(xs.max() - xs.min() + 1))
    bbox_h = max(1.0, float(ys.max() - ys.min() + 1))
    scale = max(8.0, 0.35 * math.sqrt(bbox_w * bbox_w + bbox_h * bbox_h))
    dist = float(np.linalg.norm(center_xy - centroid_xy))
    return float(np.clip(1.0 - dist / scale, 0.0, 1.0))


def compute_3d_balance_score(center_xyz, target_segment_points):
    """Prefer gripper centers that stay near the geometric middle of the target point cloud."""
    if center_xyz is None or target_segment_points is None:
        return 0.0
    target_segment_points = np.asarray(target_segment_points, dtype=np.float32)
    if target_segment_points.ndim != 2 or target_segment_points.shape[0] == 0:
        return 0.0
    points_xyz = target_segment_points[:, :3]
    centroid_xyz = np.median(points_xyz, axis=0).astype(np.float32)
    radial = np.linalg.norm(points_xyz - centroid_xyz, axis=1)
    scale = float(np.percentile(radial, 75)) if radial.size > 0 else 0.0
    scale = max(0.02, scale)
    dist = float(np.linalg.norm(np.asarray(center_xyz, dtype=np.float32) - centroid_xyz))
    return float(np.clip(1.0 - dist / scale, 0.0, 1.0))


def compute_mask_midband_score(center_uv, target_mask):
    """Prefer centers that stay away from the top/bottom extremes of the target mask."""
    if center_uv is None or target_mask is None:
        return 0.0
    target_mask = np.asarray(target_mask, dtype=bool)
    ys, xs = np.nonzero(target_mask)
    if ys.size == 0:
        return 0.0
    y_min = float(ys.min())
    y_max = float(ys.max())
    span = max(1.0, y_max - y_min)
    center_y = float(np.asarray(center_uv, dtype=np.float32)[1])
    normalized_y = np.clip((center_y - y_min) / span, 0.0, 1.0)
    dist_from_mid = abs(normalized_y - 0.5)
    return float(np.clip(1.0 - dist_from_mid / 0.35, 0.0, 1.0))


def compute_mask_top_avoidance_score(center_uv, target_mask):
    """Only mildly penalize grasps extremely close to the target's top edge."""
    if center_uv is None or target_mask is None:
        return 0.0
    target_mask = np.asarray(target_mask, dtype=bool)
    ys, _ = np.nonzero(target_mask)
    if ys.size == 0:
        return 0.0
    y_min = float(ys.min())
    y_max = float(ys.max())
    span = max(1.0, y_max - y_min)
    center_y = float(np.asarray(center_uv, dtype=np.float32)[1])
    normalized_y = np.clip((center_y - y_min) / span, 0.0, 1.0)
    # Height itself is not the issue; only the extreme top edge gets a mild penalty.
    return float(np.clip((normalized_y - 0.06) / 0.14, 0.0, 1.0))


def compute_local_mask_row_span_score(center_uv, finger_a_uv, finger_b_uv, target_mask):
    """Prefer longer finger spans on locally wider image-space target cross-sections."""
    local_width, finger_span, span_ratio = compute_local_mask_row_span_metrics(
        center_uv,
        finger_a_uv,
        finger_b_uv,
        target_mask,
    )
    if local_width <= 1e-6 or finger_span <= 1e-6:
        return 0.0

    span_fill = float(np.clip((span_ratio - 0.35) / 0.40, 0.0, 1.0))
    width_preference = float(np.clip((local_width - 18.0) / 18.0, 0.0, 1.0))

    return float(0.60 * span_fill + 0.40 * width_preference)


def compute_local_horizontal_cover_score(center_uv, finger_a_uv, finger_b_uv, target_mask):
    """Measure how much of the local target cross-section is covered horizontally."""
    local_width, finger_span, span_ratio = compute_local_mask_row_span_metrics(
        center_uv,
        finger_a_uv,
        finger_b_uv,
        target_mask,
    )
    if local_width <= 1e-6 or finger_span <= 1e-6:
        return 0.0

    dx = abs(float(finger_b_uv[0]) - float(finger_a_uv[0]))
    dy = abs(float(finger_b_uv[1]) - float(finger_a_uv[1]))
    horizontal_cover = float(np.clip(dx / max(local_width, 1e-6), 0.0, 1.0))
    orientation_score = float(np.clip(dx / max(dx + 1.8 * dy, 1e-6), 0.0, 1.0))
    row_score = float(np.clip((span_ratio - 0.34) / 0.26, 0.0, 1.0))
    return float(
        0.45 * horizontal_cover +
        0.30 * orientation_score +
        0.25 * row_score
    )


def compute_local_mask_row_span_metrics(center_uv, finger_a_uv, finger_b_uv, target_mask):
    """Return local row width, finger span, and span ratio around one grasp center."""
    if center_uv is None or finger_a_uv is None or finger_b_uv is None or target_mask is None:
        return 0.0, 0.0, 0.0
    target_mask = np.asarray(target_mask, dtype=bool)
    if target_mask.ndim != 2:
        return 0.0, 0.0, 0.0

    h, w = target_mask.shape
    center_y = int(np.clip(int(center_uv[1]), 0, h - 1))
    y0 = max(0, center_y - 2)
    y1 = min(h, center_y + 3)
    band = target_mask[y0:y1]
    ys, xs = np.nonzero(band)
    if xs.size < 8:
        return 0.0, 0.0, 0.0

    local_width = float(xs.max() - xs.min() + 1)
    finger_span = float(np.linalg.norm(
        np.asarray(finger_a_uv, dtype=np.float32) - np.asarray(finger_b_uv, dtype=np.float32)
    ))
    if local_width <= 1e-6 or finger_span <= 1e-6:
        return local_width, finger_span, 0.0

    span_ratio = finger_span / max(local_width, 1e-6)
    return local_width, finger_span, span_ratio


def compute_local_cross_section_support_score(execution, target_segment_points):
    """Prefer grasps whose fingers actually span a locally supported 3D cross-section."""
    if execution is None or target_segment_points is None:
        return 0.0
    if str(getattr(execution, 'frame', '')) != 'camera_optical_frame':
        return 0.0

    points = np.asarray(target_segment_points, dtype=np.float32)
    if points.ndim != 2 or points.shape[0] < 24:
        return 0.0
    points_xyz = points[:, :3]

    pose = np.asarray(execution.pose, dtype=np.float32)
    if pose.shape != (4, 4):
        return 0.0

    closure_axis = pose[:3, 0].astype(np.float32)
    lateral_axis = pose[:3, 1].astype(np.float32)
    approach_axis = pose[:3, 2].astype(np.float32)
    opening = max(0.0, float(getattr(execution, 'gripper_opening', 0.0)))
    if opening <= 1e-4:
        return 0.0

    closure_norm = float(np.linalg.norm(closure_axis))
    lateral_norm = float(np.linalg.norm(lateral_axis))
    approach_norm = float(np.linalg.norm(approach_axis))
    if closure_norm <= 1e-8 or lateral_norm <= 1e-8 or approach_norm <= 1e-8:
        return 0.0
    closure_axis /= closure_norm
    lateral_axis /= lateral_norm
    approach_axis /= approach_norm

    contact_point = getattr(execution, 'contact_point', None)
    if contact_point is None:
        return 0.0
    contact_point = np.asarray(contact_point, dtype=np.float32).reshape(3)
    center_xyz = contact_point + 0.5 * opening * closure_axis

    rel = points_xyz - center_xyz.reshape(1, 3)
    closure_coord = rel @ closure_axis
    lateral_coord = rel @ lateral_axis
    approach_coord = rel @ approach_axis

    slice_lateral_tol = max(0.015, 0.5 * opening)
    slice_approach_tol = 0.03
    slice_mask = (
        np.abs(lateral_coord) <= slice_lateral_tol
        ) & (
        np.abs(approach_coord) <= slice_approach_tol
    )
    if int(np.count_nonzero(slice_mask)) < 16:
        return 0.0

    slice_closure = closure_coord[slice_mask]
    pos_side = slice_closure[slice_closure > 0.004]
    neg_side = slice_closure[slice_closure < -0.004]
    if pos_side.size < 4 or neg_side.size < 4:
        return 0.0

    pos_extent = float(np.percentile(pos_side, 75))
    neg_extent = float(np.percentile(-neg_side, 75))
    observed_width = pos_extent + neg_extent
    if observed_width <= 1e-4:
        return 0.0

    side_balance = float(min(pos_extent, neg_extent) / max(pos_extent, neg_extent, 1e-6))
    width_ratio = observed_width / max(opening, 1e-6)
    width_match = float(np.clip(1.0 - abs(width_ratio - 1.0) / 0.6, 0.0, 1.0))
    span_reach = float(np.clip(min(pos_extent, neg_extent) / max(0.25 * opening, 1e-6), 0.0, 1.0))
    width_preference = float(np.clip((observed_width - 0.035) / 0.03, 0.0, 1.0))

    return float(
        0.28 * side_balance +
        0.24 * width_match +
        0.28 * span_reach +
        0.20 * width_preference
    )


def compute_gripper_opening_preference_score(execution):
    """Softly prefer larger gripper openings without rejecting narrow but valid cross-sections."""
    if execution is None:
        return 0.0
    opening = max(0.0, float(getattr(execution, 'gripper_opening', 0.0)))
    if opening <= 1e-6:
        return 0.0
    return float(np.clip((opening - 0.018) / 0.035, 0.0, 1.0))


def compute_approach_preference_score(execution):
    """Softly prefer grasps whose approach direction has enough downward component."""
    if execution is None:
        return 0.0
    approach = np.asarray(getattr(execution, 'approach_vector', None), dtype=np.float32)
    if approach.shape != (3,):
        pose = np.asarray(getattr(execution, 'pose', None), dtype=np.float32)
        if pose.shape != (4, 4):
            return 0.0
        approach = pose[:3, 2].astype(np.float32)
    norm = float(np.linalg.norm(approach))
    if norm <= 1e-8:
        return 0.0
    approach = approach / norm
    approach_z = float(approach[2])
    # 0 around very flat approaches (~-0.20), 1.0 for clearly descending approaches (~-0.70).
    return float(np.clip((-approach_z - 0.20) / 0.50, 0.0, 1.0))


def compute_upper_grasp_quality_score(center_uv, finger_a_uv, finger_b_uv, target_mask):
    """Reward grasps with enough span and horizontal coverage; only mildly punish the very top edge."""
    target_mask = np.asarray(target_mask, dtype=bool)
    ys, _ = np.nonzero(target_mask)
    if ys.size == 0:
        return 0.0

    y_min = float(ys.min())
    y_max = float(ys.max())
    target_height = max(1.0, y_max - y_min)
    normalized_y = float(np.clip((float(center_uv[1]) - y_min) / target_height, 0.0, 1.0))
    _, finger_span, span_ratio = compute_local_mask_row_span_metrics(
        center_uv,
        finger_a_uv,
        finger_b_uv,
        target_mask,
    )
    dx = abs(float(finger_b_uv[0]) - float(finger_a_uv[0]))
    dy = abs(float(finger_b_uv[1]) - float(finger_a_uv[1]))
    orientation_score = float(np.clip(dx / max(dx + 1.4 * dy, 1e-6), 0.0, 1.0))
    span_score = float(np.clip((finger_span - 10.0) / 18.0, 0.0, 1.0))
    row_score = float(np.clip((span_ratio - 0.34) / 0.30, 0.0, 1.0))
    horizontal_reach = float(np.clip((dx - 10.0) / 18.0, 0.0, 1.0))
    quality = (
        0.30 * orientation_score +
        0.25 * horizontal_reach +
        0.25 * row_score +
        0.20 * span_score
    )
    top_edge_progress = float(np.clip((0.18 - normalized_y) / 0.18, 0.0, 1.0))
    # Near the extreme top edge, short point-like grasps should lose rank, but high grasps
    # with real span should still survive.
    return float(np.clip((1.0 - top_edge_progress) * quality + top_edge_progress * min(quality, max(horizontal_reach, row_score)), 0.0, 1.0))


def compute_upper_horizontal_span_score(center_uv, finger_a_uv, finger_b_uv, target_mask):
    """Prefer true horizontal span and sufficient width, regardless of height."""
    target_mask = np.asarray(target_mask, dtype=bool)
    ys, _ = np.nonzero(target_mask)
    if ys.size == 0:
        return 0.0

    y_min = float(ys.min())
    y_max = float(ys.max())
    target_height = max(1.0, y_max - y_min)
    normalized_y = float(np.clip((float(center_uv[1]) - y_min) / target_height, 0.0, 1.0))
    _, finger_span, span_ratio = compute_local_mask_row_span_metrics(
        center_uv,
        finger_a_uv,
        finger_b_uv,
        target_mask,
    )
    dx = abs(float(finger_b_uv[0]) - float(finger_a_uv[0]))
    dy = abs(float(finger_b_uv[1]) - float(finger_a_uv[1]))
    orientation_score = float(np.clip(dx / max(dx + 1.8 * dy, 1e-6), 0.0, 1.0))
    horizontal_reach = float(np.clip((dx - 12.0) / 16.0, 0.0, 1.0))
    span_score = float(np.clip((finger_span - 12.0) / 16.0, 0.0, 1.0))
    row_score = float(np.clip((span_ratio - 0.34) / 0.28, 0.0, 1.0))
    base_score = (
        0.35 * orientation_score +
        0.30 * horizontal_reach +
        0.20 * row_score +
        0.15 * span_score
    )
    top_edge_progress = float(np.clip((0.16 - normalized_y) / 0.16, 0.0, 1.0))
    top_edge_score = min(base_score, max(horizontal_reach, row_score))
    return float(np.clip((1.0 - top_edge_progress) * base_score + top_edge_progress * top_edge_score, 0.0, 1.0))


def compute_span_validity_score(execution, center_uv, finger_a_uv, finger_b_uv, target_mask, target_segment_points):
    """Return a multiplicative quality gate based on whether the grasp truly spans the object."""
    local_width, finger_span, span_ratio = compute_local_mask_row_span_metrics(
        center_uv,
        finger_a_uv,
        finger_b_uv,
        target_mask,
    )
    if local_width <= 1e-6 or finger_span <= 1e-6:
        return 0.0

    dx = abs(float(finger_b_uv[0]) - float(finger_a_uv[0]))
    dy = abs(float(finger_b_uv[1]) - float(finger_a_uv[1]))
    orientation_score = float(np.clip(dx / max(dx + 1.8 * dy, 1e-6), 0.0, 1.0))
    horizontal_reach = float(np.clip((dx - 12.0) / 16.0, 0.0, 1.0))
    span_score = float(np.clip((finger_span - 10.0) / 16.0, 0.0, 1.0))
    row_score = float(np.clip((span_ratio - 0.34) / 0.26, 0.0, 1.0))
    opening_score = compute_gripper_opening_preference_score(execution)
    approach_score = compute_approach_preference_score(execution)
    cross_section_support = compute_local_cross_section_support_score(execution, target_segment_points)
    horizontal_cover = compute_local_horizontal_cover_score(center_uv, finger_a_uv, finger_b_uv, target_mask)

    return float(
        np.clip(
            0.24 * horizontal_cover +
            0.16 * orientation_score +
            0.16 * horizontal_reach +
            0.13 * span_score +
            0.11 * row_score +
            0.08 * opening_score +
            0.12 * approach_score +
            0.05 * cross_section_support,
            0.0,
            1.0,
        )
    )


def evaluate_execution_geometry_constraints(execution_preview,
                                            sensor_data,
                                            target_mask,
                                            max_approach_z=None,
                                            enforce_approach=True):
    """Apply hard 2D/3D sanity checks before one grasp is allowed to execute."""
    if execution_preview is None:
        return False, {'reason': 'missing_preview'}
    if sensor_data is None or target_mask is None or sensor_data.K is None:
        return False, {'reason': 'missing_projection_context'}
    if str(getattr(execution_preview, 'frame', '')) != 'camera_optical_frame':
        return False, {'reason': 'non_camera_frame'}

    debug_geom = compute_gripper_debug_geometry(execution_preview)
    if debug_geom is None:
        return False, {'reason': 'missing_geometry'}

    target_mask = np.asarray(target_mask, dtype=bool)
    image_hw = target_mask.shape[:2]
    center_uv = project_camera_point_to_pixel(debug_geom['center'], sensor_data.K, image_hw)
    finger_a_uv = project_camera_point_to_pixel(debug_geom['finger_a'], sensor_data.K, image_hw)
    finger_b_uv = project_camera_point_to_pixel(debug_geom['finger_b'], sensor_data.K, image_hw)
    if center_uv is None or finger_a_uv is None or finger_b_uv is None:
        return False, {
            'reason': 'projection_oob',
            'center_uv': center_uv,
            'finger_a_uv': finger_a_uv,
            'finger_b_uv': finger_b_uv,
        }

    center_inside = point_on_mask(target_mask, center_uv, radius=5)
    finger_a_on_target = point_on_mask(target_mask, finger_a_uv, radius=5)
    finger_b_on_target = point_on_mask(target_mask, finger_b_uv, radius=5)
    finger_contact_count = int(bool(finger_a_on_target)) + int(bool(finger_b_on_target))
    line_fraction = sample_line_mask_fraction(target_mask, finger_a_uv, finger_b_uv)
    local_width, finger_span, span_ratio = compute_local_mask_row_span_metrics(
        center_uv,
        finger_a_uv,
        finger_b_uv,
        target_mask,
    )

    ab = np.asarray(finger_b_uv, dtype=np.float32) - np.asarray(finger_a_uv, dtype=np.float32)
    ac = np.asarray(center_uv, dtype=np.float32) - np.asarray(finger_a_uv, dtype=np.float32)
    ab_norm_sq = float(np.dot(ab, ab))
    center_t = float(np.dot(ac, ab) / ab_norm_sq) if ab_norm_sq > 1e-6 else 0.5

    ys, _ = np.nonzero(target_mask)
    if ys.size == 0:
        return False, {'reason': 'empty_target_mask'}
    y_min = float(ys.min())
    y_max = float(ys.max())
    target_height = max(1.0, y_max - y_min)
    normalized_y = float(np.clip((float(center_uv[1]) - y_min) / target_height, 0.0, 1.0))

    approach = np.asarray(getattr(execution_preview, 'approach_vector', None), dtype=np.float32)
    if approach.shape != (3,):
        pose = np.asarray(execution_preview.pose, dtype=np.float32)
        approach = pose[:3, 2].astype(np.float32)
    approach_norm = float(np.linalg.norm(approach))
    if approach_norm <= 1e-8:
        return False, {'reason': 'invalid_approach'}
    approach /= approach_norm
    approach_z = float(approach[2])

    pregrasp_backoff = 0.0
    if getattr(execution_preview, 'pregrasp_pose', None) is not None:
        pregrasp_delta = (
            np.asarray(execution_preview.pregrasp_pose, dtype=np.float32)[:3, 3]
            - np.asarray(execution_preview.pose, dtype=np.float32)[:3, 3]
        )
        pregrasp_backoff = float(-np.dot(pregrasp_delta, approach))
    required_backoff = max(
        DEFAULT_EXECUTION_MIN_PREGRASP_BACKOFF_M,
        0.20 * float(getattr(execution_preview, 'pregrasp_offset', 0.0) or 0.0),
    )
    if max_approach_z is None:
        max_approach_z = DEFAULT_EXECUTION_MAX_APPROACH_Z

    reason = 'ok'
    valid = True
    if not center_inside:
        reason = 'center_off_target'
        valid = False
    elif finger_contact_count <= 0:
        reason = 'finger_off_target'
        valid = False
    elif finger_contact_count == 1 and line_fraction < max(DEFAULT_EXECUTION_MIN_LINE_FRACTION, 0.10):
        reason = 'finger_contact_weak'
        valid = False
    elif line_fraction < DEFAULT_EXECUTION_MIN_LINE_FRACTION:
        reason = 'line_fraction_low'
        valid = False
    elif (
        finger_span < DEFAULT_EXECUTION_MIN_FINGER_SPAN_PX
        and span_ratio < DEFAULT_EXECUTION_MIN_SPAN_RATIO_FOR_NARROW_OPENING
    ):
        reason = 'finger_span_short'
        valid = False
    elif span_ratio < DEFAULT_EXECUTION_MIN_ROW_SPAN_RATIO:
        reason = 'row_span_short'
        valid = False
    elif center_t < DEFAULT_EXECUTION_MIN_CENTER_T or center_t > DEFAULT_EXECUTION_MAX_CENTER_T:
        reason = 'center_not_between_fingers'
        valid = False
    elif enforce_approach and approach_z > float(max_approach_z):
        reason = 'approach_too_flat'
        valid = False
    elif pregrasp_backoff < required_backoff:
        reason = 'pregrasp_backoff_invalid'
        valid = False

    return valid, {
        'reason': reason,
        'center_uv': center_uv,
        'finger_a_uv': finger_a_uv,
        'finger_b_uv': finger_b_uv,
        'center_inside': center_inside,
        'finger_a_on_target': finger_a_on_target,
        'finger_b_on_target': finger_b_on_target,
        'finger_contact_count': finger_contact_count,
        'line_fraction': line_fraction,
        'finger_span': finger_span,
        'local_width': local_width,
        'row_span_ratio': span_ratio,
        'center_t': center_t,
        'normalized_y': normalized_y,
        'approach_z': approach_z,
        'pregrasp_backoff': pregrasp_backoff,
        'required_backoff': required_backoff,
    }


def summarize_execution_candidate_geometry(execution, sensor_data, target_mask):
    """Summarize one execution candidate using the same image-space geometry used for filtering."""
    if execution is None or sensor_data is None or target_mask is None or sensor_data.K is None:
        return None
    if str(getattr(execution, 'frame', '')) != 'camera_optical_frame':
        return None

    debug_geom = compute_gripper_debug_geometry(execution)
    if debug_geom is None:
        return None

    image_hw = np.asarray(target_mask).shape[:2]
    center_uv = project_camera_point_to_pixel(debug_geom['center'], sensor_data.K, image_hw)
    finger_a_uv = project_camera_point_to_pixel(debug_geom['finger_a'], sensor_data.K, image_hw)
    finger_b_uv = project_camera_point_to_pixel(debug_geom['finger_b'], sensor_data.K, image_hw)
    if center_uv is None or finger_a_uv is None or finger_b_uv is None:
        return None

    target_mask = np.asarray(target_mask, dtype=bool)
    local_width, finger_span, row_span_ratio = compute_local_mask_row_span_metrics(
        center_uv,
        finger_a_uv,
        finger_b_uv,
        target_mask,
    )
    line_fraction = sample_mask_fraction_along_line(target_mask, finger_a_uv, finger_b_uv, num_samples=31)
    center_inside = point_on_mask(target_mask, center_uv, radius=5)
    finger_a_on_target = point_on_mask(target_mask, finger_a_uv, radius=5)
    finger_b_on_target = point_on_mask(target_mask, finger_b_uv, radius=5)
    dx = abs(float(finger_b_uv[0]) - float(finger_a_uv[0]))
    dy = abs(float(finger_b_uv[1]) - float(finger_a_uv[1]))
    horizontal_cover = 0.0 if local_width <= 1e-6 else float(np.clip(dx / max(local_width, 1e-6), 0.0, 1.0))
    orientation_score = float(np.clip(dx / max(dx + 1.8 * dy, 1e-6), 0.0, 1.0))

    ab = np.asarray(finger_b_uv, dtype=np.float32) - np.asarray(finger_a_uv, dtype=np.float32)
    ac = np.asarray(center_uv, dtype=np.float32) - np.asarray(finger_a_uv, dtype=np.float32)
    ab_norm_sq = float(np.dot(ab, ab))
    center_t = float(np.dot(ac, ab) / ab_norm_sq) if ab_norm_sq > 1e-6 else 0.5

    ys, _ = np.nonzero(target_mask)
    if ys.size > 0:
        y_min = float(ys.min())
        y_max = float(ys.max())
        target_height = max(1.0, y_max - y_min)
        normalized_y = float(np.clip((float(center_uv[1]) - y_min) / target_height, 0.0, 1.0))
    else:
        normalized_y = 0.5

    approach_score = compute_approach_preference_score(execution)
    approach = np.asarray(getattr(execution, 'approach_vector', None), dtype=np.float32)
    if approach.shape != (3,):
        pose = np.asarray(getattr(execution, 'pose', None), dtype=np.float32)
        if pose.shape == (4, 4):
            approach = pose[:3, 2].astype(np.float32)
    approach_norm = float(np.linalg.norm(approach)) if approach.shape == (3,) else 0.0
    approach_z = 0.0 if approach_norm <= 1e-8 else float(approach[2] / approach_norm)

    return {
        'center_uv': center_uv,
        'finger_a_uv': finger_a_uv,
        'finger_b_uv': finger_b_uv,
        'dx': float(dx),
        'dy': float(dy),
        'local_width': float(local_width),
        'finger_span': float(finger_span),
        'row_span_ratio': float(row_span_ratio),
        'horizontal_cover': float(horizontal_cover),
        'orientation_score': float(orientation_score),
        'line_fraction': float(line_fraction),
        'center_t': float(center_t),
        'normalized_y': float(normalized_y),
        'center_inside': bool(center_inside),
        'finger_a_on_target': bool(finger_a_on_target),
        'finger_b_on_target': bool(finger_b_on_target),
        'finger_contact_count': int(
            int(bool(finger_a_on_target)) + int(bool(finger_b_on_target))
        ),
        'opening': float(getattr(execution, 'gripper_opening', 0.0) or 0.0),
        'approach_z': float(approach_z),
        'approach_score': float(approach_score),
    }


def score_execution_object_fit(execution,
                               sensor_data,
                               target_mask,
                               boundary_points=None,
                               target_segment_points=None):
    """
    Score how well a grasp spans the selected target in image space.

    Generic preference:
    - the finger-to-finger line should pass through object interior
    - the gripper center should be away from the mask boundary
    - the gripper center should stay near the target's geometric middle
    """
    if execution is None or sensor_data is None or target_mask is None:
        return 0.0
    if str(execution.frame) != 'camera_optical_frame':
        return 0.0

    debug_geom = compute_gripper_debug_geometry(execution)
    if debug_geom is None:
        return 0.0

    image_hw = np.asarray(target_mask).shape[:2]
    center_uv = project_camera_point_to_pixel(debug_geom['center'], sensor_data.K, image_hw)
    finger_a_uv = project_camera_point_to_pixel(debug_geom['finger_a'], sensor_data.K, image_hw)
    finger_b_uv = project_camera_point_to_pixel(debug_geom['finger_b'], sensor_data.K, image_hw)
    if center_uv is None or finger_a_uv is None or finger_b_uv is None:
        return 0.0

    target_mask = np.asarray(target_mask, dtype=bool)
    line_fraction = sample_mask_fraction_along_line(target_mask, finger_a_uv, finger_b_uv, num_samples=31)
    center_inside = 1.0 if target_mask[int(center_uv[1]), int(center_uv[0])] else 0.0
    if boundary_points is None:
        boundary_points = compute_mask_boundary_points(target_mask)
    center_clearance = compute_center_clearance_score(center_uv, boundary_points, int(target_mask.sum()))
    centroid_proximity = compute_mask_centroid_score(center_uv, target_mask)
    balance_3d = compute_3d_balance_score(debug_geom['center'], target_segment_points)
    midband_score = compute_mask_midband_score(center_uv, target_mask)
    top_avoidance = compute_mask_top_avoidance_score(center_uv, target_mask)
    row_span_score = compute_local_mask_row_span_score(center_uv, finger_a_uv, finger_b_uv, target_mask)
    horizontal_cover_score = compute_local_horizontal_cover_score(center_uv, finger_a_uv, finger_b_uv, target_mask)
    cross_section_support = compute_local_cross_section_support_score(execution, target_segment_points)
    opening_preference = compute_gripper_opening_preference_score(execution)
    approach_preference = compute_approach_preference_score(execution)
    upper_grasp_quality = compute_upper_grasp_quality_score(center_uv, finger_a_uv, finger_b_uv, target_mask)
    upper_horizontal_span = compute_upper_horizontal_span_score(center_uv, finger_a_uv, finger_b_uv, target_mask)
    span_validity = compute_span_validity_score(
        execution,
        center_uv,
        finger_a_uv,
        finger_b_uv,
        target_mask,
        target_segment_points,
    )
    additive_score = float(
        0.05 * line_fraction +
        0.05 * center_clearance +
        0.03 * center_inside +
        0.10 * centroid_proximity +
        0.09 * balance_3d +
        0.04 * midband_score +
        0.01 * top_avoidance +
        0.18 * row_span_score +
        0.16 * horizontal_cover_score +
        0.14 * cross_section_support +
        0.07 * opening_preference +
        0.16 * approach_preference +
        0.08 * upper_grasp_quality +
        0.04 * upper_horizontal_span
    )
    return float(additive_score * (0.20 + 0.80 * span_validity) * (0.35 + 0.65 * approach_preference))


def compute_execution_span_priority(execution, sensor_data, target_mask):
    """Return the primary rerank signal: how strongly the grasp spans the local cross-section."""
    if execution is None or sensor_data is None or target_mask is None:
        return 0.0
    if str(execution.frame) != 'camera_optical_frame':
        return 0.0

    debug_geom = compute_gripper_debug_geometry(execution)
    if debug_geom is None:
        return 0.0

    image_hw = np.asarray(target_mask).shape[:2]
    center_uv = project_camera_point_to_pixel(debug_geom['center'], sensor_data.K, image_hw)
    finger_a_uv = project_camera_point_to_pixel(debug_geom['finger_a'], sensor_data.K, image_hw)
    finger_b_uv = project_camera_point_to_pixel(debug_geom['finger_b'], sensor_data.K, image_hw)
    if center_uv is None or finger_a_uv is None or finger_b_uv is None:
        return 0.0
    return compute_local_horizontal_cover_score(center_uv, finger_a_uv, finger_b_uv, target_mask)


def select_best_execution_candidate(client,
                                    pred_grasps_dict,
                                    scores_dict,
                                    contact_pts_dict,
                                    gripper_openings_dict,
                                    min_score=0.0,
                                    frame='camera_optical_frame',
                                    pregrasp_offset=0.10,
                                    retreat_offset=0.10,
                                    max_pregrasp_offset=None,
                                    max_retreat_offset=None,
                                    workspace_bounds=None,
                                    top_k=None,
                                    preferred_seg_id=None,
                                    end_pose_world=None,
                                    planning_frame='world',
                                    reference_frame='world',
                                    base_position_world=None,
                                    base_yaw_deg=0.0,
                                    base_workspace_bounds=None,
                                    camera_translation_gripper=None,
                                    camera_quaternion_gripper=None,
                                    sensor_data=None,
                                    target_mask=None,
                                    target_segment_points=None,
                                    max_contact_distance_to_target=None,
                                    end_pose_is_camera_pose: bool = False):
    """Choose the best valid execution candidate with object-fit reranking."""
    workspace_bounds = client._normalize_workspace_bounds(workspace_bounds)
    base_workspace_bounds = client._normalize_workspace_bounds(base_workspace_bounds)
    stats = {
        'total_candidates': 0,
        'rejected_score': 0,
        'rejected_grasp_workspace': 0,
        'rerank_pool': 0,
        'rejected_geometry': 0,
        'rejected_pose_workspace': 0,
        'rejected_base_workspace': 0,
        'rejected_contact_distance': 0,
        'accepted_candidates': 0,
    }
    boundary_points = None if target_mask is None else compute_mask_boundary_points(target_mask)
    candidate_entries = []
    top_candidate_diagnostics = []
    accepted_candidate_diagnostics = []
    for candidate in client._iter_grasp_candidates(
        pred_grasps_dict,
        scores_dict,
        contact_pts_dict,
        gripper_openings_dict,
        preferred_seg_id=preferred_seg_id,
    ):
        stats['total_candidates'] += 1
        if candidate['score'] < min_score:
            stats['rejected_score'] += 1
            continue
        if not client._candidate_in_workspace(candidate, workspace_bounds):
            stats['rejected_grasp_workspace'] += 1
            continue
        execution = client.build_execution_command(
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
        if sensor_data is not None and target_mask is not None:
            geometry_valid, _ = evaluate_execution_geometry_constraints(
                execution,
                sensor_data,
                target_mask,
            )
            if not geometry_valid:
                stats['rejected_geometry'] += 1
                continue
        preliminary_fit_score = score_execution_object_fit(
            execution,
            sensor_data=sensor_data,
            target_mask=target_mask,
            boundary_points=boundary_points,
            target_segment_points=target_segment_points,
        )
        geometry_summary = summarize_execution_candidate_geometry(
            execution,
            sensor_data=sensor_data,
            target_mask=target_mask,
        )
        preliminary_span_priority = compute_execution_span_priority(
            execution,
            sensor_data=sensor_data,
            target_mask=target_mask,
        )
        preliminary_combined_score = float(candidate['score']) + 0.85 * float(preliminary_fit_score)
        preliminary_rank_key = (
            float(preliminary_span_priority),
            float(preliminary_fit_score),
            float(preliminary_combined_score),
            float(candidate['score']),
        )
        candidate_entries.append((
            candidate,
            execution,
            preliminary_fit_score,
            preliminary_span_priority,
            preliminary_rank_key,
            geometry_summary,
        ))
    if not candidate_entries:
        return None, stats

    candidate_entries.sort(key=lambda item: item[4], reverse=True)
    rerank_limit = DEFAULT_EXECUTION_RERANK_TOP_K if top_k is None else max(0, int(top_k))
    if rerank_limit > 0:
        candidate_entries = candidate_entries[:rerank_limit]
    stats['rerank_pool'] = len(candidate_entries)
    for candidate, _, preliminary_fit_score, preliminary_span_priority, _, geometry_summary in candidate_entries[:5]:
        top_candidate_diagnostics.append({
            'segment_id': candidate['segment_id'],
            'score': float(candidate['score']),
            'fit_score': float(preliminary_fit_score),
            'span_priority': float(preliminary_span_priority),
            'geometry': geometry_summary,
        })

    reachability_check_enabled = base_workspace_bounds is not None and base_position_world is not None
    if reachability_check_enabled:
        end_pose_world = client._normalize_matrix4(end_pose_world, 'end_pose_world')

    best_execution = None
    best_meta = None
    best_key = None

    for candidate, execution, preliminary_fit_score, preliminary_span_priority, _, geometry_summary in candidate_entries:
        if not (
            client._pose_in_workspace(execution.pose, workspace_bounds)
            and client._pose_in_workspace(execution.pregrasp_pose, workspace_bounds)
            and client._pose_in_workspace(execution.retreat_pose, workspace_bounds)
        ):
            stats['rejected_pose_workspace'] += 1
            continue
        if (
            reachability_check_enabled
            and not client._execution_in_base_workspace(
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
        ):
            stats['rejected_base_workspace'] += 1
            continue

        nearest_distance = None
        if max_contact_distance_to_target is not None:
            nearest_distance = compute_contact_to_points_distance(
                target_segment_points,
                execution.contact_point,
            )
            if nearest_distance is None or nearest_distance > float(max_contact_distance_to_target):
                stats['rejected_contact_distance'] += 1
                continue

        fit_score = preliminary_fit_score
        combined_score = float(execution.score) + 0.85 * float(fit_score)
        rank_key = (
            float(preliminary_span_priority),
            float(fit_score),
            float(combined_score),
            float(execution.score),
        )
        if best_key is None or rank_key > best_key:
            best_key = rank_key
            best_execution = execution
            best_meta = {
                'fit_score': float(fit_score),
                'combined_score': float(combined_score),
                'nearest_distance': nearest_distance,
                'span_priority': float(preliminary_span_priority),
                'selected_geometry': geometry_summary,
                'stats': stats.copy(),
            }
        accepted_candidate_diagnostics.append({
            'segment_id': candidate['segment_id'],
            'score': float(candidate['score']),
            'fit_score': float(fit_score),
            'span_priority': float(preliminary_span_priority),
            'geometry': geometry_summary,
        })
        stats['accepted_candidates'] += 1

    if best_meta is None:
        best_meta = {'stats': stats.copy()}
    else:
        best_meta['stats'] = stats.copy()
    best_meta['top_candidate_diagnostics'] = top_candidate_diagnostics
    best_meta['accepted_candidate_diagnostics'] = accepted_candidate_diagnostics[:5]
    if (
        best_execution is None
        and stats['accepted_candidates'] == 0
        and stats['rejected_base_workspace'] > 0
        and reachability_check_enabled
    ):
        log_base_workspace_debug(
            client,
            [candidate for candidate, *_ in candidate_entries],
            end_pose_world=end_pose_world,
            planning_frame=planning_frame,
            reference_frame=reference_frame,
            base_position_world=base_position_world,
            base_yaw_deg=base_yaw_deg,
            camera_translation_gripper=camera_translation_gripper,
            camera_quaternion_gripper=camera_quaternion_gripper,
            pregrasp_offset=pregrasp_offset,
            retreat_offset=retreat_offset,
            max_pregrasp_offset=max_pregrasp_offset,
            max_retreat_offset=max_retreat_offset,
            frame=frame,
            max_items=3,
            end_pose_is_camera_pose=end_pose_is_camera_pose,
        )
    return best_execution, best_meta


def freeze_execution_to_planning_frame(client,
                                       execution,
                                       end_pose_world,
                                       execution_reference_frame='world',
                                       reference_frame='world',
                                       execution_camera_translation_gripper=None,
                                       execution_camera_quaternion_gripper=None,
                                       end_pose_is_camera_pose: bool = False):
    """Convert one execution command into a frozen planning/world-frame command."""
    if execution is None:
        return None
    if end_pose_world is None:
        return execution
    if execution.frame == execution_reference_frame:
        return execution

    def regularize_planning_approach_vector(approach_vector_world):
        """Bias approach motions toward a vertical descend to avoid sweeping objects sideways."""
        vector = np.asarray(approach_vector_world, dtype=np.float32).reshape(3)
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-8:
            return np.array([0.0, 0.0, -1.0], dtype=np.float32)
        vector = vector / norm

        vertical_sign = -1.0 if float(vector[2]) <= 0.0 else 1.0
        vertical_axis = np.array([0.0, 0.0, vertical_sign], dtype=np.float32)
        safe_vector = (
            DEFAULT_PLANNING_APPROACH_VERTICAL_BLEND * vertical_axis
            + (1.0 - DEFAULT_PLANNING_APPROACH_VERTICAL_BLEND) * vector
        )
        safe_norm = float(np.linalg.norm(safe_vector))
        if safe_norm <= 1e-8:
            return vertical_axis
        safe_vector = safe_vector / safe_norm

        lateral_norm = float(np.linalg.norm(safe_vector[:2]))
        vertical_abs = abs(float(safe_vector[2]))
        max_lateral = DEFAULT_PLANNING_APPROACH_MAX_LATERAL_RATIO * max(vertical_abs, 1e-6)
        if lateral_norm > max_lateral and lateral_norm > 1e-8:
            safe_vector[:2] *= max_lateral / lateral_norm
            safe_vector /= max(float(np.linalg.norm(safe_vector)), 1e-8)

        if abs(float(safe_vector[2])) < DEFAULT_PLANNING_APPROACH_MIN_VERTICAL_COMPONENT:
            max_horizontal = math.sqrt(max(0.0, 1.0 - DEFAULT_PLANNING_APPROACH_MIN_VERTICAL_COMPONENT ** 2))
            horizontal_norm = float(np.linalg.norm(safe_vector[:2]))
            if horizontal_norm > max_horizontal and horizontal_norm > 1e-8:
                safe_vector[:2] *= max_horizontal / horizontal_norm
            safe_vector[2] = vertical_sign * DEFAULT_PLANNING_APPROACH_MIN_VERTICAL_COMPONENT
            safe_vector /= max(float(np.linalg.norm(safe_vector)), 1e-8)

        return safe_vector.astype(np.float32)

    pose_world = client._transform_pose_to_planning_frame(
        execution.pose,
        execution.frame,
        end_pose_world=end_pose_world,
        planning_frame=execution_reference_frame,
        reference_frame=reference_frame,
        camera_translation_gripper=execution_camera_translation_gripper,
        camera_quaternion_gripper=execution_camera_quaternion_gripper,
        end_pose_is_camera_pose=end_pose_is_camera_pose,
    )
    contact_pose = np.eye(4, dtype=np.float32)
    contact_pose[:3, 3] = np.asarray(execution.contact_point, dtype=np.float32)
    contact_pose_world = client._transform_pose_to_planning_frame(
        contact_pose,
        execution.frame,
        end_pose_world=end_pose_world,
        planning_frame=execution_reference_frame,
        reference_frame=reference_frame,
        camera_translation_gripper=execution_camera_translation_gripper,
        camera_quaternion_gripper=execution_camera_quaternion_gripper,
        end_pose_is_camera_pose=end_pose_is_camera_pose,
    )

    approach_tip_pose = np.eye(4, dtype=np.float32)
    approach_tip_pose[:3, 3] = (
        np.asarray(execution.contact_point, dtype=np.float32)
        + np.asarray(execution.approach_vector, dtype=np.float32)
    )
    approach_tip_world = client._transform_pose_to_planning_frame(
        approach_tip_pose,
        execution.frame,
        end_pose_world=end_pose_world,
        planning_frame=execution_reference_frame,
        reference_frame=reference_frame,
        camera_translation_gripper=execution_camera_translation_gripper,
        camera_quaternion_gripper=execution_camera_quaternion_gripper,
        end_pose_is_camera_pose=end_pose_is_camera_pose,
    )
    approach_vector_world = approach_tip_world[:3, 3] - contact_pose_world[:3, 3]
    approach_norm = float(np.linalg.norm(approach_vector_world))
    if approach_norm > 1e-8:
        approach_vector_world = approach_vector_world / approach_norm
    else:
        approach_vector_world = pose_world[:3, 2].astype(np.float32)
    planning_approach_vector_world = regularize_planning_approach_vector(approach_vector_world)

    pregrasp_world = pose_world.copy()
    pregrasp_world[:3, 3] -= planning_approach_vector_world * float(execution.pregrasp_offset)

    retreat_world = pose_world.copy()
    retreat_world[:3, 3] -= planning_approach_vector_world * float(execution.retreat_offset)

    return ExecutionCommand(
        pose=np.asarray(pose_world, dtype=np.float32),
        pregrasp_pose=np.asarray(pregrasp_world, dtype=np.float32),
        retreat_pose=np.asarray(retreat_world, dtype=np.float32),
        contact_point=np.asarray(contact_pose_world[:3, 3], dtype=np.float32),
        approach_vector=np.asarray(planning_approach_vector_world, dtype=np.float32),
        gripper_opening=float(execution.gripper_opening),
        score=float(execution.score),
        segment_id=int(execution.segment_id),
        frame=str(execution_reference_frame),
        pregrasp_offset=float(execution.pregrasp_offset),
        retreat_offset=float(execution.retreat_offset),
    )


def project_execution_to_camera_frame(client,
                                      execution,
                                      end_pose_world,
                                      execution_reference_frame='world',
                                      execution_camera_translation_gripper=None,
                                      execution_camera_quaternion_gripper=None,
                                      end_pose_is_camera_pose: bool = False):
    """Project a frozen planning/world-frame execution back into the current camera frame."""
    if execution is None:
        return None
    if end_pose_world is None:
        return execution if str(execution.frame) == 'camera_optical_frame' else None
    if str(execution.frame) == 'camera_optical_frame':
        return execution
    if str(execution.frame) != str(execution_reference_frame):
        return None

    end_pose_world = client._normalize_matrix4(end_pose_world, 'end_pose_world')
    camera_translation = client._normalize_vector3(
        execution_camera_translation_gripper if execution_camera_translation_gripper is not None else client._DEFAULT_CAMERA_TRANSLATION_GRIPPER,
        'camera_translation_gripper',
    )
    camera_quaternion = np.asarray(
        execution_camera_quaternion_gripper if execution_camera_quaternion_gripper is not None else client._DEFAULT_CAMERA_QUATERNION_GRIPPER,
        dtype=np.float32,
    )

    t_gripper_to_cam = np.eye(4, dtype=np.float32)
    t_gripper_to_cam[:3, :3] = client._quaternion_to_matrix(camera_quaternion)
    t_gripper_to_cam[:3, 3] = camera_translation

    t_cam_to_optical = np.eye(4, dtype=np.float32)
    t_cam_to_optical[:3, :3] = get_camera_to_optical_rotation(client)

    world_to_reference = np.linalg.inv(end_pose_world)

    def world_pose_to_optical(pose_world):
        pose_world = client._normalize_matrix4(pose_world, 'pose_world')
        pose_in_reference = world_to_reference @ pose_world
        if end_pose_is_camera_pose:
            pose_in_cam = pose_in_reference
        else:
            pose_in_cam = t_gripper_to_cam @ pose_in_reference
        return t_cam_to_optical @ pose_in_cam

    pose_camera = world_pose_to_optical(execution.pose)
    pregrasp_camera = world_pose_to_optical(execution.pregrasp_pose)
    retreat_camera = world_pose_to_optical(execution.retreat_pose)

    contact_pose_world = np.eye(4, dtype=np.float32)
    contact_pose_world[:3, 3] = np.asarray(execution.contact_point, dtype=np.float32)
    contact_pose_camera = world_pose_to_optical(contact_pose_world)

    approach_tip_world = np.eye(4, dtype=np.float32)
    approach_tip_world[:3, 3] = (
        np.asarray(execution.contact_point, dtype=np.float32)
        + np.asarray(execution.approach_vector, dtype=np.float32)
    )
    approach_tip_camera = world_pose_to_optical(approach_tip_world)
    approach_vector_camera = approach_tip_camera[:3, 3] - contact_pose_camera[:3, 3]
    approach_norm = float(np.linalg.norm(approach_vector_camera))
    if approach_norm > 1e-8:
        approach_vector_camera = approach_vector_camera / approach_norm
    else:
        approach_vector_camera = pose_camera[:3, 2].astype(np.float32)

    return ExecutionCommand(
        pose=np.asarray(pose_camera, dtype=np.float32),
        pregrasp_pose=np.asarray(pregrasp_camera, dtype=np.float32),
        retreat_pose=np.asarray(retreat_camera, dtype=np.float32),
        contact_point=np.asarray(contact_pose_camera[:3, 3], dtype=np.float32),
        approach_vector=np.asarray(approach_vector_camera, dtype=np.float32),
        gripper_opening=float(execution.gripper_opening),
        score=float(execution.score),
        segment_id=int(execution.segment_id),
        frame='camera_optical_frame',
        pregrasp_offset=float(execution.pregrasp_offset),
        retreat_offset=float(execution.retreat_offset),
    )


def compute_execution_world_delta(execution_a_world, execution_b_world):
    """Measure how far two frozen world-frame executions differ."""
    if execution_a_world is None or execution_b_world is None:
        return None
    center_a = np.asarray(execution_a_world.pose[:3, 3], dtype=np.float32)
    center_b = np.asarray(execution_b_world.pose[:3, 3], dtype=np.float32)
    contact_a = np.asarray(execution_a_world.contact_point, dtype=np.float32)
    contact_b = np.asarray(execution_b_world.contact_point, dtype=np.float32)
    approach_a = np.asarray(execution_a_world.approach_vector, dtype=np.float32)
    approach_b = np.asarray(execution_b_world.approach_vector, dtype=np.float32)
    approach_a_norm = float(np.linalg.norm(approach_a))
    approach_b_norm = float(np.linalg.norm(approach_b))
    if approach_a_norm > 1e-8:
        approach_a = approach_a / approach_a_norm
    if approach_b_norm > 1e-8:
        approach_b = approach_b / approach_b_norm
    dot = float(np.clip(np.dot(approach_a, approach_b), -1.0, 1.0))
    angle_deg = float(np.degrees(np.arccos(dot)))
    return {
        'center_distance_m': float(np.linalg.norm(center_a - center_b)),
        'contact_distance_m': float(np.linalg.norm(contact_a - contact_b)),
        'approach_angle_deg': angle_deg,
    }


def log_execution_target_distance(execution,
                                  pc_segments,
                                  active_segmap_id,
                                  target_points_frame='camera_optical_frame'):
    """Log whether the chosen execution contact point lies near the selected target segment."""
    if execution is None or active_segmap_id is None or active_segmap_id <= 0:
        return
    execution_frame = str(getattr(execution, 'frame', '') or '')
    if execution_frame and execution_frame != str(target_points_frame):
        print(
            "  Execution target diagnostic skipped "
            f"(execution frame={execution_frame}, target points frame={target_points_frame})"
        )
        return

    nearest_distance = compute_contact_to_segment_distance(
        pc_segments,
        active_segmap_id,
        execution.contact_point,
    )
    if nearest_distance is None:
        print("  Execution target diagnostic unavailable (missing target segment points)")
        return

    print(
        "  Execution contact point: "
        f"{np.round(np.asarray(execution.contact_point), 4).tolist()} "
        f"-> target seg {int(active_segmap_id)} nearest distance={nearest_distance:.4f} m"
    )


def get_target_segment_points(pc_segments, active_segmap_id):
    """Return the Nx3 target point cloud for the selected segment id."""
    if active_segmap_id is None or active_segmap_id <= 0:
        return None
    if active_segmap_id not in pc_segments:
        return None
    segment_points = np.asarray(pc_segments[active_segmap_id], dtype=np.float32)
    if segment_points.size == 0:
        return None
    return segment_points


def compute_target_signature(segmap_aligned, active_segmap_id, depth):
    """Compute a simple target signature for frame-to-frame stability checks."""
    if segmap_aligned is None or active_segmap_id is None or active_segmap_id <= 0:
        return None
    mask = np.asarray(segmap_aligned == active_segmap_id, dtype=bool)
    ys, xs = np.nonzero(mask)
    if ys.size == 0:
        return None

    centroid_xy = np.array([float(xs.mean()), float(ys.mean())], dtype=np.float32)
    depth_vals = np.asarray(depth)[mask]
    depth_vals = depth_vals[np.isfinite(depth_vals)]
    median_depth = float(np.median(depth_vals)) if depth_vals.size > 0 else None
    return {
        'segment_id': int(active_segmap_id),
        'centroid_xy': centroid_xy,
        'median_depth': median_depth,
        'area': int(mask.sum()),
    }


def collect_segment_signatures(segmap_aligned, depth):
    """Collect signatures for every foreground segment in the current frame."""
    if segmap_aligned is None:
        return {}

    signatures = {}
    segment_ids = np.unique(np.asarray(segmap_aligned))
    for seg_id in segment_ids:
        if int(seg_id) <= 0:
            continue
        signature = compute_target_signature(segmap_aligned, int(seg_id), depth)
        if signature is not None:
            signatures[int(seg_id)] = signature
    return signatures


def choose_locked_target_segment(segment_signatures,
                                 locked_signature,
                                 centroid_tol_px=40.0,
                                 depth_tol_m=0.05,
                                 max_area_ratio_change=3.0,
                                 allowed_segment_ids=None):
    """Choose the current segment that best matches a previously locked target."""
    if not segment_signatures or not locked_signature:
        return None

    allowed_segment_ids = None if allowed_segment_ids is None else {
        int(seg_id) for seg_id in allowed_segment_ids
    }

    best_seg_id = None
    best_cost = None
    locked_centroid = np.asarray(locked_signature['centroid_xy'], dtype=np.float32)
    locked_depth = locked_signature.get('median_depth')
    locked_area = max(1, int(locked_signature.get('area', 1)))

    for seg_id, signature in segment_signatures.items():
        if allowed_segment_ids is not None and int(seg_id) not in allowed_segment_ids:
            continue
        centroid_dist = np.linalg.norm(signature['centroid_xy'] - locked_centroid)
        if centroid_dist > float(centroid_tol_px):
            continue

        current_depth = signature.get('median_depth')
        if locked_depth is not None and current_depth is not None:
            depth_delta = abs(current_depth - locked_depth)
            if depth_delta > float(depth_tol_m):
                continue
        else:
            depth_delta = 0.0

        current_area = max(1, int(signature.get('area', 1)))
        area_ratio = max(current_area / float(locked_area), locked_area / float(current_area))
        if area_ratio > float(max_area_ratio_change):
            continue

        cost = centroid_dist + depth_delta * 200.0
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_seg_id = int(seg_id)

    return best_seg_id


def maybe_preselect_tracked_target_segment(segment_signatures,
                                           target_query,
                                           target_lock,
                                           tracking_state,
                                           target_lock_max_centroid_shift_px,
                                           target_lock_max_depth_delta_m,
                                           target_lock_max_area_ratio_change):
    """Reuse only locked target geometry to avoid semantic drift before lock."""
    if not target_query or not segment_signatures:
        return None, None

    locked_signature = tracking_state.get('locked_signature')
    if target_lock and locked_signature is not None:
        locked_seg_id = choose_locked_target_segment(
            segment_signatures,
            locked_signature,
            centroid_tol_px=target_lock_max_centroid_shift_px,
            depth_tol_m=target_lock_max_depth_delta_m,
            max_area_ratio_change=target_lock_max_area_ratio_change,
        )
        if locked_seg_id is not None:
            return int(locked_seg_id), f'{target_query}->{int(locked_seg_id)} (locked-fastpath)'

    return None, None


def target_signatures_match(signature_a,
                            signature_b,
                            centroid_tol_px=40.0,
                            depth_tol_m=0.05,
                            max_area_ratio_change=3.0):
    """Return whether two target signatures likely describe the same object instance."""
    if signature_a is None or signature_b is None:
        return False
    centroid_a = np.asarray(signature_a.get('centroid_xy'), dtype=np.float32)
    centroid_b = np.asarray(signature_b.get('centroid_xy'), dtype=np.float32)
    if centroid_a.shape != (2,) or centroid_b.shape != (2,):
        return False
    if np.linalg.norm(centroid_a - centroid_b) > float(centroid_tol_px):
        return False

    depth_a = signature_a.get('median_depth')
    depth_b = signature_b.get('median_depth')
    if depth_a is not None and depth_b is not None:
        if abs(float(depth_a) - float(depth_b)) > float(depth_tol_m):
            return False

    area_a = max(1, int(signature_a.get('area', 1)))
    area_b = max(1, int(signature_b.get('area', 1)))
    area_ratio = max(area_a / float(area_b), area_b / float(area_a))
    return bool(area_ratio <= float(max_area_ratio_change))


def update_target_stability(stability_state,
                            target_key,
                            signature,
                            centroid_tol_px=40.0,
                            depth_tol_m=0.05):
    """Update and return the number of consecutive stable frames for the target."""
    if signature is None:
        stability_state['key'] = None
        stability_state['signature'] = None
        stability_state['count'] = 0
        return 0

    prev_key = stability_state.get('key')
    prev_signature = stability_state.get('signature')
    stable = False
    if prev_key == target_key and prev_signature is not None:
        centroid_ok = (
            np.linalg.norm(signature['centroid_xy'] - prev_signature['centroid_xy'])
            <= float(centroid_tol_px)
        )
        prev_depth = prev_signature.get('median_depth')
        curr_depth = signature.get('median_depth')
        if prev_depth is None or curr_depth is None:
            depth_ok = True
        else:
            depth_ok = abs(curr_depth - prev_depth) <= float(depth_tol_m)
        stable = bool(centroid_ok and depth_ok)

    stability_state['count'] = int(stability_state.get('count', 0)) + 1 if stable else 1
    stability_state['key'] = target_key
    stability_state['signature'] = signature
    return stability_state['count']


def clear_pre_stability_execution_cache(tracking_state):
    """Clear the temporary best-candidate cache accumulated before target stability is reached."""
    tracking_state['prestable_execution_world'] = None
    tracking_state['prestable_fit_score'] = None
    tracking_state['prestable_span_priority'] = None
    tracking_state['prestable_geometry'] = None
    tracking_state['prestable_signature'] = None
    tracking_state['prestable_target_key'] = None
    tracking_state['prestable_frame_id'] = None
    tracking_state['prestable_score'] = None


def build_execution_candidate_rank(execution, execution_meta):
    """Build a comparable rank key for one execution candidate."""
    if execution is None or execution_meta is None:
        return None
    geometry = execution_meta.get('selected_geometry') or {}
    return (
        float(execution_meta.get('span_priority', -1.0)),
        float(execution_meta.get('fit_score', -1.0)),
        float(geometry.get('approach_score', 0.0)),
        float(getattr(execution, 'score', 0.0) or 0.0),
    )


def update_pre_stability_execution_cache(tracking_state,
                                         current_target_key,
                                         current_signature,
                                         execution_preview,
                                         execution_world,
                                         execution_meta,
                                         frame_id,
                                         centroid_tol_px=40.0,
                                         depth_tol_m=0.05,
                                         max_area_ratio_change=3.0):
    """Record the strongest execution candidate observed before stability threshold is met."""
    if (
        current_target_key is None
        or current_signature is None
        or execution_preview is None
        or execution_world is None
        or execution_meta is None
    ):
        return False

    previous_key = tracking_state.get('prestable_target_key')
    previous_signature = tracking_state.get('prestable_signature')
    same_target = (
        previous_key == current_target_key
        and target_signatures_match(
            current_signature,
            previous_signature,
            centroid_tol_px=centroid_tol_px,
            depth_tol_m=depth_tol_m,
            max_area_ratio_change=max_area_ratio_change,
        )
    )
    if not same_target:
        clear_pre_stability_execution_cache(tracking_state)

    current_rank = build_execution_candidate_rank(execution_preview, execution_meta)
    previous_execution_world = tracking_state.get('prestable_execution_world')
    previous_rank = None
    if previous_execution_world is not None:
        previous_rank = (
            float(tracking_state.get('prestable_span_priority', -1.0) or -1.0),
            float(tracking_state.get('prestable_fit_score', -1.0) or -1.0),
            float((tracking_state.get('prestable_geometry') or {}).get('approach_score', 0.0)),
            float(tracking_state.get('prestable_score', 0.0) or 0.0),
        )

    if previous_rank is not None and current_rank is not None and current_rank <= previous_rank:
        return False

    tracking_state['prestable_execution_world'] = execution_world
    tracking_state['prestable_fit_score'] = float(execution_meta.get('fit_score', 0.0))
    tracking_state['prestable_span_priority'] = float(execution_meta.get('span_priority', 0.0))
    tracking_state['prestable_geometry'] = execution_meta.get('selected_geometry')
    tracking_state['prestable_signature'] = current_signature
    tracking_state['prestable_target_key'] = current_target_key
    tracking_state['prestable_frame_id'] = int(frame_id)
    tracking_state['prestable_score'] = float(getattr(execution_preview, 'score', 0.0) or 0.0)
    return True


def promote_pre_stability_execution_cache(client,
                                          tracking_state,
                                          current_target_key,
                                          current_signature,
                                          sensor_data,
                                          target_mask,
                                          target_segment_points,
                                          current_execution_preview,
                                          current_execution_world,
                                          current_execution_fit,
                                          current_execution_meta,
                                          execution_reference_frame='world',
                                          execution_camera_translation_gripper=None,
                                          execution_camera_quaternion_gripper=None,
                                          end_pose_is_camera_pose: bool = False,
                                          centroid_tol_px=40.0,
                                          depth_tol_m=0.05,
                                          max_area_ratio_change=3.0):
    """Compare the current execution against the best pre-stability candidate and keep the stronger one."""
    cached_world = tracking_state.get('prestable_execution_world')
    cached_signature = tracking_state.get('prestable_signature')
    cached_target_key = tracking_state.get('prestable_target_key')
    cached_frame_id = int(tracking_state.get('prestable_frame_id') or 0)

    if (
        cached_world is None
        or current_target_key is None
        or current_signature is None
        or cached_target_key != current_target_key
        or not target_signatures_match(
            current_signature,
            cached_signature,
            centroid_tol_px=centroid_tol_px,
            depth_tol_m=depth_tol_m,
            max_area_ratio_change=max_area_ratio_change,
        )
    ):
        clear_pre_stability_execution_cache(tracking_state)
        return current_execution_preview, current_execution_world, current_execution_fit, current_execution_meta

    cached_preview = project_execution_to_camera_frame(
        client,
        cached_world,
        end_pose_world=sensor_data.end_pose,
        execution_reference_frame=execution_reference_frame,
        execution_camera_translation_gripper=execution_camera_translation_gripper,
        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
        end_pose_is_camera_pose=end_pose_is_camera_pose,
    )
    cached_valid, cached_info = evaluate_execution_preview_projection(
        cached_preview,
        sensor_data,
        target_mask,
    )
    if not cached_valid:
        print(
            f"Frame {sensor_data.frame_id}: discarding pre-stability execution from frame "
            f"{cached_frame_id} ({cached_info.get('reason')}, "
            f"line_fraction={cached_info.get('line_fraction', 0.0):.2f}, "
            f"finger_span={cached_info.get('finger_span', 0.0):.2f}, "
            f"center_t={cached_info.get('center_t', 0.5):.2f})"
        )
        clear_pre_stability_execution_cache(tracking_state)
        return current_execution_preview, current_execution_world, current_execution_fit, current_execution_meta

    cached_fit = score_execution_object_fit(
        cached_preview,
        sensor_data,
        target_mask,
        target_segment_points=target_segment_points,
    )
    cached_geometry = summarize_execution_candidate_geometry(
        cached_preview,
        sensor_data=sensor_data,
        target_mask=target_mask,
    )
    cached_span_priority = compute_execution_span_priority(
        cached_preview,
        sensor_data=sensor_data,
        target_mask=target_mask,
    )
    cached_meta = {
        'fit_score': float(cached_fit),
        'span_priority': float(cached_span_priority),
        'selected_geometry': cached_geometry,
    }

    if current_execution_preview is not None:
        current_fit = current_execution_fit
        if current_fit is None and current_execution_meta is not None:
            current_fit = current_execution_meta.get('fit_score')
        if current_fit is None:
            current_fit = score_execution_object_fit(
                current_execution_preview,
                sensor_data,
                target_mask,
                target_segment_points=target_segment_points,
            )
        current_meta = dict(current_execution_meta or {})
        current_meta['fit_score'] = float(current_fit)
        if current_meta.get('span_priority') is None:
            current_meta['span_priority'] = float(compute_execution_span_priority(
                current_execution_preview,
                sensor_data=sensor_data,
                target_mask=target_mask,
            ))
        if current_meta.get('selected_geometry') is None:
            current_meta['selected_geometry'] = summarize_execution_candidate_geometry(
                current_execution_preview,
                sensor_data=sensor_data,
                target_mask=target_mask,
            )
    else:
        current_fit = None
        current_meta = None

    cached_rank = build_execution_candidate_rank(cached_preview, cached_meta)
    current_rank = build_execution_candidate_rank(current_execution_preview, current_meta)

    clear_pre_stability_execution_cache(tracking_state)

    if current_rank is None or (cached_rank is not None and cached_rank > current_rank):
        print(
            f"Frame {sensor_data.frame_id}: promoting pre-stability execution from frame "
            f"{cached_frame_id} (cached_span={float(cached_meta.get('span_priority', 0.0)):.3f}, "
            f"cached_fit={float(cached_meta.get('fit_score', 0.0)):.3f})"
        )
        return cached_preview, cached_world, float(cached_fit), cached_meta

    print(
        f"Frame {sensor_data.frame_id}: keeping current execution over pre-stability candidate "
        f"from frame {cached_frame_id} "
        f"(current_span={float(current_meta.get('span_priority', 0.0)):.3f}, "
        f"cached_span={float(cached_meta.get('span_priority', 0.0)):.3f}, "
        f"current_fit={float(current_meta.get('fit_score', 0.0)):.3f}, "
        f"cached_fit={float(cached_meta.get('fit_score', 0.0)):.3f})"
    )
    return current_execution_preview, current_execution_world, float(current_fit), current_meta


def compute_valid_depth_mask(depth, z_range):
    """Return a boolean mask of pixels with valid depth inside the configured z-range."""
    depth = np.asarray(depth)
    return np.isfinite(depth) & (depth > float(z_range[0])) & (depth < float(z_range[1]))


def _erode_binary_mask(mask, iterations=1):
    """Erode a 2D boolean mask with a 3x3 all-neighbor structuring element."""
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2 or not np.any(mask):
        return mask.copy()
    eroded = mask.copy()
    for _ in range(max(0, int(iterations))):
        padded = np.pad(eroded, 1, mode='constant', constant_values=False)
        neighbors = [
            padded[1 + dy:1 + dy + mask.shape[0], 1 + dx:1 + dx + mask.shape[1]]
            for dy in (-1, 0, 1)
            for dx in (-1, 0, 1)
        ]
        eroded = np.logical_and.reduce(neighbors)
        if not np.any(eroded):
            break
    return eroded


def _dilate_binary_mask(mask, iterations=1):
    """Dilate a 2D boolean mask with a 3x3 all-neighbor structuring element."""
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2 or not np.any(mask):
        return mask.copy()
    dilated = mask.copy()
    for _ in range(max(0, int(iterations))):
        padded = np.pad(dilated, 1, mode='constant', constant_values=False)
        neighbors = [
            padded[1 + dy:1 + dy + mask.shape[0], 1 + dx:1 + dx + mask.shape[1]]
            for dy in (-1, 0, 1)
            for dx in (-1, 0, 1)
        ]
        dilated = np.logical_or.reduce(neighbors)
    return dilated


def _select_preferred_mask_component(mask, preferred_mask=None, min_pixels=0):
    """Keep the strongest connected component, preferring overlap with a seed mask."""
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2 or not np.any(mask):
        return np.zeros_like(mask, dtype=bool), None

    preferred_mask = None if preferred_mask is None else np.asarray(preferred_mask, dtype=bool)
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    best_component = None
    best_meta = None

    for start_y, start_x in np.argwhere(mask):
        if visited[start_y, start_x]:
            continue

        stack = [(int(start_y), int(start_x))]
        visited[start_y, start_x] = True
        pixels = []
        overlap = 0
        while stack:
            y, x = stack.pop()
            pixels.append((y, x))
            if preferred_mask is not None and preferred_mask[y, x]:
                overlap += 1
            for ny in range(max(0, y - 1), min(height, y + 2)):
                for nx in range(max(0, x - 1), min(width, x + 2)):
                    if (ny == y and nx == x) or visited[ny, nx] or not mask[ny, nx]:
                        continue
                    visited[ny, nx] = True
                    stack.append((ny, nx))

        area = len(pixels)
        if area < max(1, int(min_pixels)):
            continue

        score = (1 if overlap > 0 else 0, overlap, area)
        if best_meta is None or score > best_meta['score']:
            component_mask = np.zeros_like(mask, dtype=bool)
            ys, xs = zip(*pixels)
            component_mask[np.asarray(ys, dtype=np.int32), np.asarray(xs, dtype=np.int32)] = True
            best_component = component_mask
            best_meta = {
                'area': int(area),
                'core_overlap': int(overlap),
                'score': score,
            }

    if best_component is None:
        return np.zeros_like(mask, dtype=bool), None
    return best_component, best_meta


def refine_target_mask_by_depth(target_mask,
                                depth,
                                z_range,
                                enabled=DEFAULT_TARGET_DEPTH_CLEANING_ENABLED,
                                core_erode_iters=DEFAULT_TARGET_DEPTH_CORE_ERODE_ITERS,
                                min_core_pixels=DEFAULT_TARGET_DEPTH_MIN_CORE_PIXELS,
                                seed_percentile=DEFAULT_TARGET_DEPTH_SEED_PERCENTILE,
                                front_tolerance_m=DEFAULT_TARGET_DEPTH_FRONT_TOLERANCE_M,
                                back_tolerance_m=DEFAULT_TARGET_DEPTH_BACK_TOLERANCE_M,
                                min_component_pixels=DEFAULT_TARGET_DEPTH_MIN_COMPONENT_PIXELS,
                                component_dilate_iters=DEFAULT_TARGET_DEPTH_COMPONENT_DILATE_ITERS):
    """Remove far-background leakage from one selected target mask using local depth structure."""
    target_mask = np.asarray(target_mask, dtype=bool)
    depth = np.asarray(depth, dtype=np.float32)
    original_pixels = int(np.count_nonzero(target_mask))
    original_valid_mask = target_mask & compute_valid_depth_mask(depth, z_range)
    original_valid_pixels = int(np.count_nonzero(original_valid_mask))

    info = {
        'enabled': bool(enabled),
        'applied': False,
        'reason': 'disabled' if not enabled else 'not_run',
        'seed_depth': None,
        'original_pixels': original_pixels,
        'original_valid_pixels': original_valid_pixels,
        'cleaned_pixels': original_pixels,
        'cleaned_valid_pixels': original_valid_pixels,
        'core_pixels': 0,
        'component_pixels': 0,
        'component_core_overlap': 0,
    }
    if not enabled or original_pixels == 0:
        return target_mask.copy(), info

    core_mask = _erode_binary_mask(target_mask, iterations=core_erode_iters)
    if int(np.count_nonzero(core_mask)) < int(min_core_pixels):
        core_mask = target_mask.copy()
    info['core_pixels'] = int(np.count_nonzero(core_mask))

    core_depth = depth[core_mask]
    core_depth = core_depth[np.isfinite(core_depth) & (core_depth > 0)]
    if core_depth.size == 0:
        info['reason'] = 'no_core_depth'
        return target_mask.copy(), info

    seed_depth = float(np.percentile(core_depth, float(seed_percentile)))
    info['seed_depth'] = seed_depth

    valid_mask = compute_valid_depth_mask(depth, z_range)
    band_mask = (
        target_mask
        & valid_mask
        & (depth >= max(float(z_range[0]), seed_depth - float(front_tolerance_m)))
        & (depth <= min(float(z_range[1]), seed_depth + float(back_tolerance_m)))
    )
    if not np.any(band_mask):
        info['reason'] = 'no_band_pixels'
        return target_mask.copy(), info

    preferred_mask = core_mask & band_mask
    if not np.any(preferred_mask):
        preferred_mask = _dilate_binary_mask(core_mask, iterations=1) & band_mask

    component_mask, component_meta = _select_preferred_mask_component(
        band_mask,
        preferred_mask=preferred_mask if np.any(preferred_mask) else core_mask,
        min_pixels=min_component_pixels,
    )
    if component_meta is None or not np.any(component_mask):
        info['reason'] = 'no_component'
        return target_mask.copy(), info

    cleaned_mask = component_mask
    if int(component_dilate_iters) > 0:
        cleaned_mask = _dilate_binary_mask(cleaned_mask, iterations=component_dilate_iters)
        cleaned_mask &= target_mask & valid_mask
        cleaned_mask, _ = _select_preferred_mask_component(
            cleaned_mask,
            preferred_mask=component_mask,
            min_pixels=min_component_pixels,
        )

    cleaned_pixels = int(np.count_nonzero(cleaned_mask))
    cleaned_valid_pixels = int(np.count_nonzero(cleaned_mask & valid_mask))
    if cleaned_pixels < int(min_component_pixels) or cleaned_valid_pixels == 0:
        info['reason'] = 'cleaned_too_small'
        return target_mask.copy(), info

    info.update({
        'applied': True,
        'reason': 'cleaned',
        'cleaned_pixels': cleaned_pixels,
        'cleaned_valid_pixels': cleaned_valid_pixels,
        'component_pixels': int(component_meta.get('area', 0)),
        'component_core_overlap': int(component_meta.get('core_overlap', 0)),
    })
    return cleaned_mask, info


def apply_cleaned_target_mask(segmap, active_segmap_id, cleaned_mask):
    """Replace one target segment's mask with a cleaned boolean mask."""
    if segmap is None or active_segmap_id is None or int(active_segmap_id) <= 0:
        return segmap
    segmap = np.asarray(segmap, dtype=np.int32)
    cleaned_mask = np.asarray(cleaned_mask, dtype=bool)
    updated = segmap.copy()
    updated[updated == int(active_segmap_id)] = 0
    updated[cleaned_mask] = int(active_segmap_id)
    return updated


def print_target_depth_cleanup_info(info, prefix='  '):
    """Print compact diagnostics for target-mask depth cleanup."""
    if not info or not info.get('enabled', False):
        return
    original_ratio = (
        float(info['original_valid_pixels']) / float(max(1, info['original_pixels']))
        if info.get('original_pixels', 0) > 0 else 0.0
    )
    cleaned_ratio = (
        float(info['cleaned_valid_pixels']) / float(max(1, info['cleaned_pixels']))
        if info.get('cleaned_pixels', 0) > 0 else 0.0
    )
    seed_depth = info.get('seed_depth')
    seed_text = 'n/a' if seed_depth is None else f'{float(seed_depth):.3f}'
    print(
        f"{prefix}Target depth cleanup: reason={info.get('reason')}, "
        f"seed={seed_text}, original={int(info.get('original_pixels', 0))}/"
        f"{int(info.get('original_valid_pixels', 0))} ({original_ratio:.3f}), "
        f"cleaned={int(info.get('cleaned_pixels', 0))}/"
        f"{int(info.get('cleaned_valid_pixels', 0))} ({cleaned_ratio:.3f})"
    )


def _shift_depth_array(values, dy, dx):
    """Shift one 2D float array with NaN padding."""
    values = np.asarray(values, dtype=np.float32)
    height, width = values.shape
    padded = np.pad(values, ((1, 1), (1, 1)), mode='constant', constant_values=np.nan)
    return padded[1 + int(dy):1 + int(dy) + height, 1 + int(dx):1 + int(dx) + width]


def _compute_depth_neighbor_statistics(depth):
    """Compute local 3x3 depth statistics used for hole filling and outlier cleanup."""
    depth = np.asarray(depth, dtype=np.float32)
    neighbor_views = []
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            neighbor_views.append(_shift_depth_array(depth, dy, dx))
    stack = np.stack(neighbor_views, axis=0)
    valid = np.isfinite(stack) & (stack > 0)
    stack = np.where(valid, stack, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', category=RuntimeWarning)
        local_median = np.nanmedian(stack, axis=0)
        local_min = np.nanmin(stack, axis=0)
        local_max = np.nanmax(stack, axis=0)
    valid_count = np.sum(valid, axis=0)
    return {
        'median': np.asarray(local_median, dtype=np.float32),
        'min': np.asarray(local_min, dtype=np.float32),
        'max': np.asarray(local_max, dtype=np.float32),
        'count': np.asarray(valid_count, dtype=np.int32),
    }


def preprocess_depth_for_inference(depth,
                                   z_range,
                                   enabled=True,
                                   hole_fill_iters=DEFAULT_DEPTH_HOLE_FILL_ITERS,
                                   min_valid_neighbors=DEFAULT_DEPTH_HOLE_FILL_MIN_VALID_NEIGHBORS,
                                   hole_fill_max_delta_m=DEFAULT_DEPTH_HOLE_FILL_MAX_DELTA_M,
                                   outlier_threshold_m=DEFAULT_DEPTH_OUTLIER_THRESHOLD_M,
                                   outlier_relative_threshold=DEFAULT_DEPTH_OUTLIER_RELATIVE_THRESHOLD):
    """Apply lightweight real-robot depth cleanup before point-cloud extraction."""
    depth = np.asarray(depth, dtype=np.float32)
    cleaned = depth.copy()
    invalid_before_mask = ~np.isfinite(cleaned) | (cleaned <= 0)
    invalid_before = int(np.count_nonzero(invalid_before_mask))

    cleaned[~np.isfinite(cleaned)] = 0.0
    cleaned[cleaned < 0] = 0.0

    stats = {
        'enabled': bool(enabled),
        'invalid_before': invalid_before,
        'invalid_after': invalid_before,
        'filled_pixels': 0,
        'outlier_pixels': 0,
    }
    if not enabled:
        return cleaned, stats

    for _ in range(max(0, int(hole_fill_iters))):
        neighbor_stats = _compute_depth_neighbor_statistics(cleaned)
        invalid_mask = cleaned <= 0
        neighbor_span = neighbor_stats['max'] - neighbor_stats['min']
        fill_mask = (
            invalid_mask
            & (neighbor_stats['count'] >= int(min_valid_neighbors))
            & np.isfinite(neighbor_stats['median'])
            & np.isfinite(neighbor_span)
            & (neighbor_span <= float(hole_fill_max_delta_m))
        )
        if not np.any(fill_mask):
            break
        cleaned[fill_mask] = neighbor_stats['median'][fill_mask]
        stats['filled_pixels'] += int(np.count_nonzero(fill_mask))

    valid_in_range_mask = compute_valid_depth_mask(cleaned, z_range)
    if np.any(valid_in_range_mask):
        neighbor_stats = _compute_depth_neighbor_statistics(cleaned)
        adaptive_threshold = np.maximum(
            float(outlier_threshold_m),
            float(outlier_relative_threshold) * neighbor_stats['median'],
        )
        outlier_mask = (
            valid_in_range_mask
            & (neighbor_stats['count'] >= int(min_valid_neighbors))
            & np.isfinite(neighbor_stats['median'])
            & np.isfinite(adaptive_threshold)
            & (np.abs(cleaned - neighbor_stats['median']) > adaptive_threshold)
        )
        if np.any(outlier_mask):
            cleaned[outlier_mask] = neighbor_stats['median'][outlier_mask]
            stats['outlier_pixels'] = int(np.count_nonzero(outlier_mask))

    stats['invalid_after'] = int(np.count_nonzero(~np.isfinite(cleaned) | (cleaned <= 0)))
    return cleaned.astype(np.float32), stats


def print_depth_preprocess_stats(stats, prefix='  '):
    """Print compact stats for the real-robot depth preprocessing step."""
    if not stats or not stats.get('enabled', False):
        return
    print(
        f"{prefix}Depth preprocess: invalid_before={int(stats.get('invalid_before', 0))}, "
        f"filled={int(stats.get('filled_pixels', 0))}, "
        f"outliers={int(stats.get('outlier_pixels', 0))}, "
        f"invalid_after={int(stats.get('invalid_after', 0))}"
    )


def resize_label_map_nearest(label_map, target_hw):
    """Resize an integer label map to target (H, W) with nearest-neighbor sampling."""
    try:
        from PIL import Image
    except ImportError as exc:
        raise ImportError('Pillow is required for segmap resizing') from exc

    label_map = np.asarray(label_map)
    target_h, target_w = int(target_hw[0]), int(target_hw[1])
    resized = Image.fromarray(label_map.astype(np.int32), mode='I').resize(
        (target_w, target_h),
        resample=Image.Resampling.NEAREST,
    )
    return np.asarray(resized, dtype=np.int32)


def resize_rgb_image(rgb, target_hw):
    """Resize an RGB image to target (H, W) for debug overlays."""
    try:
        from PIL import Image
    except ImportError as exc:
        raise ImportError('Pillow is required for RGB resizing') from exc

    rgb = np.asarray(rgb, dtype=np.uint8)
    target_h, target_w = int(target_hw[0]), int(target_hw[1])
    resized = Image.fromarray(rgb, mode='RGB').resize(
        (target_w, target_h),
        resample=Image.Resampling.BILINEAR,
    )
    return np.asarray(resized, dtype=np.uint8)


def colorize_segmap(segmap, active_segmap_id=None):
    """Convert an integer segmap to a deterministic RGB visualization."""
    segmap = np.asarray(segmap, dtype=np.int32)
    colorized = np.zeros(segmap.shape + (3,), dtype=np.uint8)

    segment_ids = [int(seg_id) for seg_id in np.unique(segmap) if int(seg_id) > 0]
    for seg_id in segment_ids:
        # Deterministic pseudo-colors without adding a new dependency.
        color = np.array([
            (seg_id * 53) % 256,
            (seg_id * 97) % 256,
            (seg_id * 193) % 256,
        ], dtype=np.uint8)
        colorized[segmap == seg_id] = color

    if active_segmap_id is not None and int(active_segmap_id) > 0:
        target_mask = segmap == int(active_segmap_id)
        colorized[target_mask] = np.array([255, 64, 64], dtype=np.uint8)

    return colorized


def colorize_depth(depth, target_mask=None):
    """Convert depth to a viewable 8-bit RGB image."""
    depth = np.asarray(depth, dtype=np.float32)
    finite_mask = np.isfinite(depth) & (depth > 0)
    rgb = np.zeros(depth.shape + (3,), dtype=np.uint8)
    if not np.any(finite_mask):
        return rgb

    finite_depth = depth[finite_mask]
    depth_min = float(np.min(finite_depth))
    depth_max = float(np.max(finite_depth))
    if depth_max - depth_min < 1e-6:
        normalized = np.zeros_like(depth, dtype=np.float32)
    else:
        normalized = (depth - depth_min) / (depth_max - depth_min)
    normalized = np.clip(normalized, 0.0, 1.0)
    gray = (normalized * 255.0).astype(np.uint8)
    rgb = np.repeat(gray[..., None], 3, axis=2)

    if target_mask is not None:
        target_mask = np.asarray(target_mask, dtype=bool)
        rgb[target_mask] = (
            0.45 * rgb[target_mask].astype(np.float32)
            + 0.55 * np.array([255.0, 96.0, 0.0], dtype=np.float32)
        ).clip(0, 255).astype(np.uint8)

    return rgb


def compute_mask_bbox(mask, margin=12):
    """Return a padded bounding box [y0:y1, x0:x1] for a boolean mask."""
    ys, xs = np.nonzero(np.asarray(mask, dtype=bool))
    if ys.size == 0:
        return None

    y0 = max(0, int(ys.min()) - int(margin))
    y1 = int(ys.max()) + int(margin) + 1
    x0 = max(0, int(xs.min()) - int(margin))
    x1 = int(xs.max()) + int(margin) + 1
    return y0, y1, x0, x1


def maybe_align_segmap_to_depth(segmap, depth, resize_segmap_to_depth=False):
    """Warn on RGB/depth shape mismatch and optionally resize segmap to depth resolution."""
    if segmap is None:
        return None, False

    depth_hw = tuple(np.asarray(depth).shape[:2])
    segmap_hw = tuple(np.asarray(segmap).shape[:2])
    if segmap_hw == depth_hw:
        return np.asarray(segmap, dtype=np.int32), False

    print(f"  WARNING: segmap/depth shape mismatch: segmap={segmap_hw}, depth={depth_hw}")
    if not resize_segmap_to_depth:
        return np.asarray(segmap, dtype=np.int32), False

    aligned = resize_label_map_nearest(segmap, depth_hw)
    print(f"  Resized segmap to depth resolution: {aligned.shape[:2]}")
    return aligned, True


def summarize_target_depth(depth, target_mask, z_range):
    """Summarize how target-mask pixels distribute across depth validity buckets."""
    depth = np.asarray(depth, dtype=np.float32)
    target_mask = np.asarray(target_mask, dtype=bool)

    total = int(target_mask.sum())
    if total == 0:
        return {
            'total': 0,
            'zero': 0,
            'nonfinite': 0,
            'below_min': 0,
            'above_max': 0,
            'in_range': 0,
            'finite_count': 0,
            'min': None,
            'max': None,
            'median': None,
        }

    target_depth = depth[target_mask]
    zero_mask = target_depth == 0
    finite_mask = np.isfinite(target_depth)
    nonfinite_mask = ~finite_mask
    positive_finite = finite_mask & (target_depth > 0)
    below_min_mask = positive_finite & (target_depth <= float(z_range[0]))
    above_max_mask = positive_finite & (target_depth >= float(z_range[1]))
    in_range_mask = positive_finite & (target_depth > float(z_range[0])) & (target_depth < float(z_range[1]))

    finite_values = target_depth[positive_finite]
    if finite_values.size > 0:
        min_depth = float(np.min(finite_values))
        max_depth = float(np.max(finite_values))
        median_depth = float(np.median(finite_values))
    else:
        min_depth = None
        max_depth = None
        median_depth = None

    return {
        'total': total,
        'zero': int(zero_mask.sum()),
        'nonfinite': int(nonfinite_mask.sum()),
        'below_min': int(below_min_mask.sum()),
        'above_max': int(above_max_mask.sum()),
        'in_range': int(in_range_mask.sum()),
        'finite_count': int(positive_finite.sum()),
        'min': min_depth,
        'max': max_depth,
        'median': median_depth,
    }


def print_target_depth_summary(summary, prefix='  '):
    """Print a compact target-depth summary for debugging."""
    print(
        f"{prefix}Target depth stats: total={summary['total']}, zero={summary['zero']}, "
        f"nonfinite={summary['nonfinite']}, below_min={summary['below_min']}, "
        f"above_max={summary['above_max']}, in_range={summary['in_range']}"
    )
    if summary['finite_count'] > 0:
        print(
            f"{prefix}Target depth finite values: min={summary['min']:.3f}, "
            f"median={summary['median']:.3f}, max={summary['max']:.3f}"
        )


def print_execution_filter_stats(stats, prefix='  '):
    """Print compact execution-candidate rejection statistics."""
    if not stats:
        return
    print(
        f"{prefix}Execution filter stats: total={int(stats.get('total_candidates', 0))}, "
        f"score={int(stats.get('rejected_score', 0))}, "
        f"grasp_workspace={int(stats.get('rejected_grasp_workspace', 0))}, "
        f"rerank_pool={int(stats.get('rerank_pool', 0))}, "
        f"geometry={int(stats.get('rejected_geometry', 0))}, "
        f"pose_workspace={int(stats.get('rejected_pose_workspace', 0))}, "
        f"base_workspace={int(stats.get('rejected_base_workspace', 0))}, "
        f"contact_distance={int(stats.get('rejected_contact_distance', 0))}, "
        f"accepted={int(stats.get('accepted_candidates', 0))}"
    )


def print_execution_candidate_diagnostics(entries, prefix='  ', max_items=5, label='Top candidate geometry'):
    """Print compact geometry diagnostics for the top reranked candidates."""
    if not entries:
        return
    print(f"{prefix}{label}:")
    for idx, entry in enumerate(entries[:max(1, int(max_items))], start=1):
        geom = entry.get('geometry') or {}
        print(
            f"{prefix}  #{idx} seg={entry.get('segment_id')} score={float(entry.get('score', 0.0)):.3f} "
            f"fit={float(entry.get('fit_score', 0.0)):.3f} span_prio={float(entry.get('span_priority', 0.0)):.3f} "
            f"dx={float(geom.get('dx', 0.0)):.2f} dy={float(geom.get('dy', 0.0)):.2f} "
            f"local_w={float(geom.get('local_width', 0.0)):.2f} row_span={float(geom.get('row_span_ratio', 0.0)):.2f} "
            f"hcover={float(geom.get('horizontal_cover', 0.0)):.2f} line={float(geom.get('line_fraction', 0.0)):.2f} "
            f"center_t={float(geom.get('center_t', 0.5)):.2f} y={float(geom.get('normalized_y', 0.5)):.2f} "
            f"az={float(geom.get('approach_z', 0.0)):.2f} a_pref={float(geom.get('approach_score', 0.0)):.2f}"
        )


def project_camera_point_to_pixel(point_xyz, K, image_hw):
    """Project one 3D camera-frame point to integer pixel coordinates."""
    if point_xyz is None or K is None:
        return None
    point_xyz = np.asarray(point_xyz, dtype=np.float32).reshape(3)
    z = float(point_xyz[2])
    if not np.isfinite(z) or z <= 1e-6:
        return None

    K = np.asarray(K, dtype=np.float32).reshape(3, 3)
    u = float((K[0, 0] * point_xyz[0] / z) + K[0, 2])
    v = float((K[1, 1] * point_xyz[1] / z) + K[1, 2])
    width = int(image_hw[1])
    height = int(image_hw[0])
    if u < 0 or v < 0 or u >= width or v >= height:
        return None
    return int(round(u)), int(round(v))


def draw_debug_circle(image, center_xy, radius=6, color=(255, 32, 32)):
    """Draw a filled circle on an HxWx3 uint8 image."""
    if image is None or center_xy is None:
        return image
    image = np.asarray(image, dtype=np.uint8)
    cx, cy = int(center_xy[0]), int(center_xy[1])
    yy, xx = np.ogrid[:image.shape[0], :image.shape[1]]
    mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= int(radius) ** 2
    image[mask] = np.asarray(color, dtype=np.uint8)
    return image


def draw_debug_line(image, start_xy, end_xy, color=(255, 32, 32), thickness=2):
    """Draw a simple line segment on an HxWx3 uint8 image."""
    if image is None or start_xy is None or end_xy is None:
        return image
    image = np.asarray(image, dtype=np.uint8)
    x0, y0 = [int(v) for v in start_xy]
    x1, y1 = [int(v) for v in end_xy]
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    xs = np.linspace(x0, x1, steps + 1).astype(np.int32)
    ys = np.linspace(y0, y1, steps + 1).astype(np.int32)
    half = max(0, int(thickness) // 2)
    for x, y in zip(xs, ys):
        x0c = max(0, x - half)
        x1c = min(image.shape[1], x + half + 1)
        y0c = max(0, y - half)
        y1c = min(image.shape[0], y + half + 1)
        image[y0c:y1c, x0c:x1c] = np.asarray(color, dtype=np.uint8)
    return image


def draw_gripper_debug_on_image(image,
                                center_uv,
                                finger_a_uv,
                                finger_b_uv):
    """Overlay a minimal gripper visualization on one RGB-like image."""
    if image is None:
        return image
    if finger_a_uv is not None and finger_b_uv is not None:
        image = draw_debug_line(image, finger_a_uv, finger_b_uv, color=(255, 0, 255), thickness=2)
    if center_uv is not None:
        image = draw_debug_circle(image, center_uv, radius=5, color=(255, 255, 0))
    if finger_a_uv is not None:
        image = draw_debug_circle(image, finger_a_uv, radius=4, color=(255, 0, 255))
    if finger_b_uv is not None:
        image = draw_debug_circle(image, finger_b_uv, radius=4, color=(255, 0, 255))
    return image


def compute_gripper_debug_geometry(execution_preview):
    """Build a minimal camera-frame gripper visualization from one execution preview.

    Contact-GraspNet's pose origin is not a reliable proxy for the physical
    finger-closing center in image space. For debug projection and 2D gating,
    anchor the gripper on the predicted contact point instead.
    """
    if execution_preview is None:
        return None
    pose = np.asarray(execution_preview.pose, dtype=np.float32)
    if pose.shape != (4, 4):
        return None

    contact_point = getattr(execution_preview, 'contact_point', None)
    if contact_point is None:
        return None
    center = np.asarray(contact_point, dtype=np.float32).reshape(3)
    closure_axis = pose[:3, 0].astype(np.float32)

    closure_norm = float(np.linalg.norm(closure_axis))
    if closure_norm <= 1e-8:
        return None

    closure_axis = closure_axis / closure_norm

    opening = max(0.0, float(getattr(execution_preview, 'gripper_opening', 0.0)))
    half_opening = 0.5 * opening
    finger_a = center + half_opening * closure_axis
    finger_b = center - half_opening * closure_axis

    return {
        'center': center,
        'finger_a': finger_a.astype(np.float32),
        'finger_b': finger_b.astype(np.float32),
    }


def point_on_mask(mask, uv, radius=4):
    """Return True when one projected pixel lands on or very near the target mask."""
    if mask is None or uv is None:
        return False
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2:
        return False
    x = int(uv[0])
    y = int(uv[1])
    h, w = mask.shape
    if x < 0 or y < 0 or x >= w or y >= h:
        return False
    r = max(0, int(radius))
    y0 = max(0, y - r)
    y1 = min(h, y + r + 1)
    x0 = max(0, x - r)
    x1 = min(w, x + r + 1)
    return bool(np.any(mask[y0:y1, x0:x1]))


def sample_line_mask_fraction(mask, start_uv, end_uv):
    """Estimate how much of a 2D line segment lies on the target mask."""
    if mask is None or start_uv is None or end_uv is None:
        return 0.0
    mask = np.asarray(mask, dtype=bool)
    if mask.ndim != 2:
        return 0.0
    x0, y0 = [int(v) for v in start_uv]
    x1, y1 = [int(v) for v in end_uv]
    steps = max(abs(x1 - x0), abs(y1 - y0), 1)
    xs = np.linspace(x0, x1, steps + 1).astype(np.int32)
    ys = np.linspace(y0, y1, steps + 1).astype(np.int32)
    valid = (
        (xs >= 0) & (xs < mask.shape[1]) &
        (ys >= 0) & (ys < mask.shape[0])
    )
    if not np.any(valid):
        return 0.0
    return float(np.mean(mask[ys[valid], xs[valid]]))


def evaluate_execution_preview_projection(execution_preview, sensor_data, target_mask):
    """Check whether one camera-frame preview still lies on the selected target."""
    return evaluate_execution_geometry_constraints(
        execution_preview,
        sensor_data,
        target_mask,
        max_approach_z=DEFAULT_EXECUTION_CACHED_MAX_APPROACH_Z,
        enforce_approach=False,
    )


def compute_debug_execution_preview(client,
                                    sensor_data,
                                    pred_grasps_dict,
                                    scores_dict,
                                    contact_pts_dict,
                                    gripper_openings_dict,
                                    active_segmap_id,
                                    min_grasp_score=0.0,
                                    execution_frame='camera_optical_frame',
                                    pregrasp_offset=0.10,
                                    retreat_offset=0.10,
                                    max_pregrasp_offset=None,
                                    max_retreat_offset=None,
                                    execution_top_k=None,
                                    target_segment_points=None,
                                    max_contact_distance_to_target=None,
                                    execution_reference_frame='world',
                                    execution_base_position_world=None,
                                    execution_base_yaw_deg=0.0,
                                    execution_base_workspace_bounds=None,
                                    execution_camera_translation_gripper=None,
                                    execution_camera_quaternion_gripper=None,
                                    target_mask=None,
                                    end_pose_is_camera_pose: bool = False):
    """Build the locally selected execution candidate without sending it."""
    if sensor_data is None:
        return None, None
    end_pose_world = sensor_data.end_pose
    reference_frame = sensor_data.end_pose_frame or execution_reference_frame
    if (
        execution_base_workspace_bounds is not None
        and execution_base_position_world is not None
        and end_pose_world is None
    ):
        return None, None

    execution, execution_meta = select_best_execution_candidate(
        client,
        pred_grasps_dict,
        scores_dict,
        contact_pts_dict,
        gripper_openings_dict,
        min_score=min_grasp_score,
        frame=execution_frame,
        pregrasp_offset=pregrasp_offset,
        retreat_offset=retreat_offset,
        max_pregrasp_offset=max_pregrasp_offset,
        max_retreat_offset=max_retreat_offset,
        workspace_bounds=None,
        top_k=execution_top_k,
        preferred_seg_id=active_segmap_id,
        end_pose_world=end_pose_world,
        planning_frame=execution_reference_frame,
        reference_frame=reference_frame,
        base_position_world=execution_base_position_world,
        base_yaw_deg=execution_base_yaw_deg,
        base_workspace_bounds=execution_base_workspace_bounds,
        camera_translation_gripper=execution_camera_translation_gripper,
        camera_quaternion_gripper=execution_camera_quaternion_gripper,
        sensor_data=sensor_data,
        target_mask=target_mask,
        target_segment_points=target_segment_points,
        max_contact_distance_to_target=max_contact_distance_to_target,
        end_pose_is_camera_pose=end_pose_is_camera_pose,
    )

    return execution, execution_meta


def maybe_promote_global_execution_fallback(grasp_estimator,
                                            client,
                                            sensor_data,
                                            z_range,
                                            current_execution,
                                            current_execution_meta,
                                            active_segmap_id,
                                            min_grasp_score=0.0,
                                            execution_frame='camera_optical_frame',
                                            pregrasp_offset=0.10,
                                            retreat_offset=0.10,
                                            max_pregrasp_offset=None,
                                            max_retreat_offset=None,
                                            execution_top_k=None,
                                            target_segment_points=None,
                                            max_contact_distance_to_target=None,
                                            execution_reference_frame='world',
                                            execution_base_position_world=None,
                                            execution_base_yaw_deg=0.0,
                                            execution_base_workspace_bounds=None,
                                            execution_camera_translation_gripper=None,
                                            execution_camera_quaternion_gripper=None,
                                            target_mask=None,
                                            forward_passes=1,
                                            skip_border_objects=False,
                                            margin_px=5,
                                            end_pose_is_camera_pose: bool = False):
    """Rerun on the full scene when the target-segment candidate pool is too narrow."""
    if grasp_estimator is None or sensor_data is None or target_mask is None:
        return current_execution, current_execution_meta

    current_span = -1.0 if current_execution_meta is None else float(current_execution_meta.get('span_priority', -1.0))
    if current_execution is not None and current_span >= DEFAULT_EXECUTION_GLOBAL_FALLBACK_MIN_SPAN_PRIORITY:
        return current_execution, current_execution_meta

    _, _, _, global_pred_grasps_dict, global_scores_dict, global_contact_pts_dict, global_gripper_openings_dict = predict_scene_candidates(
        grasp_estimator,
        sensor_data,
        z_range,
        segmap=None,
        use_segmap=False,
        segmap_id=0,
        local_regions=False,
        filter_grasps=False,
        use_cam_boxes=False,
        filter_grasps_threshold=None,
        skip_border_objects=skip_border_objects,
        margin_px=margin_px,
        forward_passes=forward_passes,
    )

    fallback_execution, fallback_meta = compute_debug_execution_preview(
        client,
        sensor_data,
        global_pred_grasps_dict,
        global_scores_dict,
        global_contact_pts_dict,
        global_gripper_openings_dict,
        active_segmap_id,
        min_grasp_score=min_grasp_score,
        execution_frame=execution_frame,
        pregrasp_offset=pregrasp_offset,
        retreat_offset=retreat_offset,
        max_pregrasp_offset=max_pregrasp_offset,
        max_retreat_offset=max_retreat_offset,
        execution_top_k=execution_top_k,
        target_segment_points=target_segment_points,
        max_contact_distance_to_target=max_contact_distance_to_target,
        execution_reference_frame=execution_reference_frame,
        execution_base_position_world=execution_base_position_world,
        execution_base_yaw_deg=execution_base_yaw_deg,
        execution_base_workspace_bounds=execution_base_workspace_bounds,
        execution_camera_translation_gripper=execution_camera_translation_gripper,
        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
        target_mask=target_mask,
        end_pose_is_camera_pose=end_pose_is_camera_pose,
    )

    if fallback_execution is None or fallback_meta is None:
        return current_execution, current_execution_meta

    fallback_span = float(fallback_meta.get('span_priority', 0.0))
    fallback_fit = float(fallback_meta.get('fit_score', 0.0))
    current_fit = -1.0 if current_execution_meta is None else float(current_execution_meta.get('fit_score', -1.0))

    if current_execution is None:
        print(
            f"  Using global execution fallback: span_priority={fallback_span:.3f}, "
            f"fit_score={fallback_fit:.3f}"
        )
        return fallback_execution, fallback_meta

    better_span = fallback_span > (current_span + DEFAULT_EXECUTION_GLOBAL_FALLBACK_MIN_SPAN_IMPROVEMENT)
    better_fit = fallback_fit > (current_fit + DEFAULT_EXECUTION_GLOBAL_FALLBACK_MIN_FIT_IMPROVEMENT)
    if better_span or (fallback_span > current_span and better_fit):
        print(
            f"  Promoting global execution fallback: current_span={current_span:.3f}, "
            f"fallback_span={fallback_span:.3f}, current_fit={current_fit:.3f}, "
            f"fallback_fit={fallback_fit:.3f}"
        )
        return fallback_execution, fallback_meta

    return current_execution, current_execution_meta


def save_segmentation_debug(sensor_data,
                            segmap,
                            active_segmap_id,
                            z_range,
                            debug_save_dir,
                            target_label,
                            target_selection=None,
                            execution_preview=None):
    """Save RGB/depth alignment debug overlays for the selected target segment."""
    if not debug_save_dir or segmap is None or active_segmap_id is None or active_segmap_id <= 0:
        return

    os.makedirs(debug_save_dir, exist_ok=True)

    try:
        from PIL import Image
    except ImportError as exc:
        raise ImportError('Pillow is required for --debug-save-dir overlays') from exc

    frame_dir = os.path.join(debug_save_dir, f'frame_{int(sensor_data.frame_id):06d}')
    os.makedirs(frame_dir, exist_ok=True)

    rgb = sensor_data.rgb
    if rgb is None:
        depth = np.asarray(sensor_data.depth, dtype=np.float32)
        finite_depth = np.where(np.isfinite(depth), depth, 0.0)
        max_depth = np.max(finite_depth) if np.max(finite_depth) > 0 else 1.0
        gray = np.clip((finite_depth / max_depth) * 255.0, 0, 255).astype(np.uint8)
        rgb = np.repeat(gray[..., None], 3, axis=2)
    else:
        rgb = np.asarray(rgb, dtype=np.uint8).copy()

    if rgb.shape[:2] != np.asarray(segmap).shape[:2]:
        print(f"  Debug note: resizing RGB for overlay from {rgb.shape[:2]} to {np.asarray(segmap).shape[:2]}")
        rgb_overlay = resize_rgb_image(rgb, np.asarray(segmap).shape[:2])
    else:
        rgb_overlay = rgb

    target_mask = np.asarray(segmap == active_segmap_id, dtype=bool)
    valid_depth_mask = compute_valid_depth_mask(sensor_data.depth, z_range)
    overlap_mask = target_mask & valid_depth_mask
    segmap_rgb = colorize_segmap(segmap, active_segmap_id=active_segmap_id)
    depth_rgb = colorize_depth(sensor_data.depth, target_mask=target_mask)

    overlay = rgb_overlay.astype(np.float32)
    overlay[target_mask] = 0.55 * overlay[target_mask] + 0.45 * np.array([0.0, 255.0, 0.0], dtype=np.float32)
    overlay[valid_depth_mask] = 0.65 * overlay[valid_depth_mask] + 0.35 * np.array([0.0, 120.0, 255.0], dtype=np.float32)
    overlay[overlap_mask] = 0.40 * overlay[overlap_mask] + 0.60 * np.array([255.0, 220.0, 0.0], dtype=np.float32)
    overlay = np.clip(overlay, 0, 255).astype(np.uint8)
    crop_overlay = rgb_overlay.copy()

    if execution_preview is not None and str(execution_preview.frame) == 'camera_optical_frame':
        debug_geom = compute_gripper_debug_geometry(execution_preview)
        if debug_geom is not None:
            image_hw = np.asarray(segmap).shape[:2]
            center_uv = project_camera_point_to_pixel(
                debug_geom['center'],
                sensor_data.K,
                image_hw,
            )
            finger_a_uv = project_camera_point_to_pixel(
                debug_geom['finger_a'],
                sensor_data.K,
                image_hw,
            )
            finger_b_uv = project_camera_point_to_pixel(
                debug_geom['finger_b'],
                sensor_data.K,
                image_hw,
            )

            overlay = draw_gripper_debug_on_image(
                overlay, center_uv, finger_a_uv, finger_b_uv
            )
            crop_overlay = draw_gripper_debug_on_image(
                crop_overlay, center_uv, finger_a_uv, finger_b_uv
            )
            depth_rgb = draw_gripper_debug_on_image(
                depth_rgb, center_uv, finger_a_uv, finger_b_uv
            )
            segmap_rgb = draw_gripper_debug_on_image(
                segmap_rgb, center_uv, finger_a_uv, finger_b_uv
            )

    frame_tag = f'frame_{int(sensor_data.frame_id):06d}'
    target_tag = str(target_label).replace('/', '_').replace(' ', '_')
    base_name = f'{target_tag}_seg{int(active_segmap_id)}'

    Image.fromarray(rgb).save(os.path.join(frame_dir, f'{base_name}_rgb.png'))
    Image.fromarray(rgb_overlay).save(os.path.join(frame_dir, f'{base_name}_rgb_aligned.png'))
    Image.fromarray(segmap_rgb).save(os.path.join(frame_dir, f'{base_name}_segmap.png'))
    Image.fromarray(depth_rgb).save(os.path.join(frame_dir, f'{base_name}_depth.png'))
    Image.fromarray(overlay).save(os.path.join(frame_dir, f'{base_name}_overlay.png'))
    Image.fromarray((target_mask.astype(np.uint8) * 255)).save(os.path.join(frame_dir, f'{base_name}_target_mask.png'))
    Image.fromarray((valid_depth_mask.astype(np.uint8) * 255)).save(os.path.join(frame_dir, f'{base_name}_valid_depth.png'))
    target_pixels = int(target_mask.sum())
    valid_depth_pixels = int(valid_depth_mask.sum())
    overlap_pixels = int(overlap_mask.sum())
    valid_ratio = float(overlap_pixels / target_pixels) if target_pixels > 0 else 0.0
    target_depth_summary = summarize_target_depth(sensor_data.depth, target_mask, z_range)
    bbox = compute_mask_bbox(target_mask)

    if bbox is not None:
        y0, y1, x0, x1 = bbox
        rgb_crop = crop_overlay[y0:y1, x0:x1].copy()
        segmap_crop = segmap_rgb[y0:y1, x0:x1].copy()
        depth_crop = depth_rgb[y0:y1, x0:x1].copy()
        Image.fromarray(rgb_crop).save(
            os.path.join(frame_dir, f'{base_name}_target_rgb_crop.png')
        )
        Image.fromarray(segmap_crop).save(
            os.path.join(frame_dir, f'{base_name}_target_segmap_crop.png')
        )
        Image.fromarray(depth_crop).save(
            os.path.join(frame_dir, f'{base_name}_target_depth_crop.png')
        )

    print(f"  Debug saved: {os.path.join(frame_dir, f'{base_name}_overlay.png')}")
    print(
        f"  Target mask pixels: {target_pixels}, valid depth pixels: {valid_depth_pixels}, "
        f"overlap pixels: {overlap_pixels}, target valid ratio: {valid_ratio:.3f}"
    )
    print_target_depth_summary(target_depth_summary)
    if target_selection is not None:
        print(f"  Target selector score snapshot: {target_selection.score:.3f}")


def flatten_predictions(pred_grasps_dict, scores_dict, contact_pts_dict, gripper_openings_dict):
    """Flatten segmented predictions for visualization/debugging."""
    flat_grasps, flat_scores, flat_contacts, flat_openings = [], [], [], []

    for seg_id, poses in pred_grasps_dict.items():
        poses = np.asarray(poses)
        if poses.size == 0:
            continue
        if poses.ndim == 2:
            poses = poses[np.newaxis, ...]

        seg_scores = np.atleast_1d(np.asarray(scores_dict[seg_id]))
        seg_contacts = np.asarray(contact_pts_dict[seg_id])
        seg_openings = np.atleast_1d(np.asarray(gripper_openings_dict[seg_id]))

        if seg_contacts.ndim == 1 and seg_contacts.size > 0:
            seg_contacts = seg_contacts[np.newaxis, ...]

        count = min(len(poses), len(seg_scores), len(seg_openings))
        if seg_contacts.size > 0:
            count = min(count, len(seg_contacts))

        if count == 0:
            continue

        flat_grasps.append(poses[:count])
        flat_scores.append(seg_scores[:count])
        if seg_contacts.size > 0:
            flat_contacts.append(seg_contacts[:count])
        flat_openings.append(seg_openings[:count])

    if not flat_grasps:
        return (
            np.empty((0, 4, 4), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
            np.empty((0, 3), dtype=np.float32),
            np.empty((0,), dtype=np.float32),
        )

    contacts = np.concatenate(flat_contacts, axis=0) if flat_contacts else np.empty((0, 3), dtype=np.float32)
    return (
        np.concatenate(flat_grasps, axis=0),
        np.concatenate(flat_scores, axis=0),
        contacts,
        np.concatenate(flat_openings, axis=0),
    )


def resolve_segmap(sensor_data, use_segmap, segmentation_source, segmenter):
    """Resolve the segmentation map used for object-wise grasp generation."""
    if not use_segmap:
        return None, 'disabled'

    if segmentation_source == 'remote':
        if sensor_data.segmap is None:
            raise ValueError('Remote segmap requested but sensor_data.segmap is empty')
        return np.asarray(sensor_data.segmap, dtype=np.int32), 'remote'

    if segmentation_source == 'sam':
        if segmenter is None:
            raise ValueError('SAM segmentation requested but no local segmenter is configured')
        segmap = segmenter.generate(sensor_data.rgb)
        return segmap, segmenter.describe()

    raise ValueError(f'Unsupported segmentation source: {segmentation_source}')


def resolve_target_segment(sensor_data,
                           segmap,
                           explicit_segmap_id=0,
                           target_query='',
                           target_selector=None,
                           target_min_score=None,
                           target_min_mask_area=400,
                           candidate_segment_ids=None):
    """Choose a single target segment either explicitly or from a text query."""
    if explicit_segmap_id and explicit_segmap_id > 0:
        return int(explicit_segmap_id), f'manual:{int(explicit_segmap_id)}', None

    if not target_query:
        return 0, 'all_segments', None

    if segmap is None:
        raise ValueError('Target query requires a valid segmap')
    if target_selector is None:
        raise ValueError('Target query requires a configured target selector')

    selection = target_selector.select(
        sensor_data.rgb,
        segmap,
        query=target_query,
        candidate_ids=candidate_segment_ids,
        min_score=target_min_score,
        min_mask_area=target_min_mask_area,
    )
    if selection is None:
        return None, f'query:{target_query}', None

    return selection.segment_id, f'{target_query}->{selection.segment_id}', selection


def predict_scene_candidates(grasp_estimator,
                             sensor_data,
                             z_range,
                             segmap=None,
                             use_segmap=False,
                             segmap_id=0,
                             local_regions=False,
                             filter_grasps=False,
                             use_cam_boxes=True,
                             filter_grasps_threshold=None,
                             skip_border_objects=False,
                             margin_px=5,
                             forward_passes=1):
    """Build point clouds and predict grasp candidates for a frame."""
    pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(
        sensor_data.depth,
        sensor_data.K,
        segmap=segmap if use_segmap else None,
        rgb=sensor_data.rgb,
        z_range=z_range,
        segmap_id=segmap_id,
        skip_border_objects=skip_border_objects,
        margin_px=margin_px,
    )

    if pc_full.shape[0] == 0:
        return pc_full, pc_segments, pc_colors, {}, {}, {}, {}

    if use_segmap:
        if segmap is None or not pc_segments:
            return pc_full, pc_segments, pc_colors, {}, {}, {}, {}

        old_filter_thres = grasp_estimator._contact_grasp_cfg['TEST'].get('filter_thres')
        if filter_grasps_threshold is not None:
            grasp_estimator._contact_grasp_cfg['TEST']['filter_thres'] = float(filter_grasps_threshold)
        try:
            pred_grasps, scores, contact_pts, gripper_openings = grasp_estimator.predict_scene_grasps(
                pc_full,
                pc_segments=pc_segments,
                local_regions=local_regions,
                filter_grasps=filter_grasps,
                forward_passes=forward_passes,
                use_cam_boxes=use_cam_boxes,
            )
        finally:
            if filter_grasps_threshold is not None:
                grasp_estimator._contact_grasp_cfg['TEST']['filter_thres'] = old_filter_thres
    else:
        pred_grasps, scores, contact_pts, gripper_openings = grasp_estimator.predict_grasps(
            pc_full,
            forward_passes=forward_passes,
        )
        pred_grasps = {-1: pred_grasps}
        scores = {-1: scores}
        contact_pts = {-1: contact_pts}
        gripper_openings = {-1: gripper_openings}

    return pc_full, pc_segments, pc_colors, pred_grasps, scores, contact_pts, gripper_openings


def load_model(ckpt_dir: str, batch_size: int = 1):
    """Load Contact-GraspNet model"""
    global_config = config_utils.load_config(ckpt_dir, batch_size=batch_size)
    grasp_estimator = GraspEstimator(global_config)

    model_checkpoint_dir = os.path.join(ckpt_dir, 'checkpoints')
    checkpoint_io = CheckpointIO(checkpoint_dir=model_checkpoint_dir, model=grasp_estimator.model)

    try:
        load_dict = checkpoint_io.load('model.pt')
        print("Model weights loaded successfully")
    except Exception as e:
        print(f"Warning: Could not load model weights: {e}")

    return grasp_estimator


def create_gripper_mesh():
    """Create a simple gripper mesh for visualization"""
    # Create gripper fingers as boxes
    finger_length = 0.08
    finger_width = 0.01
    finger_height = 0.02

    # Two fingers
    mesh1 = o3d.geometry.TriangleMesh.create_box(finger_width, finger_height, finger_length)
    mesh2 = o3d.geometry.TriangleMesh.create_box(finger_width, finger_height, finger_length)

    mesh1.translate([0, 0, -finger_length/2])
    mesh2.translate([0, 0, -finger_length/2])

    gripper = mesh1 + mesh2
    gripper.compute_vertex_normals()

    return gripper


def visualize_grasps_o3d(pc_full, pred_grasps, scores, pc_colors=None, num_vis=10):
    """
    Visualize point cloud and grasp poses using Open3D

    Args:
        pc_full: Nx3 point cloud
        pred_grasps: Nx4x4 grasp poses
        scores: N confidence scores
        pc_colors: Nx3 point cloud colors (optional)
        num_vis: Number of grasps to visualize
    """
    # Create point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_full)
    if pc_colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(pc_colors / 255.0)
    else:
        pcd.paint_uniform_color([0.5, 0.5, 0.5])

    # Create grasp visualizations (top N)
    grasp_meshes = []
    best_indices = np.argsort(scores)[::-1][:min(num_vis, len(scores))]

    gripper_template = create_gripper_mesh()

    for i, idx in enumerate(best_indices):
        pose = pred_grasps[idx]
        score = scores[idx]

        # Copy gripper mesh and transform
        gripper_mesh = o3d.geometry.TriangleMesh(gripper_template)
        gripper_mesh.transform(pose)

        # Color based on score (green=high, red=low)
        color = [score, 1.0 - score, 0.0]  # R,G,B based on confidence
        gripper_mesh.paint_uniform_color(color)
        gripper_mesh.compute_vertex_normals()

        grasp_meshes.append(gripper_mesh)

    # Visualize
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='Contact-GraspNet Visualization')

    vis.add_geometry(pcd)
    for mesh in grasp_meshes:
        vis.add_geometry(mesh)

    # Set view
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -1])
    ctr.set_up([0, -1, 0])

    print(f"Visualizing {len(grasp_meshes)} grasps (green=high confidence, red=low)")
    print("Press Q or close window to continue...")

    vis.run()
    vis.destroy_window()


def send_prediction_result(client,
                           pred_grasps,
                           scores,
                           contact_pts,
                           gripper_openings,
                           frame_id,
                           sensor_data=None,
                           execute_best_grasp=False,
                           min_grasp_score=0.0,
                           execution_frame='camera_optical_frame',
                           pregrasp_offset=0.10,
                           retreat_offset=0.10,
                           max_pregrasp_offset=None,
                           max_retreat_offset=None,
                           workspace_bounds=None,
                           execution_top_k=None,
                           preferred_segment_id=None,
                           target_segment_points=None,
                           max_contact_distance_to_target=None,
                           execution_reference_frame='world',
                           execution_base_position_world=None,
                           execution_base_yaw_deg=0.0,
                           execution_base_workspace_bounds=None,
                           execution_camera_translation_gripper=None,
                           execution_camera_quaternion_gripper=None,
                           target_mask=None,
                           override_execution=None,
                           status='success',
                           message='',
                           end_pose_is_camera_pose: bool = False):
    """Send raw predictions, and optionally an executable best-grasp command."""
    pred_grasps_dict = normalize_prediction_dict(pred_grasps)
    scores_dict = normalize_prediction_dict(scores)
    contact_pts_dict = normalize_prediction_dict(contact_pts)
    gripper_openings_dict = normalize_prediction_dict(gripper_openings)

    execution = None
    execution_to_send = None
    execution_blocked_reason = None
    execution_stats = None
    execution_selected_by = None
    if execute_best_grasp:
        latest_sensor_data = client.get_latest_data()
        busy_sensor_data = latest_sensor_data if latest_sensor_data is not None else sensor_data
        if busy_sensor_data is not None and getattr(busy_sensor_data, 'execution_busy', False):
            busy_status = getattr(busy_sensor_data, 'execution_status', '') or 'busy'
            print(f"  Skip execution: remote executor busy ({busy_status})")
            execution_blocked_reason = f'remote executor busy ({busy_status})'
            execute_best_grasp = False

    if execute_best_grasp:
        require_reachability = (
            execution_base_workspace_bounds is not None
            and execution_base_position_world is not None
        )
        end_pose_world = None if sensor_data is None else sensor_data.end_pose
        reference_frame = execution_reference_frame if sensor_data is None else (
            sensor_data.end_pose_frame or execution_reference_frame
        )

        if require_reachability and end_pose_world is None:
            print("  Skip execution: sensor data missing end_pose required for local reachability filter")
            execution_blocked_reason = 'sensor data missing end_pose required for local reachability filter'
        elif override_execution is not None:
            execution = override_execution
            execution_to_send = override_execution
            execution_selected_by = 'override'
        else:
            execution, execution_meta = select_best_execution_candidate(
                client,
                pred_grasps_dict,
                scores_dict,
                contact_pts_dict,
                gripper_openings_dict,
                min_score=min_grasp_score,
                frame=execution_frame,
                pregrasp_offset=pregrasp_offset,
                retreat_offset=retreat_offset,
                max_pregrasp_offset=max_pregrasp_offset,
                max_retreat_offset=max_retreat_offset,
                workspace_bounds=workspace_bounds,
                top_k=execution_top_k,
                preferred_seg_id=preferred_segment_id,
                end_pose_world=end_pose_world,
                planning_frame=execution_reference_frame,
                reference_frame=reference_frame,
                base_position_world=execution_base_position_world,
                base_yaw_deg=execution_base_yaw_deg,
                base_workspace_bounds=execution_base_workspace_bounds,
                camera_translation_gripper=execution_camera_translation_gripper,
                camera_quaternion_gripper=execution_camera_quaternion_gripper,
                sensor_data=sensor_data,
                target_mask=target_mask,
                target_segment_points=target_segment_points,
                max_contact_distance_to_target=max_contact_distance_to_target,
                end_pose_is_camera_pose=end_pose_is_camera_pose,
            )
            execution_stats = None if execution_meta is None else execution_meta.get('stats')
            if execution is not None:
                execution_selected_by = 'strict'
            if execution is None:
                print(f"  No execution candidate passed local score/workspace/reachability filters (min_score={min_grasp_score:.3f})")
                print_execution_filter_stats(execution_stats)
                if execution_meta is not None:
                    print_execution_candidate_diagnostics(
                        execution_meta.get('top_candidate_diagnostics'),
                        label='Top reranked candidate geometry',
                    )
                    print_execution_candidate_diagnostics(
                        execution_meta.get('accepted_candidate_diagnostics'),
                        label='Accepted candidate geometry',
                    )
                execution_blocked_reason = (
                    f'no execution candidate passed local score/workspace/reachability filters '
                    f'(min_score={min_grasp_score:.3f})'
                )
            else:
                print(f"  Execution grasp score: {execution.score:.3f} (segment={execution.segment_id})")
                if execution_meta is not None and execution_meta.get('fit_score') is not None:
                    print(f"  Execution fit score: {float(execution_meta['fit_score']):.3f}")
                if execution_meta is not None and execution_meta.get('span_priority') is not None:
                    print(f"  Execution span priority: {float(execution_meta['span_priority']):.3f}")
                if execution_meta is not None:
                    print_execution_candidate_diagnostics(
                        [
                            {
                                'segment_id': execution.segment_id,
                                'score': float(execution.score),
                                'fit_score': float(execution_meta.get('fit_score', 0.0)),
                                'span_priority': float(execution_meta.get('span_priority', 0.0)),
                                'geometry': execution_meta.get('selected_geometry'),
                            }
                        ],
                        label='Selected execution geometry',
                        max_items=1,
                    )
                if execution_selected_by == 'override':
                    print("  Execution selection: cached override")
                # Print original execution pose in optical frame
                if execution is not None:
                    print(f"  Execution grasp pose (optical): pos={np.round(execution.pose[:3, 3], 4).tolist()}")
                print_execution_filter_stats(execution_stats)
                if execution is not None:
                    execution_to_send = freeze_execution_to_planning_frame(
                        client,
                        execution,
                        end_pose_world=end_pose_world,
                        execution_reference_frame=execution_reference_frame,
                        reference_frame=reference_frame,
                        execution_camera_translation_gripper=execution_camera_translation_gripper,
                        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                    )
                    # Print converted world pose
                    if execution_to_send is not None:
                        print(f"  Execution grasp pose (world): pos={np.round(execution_to_send.pose[:3, 3], 4).tolist()}")

    if execution is None and execution_blocked_reason is not None:
        status = 'no_grasp'
        if not message or message.startswith('Generated '):
            message = execution_blocked_reason

    client.send_grasp_from_prediction(
        pred_grasps_dict,
        scores_dict,
        contact_pts_dict,
        gripper_openings_dict,
        frame_id=frame_id,
        status=status,
        message=message,
        execution=execution_to_send,
    )

    return execution


def run_single_inference(grasp_estimator, client, z_range, visualize=True,
                         execute_best_grasp=False, min_grasp_score=0.0,
                         execution_frame='camera_optical_frame', pregrasp_offset=0.10,
                         retreat_offset=0.10, max_pregrasp_offset=None,
                         max_retreat_offset=None, timeout_s=30.0,
                         segmentation_source='sam', segmenter=None,
                         target_query='', target_selector=None,
                         target_min_score=None, target_min_mask_area=400,
                         debug_save_dir='', debug_save_every=1,
                         resize_segmap_to_depth=False,
                         use_segmap=False, segmap_id=0, local_regions=False,
                         filter_grasps=False, use_cam_boxes=True,
                         filter_grasps_threshold=None,
                         skip_border_objects=False,
                         margin_px=5, forward_passes=1,
                         workspace_bounds=None, execution_top_k=None,
                         target_min_points=0,
                         max_contact_distance_to_target=None,
                         execution_reference_frame='world',
                         execution_base_position_world=None,
                         execution_base_yaw_deg=0.0,
                         execution_base_workspace_bounds=None,
                         execution_camera_translation_gripper=None,
                         execution_camera_quaternion_gripper=None,
                         depth_preprocess=DEFAULT_DEPTH_PREPROCESS_ENABLED,
                         end_pose_is_camera_pose: bool = False):
    """
    Run inference on a single frame

    Returns:
        Dictionary with results or None if no data
    """
    sensor_data = client.wait_for_data(timeout_s)

    if sensor_data is None:
        return None

    sensor_data.depth, depth_preprocess_stats = preprocess_depth_for_inference(
        sensor_data.depth,
        z_range,
        enabled=depth_preprocess,
    )

    print(f"Received frame {sensor_data.frame_id}")
    print(f"  Depth shape: {sensor_data.depth.shape}")
    print_depth_preprocess_stats(depth_preprocess_stats)
    if sensor_data.rgb is not None:
        print(f"  RGB shape: {sensor_data.rgb.shape}")
        if sensor_data.rgb.shape[:2] != sensor_data.depth.shape[:2]:
            print("  WARNING: RGB/depth resolution mismatch detected")

    try:
        segmap, segmap_backend = resolve_segmap(
            sensor_data,
            use_segmap=use_segmap,
            segmentation_source=segmentation_source,
            segmenter=segmenter,
        )
    except Exception as exc:
        print(f"  Segmentation error: {exc}")
        client.send_grasp_from_prediction(
            {}, {}, {}, {},
            frame_id=sensor_data.frame_id,
            status='error',
            message=f'Segmentation failed: {exc}'
        )
        return None

    segmap_aligned = segmap
    segmap_resized = False
    if use_segmap and segmap is not None:
        segmap_aligned, segmap_resized = maybe_align_segmap_to_depth(
            segmap,
            sensor_data.depth,
            resize_segmap_to_depth=resize_segmap_to_depth,
        )
        segmap_aligned = complete_fragmented_segments(segmap_aligned, sensor_data.depth)
        print(f"  Segmap shape: {segmap.shape}")
        if segmap_resized:
            print(f"  Aligned segmap shape: {segmap_aligned.shape}")

    selection_candidate_ids = None
    if use_segmap and segmap_aligned is not None:
        self_mask = build_normalized_rect_mask(segmap_aligned.shape[:2], DEFAULT_SELF_EXCLUSION_RECTS)
        excluded_segment_ids = compute_excluded_segment_ids(
            segmap_aligned,
            self_mask,
            overlap_ratio_threshold=DEFAULT_SELF_EXCLUSION_OVERLAP_RATIO,
        )
        selection_candidate_ids = compute_candidate_segment_ids(segmap_aligned, excluded_segment_ids)

    try:
        active_segmap_id, target_label, target_selection = resolve_target_segment(
            sensor_data,
            segmap_aligned,
            explicit_segmap_id=segmap_id,
            target_query=target_query,
            target_selector=target_selector,
            target_min_score=target_min_score,
            target_min_mask_area=target_min_mask_area,
            candidate_segment_ids=selection_candidate_ids,
        )
    except Exception as exc:
        print(f"  Target selection error: {exc}")
        client.send_grasp_from_prediction(
            {}, {}, {}, {},
            frame_id=sensor_data.frame_id,
            status='error',
            message=f'Target selection failed: {exc}'
        )
        return None

    if (
        use_segmap
        and segmap_aligned is not None
        and target_query
        and active_segmap_id is not None
        and active_segmap_id > 0
    ):
        instance_cluster_ids = collect_instance_companion_segments(
            segmap_aligned,
            sensor_data.depth,
            active_segmap_id,
            candidate_segment_ids=selection_candidate_ids,
        )
        if len(instance_cluster_ids) > 1:
            segmap_aligned = merge_selected_instance_segments(
                segmap_aligned,
                active_segmap_id,
                instance_cluster_ids,
            )

    if target_query and active_segmap_id is None:
        selection_debug = get_target_selection_debug_info(target_selector)
        print(f"  Target '{target_query}' not found in current frame")
        print_target_selection_debug_info(selection_debug)
        rejected_seg_id = selection_debug.get('best_segment_id')
        if (
            debug_save_dir
            and (sensor_data.frame_id % max(1, debug_save_every) == 0)
            and segmap_aligned is not None
            and rejected_seg_id is not None
            and int(rejected_seg_id) > 0
        ):
            save_segmentation_debug(
                sensor_data,
                segmap_aligned,
                int(rejected_seg_id),
                z_range,
                debug_save_dir,
                f"{target_query}->rejected",
                target_selection=None,
                execution_preview=None,
            )
        client.send_grasp_from_prediction(
            {}, {}, {}, {},
            frame_id=sensor_data.frame_id,
            status='no_grasp',
            message=f"Target '{target_query}' not found in current frame"
        )
        return None

    target_depth_cleanup_info = None
    if use_segmap and segmap_aligned is not None and active_segmap_id is not None and active_segmap_id > 0:
        raw_target_mask = np.asarray(segmap_aligned == active_segmap_id, dtype=bool)
        cleaned_target_mask, target_depth_cleanup_info = refine_target_mask_by_depth(
            raw_target_mask,
            sensor_data.depth,
            z_range,
        )
        if target_depth_cleanup_info is not None:
            print_target_depth_cleanup_info(target_depth_cleanup_info)
        if bool(target_depth_cleanup_info and target_depth_cleanup_info.get('applied')):
            segmap_aligned = apply_cleaned_target_mask(
                segmap_aligned,
                active_segmap_id,
                cleaned_target_mask,
            )

    prediction_segmap_id = active_segmap_id or 0
    if should_refine_target_with_reachability(use_segmap, segmap_id, target_query):
        prediction_segmap_id = 0

    # Extract point clouds
    pc_full, pc_segments, pc_colors, pred_grasps_dict, scores_dict, contact_pts_dict, gripper_openings_dict = predict_scene_candidates(
        grasp_estimator,
        sensor_data,
        z_range,
        segmap=segmap_aligned,
        use_segmap=use_segmap,
        segmap_id=prediction_segmap_id,
        local_regions=local_regions,
        filter_grasps=filter_grasps,
        use_cam_boxes=use_cam_boxes,
        filter_grasps_threshold=filter_grasps_threshold,
        skip_border_objects=skip_border_objects,
        margin_px=margin_px,
        forward_passes=forward_passes,
    )

    print(f"  Point cloud: {pc_full.shape[0]} points")
    if use_segmap:
        seg_count = len(np.unique(segmap_aligned[segmap_aligned > 0])) if segmap_aligned is not None else 0
        print(f"  Segmentation backend: {segmap_backend}")
        print(f"  Segments in segmap: {seg_count}")
        print(f"  Segments with points: {len(pc_segments)}")
        print(f"  Target selection: {target_label}")
        if target_selection is not None:
            print(f"  Target score: {target_selection.score:.3f}")
            print(
                "  Target semantics: "
                f"masked={float(getattr(target_selection, 'masked_score', 0.0)):.3f}, "
                f"context={float(getattr(target_selection, 'context_score', 0.0)):.3f}, "
                f"distractor={float(getattr(target_selection, 'distractor_score', 0.0)):.3f}, "
                f"agreement={float(getattr(target_selection, 'agreement_score', 0.0)):.3f}, "
                f"advantage={float(getattr(target_selection, 'query_advantage', 0.0)):.3f}, "
                f"margin={float(getattr(target_selection, 'selection_margin', 0.0)):.3f}"
            )
        print(f"  Segmap resized to depth: {segmap_resized}")
        target_segment_points = get_segment_point_count(pc_segments, active_segmap_id)
        if target_segment_points is not None:
            print(f"  Target segment points: {target_segment_points}")
        if segmap_aligned is not None and active_segmap_id is not None and active_segmap_id > 0:
            target_mask = np.asarray(segmap_aligned == active_segmap_id, dtype=bool)
            target_depth_summary = summarize_target_depth(sensor_data.depth, target_mask, z_range)
            print_target_depth_summary(target_depth_summary)

    if pc_full.shape[0] == 0:
        print("  No points in range, skipping")
        client.send_grasp_from_prediction(
            {}, {}, {}, {},
            frame_id=sensor_data.frame_id,
            status='no_grasp',
            message='No points in configured z-range'
        )
        return None

    if use_segmap and not pc_segments:
        print("  No valid segmented objects, skipping")
        client.send_grasp_from_prediction(
            {}, {}, {}, {},
            frame_id=sensor_data.frame_id,
            status='no_grasp',
            message='No valid segmented objects after local segmentation/filtering'
        )
        return None

    if should_refine_target_with_reachability(use_segmap, segmap_id, target_query):
        if not target_selection_is_semantically_confident(target_selection):
            print(
                "  Skipping target refinement due to weak semantic confidence: "
                f"agreement={float(getattr(target_selection, 'agreement_score', 0.0)):.3f}, "
                f"advantage={float(getattr(target_selection, 'query_advantage', 0.0)):.3f}, "
                f"margin={float(getattr(target_selection, 'selection_margin', 0.0)):.3f}"
            )
        else:
            preferred_target = choose_execution_preferred_target_segment(
                client,
                sensor_data,
                target_selection,
                pc_segments,
                segmap_aligned,
                pred_grasps_dict,
                scores_dict,
                contact_pts_dict,
                gripper_openings_dict,
                min_grasp_score=min_grasp_score,
                execution_frame=execution_frame,
                pregrasp_offset=pregrasp_offset,
                retreat_offset=retreat_offset,
                max_pregrasp_offset=max_pregrasp_offset,
                max_retreat_offset=max_retreat_offset,
                target_min_points=target_min_points,
                execution_top_k=execution_top_k,
                execution_reference_frame=execution_reference_frame,
                execution_base_position_world=execution_base_position_world,
                execution_base_yaw_deg=execution_base_yaw_deg,
                execution_base_workspace_bounds=execution_base_workspace_bounds,
                execution_camera_translation_gripper=execution_camera_translation_gripper,
                execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                end_pose_is_camera_pose=end_pose_is_camera_pose,
            )
            if preferred_target is None:
                preferred_target = choose_reachability_preferred_target_segment(
                    client,
                    sensor_data,
                    target_selection,
                    pc_segments,
                    segmap_aligned,
                    target_min_points=target_min_points,
                    execution_reference_frame=execution_reference_frame,
                    execution_base_position_world=execution_base_position_world,
                    execution_base_yaw_deg=execution_base_yaw_deg,
                    execution_base_workspace_bounds=execution_base_workspace_bounds,
                    execution_camera_translation_gripper=execution_camera_translation_gripper,
                    execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                    end_pose_is_camera_pose=end_pose_is_camera_pose,
                )
            if preferred_target is not None:
                original_segmap_id = active_segmap_id
                active_segmap_id = int(preferred_target['segment_id'])
                target_label = f"{target_query}->{active_segmap_id}"
                refinement_reason = 'execution-preferred' if preferred_target.get('executable') else 'reachability-preferred'
                if original_segmap_id != active_segmap_id:
                    print(
                        "  Refined target selection: "
                        f"{target_query}->{original_segmap_id} -> {target_query}->{active_segmap_id} "
                        f"({refinement_reason}, "
                        f"reachable={preferred_target.get('reachable', preferred_target.get('executable', False))}, "
                        f"points={preferred_target['point_count']}, "
                        f"depth={preferred_target['median_depth']:.3f} m)"
                    )
                else:
                    print(
                        "  Refined target selection kept: "
                        f"{target_query}->{active_segmap_id} "
                        f"({refinement_reason}, "
                        f"reachable={preferred_target.get('reachable', preferred_target.get('executable', False))}, "
                        f"points={preferred_target['point_count']}, "
                        f"depth={preferred_target['median_depth']:.3f} m)"
                    )

    debug_execution_preview = None
    debug_execution_meta = None
    target_mask = None if segmap_aligned is None or active_segmap_id is None or active_segmap_id <= 0 else (
        np.asarray(segmap_aligned == active_segmap_id, dtype=bool)
    )
    if execute_best_grasp:
        target_segment_pc = get_target_segment_points(pc_segments, active_segmap_id)
        debug_execution_preview, debug_execution_meta = compute_debug_execution_preview(
            client,
            sensor_data,
            pred_grasps_dict,
            scores_dict,
            contact_pts_dict,
            gripper_openings_dict,
            active_segmap_id,
            min_grasp_score=min_grasp_score,
            execution_frame=execution_frame,
            pregrasp_offset=pregrasp_offset,
            retreat_offset=retreat_offset,
            max_pregrasp_offset=max_pregrasp_offset,
            max_retreat_offset=max_retreat_offset,
            execution_top_k=execution_top_k,
            target_segment_points=target_segment_pc,
            max_contact_distance_to_target=max_contact_distance_to_target,
            execution_reference_frame=execution_reference_frame,
            execution_base_position_world=execution_base_position_world,
            execution_base_yaw_deg=execution_base_yaw_deg,
            execution_base_workspace_bounds=execution_base_workspace_bounds,
            execution_camera_translation_gripper=execution_camera_translation_gripper,
            execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
            target_mask=target_mask,
            end_pose_is_camera_pose=end_pose_is_camera_pose,
        )
        debug_execution_preview, debug_execution_meta = maybe_promote_global_execution_fallback(
            grasp_estimator,
            client,
            sensor_data,
            z_range,
            debug_execution_preview,
            debug_execution_meta,
            active_segmap_id,
            min_grasp_score=min_grasp_score,
            execution_frame=execution_frame,
            pregrasp_offset=pregrasp_offset,
            retreat_offset=retreat_offset,
            max_pregrasp_offset=max_pregrasp_offset,
            max_retreat_offset=max_retreat_offset,
            execution_top_k=execution_top_k,
            target_segment_points=target_segment_pc,
            max_contact_distance_to_target=max_contact_distance_to_target,
            execution_reference_frame=execution_reference_frame,
            execution_base_position_world=execution_base_position_world,
            execution_base_yaw_deg=execution_base_yaw_deg,
            execution_base_workspace_bounds=execution_base_workspace_bounds,
            execution_camera_translation_gripper=execution_camera_translation_gripper,
            execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
            target_mask=target_mask,
            forward_passes=forward_passes,
            skip_border_objects=skip_border_objects,
            margin_px=margin_px,
            end_pose_is_camera_pose=end_pose_is_camera_pose,
        )

    if debug_save_dir and (sensor_data.frame_id % max(1, debug_save_every) == 0):
        save_segmentation_debug(
            sensor_data,
            segmap_aligned,
            active_segmap_id,
            z_range,
            debug_save_dir,
            target_label,
            target_selection=target_selection,
            execution_preview=debug_execution_preview,
        )

    target_segment_pc = get_target_segment_points(pc_segments, active_segmap_id)
    target_point_count = None if target_segment_pc is None else int(target_segment_pc.shape[0])
    if target_point_count is not None and target_point_count < int(target_min_points):
        print(
            f"  Target segment has too few points for execution: "
            f"{target_point_count} < {int(target_min_points)}"
        )
        client.send_grasp_from_prediction(
            {}, {}, {}, {},
            frame_id=sensor_data.frame_id,
            status='no_grasp',
            message=f'Target segment too small: {target_point_count} points'
        )
        return None

    pred_grasps, scores, contact_pts, gripper_openings = flatten_predictions(
        pred_grasps_dict,
        scores_dict,
        contact_pts_dict,
        gripper_openings_dict,
    )
    total_grasps = count_predicted_grasps(scores_dict)

    print(f"  Generated {total_grasps} grasps")

    if total_grasps > 0:
        best_idx = np.argmax(scores)
        print(f"  Best grasp score: {scores[best_idx]:.3f}")
        print(f"  Best grasp position: {pred_grasps[best_idx][:3, 3]}")

        execution = send_prediction_result(
            client,
            pred_grasps_dict,
            scores_dict,
            contact_pts_dict,
            gripper_openings_dict,
            frame_id=sensor_data.frame_id,
            sensor_data=sensor_data,
            execute_best_grasp=execute_best_grasp,
            min_grasp_score=min_grasp_score,
            execution_frame=execution_frame,
            pregrasp_offset=pregrasp_offset,
            retreat_offset=retreat_offset,
            max_pregrasp_offset=max_pregrasp_offset,
            max_retreat_offset=max_retreat_offset,
            workspace_bounds=workspace_bounds,
            execution_top_k=execution_top_k,
            preferred_segment_id=resolve_preferred_prediction_segment(
                scores_dict,
                active_segmap_id if active_segmap_id and active_segmap_id > 0 else None,
            ),
            target_segment_points=target_segment_pc,
            max_contact_distance_to_target=max_contact_distance_to_target,
            execution_reference_frame=execution_reference_frame,
            execution_base_position_world=execution_base_position_world,
            execution_base_yaw_deg=execution_base_yaw_deg,
            execution_base_workspace_bounds=execution_base_workspace_bounds,
            execution_camera_translation_gripper=execution_camera_translation_gripper,
            execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
            target_mask=target_mask,
            status='success',
            message=f'Generated {total_grasps} grasps ({target_label})',
            end_pose_is_camera_pose=end_pose_is_camera_pose,
        )
        log_execution_target_distance(execution, pc_segments, active_segmap_id)

        # Visualize if requested
        if visualize:
            visualize_grasps_o3d(pc_full, pred_grasps, scores, pc_colors, num_vis=20)
    else:
        client.send_grasp_from_prediction(
            {}, {}, {}, {},
            frame_id=sensor_data.frame_id,
            status='no_grasp',
            message='Model returned no valid grasps'
        )

    return {
        'frame_id': sensor_data.frame_id,
        'pc_full': pc_full,
        'pred_grasps': pred_grasps_dict,
        'scores': scores_dict,
        'pc_colors': pc_colors,
    }


def run_continuous_inference(grasp_estimator, remote_ip, z_range, visualize=False,
                             sensor_port=5555, grasp_port=5556,
                             execute_best_grasp=False, min_grasp_score=0.0,
                             execution_frame='camera_optical_frame', pregrasp_offset=0.10,
                             retreat_offset=0.10, max_pregrasp_offset=None,
                             max_retreat_offset=None, segmentation_source='sam',
                             segmenter=None, target_query='', target_selector=None,
                             target_min_score=None, target_min_mask_area=400,
                             debug_save_dir='', debug_save_every=1,
                             resize_segmap_to_depth=False,
                             use_segmap=False, segmap_id=0,
                             local_regions=False, filter_grasps=False,
                             use_cam_boxes=True, filter_grasps_threshold=None,
                             skip_border_objects=False, margin_px=5,
                             forward_passes=1, workspace_bounds=None,
                             execution_top_k=None,
                             target_min_points=0,
                             target_stable_frames=1,
                             target_stability_centroid_tol_px=40.0,
                             target_stability_depth_tol_m=0.05,
                             max_contact_distance_to_target=None,
                             target_lock=False,
                             target_lock_max_centroid_shift_px=60.0,
                             target_lock_max_depth_delta_m=0.08,
                             target_lock_max_area_ratio_change=3.0,
                             execution_reference_frame='world',
                             execution_base_position_world=None,
                             execution_base_yaw_deg=0.0,
                             execution_base_workspace_bounds=None,
                             execution_camera_translation_gripper=None,
                             execution_camera_quaternion_gripper=None,
                             depth_preprocess=DEFAULT_DEPTH_PREPROCESS_ENABLED,
                             end_pose_is_camera_pose: bool = False):
    """
    Run continuous inference with optional visualization window
    """
    client = IsaacSimClient(
        remote_ip=remote_ip,
        sensor_port=sensor_port,
        grasp_port=grasp_port,
    )
    client.connect()

    print("\n=== Continuous Inference Mode ===")
    print("Press Ctrl+C to stop\n")

    # For visualization in continuous mode, we'll use a persistent window
    if visualize:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name='Contact-GraspNet Live Visualization')

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
        vis.add_geometry(pcd)

        gripper_meshes = []
        gripper_template = create_gripper_mesh()

    frame_count = 0
    last_update_time = time.time()
    stability_state = {'key': None, 'signature': None, 'count': 0}
    tracking_state = {
        'locked_signature': None,
        'cached_execution_preview': None,
        'cached_execution_world': None,
        'cached_fit_score': None,
        'cached_signature': None,
        'cached_target_key': None,
        'cached_frame_id': None,
        'execution_dispatch_latched': False,
        'execution_dispatch_frame_id': None,
        'prestable_execution_world': None,
        'prestable_fit_score': None,
        'prestable_span_priority': None,
        'prestable_geometry': None,
        'prestable_signature': None,
        'prestable_target_key': None,
        'prestable_frame_id': None,
        'prestable_score': None,
    }
    last_processed_frame_id = None

    try:
        while True:
            sensor_data = client.wait_for_data(5)

            if sensor_data is None:
                continue

            # Continuous mode should operate on the freshest frame available, not
            # an older queued frame captured before the executor entered busy state.
            latest_sensor_data = client.get_latest_data()
            if (
                latest_sensor_data is not None
                and latest_sensor_data.frame_id > sensor_data.frame_id
            ):
                sensor_data = latest_sensor_data

            if sensor_data.frame_id == last_processed_frame_id:
                continue
            last_processed_frame_id = sensor_data.frame_id

            sensor_data.depth, depth_preprocess_stats = preprocess_depth_for_inference(
                sensor_data.depth,
                z_range,
                enabled=depth_preprocess,
            )

            frame_count += 1
            loop_time = time.time()
            remote_busy = bool(getattr(sensor_data, 'execution_busy', False))
            remote_status = str(getattr(sensor_data, 'execution_status', '') or ('busy' if remote_busy else ''))
            if (
                depth_preprocess_stats.get('filled_pixels', 0) > 0
                or depth_preprocess_stats.get('outlier_pixels', 0) > 0
            ):
                print_depth_preprocess_stats(depth_preprocess_stats, prefix=f"Frame {sensor_data.frame_id}: ")

            if tracking_state['execution_dispatch_latched']:
                if (not remote_busy) and remote_status == 'idle':
                    print(
                        f"Frame {sensor_data.frame_id}: remote executor returned to idle; "
                        f"resuming grasp-result publishing after dispatched frame "
                        f"{int(tracking_state.get('execution_dispatch_frame_id') or 0)}"
                    )
                    tracking_state['execution_dispatch_latched'] = False
                    tracking_state['execution_dispatch_frame_id'] = None
                else:
                    if loop_time - last_update_time > 1:
                        print(
                            f"Frame {sensor_data.frame_id}: holding grasp-result publishing "
                            f"until remote executor returns idle "
                            f"(status={remote_status or 'busy'}, "
                            f"dispatched_frame={int(tracking_state.get('execution_dispatch_frame_id') or 0)})"
                        )
                    continue

            try:
                segmap, _ = resolve_segmap(
                    sensor_data,
                    use_segmap=use_segmap,
                    segmentation_source=segmentation_source,
                    segmenter=segmenter,
                )
            except Exception as exc:
                client.send_grasp_from_prediction(
                    {}, {}, {}, {},
                    frame_id=sensor_data.frame_id,
                    status='error',
                    message=f'Segmentation failed: {exc}'
                )
                print(f"Frame {sensor_data.frame_id}: segmentation failed: {exc}")
                continue

            segmap_aligned = segmap
            segmap_resized = False
            if use_segmap and segmap is not None:
                segmap_aligned, segmap_resized = maybe_align_segmap_to_depth(
                    segmap,
                    sensor_data.depth,
                    resize_segmap_to_depth=resize_segmap_to_depth,
                )
                segmap_aligned = complete_fragmented_segments(segmap_aligned, sensor_data.depth)
            segment_signatures = collect_segment_signatures(segmap_aligned, sensor_data.depth)
            selection_candidate_ids = None
            if use_segmap and segmap_aligned is not None:
                self_mask = build_normalized_rect_mask(segmap_aligned.shape[:2], DEFAULT_SELF_EXCLUSION_RECTS)
                excluded_segment_ids = compute_excluded_segment_ids(
                    segmap_aligned,
                    self_mask,
                    overlap_ratio_threshold=DEFAULT_SELF_EXCLUSION_OVERLAP_RATIO,
                )
                selection_candidate_ids = compute_candidate_segment_ids(segmap_aligned, excluded_segment_ids)

            active_segmap_id = None
            target_label = f'query:{target_query}' if target_query else (
                f'manual:{int(segmap_id)}' if segmap_id > 0 else 'all_segments'
            )
            target_selection = None

            fast_target_seg_id, fast_target_label = maybe_preselect_tracked_target_segment(
                segment_signatures,
                target_query=target_query,
                target_lock=target_lock,
                tracking_state=tracking_state,
                target_lock_max_centroid_shift_px=target_lock_max_centroid_shift_px,
                target_lock_max_depth_delta_m=target_lock_max_depth_delta_m,
                target_lock_max_area_ratio_change=target_lock_max_area_ratio_change,
            )
            if fast_target_seg_id is not None:
                active_segmap_id = int(fast_target_seg_id)
                target_label = fast_target_label
                if loop_time - last_update_time > 1:
                    print(
                        f"Frame {sensor_data.frame_id}: skipping text target selection, "
                        f"reusing tracked target {target_label}"
                    )
            else:
                try:
                    active_segmap_id, target_label, target_selection = resolve_target_segment(
                        sensor_data,
                        segmap_aligned,
                        explicit_segmap_id=segmap_id,
                        target_query=target_query,
                        target_selector=target_selector,
                        target_min_score=target_min_score,
                        target_min_mask_area=target_min_mask_area,
                        candidate_segment_ids=selection_candidate_ids,
                    )
                except Exception as exc:
                    client.send_grasp_from_prediction(
                        {}, {}, {}, {},
                        frame_id=sensor_data.frame_id,
                        status='error',
                        message=f'Target selection failed: {exc}'
                    )
                    print(f"Frame {sensor_data.frame_id}: target selection failed: {exc}")
                    continue

            if (
                use_segmap
                and segmap_aligned is not None
                and target_query
                and active_segmap_id is not None
                and active_segmap_id > 0
            ):
                instance_cluster_ids = collect_instance_companion_segments(
                    segmap_aligned,
                    sensor_data.depth,
                    active_segmap_id,
                    candidate_segment_ids=selection_candidate_ids,
                )
                if len(instance_cluster_ids) > 1:
                    segmap_aligned = merge_selected_instance_segments(
                        segmap_aligned,
                        active_segmap_id,
                        instance_cluster_ids,
                    )
                    segment_signatures = collect_segment_signatures(segmap_aligned, sensor_data.depth)

            current_execution_fit = None

            if target_query and active_segmap_id is None:
                selection_debug = get_target_selection_debug_info(target_selector)
                tracking_state['cached_execution_preview'] = None
                tracking_state['cached_execution_world'] = None
                tracking_state['cached_fit_score'] = None
                tracking_state['cached_signature'] = None
                tracking_state['cached_target_key'] = None
                tracking_state['cached_frame_id'] = None
                clear_pre_stability_execution_cache(tracking_state)
                print(f"Frame {sensor_data.frame_id}: target '{target_query}' not found in current frame")
                print_target_selection_debug_info(selection_debug, prefix=f"Frame {sensor_data.frame_id}: ")
                rejected_seg_id = selection_debug.get('best_segment_id')
                if (
                    debug_save_dir
                    and (sensor_data.frame_id % max(1, debug_save_every) == 0)
                    and segmap_aligned is not None
                    and rejected_seg_id is not None
                    and int(rejected_seg_id) > 0
                ):
                    save_segmentation_debug(
                        sensor_data,
                        segmap_aligned,
                        int(rejected_seg_id),
                        z_range,
                        debug_save_dir,
                        f"{target_query}->rejected",
                        target_selection=None,
                        execution_preview=None,
                    )
                client.send_grasp_from_prediction(
                    {}, {}, {}, {},
                    frame_id=sensor_data.frame_id,
                    status='no_grasp',
                    message=f"Target '{target_query}' not found in current frame"
                )
                continue

            target_depth_cleanup_info = None
            if use_segmap and segmap_aligned is not None and active_segmap_id is not None and active_segmap_id > 0:
                raw_target_mask = np.asarray(segmap_aligned == active_segmap_id, dtype=bool)
                cleaned_target_mask, target_depth_cleanup_info = refine_target_mask_by_depth(
                    raw_target_mask,
                    sensor_data.depth,
                    z_range,
                )
                if (
                    target_depth_cleanup_info is not None
                    and (
                        bool(target_depth_cleanup_info.get('applied'))
                        or float(target_depth_cleanup_info.get('original_valid_pixels', 0))
                        / float(max(1, target_depth_cleanup_info.get('original_pixels', 1)))
                        < 0.70
                    )
                    and loop_time - last_update_time > 1
                ):
                    print_target_depth_cleanup_info(
                        target_depth_cleanup_info,
                        prefix=f"Frame {sensor_data.frame_id}: ",
                    )
                if bool(target_depth_cleanup_info and target_depth_cleanup_info.get('applied')):
                    segmap_aligned = apply_cleaned_target_mask(
                        segmap_aligned,
                        active_segmap_id,
                        cleaned_target_mask,
                    )
                    segment_signatures = collect_segment_signatures(segmap_aligned, sensor_data.depth)

            prediction_segmap_id = active_segmap_id or 0
            if should_refine_target_with_reachability(use_segmap, segmap_id, target_query):
                prediction_segmap_id = 0

            # Extract point clouds
            pc_full, pc_segments, pc_colors, pred_grasps_dict, scores_dict, contact_pts_dict, gripper_openings_dict = predict_scene_candidates(
                grasp_estimator,
                sensor_data,
                z_range,
                segmap=segmap_aligned,
                use_segmap=use_segmap,
                segmap_id=prediction_segmap_id,
                local_regions=local_regions,
                filter_grasps=filter_grasps,
                use_cam_boxes=use_cam_boxes,
                filter_grasps_threshold=filter_grasps_threshold,
                skip_border_objects=skip_border_objects,
                margin_px=margin_px,
                forward_passes=forward_passes,
            )

            if pc_full.shape[0] == 0:
                client.send_grasp_from_prediction(
                    {}, {}, {}, {},
                    frame_id=sensor_data.frame_id,
                    status='no_grasp',
                    message='No points in configured z-range'
                )
                continue

            if use_segmap and not pc_segments:
                client.send_grasp_from_prediction(
                    {}, {}, {}, {},
                    frame_id=sensor_data.frame_id,
                    status='no_grasp',
                    message='No valid segmented objects after local segmentation/filtering'
                )
                continue

            allowed_locked_seg_ids = None
            if target_selection is not None and target_selection.scores_by_segment:
                allowed_locked_seg_ids = {
                    int(seg_id) for seg_id in target_selection.scores_by_segment.keys()
                }

            locked_seg_id = None
            sticky_seg_id = None
            if (
                target_lock
                and target_query
                and tracking_state['locked_signature'] is not None
            ):
                locked_seg_id = choose_locked_target_segment(
                    segment_signatures,
                    tracking_state['locked_signature'],
                    centroid_tol_px=target_lock_max_centroid_shift_px,
                    depth_tol_m=target_lock_max_depth_delta_m,
                    max_area_ratio_change=target_lock_max_area_ratio_change,
                    allowed_segment_ids=allowed_locked_seg_ids,
                )
                if locked_seg_id is not None:
                    active_segmap_id = locked_seg_id
                    target_label = f'{target_query}->{locked_seg_id} (locked)'

            if (
                locked_seg_id is None
                and target_query
                and stability_state.get('count', 0) > 0
                and stability_state.get('key') == f'query:{target_query}'
                and stability_state.get('signature') is not None
            ):
                sticky_seg_id = choose_locked_target_segment(
                    segment_signatures,
                    stability_state['signature'],
                    centroid_tol_px=target_stability_centroid_tol_px,
                    depth_tol_m=target_stability_depth_tol_m,
                    max_area_ratio_change=max(2.0, float(target_lock_max_area_ratio_change)),
                    allowed_segment_ids=allowed_locked_seg_ids,
                )
                if sticky_seg_id is not None:
                    active_segmap_id = sticky_seg_id
                    target_label = f'{target_query}->{sticky_seg_id}'
                    if loop_time - last_update_time > 1:
                        print(
                            f"Frame {sensor_data.frame_id}: keeping sticky target "
                            f"{target_query}->{sticky_seg_id} "
                            f"(stability={int(stability_state.get('count', 0))}/{int(target_stable_frames)})"
                        )

            if (
                locked_seg_id is None
                and sticky_seg_id is None
                and should_refine_target_with_reachability(use_segmap, segmap_id, target_query)
            ):
                if not target_selection_is_semantically_confident(target_selection):
                    if loop_time - last_update_time > 1:
                        print(
                            f"Frame {sensor_data.frame_id}: skip target refinement due to weak semantics "
                            f"(agreement={float(getattr(target_selection, 'agreement_score', 0.0)):.3f}, "
                            f"advantage={float(getattr(target_selection, 'query_advantage', 0.0)):.3f}, "
                            f"margin={float(getattr(target_selection, 'selection_margin', 0.0)):.3f})"
                        )
                else:
                    preferred_target = choose_execution_preferred_target_segment(
                        client,
                        sensor_data,
                        target_selection,
                        pc_segments,
                        segmap_aligned,
                        pred_grasps_dict,
                        scores_dict,
                        contact_pts_dict,
                        gripper_openings_dict,
                        min_grasp_score=min_grasp_score,
                        execution_frame=execution_frame,
                        pregrasp_offset=pregrasp_offset,
                        retreat_offset=retreat_offset,
                        max_pregrasp_offset=max_pregrasp_offset,
                        max_retreat_offset=max_retreat_offset,
                        target_min_points=target_min_points,
                        execution_top_k=execution_top_k,
                        execution_reference_frame=execution_reference_frame,
                        execution_base_position_world=execution_base_position_world,
                        execution_base_yaw_deg=execution_base_yaw_deg,
                        execution_base_workspace_bounds=execution_base_workspace_bounds,
                        execution_camera_translation_gripper=execution_camera_translation_gripper,
                        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                    )
                    if preferred_target is None:
                        preferred_target = choose_reachability_preferred_target_segment(
                            client,
                            sensor_data,
                            target_selection,
                            pc_segments,
                            segmap_aligned,
                            target_min_points=target_min_points,
                            execution_reference_frame=execution_reference_frame,
                            execution_base_position_world=execution_base_position_world,
                            execution_base_yaw_deg=execution_base_yaw_deg,
                            execution_base_workspace_bounds=execution_base_workspace_bounds,
                            execution_camera_translation_gripper=execution_camera_translation_gripper,
                            execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                            end_pose_is_camera_pose=end_pose_is_camera_pose,
                        )
                    if preferred_target is not None:
                        original_segmap_id = active_segmap_id
                        active_segmap_id = int(preferred_target['segment_id'])
                        target_label = f'{target_query}->{active_segmap_id}'
                        refinement_reason = 'execution-preferred' if preferred_target.get('executable') else 'reachability-preferred'
                        if loop_time - last_update_time > 1:
                            if original_segmap_id != active_segmap_id:
                                print(
                                    f"Frame {sensor_data.frame_id}: refined target "
                                    f"{target_query}->{original_segmap_id} -> {target_query}->{active_segmap_id} "
                                    f"({refinement_reason}, "
                                    f"reachable={preferred_target.get('reachable', preferred_target.get('executable', False))}, "
                                    f"points={preferred_target['point_count']}, "
                                    f"depth={preferred_target['median_depth']:.3f} m)"
                                )
                            else:
                                print(
                                    f"Frame {sensor_data.frame_id}: refined target kept "
                                    f"{target_query}->{active_segmap_id} "
                                    f"({refinement_reason}, "
                                    f"reachable={preferred_target.get('reachable', preferred_target.get('executable', False))}, "
                                    f"points={preferred_target['point_count']}, "
                                    f"depth={preferred_target['median_depth']:.3f} m)"
                                )

            debug_execution_preview = None
            debug_execution_meta = None
            target_mask = None if segmap_aligned is None or active_segmap_id is None or active_segmap_id <= 0 else (
                np.asarray(segmap_aligned == active_segmap_id, dtype=bool)
            )
            current_target_key = f'query:{target_query}' if target_query else (
                f'seg:{active_segmap_id}' if active_segmap_id else None
            )
            current_execution_world = None
            if execute_best_grasp:
                target_segment_pc = get_target_segment_points(pc_segments, active_segmap_id)
                debug_execution_preview, debug_execution_meta = compute_debug_execution_preview(
                    client,
                    sensor_data,
                    pred_grasps_dict,
                    scores_dict,
                    contact_pts_dict,
                    gripper_openings_dict,
                    active_segmap_id,
                    min_grasp_score=min_grasp_score,
                    execution_frame=execution_frame,
                    pregrasp_offset=pregrasp_offset,
                    retreat_offset=retreat_offset,
                    max_pregrasp_offset=max_pregrasp_offset,
                    max_retreat_offset=max_retreat_offset,
                    execution_top_k=execution_top_k,
                    target_segment_points=target_segment_pc,
                    max_contact_distance_to_target=max_contact_distance_to_target,
                    execution_reference_frame=execution_reference_frame,
                    execution_base_position_world=execution_base_position_world,
                    execution_base_yaw_deg=execution_base_yaw_deg,
                    execution_base_workspace_bounds=execution_base_workspace_bounds,
                    execution_camera_translation_gripper=execution_camera_translation_gripper,
                    execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                    target_mask=target_mask,
                    end_pose_is_camera_pose=end_pose_is_camera_pose,
                )
                debug_execution_preview, debug_execution_meta = maybe_promote_global_execution_fallback(
                    grasp_estimator,
                    client,
                    sensor_data,
                    z_range,
                    debug_execution_preview,
                    debug_execution_meta,
                    active_segmap_id,
                    min_grasp_score=min_grasp_score,
                    execution_frame=execution_frame,
                    pregrasp_offset=pregrasp_offset,
                    retreat_offset=retreat_offset,
                    max_pregrasp_offset=max_pregrasp_offset,
                    max_retreat_offset=max_retreat_offset,
                    execution_top_k=execution_top_k,
                    target_segment_points=target_segment_pc,
                    max_contact_distance_to_target=max_contact_distance_to_target,
                    execution_reference_frame=execution_reference_frame,
                    execution_base_position_world=execution_base_position_world,
                    execution_base_yaw_deg=execution_base_yaw_deg,
                    execution_base_workspace_bounds=execution_base_workspace_bounds,
                    execution_camera_translation_gripper=execution_camera_translation_gripper,
                    execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                    target_mask=target_mask,
                    forward_passes=forward_passes,
                    skip_border_objects=skip_border_objects,
                    margin_px=margin_px,
                    end_pose_is_camera_pose=end_pose_is_camera_pose,
                )
                if debug_execution_preview is not None and debug_execution_meta is not None:
                    if debug_execution_meta.get('fit_score') is not None:
                        print(f"  Execution fit score: {float(debug_execution_meta['fit_score']):.3f}")
                    if debug_execution_meta.get('span_priority') is not None:
                        print(f"  Execution span priority: {float(debug_execution_meta['span_priority']):.3f}")
                    print_execution_candidate_diagnostics(
                        [
                            {
                                'segment_id': debug_execution_preview.segment_id,
                                'score': float(debug_execution_preview.score),
                                'fit_score': float(debug_execution_meta.get('fit_score', 0.0)),
                                'span_priority': float(debug_execution_meta.get('span_priority', 0.0)),
                                'geometry': debug_execution_meta.get('selected_geometry'),
                            }
                        ],
                        label='Selected execution geometry',
                        max_items=1,
                    )
                elif debug_execution_meta is not None:
                    print_execution_candidate_diagnostics(
                        debug_execution_meta.get('top_candidate_diagnostics'),
                        label='Top reranked candidate geometry',
                    )
                    print_execution_candidate_diagnostics(
                        debug_execution_meta.get('accepted_candidate_diagnostics'),
                        label='Accepted candidate geometry',
                    )
                if debug_execution_preview is not None and sensor_data.end_pose is not None:
                    current_execution_world = freeze_execution_to_planning_frame(
                        client,
                        debug_execution_preview,
                        end_pose_world=sensor_data.end_pose,
                        execution_reference_frame=execution_reference_frame,
                        reference_frame=sensor_data.end_pose_frame or execution_reference_frame,
                        execution_camera_translation_gripper=execution_camera_translation_gripper,
                        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                    )
                # Fresh candidates already passed geometry constraints before entering
                # the rerank pool. Do not apply the same local reject path again here,
                # otherwise rerank can still collapse to "no_grasp" after selection.
            cached_execution_preview = None
            cached_execution_world = None
            if (
                execute_best_grasp
                and current_target_key is not None
                and current_target_key == tracking_state.get('cached_target_key')
                and target_signatures_match(
                    segment_signatures.get(active_segmap_id),
                    tracking_state.get('cached_signature'),
                    centroid_tol_px=target_lock_max_centroid_shift_px,
                    depth_tol_m=target_lock_max_depth_delta_m,
                    max_area_ratio_change=target_lock_max_area_ratio_change,
                )
            ):
                cached_execution_world = tracking_state.get('cached_execution_world')
                if cached_execution_world is not None:
                    cached_execution_preview = project_execution_to_camera_frame(
                        client,
                        cached_execution_world,
                        end_pose_world=sensor_data.end_pose,
                        execution_reference_frame=execution_reference_frame,
                        execution_camera_translation_gripper=execution_camera_translation_gripper,
                        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                    )
                    cached_valid, cached_info = evaluate_execution_preview_projection(
                        cached_execution_preview,
                        sensor_data,
                        target_mask,
                    )
                    if not cached_valid:
                        print(
                            f"Frame {sensor_data.frame_id}: discarding cached execution from frame "
                            f"{int(tracking_state.get('cached_frame_id') or 0)} "
                            f"({cached_info.get('reason')}, center_inside={cached_info.get('center_inside')}, "
                            f"finger_a_on_target={cached_info.get('finger_a_on_target')}, "
                            f"finger_b_on_target={cached_info.get('finger_b_on_target')}, "
                            f"line_fraction={cached_info.get('line_fraction', 0.0):.2f}, "
                            f"finger_span={cached_info.get('finger_span', 0.0):.2f}, "
                            f"center_t={cached_info.get('center_t', 0.5):.2f})"
                        )
                        cached_execution_preview = None
                        cached_execution_world = None
                        tracking_state['cached_execution_preview'] = None
                        tracking_state['cached_execution_world'] = None
                        tracking_state['cached_fit_score'] = None
                        tracking_state['cached_signature'] = None
                        tracking_state['cached_target_key'] = None
                        tracking_state['cached_frame_id'] = None

                if debug_execution_preview is None and cached_execution_preview is not None:
                    debug_execution_preview = cached_execution_preview
                    current_execution_world = cached_execution_world
                    current_execution_fit = tracking_state.get('cached_fit_score')
                    print(
                        f"Frame {sensor_data.frame_id}: reusing cached execution preview "
                        f"from frame {int(tracking_state.get('cached_frame_id') or 0)}"
                    )
                elif (
                    debug_execution_preview is not None
                    and cached_execution_preview is not None
                    and current_execution_world is not None
                ):
                    current_fit = score_execution_object_fit(
                        debug_execution_preview,
                        sensor_data,
                        target_mask,
                        target_segment_points=target_segment_pc,
                    )
                    cached_fit = score_execution_object_fit(
                        cached_execution_preview,
                        sensor_data,
                        target_mask,
                        target_segment_points=target_segment_pc,
                    )
                    world_delta = compute_execution_world_delta(
                        current_execution_world,
                        cached_execution_world,
                    )
                    if world_delta is not None:
                        hard_drift_exceeded = (
                            world_delta['center_distance_m'] > DEFAULT_EXECUTION_TEMPORAL_MAX_POSITION_DELTA_M
                            or world_delta['contact_distance_m'] > DEFAULT_EXECUTION_TEMPORAL_MAX_POSITION_DELTA_M
                            or world_delta['approach_angle_deg'] > DEFAULT_EXECUTION_TEMPORAL_MAX_ANGLE_DELTA_DEG
                        )
                        drift_too_large = (
                            world_delta['center_distance_m'] > DEFAULT_EXECUTION_TEMPORAL_POSITION_DELTA_M
                            or world_delta['contact_distance_m'] > DEFAULT_EXECUTION_TEMPORAL_POSITION_DELTA_M
                            or world_delta['approach_angle_deg'] > DEFAULT_EXECUTION_TEMPORAL_ANGLE_DELTA_DEG
                        )
                        current_not_better = current_fit < (cached_fit + DEFAULT_EXECUTION_TEMPORAL_FIT_MARGIN)
                        cached_clearly_better = cached_fit > (current_fit + DEFAULT_EXECUTION_TEMPORAL_FIT_MARGIN)
                        if hard_drift_exceeded:
                            if cached_fit > (current_fit + DEFAULT_EXECUTION_CACHE_REPLACE_FIT_MARGIN):
                                debug_execution_preview = cached_execution_preview
                                current_execution_world = cached_execution_world
                                current_execution_fit = cached_fit
                                print(
                                    f"Frame {sensor_data.frame_id}: using cached execution from frame "
                                    f"{int(tracking_state.get('cached_frame_id') or 0)} despite hard drift "
                                    f"(center_delta={world_delta['center_distance_m']:.3f} m, "
                                    f"contact_delta={world_delta['contact_distance_m']:.3f} m, "
                                    f"angle_delta={world_delta['approach_angle_deg']:.1f} deg, "
                                    f"current_fit={current_fit:.3f}, cached_fit={cached_fit:.3f})"
                                )
                            else:
                                print(
                                    f"Frame {sensor_data.frame_id}: refusing cached execution from frame "
                                    f"{int(tracking_state.get('cached_frame_id') or 0)} "
                                    f"(center_delta={world_delta['center_distance_m']:.3f} m, "
                                    f"contact_delta={world_delta['contact_distance_m']:.3f} m, "
                                    f"angle_delta={world_delta['approach_angle_deg']:.1f} deg exceeds hard limit)"
                                )
                                current_execution_fit = current_fit
                        elif cached_clearly_better or (drift_too_large and current_not_better):
                            debug_execution_preview = cached_execution_preview
                            current_execution_world = cached_execution_world
                            current_execution_fit = cached_fit
                            print(
                                f"Frame {sensor_data.frame_id}: keeping cached execution from frame "
                                f"{int(tracking_state.get('cached_frame_id') or 0)} "
                                f"(center_delta={world_delta['center_distance_m']:.3f} m, "
                                f"contact_delta={world_delta['contact_distance_m']:.3f} m, "
                                f"angle_delta={world_delta['approach_angle_deg']:.1f} deg, "
                                f"current_fit={current_fit:.3f}, cached_fit={cached_fit:.3f})"
                            )
                        else:
                            current_execution_fit = current_fit
                elif debug_execution_preview is not None:
                    current_execution_fit = score_execution_object_fit(
                        debug_execution_preview,
                        sensor_data,
                        target_mask,
                        target_segment_points=target_segment_pc,
                    )

            if debug_save_dir and (sensor_data.frame_id % max(1, debug_save_every) == 0):
                save_segmentation_debug(
                    sensor_data,
                    segmap_aligned,
                    active_segmap_id,
                    z_range,
                    debug_save_dir,
                    target_label,
                    target_selection=target_selection,
                    execution_preview=debug_execution_preview,
                )

            target_segment_pc = get_target_segment_points(pc_segments, active_segmap_id)
            target_point_count = None if target_segment_pc is None else int(target_segment_pc.shape[0])
            if target_point_count is not None and target_point_count < int(target_min_points):
                client.send_grasp_from_prediction(
                    {}, {}, {}, {},
                    frame_id=sensor_data.frame_id,
                    status='no_grasp',
                    message=f'Target segment too small: {target_point_count} points'
                )
                if loop_time - last_update_time > 1:
                    print(
                        f"Frame {sensor_data.frame_id}: skip target with too few points "
                        f"({target_point_count} < {int(target_min_points)})"
                    )
                continue

            stable_count = 0
            if use_segmap and active_segmap_id is not None and active_segmap_id > 0:
                stability_signature = compute_target_signature(
                    segmap_aligned,
                    active_segmap_id,
                    sensor_data.depth,
                )
                if target_query:
                    stability_key = f'query:{target_query}'
                elif segmap_id > 0:
                    stability_key = f'seg:{segmap_id}'
                else:
                    stability_key = 'selected_target'
                stable_count = update_target_stability(
                    stability_state,
                    stability_key,
                    stability_signature,
                    centroid_tol_px=target_stability_centroid_tol_px,
                    depth_tol_m=target_stability_depth_tol_m,
                )
                if (
                    execute_best_grasp
                    and debug_execution_preview is not None
                    and current_execution_world is not None
                    and debug_execution_meta is not None
                    and stable_count < int(target_stable_frames)
                ):
                    updated_prestable = update_pre_stability_execution_cache(
                        tracking_state,
                        current_target_key,
                        stability_signature,
                        debug_execution_preview,
                        current_execution_world,
                        debug_execution_meta,
                        sensor_data.frame_id,
                        centroid_tol_px=target_stability_centroid_tol_px,
                        depth_tol_m=target_stability_depth_tol_m,
                        max_area_ratio_change=max(2.0, float(target_lock_max_area_ratio_change)),
                    )
                    if updated_prestable:
                        print(
                            f"Frame {sensor_data.frame_id}: updated pre-stability best execution "
                            f"(stability={stable_count}/{int(target_stable_frames)}, "
                            f"span={float(debug_execution_meta.get('span_priority', 0.0)):.3f}, "
                            f"fit={float(debug_execution_meta.get('fit_score', 0.0)):.3f})"
                        )
                if stable_count < int(target_stable_frames):
                    client.send_grasp_from_prediction(
                        {}, {}, {}, {},
                        frame_id=sensor_data.frame_id,
                        status='no_grasp',
                        message=(
                            f'Target not stable yet: {stable_count}/{int(target_stable_frames)} frames'
                        ),
                    )
                    if loop_time - last_update_time > 1:
                        print(
                            f"Frame {sensor_data.frame_id}: target stability "
                            f"{stable_count}/{int(target_stable_frames)}"
                        )
                    continue
                if execute_best_grasp:
                    debug_execution_preview, current_execution_world, current_execution_fit, debug_execution_meta = promote_pre_stability_execution_cache(
                        client,
                        tracking_state,
                        current_target_key,
                        stability_signature,
                        sensor_data,
                        target_mask,
                        target_segment_pc,
                        debug_execution_preview,
                        current_execution_world,
                        current_execution_fit,
                        debug_execution_meta,
                        execution_reference_frame=execution_reference_frame,
                        execution_camera_translation_gripper=execution_camera_translation_gripper,
                        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                        centroid_tol_px=target_stability_centroid_tol_px,
                        depth_tol_m=target_stability_depth_tol_m,
                        max_area_ratio_change=max(2.0, float(target_lock_max_area_ratio_change)),
                    )
                if target_lock:
                    tracking_state['locked_signature'] = segment_signatures.get(active_segmap_id)
            elif target_lock:
                tracking_state['locked_signature'] = None
                clear_pre_stability_execution_cache(tracking_state)

            execution_override_world = None
            execution_dispatch_blocked_reason = None
            if (
                execute_best_grasp
                and debug_execution_preview is not None
                and active_segmap_id is not None
                and active_segmap_id > 0
                and stable_count >= int(target_stable_frames)
            ):
                selected_world = current_execution_world
                if selected_world is None and sensor_data.end_pose is not None:
                    selected_world = freeze_execution_to_planning_frame(
                        client,
                        debug_execution_preview,
                        end_pose_world=sensor_data.end_pose,
                        execution_reference_frame=execution_reference_frame,
                        reference_frame=sensor_data.end_pose_frame or execution_reference_frame,
                        execution_camera_translation_gripper=execution_camera_translation_gripper,
                        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                    )
                cacheable_preview, cacheable_info = evaluate_execution_preview_projection(
                    debug_execution_preview,
                    sensor_data,
                    target_mask,
                )
                if cacheable_preview and selected_world is not None:
                    execution_override_world = selected_world
                    selected_fit = current_execution_fit
                    if selected_fit is None:
                        selected_fit = score_execution_object_fit(
                            debug_execution_preview,
                            sensor_data,
                            target_mask,
                            target_segment_points=target_segment_pc,
                        )
                    previous_fit = tracking_state.get('cached_fit_score')
                    previous_world = tracking_state.get('cached_execution_world')
                    replace_cache = (
                        previous_world is None
                        or previous_fit is None
                        or selected_fit >= (float(previous_fit) + DEFAULT_EXECUTION_CACHE_REPLACE_FIT_MARGIN)
                        or np.allclose(previous_world.pose, selected_world.pose)
                    )
                    if replace_cache:
                        tracking_state['cached_execution_preview'] = debug_execution_preview
                        tracking_state['cached_execution_world'] = selected_world
                        tracking_state['cached_fit_score'] = float(selected_fit)
                        tracking_state['cached_signature'] = segment_signatures.get(active_segmap_id)
                        tracking_state['cached_target_key'] = current_target_key
                        tracking_state['cached_frame_id'] = sensor_data.frame_id
                    else:
                        print(
                            f"Frame {sensor_data.frame_id}: preserving cached execution from frame "
                            f"{int(tracking_state.get('cached_frame_id') or 0)} "
                            f"(current_fit={float(selected_fit):.3f}, cached_fit={float(previous_fit):.3f})"
                        )
                else:
                    execution_dispatch_blocked_reason = (
                        f"current best execution preview rejected "
                        f"({cacheable_info.get('reason')}, center_inside={cacheable_info.get('center_inside')}, "
                        f"finger_a_on_target={cacheable_info.get('finger_a_on_target')}, "
                        f"finger_b_on_target={cacheable_info.get('finger_b_on_target')}, "
                        f"line_fraction={cacheable_info.get('line_fraction', 0.0):.2f}, "
                        f"finger_span={cacheable_info.get('finger_span', 0.0):.2f}, "
                        f"center_t={cacheable_info.get('center_t', 0.5):.2f})"
                    )
                    print(
                        f"Frame {sensor_data.frame_id}: not caching execution preview "
                        f"({cacheable_info.get('reason')}, center_inside={cacheable_info.get('center_inside')}, "
                        f"finger_a_on_target={cacheable_info.get('finger_a_on_target')}, "
                        f"finger_b_on_target={cacheable_info.get('finger_b_on_target')}, "
                        f"line_fraction={cacheable_info.get('line_fraction', 0.0):.2f}, "
                        f"finger_span={cacheable_info.get('finger_span', 0.0):.2f}, "
                        f"center_t={cacheable_info.get('center_t', 0.5):.2f})"
                    )

            pred_grasps, scores, contact_pts, gripper_openings = flatten_predictions(
                pred_grasps_dict,
                scores_dict,
                contact_pts_dict,
                gripper_openings_dict,
            )
            total_grasps = count_predicted_grasps(scores_dict)

            # Send results
            if total_grasps > 0:
                if execute_best_grasp and debug_execution_preview is not None and execution_override_world is None:
                    client.send_grasp_from_prediction(
                        pred_grasps_dict,
                        scores_dict,
                        contact_pts_dict,
                        gripper_openings_dict,
                        frame_id=sensor_data.frame_id,
                        status='no_grasp',
                        message=execution_dispatch_blocked_reason or 'Current best execution preview rejected',
                    )
                    execution = None
                else:
                    execution = send_prediction_result(
                        client,
                        pred_grasps_dict,
                        scores_dict,
                        contact_pts_dict,
                        gripper_openings_dict,
                        frame_id=sensor_data.frame_id,
                        sensor_data=sensor_data,
                        execute_best_grasp=execute_best_grasp,
                        min_grasp_score=min_grasp_score,
                        execution_frame=execution_frame,
                        pregrasp_offset=pregrasp_offset,
                        retreat_offset=retreat_offset,
                        max_pregrasp_offset=max_pregrasp_offset,
                        max_retreat_offset=max_retreat_offset,
                        workspace_bounds=workspace_bounds,
                        execution_top_k=execution_top_k,
                        preferred_segment_id=resolve_preferred_prediction_segment(
                            scores_dict,
                            active_segmap_id if active_segmap_id and active_segmap_id > 0 else None,
                        ),
                        target_segment_points=target_segment_pc,
                        max_contact_distance_to_target=max_contact_distance_to_target,
                        execution_reference_frame=execution_reference_frame,
                        execution_base_position_world=execution_base_position_world,
                        execution_base_yaw_deg=execution_base_yaw_deg,
                        execution_base_workspace_bounds=execution_base_workspace_bounds,
                        execution_camera_translation_gripper=execution_camera_translation_gripper,
                        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                        target_mask=target_mask,
                        override_execution=execution_override_world,
                        status='success',
                        message=f'Generated {total_grasps} grasps ({target_label})',
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                    )
                    if execution is not None:
                        tracking_state['execution_dispatch_latched'] = True
                        tracking_state['execution_dispatch_frame_id'] = sensor_data.frame_id
                        print(
                            f"Frame {sensor_data.frame_id}: dispatched execution command; "
                            f"suspending further grasp-result publishing until remote executor returns idle"
                        )
                    log_execution_target_distance(execution, pc_segments, active_segmap_id)
            else:
                if cached_execution_world is not None:
                    print(
                        f"Frame {sensor_data.frame_id}: reusing cached execution "
                        f"from frame {int(tracking_state.get('cached_frame_id') or 0)} despite no current grasps"
                    )
                    send_prediction_result(
                        client,
                        {}, {}, {}, {},
                        frame_id=sensor_data.frame_id,
                        sensor_data=sensor_data,
                        execute_best_grasp=execute_best_grasp,
                        min_grasp_score=min_grasp_score,
                        execution_frame=execution_frame,
                        pregrasp_offset=pregrasp_offset,
                        retreat_offset=retreat_offset,
                        max_pregrasp_offset=max_pregrasp_offset,
                        max_retreat_offset=max_retreat_offset,
                        workspace_bounds=workspace_bounds,
                        execution_top_k=execution_top_k,
                        preferred_segment_id=resolve_preferred_prediction_segment(
                            scores_dict,
                            active_segmap_id if active_segmap_id and active_segmap_id > 0 else None,
                        ),
                        target_segment_points=target_segment_pc,
                        max_contact_distance_to_target=max_contact_distance_to_target,
                        execution_reference_frame=execution_reference_frame,
                        execution_base_position_world=execution_base_position_world,
                        execution_base_yaw_deg=execution_base_yaw_deg,
                        execution_base_workspace_bounds=execution_base_workspace_bounds,
                        execution_camera_translation_gripper=execution_camera_translation_gripper,
                        execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
                        target_mask=target_mask,
                        override_execution=cached_execution_world,
                        status='success',
                        message=f'Reused cached execution ({target_label})',
                        end_pose_is_camera_pose=end_pose_is_camera_pose,
                    )
                    tracking_state['execution_dispatch_latched'] = True
                    tracking_state['execution_dispatch_frame_id'] = sensor_data.frame_id
                    print(
                        f"Frame {sensor_data.frame_id}: dispatched cached execution command; "
                        f"suspending further grasp-result publishing until remote executor returns idle"
                    )
                else:
                    client.send_grasp_from_prediction(
                        {}, {}, {}, {},
                        frame_id=sensor_data.frame_id,
                        status='no_grasp',
                        message='Model returned no valid grasps'
                    )

            # Update visualization
            if visualize and frame_count % 3 == 0:  # Update every 3 frames
                # Update point cloud
                pcd.points = o3d.utility.Vector3dVector(pc_full)
                if pc_colors is not None:
                    pcd.colors = o3d.utility.Vector3dVector(pc_colors / 255.0)
                else:
                    pcd.paint_uniform_color([0.5, 0.5, 0.5])

                # Remove old gripper meshes
                for mesh in gripper_meshes:
                    vis.remove_geometry(mesh, reset_bounding_box=False)
                gripper_meshes.clear()

                # Add new gripper meshes (top 10)
                if total_grasps > 0:
                    best_indices = np.argsort(scores)[::-1][:10]
                    for idx in best_indices:
                        gripper_mesh = o3d.geometry.TriangleMesh(gripper_template)
                        gripper_mesh.transform(pred_grasps[idx])
                        score = scores[idx]
                        gripper_mesh.paint_uniform_color([score, 1.0 - score, 0.0])
                        gripper_mesh.compute_vertex_normals()
                        vis.add_geometry(gripper_mesh, reset_bounding_box=False)
                        gripper_meshes.append(gripper_mesh)

                vis.poll_events()
                vis.update_renderer()

            # Print status every 5 seconds
            current_time = time.time()
            if current_time - last_update_time > 5:
                print(f"Frame {sensor_data.frame_id}: {pc_full.shape[0]} points, {total_grasps} grasps")
                if sensor_data.rgb is not None:
                    print(f"  RGB shape: {sensor_data.rgb.shape}, depth shape: {sensor_data.depth.shape}")
                    if sensor_data.rgb.shape[:2] != sensor_data.depth.shape[:2]:
                        print("  WARNING: RGB/depth resolution mismatch detected")
                if use_segmap:
                    print(f"  Target selection: {target_label}")
                    if target_selection is not None:
                        print(f"  Target score: {target_selection.score:.3f}")
                    if segmap is not None:
                        print(f"  Segmap shape: {np.asarray(segmap).shape}, resized_to_depth={segmap_resized}")
                    target_segment_points = get_segment_point_count(pc_segments, active_segmap_id)
                    if target_segment_points is not None:
                        print(f"  Target segment points: {target_segment_points}")
                    if target_stable_frames > 1:
                        print(f"  Target stability: {stable_count}/{int(target_stable_frames)}")
                    if target_lock and tracking_state['locked_signature'] is not None:
                        print(
                            f"  Target lock: active on seg "
                            f"{int(tracking_state['locked_signature']['segment_id'])}"
                        )
                    if segmap_aligned is not None and active_segmap_id is not None and active_segmap_id > 0:
                        target_mask = np.asarray(segmap_aligned == active_segmap_id, dtype=bool)
                        target_depth_summary = summarize_target_depth(sensor_data.depth, target_mask, z_range)
                        print_target_depth_summary(target_depth_summary)
                if total_grasps > 0:
                    print(f"  Best score: {scores.max():.3f}")
                last_update_time = current_time

    except KeyboardInterrupt:
        print("\nStopping...")

    finally:
        if visualize:
            vis.destroy_window()
        client.disconnect()
        print("Disconnected")


def main():
    parser = argparse.ArgumentParser(description='Contact-GraspNet Isaac Sim Interface')
    parser.add_argument('--ckpt_dir', default='checkpoints/contact_graspnet',
                        help='Checkpoint directory')
    parser.add_argument('--remote-ip', default='192.168.100.12',
                        help='IP of remote Isaac Sim forwarder')
    parser.add_argument('--sensor-port', type=int, default=5555,
                        help='Port for receiving sensor data')
    parser.add_argument('--grasp-port', type=int, default=5556,
                        help='Port for sending grasp results')
    parser.add_argument('--timeout', type=float, default=30.0,
                        help='Timeout for receiving data (seconds)')
    parser.add_argument('--z-range', type=str, default='[1.5,5.0]',
                        help='Z range for point cloud filtering (default: [1.5,5.0] for Piper camera)')
    parser.add_argument('--continuous', action='store_true',
                        help='Run continuous inference pipeline')
    parser.add_argument('--visualize', action='store_true', default=False,
                        help='Enable visualization (default: disabled for speed)')
    parser.add_argument('--no-visualize', action='store_true',
                        help='Disable visualization (for faster inference)')
    parser.add_argument('--execute-best-grasp', action='store_true',
                        help='Attach an executable best-grasp command for the remote simulator/controller')
    parser.add_argument('--min-grasp-score', type=float, default=0.0,
                        help='Minimum score required before sending an execution command')
    parser.add_argument('--execution-frame', type=str, default='camera_optical_frame',
                        help='Frame name used by the remote executor for the execution command')
    parser.add_argument('--pregrasp-offset', type=float, default=0.10,
                        help='Distance in meters to retreat from the grasp along the approach axis before closing')
    parser.add_argument('--retreat-offset', type=float, default=0.10,
                        help='Distance in meters to retreat after grasp closure along the approach axis')
    parser.add_argument('--max-pregrasp-offset', type=float, default=0.06,
                        help='Clamp the generated pregrasp offset to this value before sending execution commands')
    parser.add_argument('--max-retreat-offset', type=float, default=0.08,
                        help='Clamp the generated retreat offset to this value before sending execution commands')
    parser.add_argument('--use-segmap', action='store_true',
                        help='Use segmented scene grasp prediction instead of whole-scene blind grasping')
    parser.add_argument('--segmentation-source', type=str, default='sam', choices=['sam', 'remote'],
                        help='Where the segmap comes from: local SAM inference or remote forwarded data')
    parser.add_argument('--segmap-id', type=int, default=0,
                        help='Instance id to target inside segmap; 0 means all foreground instances')
    parser.add_argument('--target-query', type=str, default='',
                        help='Text query for the target object class, e.g. "cup" or "bottle"')
    parser.add_argument('--target-selector', type=str, default='open_clip', choices=['clip', 'open_clip'],
                        help='Backend used to map a text query onto SAM segments (open_clip recommended)')
    parser.add_argument('--target-selector-model', type=str, default='ViT-H-14',
                        help='Model name/path for the text-guided target selector (open_clip: ViT-H-14 or safetensors path)')
    parser.add_argument('--target-selector-device', type=str, default='',
                        help='Device for target selector inference, defaults to cuda if available else cpu')
    parser.add_argument('--target-min-score', type=float, default=None,
                        help='Optional minimum selector score required to accept a text-selected segment')
    parser.add_argument('--target-min-mask-area', type=int, default=400,
                        help='Minimum SAM mask area considered by the text-guided target selector')
    parser.add_argument('--debug-save-dir', type=str, default='',
                        help='Directory for saving RGB/segmentation/depth alignment debug images')
    parser.add_argument('--debug-save-every', type=int, default=1,
                        help='Save debug overlays every N frames when --debug-save-dir is set')
    parser.add_argument('--resize-segmap-to-depth', action='store_true',
                        help='Resize segmap to depth resolution with nearest-neighbor sampling before point-cloud extraction')
    parser.add_argument('--local-regions', action='store_true',
                        help='Crop 3D local regions around segmented objects before predicting grasps')
    parser.add_argument('--filter-grasps', action='store_true',
                        help='Filter grasp contacts so they stay on the segmented object surface')
    parser.add_argument('--filter-grasps-threshold', type=float, default=None,
                        help='Optional contact distance threshold in meters used by --filter-grasps')
    parser.add_argument('--segment-grasp-only', action='store_true',
                        help='Predict grasps directly on the target segment point cloud instead of a surrounding 3D box')
    parser.add_argument('--skip-border-objects', action='store_true',
                        help='Ignore segmented objects that touch the image border')
    parser.add_argument('--margin-px', type=int, default=5,
                        help='Pixel border margin used with --skip-border-objects')
    parser.add_argument('--forward-passes', type=int, default=1,
                        help='Number of model forward passes per frame')
    parser.add_argument('--sam-checkpoint', type=str, default='',
                        help='Path to a SAM checkpoint used when --segmentation-source sam')
    parser.add_argument('--sam-model-type', type=str, default='vit_b',
                        help='SAM backbone type, e.g. vit_b/vit_l/vit_h')
    parser.add_argument('--sam-device', type=str, default='',
                        help='Device for SAM inference, defaults to cuda if available else cpu')
    parser.add_argument('--sam-points-per-side', type=int, default=32,
                        help='SAM AutomaticMaskGenerator points_per_side')
    parser.add_argument('--sam-pred-iou-thresh', type=float, default=0.88,
                        help='SAM AutomaticMaskGenerator pred_iou_thresh')
    parser.add_argument('--sam-stability-score-thresh', type=float, default=0.95,
                        help='SAM AutomaticMaskGenerator stability_score_thresh')
    parser.add_argument('--sam-min-mask-area', type=int, default=400,
                        help='Minimum mask area kept in the SAM-generated segmap')
    parser.add_argument('--sam-max-mask-area-ratio', type=float, default=0.70,
                        help='Discard SAM masks larger than this fraction of the image area')
    parser.add_argument('--workspace-bounds', type=str, default='',
                        help='Execution workspace bounds as [[xmin,xmax],[ymin,ymax],[zmin,zmax]] in execution frame')
    parser.add_argument('--execution-reference-frame', type=str, default='world',
                        help='Planning/reference frame used for local reachability filtering')
    parser.add_argument('--execution-base-position-world', type=str, default='[-50.6,-27.7,0.85]',
                        help='Robot base position in the execution reference frame, used for local reachability filtering')
    parser.add_argument('--execution-base-yaw-deg', type=float, default=180.0,
                        help='Robot base yaw in degrees in the execution reference frame, used for local reachability filtering')
    parser.add_argument('--execution-base-workspace-bounds', type=str, default='[[-0.20,0.55],[-0.55,0.35],[-0.20,0.60]]',
                        help='Robot workspace bounds in base frame as [[xmin,xmax],[ymin,ymax],[zmin,zmax]]')
    parser.add_argument('--execution-camera-translation-gripper', type=str, default='[-0.0763,0.0039,0.035]',
                        help='Translation from gripper_base to end_cam used by local reachability filtering')
    parser.add_argument('--execution-camera-quaternion-gripper', type=str, default='[-0.120,0.124,-0.697,0.696]',
                        help='Quaternion [x,y,z,w] from gripper_base to end_cam used by local reachability filtering')
    parser.add_argument('--execution-top-k', type=int, default=None,
                        help='Restrict execution selection to the top-k scored candidates after workspace filtering')
    parser.add_argument('--target-min-points', type=int, default=0,
                        help='Minimum number of 3D points required on the selected target segment before execution is allowed')
    parser.add_argument('--target-stable-frames', type=int, default=1,
                        help='Require the selected target to remain spatially stable for this many consecutive frames before execution')
    parser.add_argument('--target-stability-centroid-tol-px', type=float, default=40.0,
                        help='Maximum target centroid motion in pixels allowed between consecutive stable frames')
    parser.add_argument('--target-stability-depth-tol-m', type=float, default=0.05,
                        help='Maximum target median-depth change in meters allowed between consecutive stable frames')
    parser.add_argument('--max-contact-distance-to-target', type=float, default=None,
                        help='Reject execution when the chosen grasp contact point is farther than this distance from the target segment point cloud')
    parser.add_argument('--target-lock', action='store_true',
                        help='Lock onto the selected target across frames using segmap geometry after it becomes stable')
    parser.add_argument('--target-lock-max-centroid-shift-px', type=float, default=60.0,
                        help='Maximum centroid motion in pixels allowed when matching the locked target across frames')
    parser.add_argument('--target-lock-max-depth-delta-m', type=float, default=0.08,
                        help='Maximum median-depth change in meters allowed when matching the locked target across frames')
    parser.add_argument('--target-lock-max-area-ratio-change', type=float, default=3.0,
                        help='Maximum area ratio change allowed when matching the locked target across frames')
    parser.add_argument('--disable-depth-preprocess', action='store_true',
                        help='Disable lightweight real-robot depth cleanup before point-cloud extraction')
    parser.add_argument('--end-pose-is-camera-pose', action='store_true',
                        help='If set, sensor_data.end_pose is camera pose in world frame, not gripper pose. '
                             'Use when Jetson camera_pose_publisher has already applied hand-eye calibration.')
    args = parser.parse_args()

    # Parse z_range
    z_range = eval(args.z_range)

    # Determine visualization setting
    visualize = args.visualize and not args.no_visualize
    workspace_bounds = parse_workspace_bounds(args.workspace_bounds)
    execution_base_position_world = parse_vector3(args.execution_base_position_world)
    execution_base_workspace_bounds = parse_workspace_bounds(args.execution_base_workspace_bounds)
    execution_camera_translation_gripper = parse_vector3(args.execution_camera_translation_gripper)
    execution_camera_quaternion_gripper = parse_quaternion(args.execution_camera_quaternion_gripper)
    segmenter = None
    target_selector = None
    effective_use_segmap = args.use_segmap or bool(args.target_query)
    effective_local_regions = args.local_regions or args.segment_grasp_only

    if effective_use_segmap and args.segmentation_source == 'sam':
        print(f"Loading segmentation backend: SAM ({args.sam_model_type})")
        segmenter = build_segmenter(
            'sam',
            checkpoint=args.sam_checkpoint,
            model_type=args.sam_model_type,
            device=args.sam_device,
            points_per_side=args.sam_points_per_side,
            pred_iou_thresh=args.sam_pred_iou_thresh,
            stability_score_thresh=args.sam_stability_score_thresh,
            min_mask_area=args.sam_min_mask_area,
            max_mask_area_ratio=args.sam_max_mask_area_ratio,
        )

    if args.target_query:
        print(f"Loading target selector: {args.target_selector} ({args.target_selector_model})")
        target_selector = build_target_selector(
            selector_type=args.target_selector,
            model_name=args.target_selector_model,
            device=args.target_selector_device,
        )

    # Load model
    print(f"Loading model from {args.ckpt_dir}")
    grasp_estimator = load_model(args.ckpt_dir)

    # Check CUDA
    if torch.cuda.is_available():
        print(f"CUDA available: {torch.cuda.get_device_name(0)}")
    else:
        print("CUDA not available, using CPU")

    print(f"\nConfiguration:")
    print(f"  Remote IP: {args.remote_ip}")
    print(f"  Sensor port: {args.sensor_port}")
    print(f"  Grasp port: {args.grasp_port}")
    print(f"  Z range: {z_range}")
    print(f"  Visualization: {visualize}")
    print(f"  Depth preprocess: {not args.disable_depth_preprocess}")
    print(f"  Use segmap: {effective_use_segmap}")
    if effective_use_segmap:
        print(f"  Segmentation source: {args.segmentation_source}")
        print(f"  Segmap id: {args.segmap_id}")
        print(f"  Target query: {args.target_query or '(none)'}")
        print(f"  Local regions: {effective_local_regions}")
        print(f"  Filter grasps: {args.filter_grasps}")
        print(f"  Segment grasp only: {args.segment_grasp_only}")
        if args.filter_grasps_threshold is not None:
            print(f"  Filter grasps threshold: {args.filter_grasps_threshold}")
        if args.segmentation_source == 'sam':
            print(f"  SAM checkpoint: {args.sam_checkpoint}")
            print(f"  SAM model type: {args.sam_model_type}")
            print(f"  SAM device: {segmenter.device}")
        if target_selector is not None:
            print(f"  Target selector: {target_selector.describe()}")
        print(f"  Resize segmap to depth: {args.resize_segmap_to_depth}")
        if args.target_min_points > 0:
            print(f"  Target min points: {args.target_min_points}")
        if args.target_stable_frames > 1:
            print(f"  Target stable frames: {args.target_stable_frames}")
            print(f"  Target centroid tol px: {args.target_stability_centroid_tol_px}")
            print(f"  Target depth tol m: {args.target_stability_depth_tol_m}")
        print(f"  Target lock: {args.target_lock}")
        if args.target_lock:
            print(f"  Target lock centroid tol px: {args.target_lock_max_centroid_shift_px}")
            print(f"  Target lock depth tol m: {args.target_lock_max_depth_delta_m}")
            print(f"  Target lock max area ratio: {args.target_lock_max_area_ratio_change}")
    if args.debug_save_dir:
        print(f"  Debug save dir: {args.debug_save_dir}")
        print(f"  Debug save every: {args.debug_save_every}")
    if workspace_bounds is not None:
        print(f"  Workspace bounds: {workspace_bounds.tolist()}")
    print(f"  Execute best grasp: {args.execute_best_grasp}")
    if args.execute_best_grasp:
        print(f"  Execution frame: {args.execution_frame}")
        print(f"  Execution reference frame: {args.execution_reference_frame}")
        print(f"  Min grasp score: {args.min_grasp_score}")
        print(f"  Pregrasp offset: {args.pregrasp_offset}")
        print(f"  Retreat offset: {args.retreat_offset}")
        print(f"  Max pregrasp offset: {args.max_pregrasp_offset}")
        print(f"  Max retreat offset: {args.max_retreat_offset}")
        print(f"  Execution base position: {execution_base_position_world.tolist()}")
        print(f"  Execution base yaw deg: {args.execution_base_yaw_deg}")
        if execution_base_workspace_bounds is None:
            print("  Execution base workspace bounds: disabled")
        else:
            print(f"  Execution base workspace bounds: {execution_base_workspace_bounds.tolist()}")
        print(f"  Execution camera translation: {execution_camera_translation_gripper.tolist()}")
        print(f"  Execution camera quaternion: {execution_camera_quaternion_gripper.tolist()}")
        print(f"  End pose is camera pose: {args.end_pose_is_camera_pose}")
        print(f"  Execution top-k: {args.execution_top_k}")
        if args.max_contact_distance_to_target is not None:
            print(f"  Max contact distance to target: {args.max_contact_distance_to_target}")
    print(f"  Mode: {'continuous' if args.continuous else 'single frame'}")

    if args.continuous:
        run_continuous_inference(
            grasp_estimator,
            args.remote_ip,
            z_range,
            visualize,
            sensor_port=args.sensor_port,
            grasp_port=args.grasp_port,
            execute_best_grasp=args.execute_best_grasp,
            min_grasp_score=args.min_grasp_score,
            execution_frame=args.execution_frame,
            pregrasp_offset=args.pregrasp_offset,
            retreat_offset=args.retreat_offset,
            max_pregrasp_offset=args.max_pregrasp_offset,
            max_retreat_offset=args.max_retreat_offset,
            segmentation_source=args.segmentation_source,
            segmenter=segmenter,
            target_query=args.target_query,
            target_selector=target_selector,
            target_min_score=args.target_min_score,
            target_min_mask_area=args.target_min_mask_area,
            debug_save_dir=args.debug_save_dir,
            debug_save_every=args.debug_save_every,
            resize_segmap_to_depth=args.resize_segmap_to_depth,
            use_segmap=effective_use_segmap,
            segmap_id=args.segmap_id,
            local_regions=effective_local_regions,
            filter_grasps=args.filter_grasps,
            use_cam_boxes=not args.segment_grasp_only,
            filter_grasps_threshold=args.filter_grasps_threshold,
            skip_border_objects=args.skip_border_objects,
            margin_px=args.margin_px,
            forward_passes=args.forward_passes,
            workspace_bounds=workspace_bounds,
            execution_top_k=args.execution_top_k,
            target_min_points=args.target_min_points,
            target_stable_frames=args.target_stable_frames,
            target_stability_centroid_tol_px=args.target_stability_centroid_tol_px,
            target_stability_depth_tol_m=args.target_stability_depth_tol_m,
            max_contact_distance_to_target=args.max_contact_distance_to_target,
            target_lock=args.target_lock,
            target_lock_max_centroid_shift_px=args.target_lock_max_centroid_shift_px,
            target_lock_max_depth_delta_m=args.target_lock_max_depth_delta_m,
            target_lock_max_area_ratio_change=args.target_lock_max_area_ratio_change,
            execution_reference_frame=args.execution_reference_frame,
            execution_base_position_world=execution_base_position_world,
            execution_base_yaw_deg=args.execution_base_yaw_deg,
            execution_base_workspace_bounds=execution_base_workspace_bounds,
            execution_camera_translation_gripper=execution_camera_translation_gripper,
            execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
            depth_preprocess=not args.disable_depth_preprocess,
            end_pose_is_camera_pose=args.end_pose_is_camera_pose,
        )
    else:
        # Single frame mode
        client = IsaacSimClient(
            remote_ip=args.remote_ip,
            sensor_port=args.sensor_port,
            grasp_port=args.grasp_port,
        )
        client.connect()

        print(f"\nWaiting for sensor data (timeout: {args.timeout}s)...")

        result = run_single_inference(
            grasp_estimator,
            client,
            z_range,
            visualize,
            execute_best_grasp=args.execute_best_grasp,
            min_grasp_score=args.min_grasp_score,
            execution_frame=args.execution_frame,
            pregrasp_offset=args.pregrasp_offset,
            retreat_offset=args.retreat_offset,
            max_pregrasp_offset=args.max_pregrasp_offset,
            max_retreat_offset=args.max_retreat_offset,
            segmentation_source=args.segmentation_source,
            segmenter=segmenter,
            target_query=args.target_query,
            target_selector=target_selector,
            target_min_score=args.target_min_score,
            target_min_mask_area=args.target_min_mask_area,
            debug_save_dir=args.debug_save_dir,
            debug_save_every=args.debug_save_every,
            resize_segmap_to_depth=args.resize_segmap_to_depth,
            use_segmap=effective_use_segmap,
            segmap_id=args.segmap_id,
            local_regions=effective_local_regions,
            filter_grasps=args.filter_grasps,
            use_cam_boxes=not args.segment_grasp_only,
            filter_grasps_threshold=args.filter_grasps_threshold,
            skip_border_objects=args.skip_border_objects,
            margin_px=args.margin_px,
            forward_passes=args.forward_passes,
            workspace_bounds=workspace_bounds,
            execution_top_k=args.execution_top_k,
            target_min_points=args.target_min_points,
            max_contact_distance_to_target=args.max_contact_distance_to_target,
            execution_reference_frame=args.execution_reference_frame,
            execution_base_position_world=execution_base_position_world,
            execution_base_yaw_deg=args.execution_base_yaw_deg,
            execution_base_workspace_bounds=execution_base_workspace_bounds,
            execution_camera_translation_gripper=execution_camera_translation_gripper,
            execution_camera_quaternion_gripper=execution_camera_quaternion_gripper,
            timeout_s=args.timeout,
            depth_preprocess=not args.disable_depth_preprocess,
            end_pose_is_camera_pose=args.end_pose_is_camera_pose,
        )

        client.disconnect()

        if result and count_predicted_grasps(result['scores']) > 0:
            print("\n=== Inference Completed ===")
            print(f"Generated {count_predicted_grasps(result['scores'])} grasps")
        else:
            print("\nNo grasps generated or no data received")


if __name__ == '__main__':
    main()