"""Utilities for generating instance segmaps from raw RGB images."""

from dataclasses import dataclass
from typing import Optional

import numpy as np
import torch


@dataclass
class SamConfig:
    """Configuration for SAM automatic mask generation."""
    checkpoint: str
    model_type: str = 'vit_b'
    device: str = ''
    points_per_side: int = 32
    pred_iou_thresh: float = 0.88
    stability_score_thresh: float = 0.95
    min_mask_area: int = 400
    max_mask_area_ratio: float = 0.70


class SamAutomaticSegmenter:
    """Generate Contact-GraspNet-compatible integer instance maps using SAM."""

    def __init__(self, config: SamConfig):
        try:
            from segment_anything import SamAutomaticMaskGenerator, sam_model_registry
        except ImportError as exc:
            raise ImportError(
                "segment_anything is required for --segmentation-source sam. "
                "Install it in the current environment before running inference."
            ) from exc

        if not config.checkpoint:
            raise ValueError('SAM checkpoint path is required')

        device = config.device or ('cuda' if torch.cuda.is_available() else 'cpu')
        if device.startswith('cuda') and not torch.cuda.is_available():
            raise ValueError('SAM requested CUDA device but torch.cuda.is_available() is False')

        if config.model_type not in sam_model_registry:
            raise ValueError(f"Unsupported SAM model type '{config.model_type}'")

        sam_model = sam_model_registry[config.model_type](checkpoint=config.checkpoint)
        sam_model.to(device=device)

        self._generator = SamAutomaticMaskGenerator(
            model=sam_model,
            points_per_side=config.points_per_side,
            pred_iou_thresh=config.pred_iou_thresh,
            stability_score_thresh=config.stability_score_thresh,
            min_mask_region_area=config.min_mask_area,
        )
        self._config = config
        self.device = device

    def describe(self) -> str:
        """Return a compact human-readable backend description."""
        return f"sam:{self._config.model_type}@{self.device}"

    def generate(self, rgb: np.ndarray) -> np.ndarray:
        """Generate an HxW integer instance segmap with 0 reserved for background."""
        if rgb is None:
            raise ValueError('RGB image is required for SAM segmentation')

        rgb = np.asarray(rgb)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f'Expected RGB image with shape HxWx3, got {rgb.shape}')

        rgb = rgb.astype(np.uint8, copy=False)
        image_area = rgb.shape[0] * rgb.shape[1]
        max_mask_area = int(self._config.max_mask_area_ratio * image_area)

        masks = self._generator.generate(rgb)
        segmap = np.zeros(rgb.shape[:2], dtype=np.int32)

        next_id = 1
        for mask_data in sorted(masks, key=lambda item: item.get('area', 0), reverse=True):
            area = int(mask_data.get('area', 0))
            if area < self._config.min_mask_area:
                continue
            if area > max_mask_area:
                continue

            mask = np.asarray(mask_data.get('segmentation'), dtype=bool)
            if mask.shape != segmap.shape:
                continue
            if not np.any(mask):
                continue

            segmap[mask] = next_id
            next_id += 1

        return segmap


def build_segmenter(source: str, **kwargs) -> Optional[SamAutomaticSegmenter]:
    """Build a local segmentation backend."""
    if source == 'sam':
        return SamAutomaticSegmenter(SamConfig(**kwargs))
    if source == 'remote':
        return None
    raise ValueError(f"Unsupported segmentation source '{source}'")
