"""Text-guided target selection over instance segmentation masks."""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

import numpy as np
import torch


@dataclass
class SegmentCrop:
    """Masked crop extracted for one segment instance."""
    segment_id: int
    area: int
    bbox_xyxy: np.ndarray
    bbox_width: int
    bbox_height: int
    masked_image: np.ndarray
    context_image: np.ndarray
    fill_ratio: float
    bbox_fraction: float
    min_side_fraction: float
    aspect_ratio: float
    touches_image_border: bool


@dataclass
class TargetSelection:
    """Result of selecting a target instance from a segmap."""
    segment_id: int
    score: float
    query: str
    scores_by_segment: Dict[int, float]
    masked_score: float = 0.0
    context_score: float = 0.0
    distractor_score: float = 0.0
    agreement_score: float = 0.0
    query_advantage: float = 0.0
    masked_query_advantage: float = 0.0
    context_bias: float = 0.0
    selection_margin: float = 0.0


def build_segment_crops(rgb: np.ndarray,
                        segmap: np.ndarray,
                        candidate_ids: Optional[Iterable[int]] = None,
                        min_area: int = 400,
                        pad_px: int = 8) -> List[SegmentCrop]:
    """Extract masked and contextual RGB crops for each candidate segment id."""
    if rgb is None:
        raise ValueError('RGB image is required for text-guided target selection')

    rgb = np.asarray(rgb)
    segmap = np.asarray(segmap)
    if rgb.ndim != 3 or rgb.shape[:2] != segmap.shape[:2]:
        raise ValueError(
            f'RGB/segmap shape mismatch: rgb={rgb.shape}, segmap={segmap.shape}'
        )

    segment_ids = list(candidate_ids) if candidate_ids is not None else [
        int(seg_id) for seg_id in np.unique(segmap) if int(seg_id) > 0
    ]

    crops: List[SegmentCrop] = []
    height, width = segmap.shape[:2]

    for seg_id in segment_ids:
        mask = segmap == seg_id
        area = int(mask.sum())
        if area < min_area:
            continue

        ys, xs = np.where(mask)
        if ys.size == 0:
            continue

        x0 = max(0, int(xs.min()) - pad_px)
        y0 = max(0, int(ys.min()) - pad_px)
        x1 = min(width, int(xs.max()) + 1 + pad_px)
        y1 = min(height, int(ys.max()) + 1 + pad_px)
        bbox_width = max(1, x1 - x0)
        bbox_height = max(1, y1 - y0)

        context_crop = rgb[y0:y1, x0:x1].copy()
        crop_mask = mask[y0:y1, x0:x1]
        masked_crop = context_crop.copy()
        masked_crop[~crop_mask] = 0
        fill_ratio = float(area) / float(max(1, bbox_height * bbox_width))
        bbox_fraction = float(bbox_height * bbox_width) / float(max(1, height * width))
        min_side_fraction = float(min(bbox_width, bbox_height)) / float(max(1, min(height, width)))
        aspect_ratio = float(max(bbox_width, bbox_height)) / float(max(1, min(bbox_width, bbox_height)))
        touches_image_border = bool(x0 == 0 or y0 == 0 or x1 == width or y1 == height)

        crops.append(
            SegmentCrop(
                segment_id=int(seg_id),
                area=area,
                bbox_xyxy=np.asarray([x0, y0, x1, y1], dtype=np.int32),
                bbox_width=bbox_width,
                bbox_height=bbox_height,
                masked_image=masked_crop,
                context_image=context_crop,
                fill_ratio=fill_ratio,
                bbox_fraction=bbox_fraction,
                min_side_fraction=min_side_fraction,
                aspect_ratio=aspect_ratio,
                touches_image_border=touches_image_border,
            )
        )

    return crops


def _bbox_overlap_metrics(bbox_a: np.ndarray, bbox_b: np.ndarray):
    """Return x/y overlap ratios and IoU for two xyxy boxes."""
    ax0, ay0, ax1, ay1 = [int(v) for v in np.asarray(bbox_a, dtype=np.int32)]
    bx0, by0, bx1, by1 = [int(v) for v in np.asarray(bbox_b, dtype=np.int32)]
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


def _crops_likely_same_instance(crop_a: SegmentCrop, crop_b: SegmentCrop) -> bool:
    """Heuristic grouping for duplicated/fragmented segments of one instance."""
    x_overlap_ratio, y_overlap_ratio, iou = _bbox_overlap_metrics(
        crop_a.bbox_xyxy,
        crop_b.bbox_xyxy,
    )
    area_a = max(1, int(crop_a.area))
    area_b = max(1, int(crop_b.area))
    area_ratio = max(area_a / float(area_b), area_b / float(area_a))

    strongly_nested = (
        x_overlap_ratio >= 0.75 and y_overlap_ratio >= 0.45 and area_ratio <= 8.0
    )
    overlapping_body = (
        iou >= 0.10 and x_overlap_ratio >= 0.45 and y_overlap_ratio >= 0.45 and area_ratio <= 6.0
    )
    return bool(strongly_nested or overlapping_body)


class ClipTargetSelector:
    """Score masked instance crops against a text query with CLIP."""

    _MIN_SELECTION_MARGIN = 0.006
    _MIN_QUERY_ADVANTAGE = 0.000
    _MIN_AGREEMENT_SCORE = 0.22
    _MIN_MASKED_QUERY_ADVANTAGE = -0.025
    _MAX_CONTEXT_BIAS = 0.090

    _DISTRACTOR_LABELS = (
        'robot arm',
        'gripper',
        'machine',
        'control panel',
        'instrument',
        'device',
        'table',
        'workbench',
        'chair',
        'floor',
        'wall',
        'cabinet',
        'drawer',
        'shelf',
        'door',
        'window',
        'box',
        'paper',
        'cardboard',
        'background',
    )

    def __init__(self, model_name: str, device: str = '', use_open_clip: bool = False):
        self.use_open_clip = use_open_clip
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
        self._last_selection_debug: Dict[str, object] = {}
        if self.device.startswith('cuda') and not torch.cuda.is_available():
            raise ValueError('CLIP target selector requested CUDA device but CUDA is unavailable')

        if use_open_clip:
            try:
                import open_clip
            except ImportError as exc:
                raise ImportError(
                    'open-clip-torch is required for open_clip models. '
                    'Install it in the current environment before running inference.'
                ) from exc

            # Parse model_name: could be "ViT-H-14" or a path to safetensors
            if model_name.endswith('.safetensors') or '/' in model_name:
                # Path to model weights
                self._model, _, self._preprocess = open_clip.create_model_and_transforms(
                    'ViT-H-14', pretrained=model_name
                )
                self._tokenizer = open_clip.get_tokenizer('ViT-H-14')
                self._arch = 'ViT-H-14'
            else:
                # Architecture name, use default pretrained
                self._model, _, self._preprocess = open_clip.create_model_and_transforms(
                    model_name, pretrained='laion2B_s32B_b79K'
                )
                self._tokenizer = open_clip.get_tokenizer(model_name)
                self._arch = model_name

            self._model.to(self.device)
            self._model.eval()
        else:
            try:
                from transformers import AutoProcessor, CLIPModel
            except ImportError as exc:
                raise ImportError(
                    'transformers is required for --target-selector clip. '
                    'Install it in the current environment before running inference.'
                ) from exc

            self._processor = AutoProcessor.from_pretrained(model_name)
            self._model = CLIPModel.from_pretrained(model_name)
            self._model.to(self.device)
            self._model.eval()

        self.model_name = model_name

    @staticmethod
    def _build_query_prompts(query: str) -> List[str]:
        """Use a small prompt ensemble for more robust zero-shot matching."""
        query = str(query).strip()
        return [
            query,
            f'a photo of a {query}',
            f'a close-up photo of a {query}',
        ]

    @staticmethod
    def _score_agreement(masked_scores: np.ndarray, context_scores: np.ndarray) -> np.ndarray:
        """Prefer candidates whose masked/context crops agree semantically."""
        masked_scores = np.asarray(masked_scores, dtype=np.float32)
        context_scores = np.asarray(context_scores, dtype=np.float32)
        score_gap = np.abs(masked_scores - context_scores)
        return np.clip(1.0 - score_gap / 0.12, 0.0, 1.0).astype(np.float32)

    @staticmethod
    def _combine_scores(masked_scores: np.ndarray, context_scores: np.ndarray) -> np.ndarray:
        """
        Blend masked-object and contextual scores.

        The minimum term rewards agreement between the isolated object crop and
        the surrounding local context, which helps suppress foreground fragments
        that look target-like only after masking.
        """
        masked_scores = np.asarray(masked_scores, dtype=np.float32)
        context_scores = np.asarray(context_scores, dtype=np.float32)
        agreement_scores = np.minimum(masked_scores, context_scores)
        return (
            0.50 * masked_scores +
            0.10 * context_scores +
            0.40 * agreement_scores
        )

    @staticmethod
    def _compute_geometry_priors(crops: List[SegmentCrop]) -> np.ndarray:
        """
        Soft instance-completeness prior.

        Prefer candidates that cover a larger portion of a plausible object
        instance, but softly penalize very large segments that likely describe
        support surfaces or merged background structures.
        """
        if not crops:
            return np.empty((0,), dtype=np.float32)

        areas = np.asarray([max(1, int(crop.area)) for crop in crops], dtype=np.float32)
        fills = np.asarray([float(crop.fill_ratio) for crop in crops], dtype=np.float32)
        bbox_fracs = np.asarray([float(crop.bbox_fraction) for crop in crops], dtype=np.float32)
        min_side_fracs = np.asarray([float(crop.min_side_fraction) for crop in crops], dtype=np.float32)
        aspect_ratios = np.asarray([float(crop.aspect_ratio) for crop in crops], dtype=np.float32)
        border_penalties = np.asarray(
            [1.0 if crop.touches_image_border else 0.0 for crop in crops],
            dtype=np.float32,
        )

        log_areas = np.log1p(areas)
        if float(np.max(log_areas) - np.min(log_areas)) > 1e-6:
            area_scores = (log_areas - np.min(log_areas)) / (np.max(log_areas) - np.min(log_areas))
        else:
            area_scores = np.ones_like(log_areas, dtype=np.float32)

        fill_scores = np.clip((fills - 0.12) / 0.50, 0.0, 1.0)
        span_scores = np.clip((min_side_fracs - 0.03) / 0.10, 0.0, 1.0)
        thin_penalties = np.clip((aspect_ratios - 5.5) / 5.0, 0.0, 1.0)
        huge_penalty = np.clip((bbox_fracs - 0.18) / 0.18, 0.0, 1.0)
        return (
            0.32 * area_scores +
            0.28 * fill_scores +
            0.25 * span_scores -
            0.20 * huge_penalty -
            0.14 * border_penalties -
            0.16 * thin_penalties
        ).astype(np.float32)

    def _score_open_clip_query(self, images: List[np.ndarray], query: str) -> np.ndarray:
        return self._score_open_clip_images(images, self._build_query_prompts(query))

    def _score_hf_clip_query(self, images: List[np.ndarray], query: str) -> np.ndarray:
        return self._score_hf_clip_images(images, self._build_query_prompts(query))

    def _score_open_clip_distractors(self, images: List[np.ndarray]) -> np.ndarray:
        distractor_scores = [
            self._score_open_clip_query(images, label)
            for label in self._DISTRACTOR_LABELS
        ]
        return np.max(np.stack(distractor_scores, axis=0), axis=0)

    def _score_hf_clip_distractors(self, images: List[np.ndarray]) -> np.ndarray:
        distractor_scores = [
            self._score_hf_clip_query(images, label)
            for label in self._DISTRACTOR_LABELS
        ]
        return np.max(np.stack(distractor_scores, axis=0), axis=0)

    def _encode_open_clip_images(self, images: List[np.ndarray]) -> torch.Tensor:
        from PIL import Image
        pil_images = [Image.fromarray(img) for img in images]
        preprocessed = torch.stack([self._preprocess(img) for img in pil_images]).to(self.device)
        with torch.no_grad():
            image_features = self._model.encode_image(preprocessed)
        image_features = image_features / image_features.norm(dim=-1, keepdim=True)
        return image_features

    def _encode_open_clip_text(self, prompts: List[str]) -> torch.Tensor:
        text_tokens = self._tokenizer(prompts).to(self.device)
        with torch.no_grad():
            text_features = self._model.encode_text(text_tokens)
        text_features = text_features / text_features.norm(dim=-1, keepdim=True)
        return text_features.mean(dim=0, keepdim=True)

    def _score_open_clip_images(self, images: List[np.ndarray], prompts: List[str]) -> np.ndarray:
        image_features = self._encode_open_clip_images(images)
        text_feature = self._encode_open_clip_text(prompts)
        logits = (image_features @ text_feature.T).squeeze(-1)
        return logits.detach().float().cpu().numpy()

    def _score_hf_clip_images(self, images: List[np.ndarray], prompts: List[str]) -> np.ndarray:
        inputs = self._processor(
            text=prompts,
            images=images,
            return_tensors='pt',
            padding=True,
        )
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        with torch.no_grad():
            outputs = self._model(**inputs)
            image_embeds = outputs.image_embeds
            text_embeds = outputs.text_embeds
        image_embeds = image_embeds / image_embeds.norm(dim=-1, keepdim=True)
        text_embeds = text_embeds / text_embeds.norm(dim=-1, keepdim=True)
        text_feature = text_embeds.mean(dim=0, keepdim=True)
        logits = (image_embeds @ text_feature.T).squeeze(-1)
        return logits.detach().float().cpu().numpy()

    def describe(self) -> str:
        """Return a compact backend description."""
        backend = 'open_clip' if self.use_open_clip else 'clip'
        return f'{backend}:{self.model_name}@{self.device}'

    def get_last_selection_debug(self) -> Dict[str, object]:
        """Return diagnostics for the most recent selection attempt."""
        return dict(self._last_selection_debug)

    def select(self,
               rgb: np.ndarray,
               segmap: np.ndarray,
               query: str,
               candidate_ids: Optional[Iterable[int]] = None,
               min_mask_area: int = 400,
               pad_px: int = 8,
               min_score: Optional[float] = None) -> Optional[TargetSelection]:
        """Select the best-matching segment id for the text query."""
        crops = build_segment_crops(
            rgb,
            segmap,
            candidate_ids=candidate_ids,
            min_area=min_mask_area,
            pad_px=pad_px,
        )
        self._last_selection_debug = {
            'query': str(query),
            'selected': False,
            'reason': 'no_crops',
        }
        if not crops:
            return None

        masked_images = [crop.masked_image for crop in crops]
        context_images = [crop.context_image for crop in crops]

        if self.use_open_clip:
            masked_scores = self._score_open_clip_query(masked_images, query)
            context_scores = self._score_open_clip_query(context_images, query)
            masked_distractor_scores = self._score_open_clip_distractors(masked_images)
            context_distractor_scores = self._score_open_clip_distractors(context_images)
        else:
            masked_scores = self._score_hf_clip_query(masked_images, query)
            context_scores = self._score_hf_clip_query(context_images, query)
            masked_distractor_scores = self._score_hf_clip_distractors(masked_images)
            context_distractor_scores = self._score_hf_clip_distractors(context_images)

        positive_scores = self._combine_scores(masked_scores, context_scores)
        distractor_scores = self._combine_scores(
            masked_distractor_scores,
            context_distractor_scores,
        )
        geometry_priors = self._compute_geometry_priors(crops)
        agreement_scores = self._score_agreement(masked_scores, context_scores)
        query_advantages = positive_scores - distractor_scores
        masked_query_advantages = masked_scores - masked_distractor_scores
        context_biases = context_scores - masked_scores
        scores = (
            positive_scores -
            0.55 * distractor_scores +
            0.20 * geometry_priors +
            0.10 * agreement_scores +
            0.10 * np.clip(masked_query_advantages, -1.0, 1.0) -
            0.08 * np.clip(context_biases, 0.0, 1.0)
        )
        best_idx = int(np.argmax(scores))
        best_score = float(scores[best_idx])
        best_agreement = float(agreement_scores[best_idx])
        best_query_advantage = float(query_advantages[best_idx])
        best_masked_query_advantage = float(masked_query_advantages[best_idx])
        best_context_bias = float(context_biases[best_idx])
        raw_sorted_scores = np.sort(np.asarray(scores, dtype=np.float32))[::-1]
        raw_second_best_score = float(raw_sorted_scores[1]) if raw_sorted_scores.size > 1 else float('-inf')
        competing_score = float('-inf')
        competing_idx = None
        for idx, candidate_score in enumerate(scores):
            if idx == best_idx:
                continue
            if _crops_likely_same_instance(crops[best_idx], crops[idx]):
                continue
            if float(candidate_score) > competing_score:
                competing_score = float(candidate_score)
                competing_idx = int(idx)
        selection_margin = (
            best_score - competing_score
            if competing_idx is not None
            else float('inf')
        )
        self._last_selection_debug = {
            'query': str(query),
            'selected': True,
            'reason': 'selected',
            'best_segment_id': int(crops[best_idx].segment_id),
            'best_score': best_score,
            'masked_score': float(masked_scores[best_idx]),
            'context_score': float(context_scores[best_idx]),
            'distractor_score': float(distractor_scores[best_idx]),
            'agreement_score': best_agreement,
            'query_advantage': best_query_advantage,
            'masked_query_advantage': best_masked_query_advantage,
            'context_bias': best_context_bias,
            'selection_margin': float(selection_margin),
            'raw_selection_margin': float(best_score - raw_second_best_score),
            'competing_segment_id': None if competing_idx is None else int(crops[competing_idx].segment_id),
            'competing_score': None if competing_idx is None else float(competing_score),
        }
        if min_score is not None and best_score < min_score:
            self._last_selection_debug['selected'] = False
            self._last_selection_debug['reason'] = 'min_score'
            return None
        if best_agreement < self._MIN_AGREEMENT_SCORE:
            self._last_selection_debug['selected'] = False
            self._last_selection_debug['reason'] = 'agreement'
            return None
        if best_query_advantage < self._MIN_QUERY_ADVANTAGE:
            self._last_selection_debug['selected'] = False
            self._last_selection_debug['reason'] = 'query_advantage'
            return None
        if best_masked_query_advantage < self._MIN_MASKED_QUERY_ADVANTAGE:
            self._last_selection_debug['selected'] = False
            self._last_selection_debug['reason'] = 'masked_query_advantage'
            return None
        if best_context_bias > self._MAX_CONTEXT_BIAS:
            self._last_selection_debug['selected'] = False
            self._last_selection_debug['reason'] = 'context_bias'
            return None
        if selection_margin < self._MIN_SELECTION_MARGIN:
            self._last_selection_debug['selected'] = False
            self._last_selection_debug['reason'] = 'selection_margin'
            return None

        scores_by_segment = {
            crops[idx].segment_id: float(scores[idx])
            for idx in range(len(crops))
        }
        return TargetSelection(
            segment_id=crops[best_idx].segment_id,
            score=best_score,
            query=query,
            scores_by_segment=scores_by_segment,
            masked_score=float(masked_scores[best_idx]),
            context_score=float(context_scores[best_idx]),
            distractor_score=float(distractor_scores[best_idx]),
            agreement_score=best_agreement,
            query_advantage=best_query_advantage,
            masked_query_advantage=best_masked_query_advantage,
            context_bias=best_context_bias,
            selection_margin=float(selection_margin),
        )


def build_target_selector(selector_type: str,
                          model_name: str,
                          device: str = '',
                          use_open_clip: bool = False) -> ClipTargetSelector:
    """Build a text-guided target selector backend."""
    if selector_type == 'clip':
        return ClipTargetSelector(model_name=model_name, device=device, use_open_clip=use_open_clip)
    elif selector_type == 'open_clip':
        return ClipTargetSelector(model_name=model_name, device=device, use_open_clip=True)
    raise ValueError(f"Unsupported target selector '{selector_type}'")