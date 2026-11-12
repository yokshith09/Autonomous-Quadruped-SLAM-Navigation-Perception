"""Camera check for loop closures.

A 2D scan of a room full of look-alike walls can match the wrong place well (F36/F51: about a third of accepted
closures made the estimate worse). The colour image of the same spot is far more distinctive, so a closure is only
trusted when ORB features of the current view and of the matched keyframe's view agree geometrically.
"""
from typing import Optional, Tuple

import cv2
import numpy as np

THUMB_SIZE = (192, 144)   # (width, height) of the stored grayscale view
Features = Tuple[np.ndarray, np.ndarray]  # (keypoint xy float32 (N, 2), ORB descriptors uint8 (N, 32))

_orb = cv2.ORB_create(nfeatures=400, fastThreshold=12)
_matcher = cv2.BFMatcher(cv2.NORM_HAMMING)


def make_thumb(bgr: np.ndarray) -> np.ndarray:
    """Small grayscale view used for matching (and recorded, so closures can be re-judged offline)."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
    return cv2.resize(gray, THUMB_SIZE, interpolation=cv2.INTER_AREA)


def describe(thumb: np.ndarray) -> Optional[Features]:
    kps, desc = _orb.detectAndCompute(thumb, None)
    if desc is None or len(kps) < 8:
        return None
    return np.float32([k.pt for k in kps]), desc


def count_inliers(a: Optional[Features], b: Optional[Features], ratio: float = 0.75, thresh_px: float = 2.0) -> int:
    """Number of feature matches between two views that agree with one camera motion (epipolar RANSAC)."""
    if a is None or b is None:
        return 0
    pairs = _matcher.knnMatch(a[1], b[1], k=2)
    good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < ratio * p[1].distance]
    if len(good) < 8:
        return 0
    pa = a[0][[m.queryIdx for m in good]]
    pb = b[0][[m.trainIdx for m in good]]
    # Same pose again: features have not moved, and a fundamental matrix is undefined for zero motion.
    static = int((np.hypot(*(pa - pb).T) < thresh_px).sum())
    try:
        _, mask = cv2.findFundamentalMat(pa, pb, cv2.FM_RANSAC, thresh_px, 0.99)
    except cv2.error:  # degenerate point sets (all matches on one spot or line)
        return static
    return max(static, 0 if mask is None else int(mask.sum()))
