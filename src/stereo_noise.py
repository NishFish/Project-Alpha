#!/usr/bin/env python3
"""
Corrupt a clean depth frame the way real stereo matching does.

Gazebo's depth camera is perfect: every pixel is exact, nothing ever drops out.
That makes the percentile aggregation and the temporal filter in
`depth_to_obstacle_ring.py` look unnecessary, because in a noiseless world they
are. The point of this module is to find out whether those filters are actually
tuned correctly, now, rather than on the aircraft.

The error model is derived rather than invented. Stereo depth comes from
disparity, z = f * B / d, so an error in disparity propagates as

    dz = z^2 * dd / (f * B)

with f the focal length in pixels, B the baseline in metres, and dd the
disparity matching error in pixels (subpixel interpolation gets this to roughly
0.1-0.3 px on a decent matcher). The z-squared term is the important part: error
grows quadratically with range, which is why a camera that is accurate at 2 m is
useless at 20 m.

For the OAK-D Lite -- B = 0.075 m -- and our 320 px wide 72 degree camera,
f = 220 px, dd = 0.2 px:

    2 m  -> 0.05 m
    5 m  -> 0.30 m
    10 m -> 1.21 m
    20 m -> 4.85 m

Dropouts are modelled as blobs, not speckle. Real stereo fails over *regions*
that lack texture -- a blank wall, calm water, open sky -- and a blob that wipes
out most of a sector is a completely different problem from scattered bad pixels
that a percentile shrugs off.
"""

import numpy as np

OAKD_LITE_BASELINE_M = 0.075


def depth_sigma(depth_m, focal_px, baseline_m, disparity_sigma_px):
    """Standard deviation of the depth error, in metres, per pixel."""
    return (depth_m ** 2) * disparity_sigma_px / (focal_px * baseline_m)


def corrupt(depth_m, focal_px, baseline_m=OAKD_LITE_BASELINE_M,
            disparity_sigma_px=0.2, dropout_near=0.02, dropout_far=0.15,
            blob_px=8, max_range_m=20.0, rng=None):
    """Return a corrupted copy of `depth_m`. Dropped pixels become +inf.

    dropout_near / dropout_far bracket the probability that a region is lost:
    near surfaces usually match, distant ones often do not. Everything in
    between is interpolated on (range / max_range)^2, matching how quickly
    matching confidence falls away.
    """
    if rng is None:
        rng = np.random.default_rng()

    depth = np.asarray(depth_m, dtype=np.float32).copy()
    h, w = depth.shape
    finite = np.isfinite(depth)

    # --- range-dependent gaussian error -------------------------------------
    sigma = np.zeros_like(depth)
    sigma[finite] = depth_sigma(depth[finite], focal_px, baseline_m,
                                disparity_sigma_px)
    noise = rng.standard_normal(depth.shape).astype(np.float32) * sigma
    depth[finite] += noise[finite]

    # A negative or zero range is not a plausible reading, it is a failure.
    depth[finite & (depth <= 0.0)] = np.inf

    # --- blob dropouts -------------------------------------------------------
    # A coarse random field, upsampled, so losses come in patches rather than as
    # isolated pixels. Anything finer than this is speckle, which the percentile
    # already handles and which therefore tests nothing.
    bh = max(1, h // blob_px)
    bw = max(1, w // blob_px)
    coarse = rng.random((bh, bw)).astype(np.float32)
    field = np.repeat(np.repeat(coarse, int(np.ceil(h / bh)), axis=0),
                      int(np.ceil(w / bw)), axis=1)[:h, :w]

    frac = np.zeros_like(depth)
    ok = np.isfinite(depth)
    frac[ok] = np.clip(depth[ok] / max_range_m, 0.0, 1.0)
    p_drop = dropout_near + (dropout_far - dropout_near) * frac ** 2

    depth[ok & (field < p_drop)] = np.inf
    return depth


def summarise(depth_m):
    """Quick stats, for logging what the corruption actually did."""
    finite = np.isfinite(depth_m)
    n = depth_m.size
    out = {"valid_frac": float(finite.sum()) / n if n else 0.0}
    if finite.any():
        v = depth_m[finite]
        out["min_m"] = float(v.min())
        out["max_m"] = float(v.max())
        out["median_m"] = float(np.median(v))
    return out
