#!/usr/bin/env python3
"""Checks for the stereo noise model. No simulator needed."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stereo_noise import corrupt, depth_sigma, OAKD_LITE_BASELINE_M

passed = failed = 0


def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print("  ok   %s" % name)
    else:
        failed += 1
        print("  FAIL %s  %s" % (name, detail))


F = 220.2   # our simulated camera
B = OAKD_LITE_BASELINE_M

# --- the error model itself ------------------------------------------------
s2 = depth_sigma(2.0, F, B, 0.2)
s5 = depth_sigma(5.0, F, B, 0.2)
s10 = depth_sigma(10.0, F, B, 0.2)
s20 = depth_sigma(20.0, F, B, 0.2)
print("  sigma: 2m=%.3f  5m=%.3f  10m=%.3f  20m=%.3f" % (s2, s5, s10, s20))

check("error grows with range", s2 < s5 < s10 < s20)
check("error is quadratic, not linear",
      abs((s20 / s10) - 4.0) < 0.01, "ratio %.3f, want 4" % (s20 / s10))
check("2 m error is a few centimetres", 0.02 < s2 < 0.10, "%.3f" % s2)
check("20 m error is metres, not centimetres", s20 > 3.0, "%.3f" % s20)

# --- corruption ------------------------------------------------------------
rng = np.random.default_rng(1234)
clean = np.full((240, 320), 5.0, dtype=np.float32)
dirty = corrupt(clean, F, rng=rng)

check("shape preserved", dirty.shape == clean.shape)
check("dtype stays float32", dirty.dtype == np.float32)

finite = np.isfinite(dirty)
check("some pixels dropped out", (~finite).any())
check("most pixels survive at 5 m", finite.mean() > 0.8,
      "%.2f survived" % finite.mean())

err = dirty[finite] - 5.0
check("measured spread matches the model",
      abs(err.std() - s5) < 0.05, "measured %.3f, model %.3f" % (err.std(), s5))
check("error is unbiased", abs(err.mean()) < 0.03, "%.4f" % err.mean())

# --- dropouts increase with range -----------------------------------------
near = corrupt(np.full((240, 320), 2.0, np.float32), F,
               rng=np.random.default_rng(7))
far = corrupt(np.full((240, 320), 18.0, np.float32), F,
              rng=np.random.default_rng(7))
near_lost = 1.0 - np.isfinite(near).mean()
far_lost = 1.0 - np.isfinite(far).mean()
print("  dropout: 2m=%.1f%%  18m=%.1f%%" % (near_lost * 100, far_lost * 100))
check("distant surfaces drop out more often", far_lost > near_lost * 2)

# --- dropouts are blobs, not speckle --------------------------------------
mask = ~np.isfinite(far)
if mask.any():
    # A blobby mask has far fewer horizontal transitions than random speckle.
    transitions = np.diff(mask.astype(np.int8), axis=1) != 0
    rate = transitions.mean()
    check("dropouts come in patches, not speckle", rate < 0.06,
          "transition rate %.3f" % rate)
else:
    check("dropouts come in patches, not speckle", False, "no dropouts at all")

# --- infinities survive, negatives never appear ----------------------------
with_inf = np.full((60, 80), 5.0, np.float32)
with_inf[:20, :] = np.inf
out = corrupt(with_inf, F, rng=np.random.default_rng(3))
check("pre-existing inf stays inf", np.isinf(out[:20, :]).all())
check("no negative or zero ranges",
      not (np.isfinite(out) & (out <= 0)).any())

# --- reproducible ----------------------------------------------------------
a = corrupt(clean, F, rng=np.random.default_rng(99))
b = corrupt(clean, F, rng=np.random.default_rng(99))
check("same seed gives the same frame", np.array_equal(np.nan_to_num(a, posinf=-1),
                                                       np.nan_to_num(b, posinf=-1)))

print("\n" + "=" * 44)
print("%d passed, %d failed" % (passed, failed))
sys.exit(1 if failed else 0)
