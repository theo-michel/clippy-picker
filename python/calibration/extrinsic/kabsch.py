"""Kabsch rigid-body registration (SVD-based point-cloud alignment)."""

# This code is used to to calculate the transformation matrix between the camera and the delta robot
# using a set of known points in the camera and delta robot frames (typically from an aruco marker)

from __future__ import annotations

import numpy as np


def _kabsch_core(
    C: np.ndarray, D: np.ndarray
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """Single-pass Kabsch solve on (N, 3) point sets C → D."""
    c_bar = C.mean(axis=0)
    d_bar = D.mean(axis=0)
    Cc = C - c_bar
    Dc = D - d_bar

    H = Cc.T @ Dc
    U, _S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    t = d_bar - R @ c_bar

    aligned = (R @ C.T).T + t
    residuals = np.linalg.norm(D - aligned, axis=1)
    rmsd = float(np.sqrt(np.mean(residuals**2)))

    return R, t, rmsd, residuals


def kabsch(
    C: np.ndarray,
    D: np.ndarray,
    *,
    max_iters: int = 5,
    threshold_sigma: float = 1.5,
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """Kabsch with iterative outlier rejection.

    After each solve, points whose residual exceeds
    ``threshold_sigma * RMSD`` are removed.  Iteration stops when no
    points are dropped or ``max_iters`` is reached.  At least 4 inliers
    are always kept.

    Returns R, t, rmsd, and **full-set** residuals (all original points
    evaluated against the final R, t) so callers can see per-point error.
    """
    C = np.asarray(C, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64)
    assert C.shape == D.shape and C.shape[1] == 3

    mask = np.ones(len(C), dtype=bool)

    for _ in range(max_iters):
        R, t, rmsd, res = _kabsch_core(C[mask], D[mask])

        cutoff = threshold_sigma * rmsd
        inlier = res <= cutoff
        if inlier.all() or inlier.sum() < 4:
            break

        idx = np.where(mask)[0]
        mask[idx[~inlier]] = False

    R, t, rmsd, _ = _kabsch_core(C[mask], D[mask])

    full_aligned = (R @ C.T).T + t  # camera detected points in delta robot frame
    full_residuals = np.linalg.norm(D - full_aligned, axis=1)  # residuals of all points

    return R, t, rmsd, full_residuals
