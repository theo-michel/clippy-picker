"""Kabsch rigid-body registration (SVD-based point-cloud alignment)."""

from __future__ import annotations

import numpy as np


def kabsch(
    C: np.ndarray, D: np.ndarray
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """Find the optimal rigid transform mapping point set C to D.

    Both C and D are (N, 3) arrays of corresponding 3-D points.
    Solves:  D_i ≈ R @ C_i + t   (least-squares).

    Returns:
        R:         (3, 3) optimal rotation matrix in SO(3).
        t:         (3,)   optimal translation vector.
        rmsd:      root-mean-square deviation after alignment.
        residuals: (N,) per-point Euclidean error after alignment.
    """
    C = np.asarray(C, dtype=np.float64)
    D = np.asarray(D, dtype=np.float64)
    assert C.shape == D.shape and C.shape[1] == 3

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
