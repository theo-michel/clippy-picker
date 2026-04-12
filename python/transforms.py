"""Lightweight SE(3) transform utilities and Kabsch rigid-body registration."""

from __future__ import annotations

import numpy as np


def make_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Build a 4x4 homogeneous matrix from a 3x3 rotation and 3-vector translation."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).ravel()
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """Efficient SE(3) inverse: [R^T, -R^T t; 0 1]."""
    R = T[:3, :3]
    t = T[:3, 3]
    Rt = R.T
    return make_transform(Rt, -Rt @ t)


def apply_transform(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) array of points. Also accepts a single (3,) vector."""
    pts = np.asarray(points)
    single = pts.ndim == 1
    if single:
        pts = pts.reshape(1, 3)
    R = T[:3, :3]
    t = T[:3, 3]
    result = (R @ pts.T).T + t
    return result.ravel() if single else result


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
