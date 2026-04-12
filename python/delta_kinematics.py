"""
Delta Robot Kinematics — inverse and forward.

Reference frame (right-handed):
  - Origin at the centre of the base platform.
  - +X points toward arm 1 (motor 1).
  - +Z points downward (workspace is at positive z).
  - +Y completes the right-hand rule.

  Arms in the XY plane (anticlockwise when viewed from below / +Z):
    Arm 1 at   0° (along +X).
    Arm 2 at 120°.
    Arm 3 at 240°.

Joint angle convention (kinematic space):
  - θ = 0°  → upper arm horizontal.
  - θ > 0°  → arm tilted downward.
  - θ < 0°  → arm tilted upward.

  The mechanical stop is 21.8° above horizontal → θ = −21.8° kinematic.
  The firmware reports 0 at that pose (after ZERO); use the offset in
  coordinates.DELTA_KINEMATIC_AT_FIRMWARE_ZERO when converting.

Robot geometry parameters (all in mm):
  upper_arm (L) — shoulder-to-elbow length
  lower_arm (l) — elbow-to-effector-joint rod length
  Fd            — base platform joint offset from centre
  Ed            — effector platform joint offset from centre

Usage::

    from delta_kinematics import DeltaKinematics

    dk = DeltaKinematics(upper_arm=150, lower_arm=268, Fd=82.5, Ed=27.3)
    angles = dk.inverse(x=0, y=0, z=250)
    print(angles)  # (θ1, θ2, θ3) in degrees
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class DeltaKinematics:
    """Inverse / forward kinematics solver for a symmetric delta robot."""

    upper_arm: float  # L  — shoulder-to-elbow
    lower_arm: float  # l  — elbow-to-effector-joint
    Fd: float  # base joint offset from centre
    Ed: float  # effector joint offset from centre

    def __post_init__(self) -> None:
        self._sqrt3 = math.sqrt(3.0)

    # ------------------------------------------------------------------ #
    #  Public API
    # ------------------------------------------------------------------ #

    def inverse(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Compute joint angles for a desired end-effector position.

        Args:
            x, y, z: Target in the robot frame (mm).  Workspace at z > 0.

        Returns:
            (θ1, θ2, θ3) in degrees.

        Raises:
            ValueError: If the target is unreachable.
        """
        t1 = self._solve_arm(x, y, z)

        # Arm 2 at +120°: rotate target by −120° (= +240°) to align with arm 1.
        x2, y2 = self._rotate_240(x, y)
        t2 = self._solve_arm(x2, y2, z)

        # Arm 3 at +240°: rotate target by −240° (= +120°) to align with arm 1.
        x3, y3 = self._rotate_120(x, y)
        t3 = self._solve_arm(x3, y3, z)

        return (t1, t2, t3)

    def forward(
        self,
        theta1_deg: float,
        theta2_deg: float,
        theta3_deg: float,
    ) -> Tuple[float, float, float]:
        """
        Compute end-effector (x, y, z) from joint angles.

        Returns the solution with z > 0 (below the base).

        Args:
            theta1_deg, theta2_deg, theta3_deg: Joint angles in degrees.

        Returns:
            (x, y, z) in mm.

        Raises:
            ValueError: If the configuration is singular or unreachable.
        """
        L = self.upper_arm
        l = self.lower_arm
        f = self.Fd - self.Ed
        sqrt3 = self._sqrt3

        t1 = math.radians(theta1_deg)
        t2 = math.radians(theta2_deg)
        t3 = math.radians(theta3_deg)

        c1, s1 = math.cos(t1), math.sin(t1)
        c2, s2 = math.cos(t2), math.sin(t2)
        c3, s3 = math.cos(t3), math.sin(t3)

        # Sphere centres — elbow positions shifted by Ed so the
        # intersection directly yields the effector centre.
        # Arm 1 at 0°, arm 2 at +120°, arm 3 at +240°.
        E1 = (
            f + L * c1,
            0.0,
            L * s1,
        )
        E2 = (
            -(f + L * c2) / 2.0,
            (f + L * c2) * sqrt3 / 2.0,
            L * s2,
        )
        E3 = (
            -(f + L * c3) / 2.0,
            -(f + L * c3) * sqrt3 / 2.0,
            L * s3,
        )

        # Intersect three spheres  |P − Eᵢ|² = l²
        def dot(a: tuple, b: tuple) -> float:
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

        def sq_norm(a: tuple) -> float:
            return dot(a, a)

        A = (E1[0] - E2[0], E1[1] - E2[1], E1[2] - E2[2])
        B = (E1[0] - E3[0], E1[1] - E3[1], E1[2] - E3[2])
        rhs_a = (sq_norm(E1) - sq_norm(E2)) / 2.0
        rhs_b = (sq_norm(E1) - sq_norm(E3)) / 2.0

        ax, ay, az = A
        bx, by, bz = B
        det = ax * by - ay * bx
        if abs(det) < 1e-12:
            raise ValueError(
                "Forward kinematics singular (elbows aligned or degenerate)"
            )

        kx = (az * by - ay * bz) / det
        ky = (ax * bz - az * bx) / det
        cx = (rhs_a * by - rhs_b * ay) / det
        cy = (rhs_b * ax - rhs_a * bx) / det

        ex, ey, ez = E1
        px0 = cx - ex
        py0 = cy - ey
        qa = kx * kx + ky * ky + 1.0
        qb = -2.0 * (px0 * kx + py0 * ky + ez)
        qc = px0 * px0 + py0 * py0 + ez * ez - l * l
        disc = qb * qb - 4.0 * qa * qc
        if disc < 0.0:
            raise ValueError(
                "Forward kinematics: no intersection (invalid joint angles)"
            )

        sqrt_d = math.sqrt(disc)
        pz1 = (-qb + sqrt_d) / (2.0 * qa)
        pz2 = (-qb - sqrt_d) / (2.0 * qa)

        # Pick the solution below the base (positive z).
        if pz1 >= 0 and pz2 >= 0:
            pz = min(pz1, pz2)
        elif pz1 >= 0:
            pz = pz1
        elif pz2 >= 0:
            pz = pz2
        else:
            pz = max(pz1, pz2)

        px = cx - kx * pz
        py = cy - ky * pz
        return (px, py, pz)

    # ------------------------------------------------------------------ #
    #  Single-arm solver
    # ------------------------------------------------------------------ #

    def _solve_arm(self, x: float, y: float, z: float) -> float:
        """
        Solve one arm's joint angle via the cosine rule.

        Expects the target rotated so the arm under consideration lies
        along +X.  The arm moves in the XZ plane; *y* is perpendicular.
        """
        L = self.upper_arm
        l = self.lower_arm

        # Effective lower-arm length projected into the arm's XZ plane.
        A2 = l * l - y * y
        if A2 < 0.0:
            raise ValueError(
                f"Target unreachable: |y| = {abs(y):.1f} exceeds lower arm {l:.1f}"
            )

        # In-plane distance from shoulder (Fd, 0) to effector joint (x+Ed, z).
        d = x + self.Ed - self.Fd
        W2 = d * d + z * z
        W = math.sqrt(W2)
        if W < 1e-12:
            raise ValueError("Target coincides with shoulder pivot")

        cos_omega = (W2 + L * L - A2) / (2.0 * L * W)
        if abs(cos_omega) > 1.0:
            raise ValueError(
                f"Target ({x:.1f}, {y:.1f}, {z:.1f}) unreachable "
                f"(cos_omega = {cos_omega:.6f})"
            )

        beta = math.atan2(z, d)
        omega = math.acos(cos_omega)
        return math.degrees(beta - omega)

    # ------------------------------------------------------------------ #
    #  Rotation helpers (120° / 240° around Z)
    # ------------------------------------------------------------------ #

    def _rotate_120(self, x: float, y: float) -> Tuple[float, float]:
        """Rotate (x, y) by +120° around Z."""
        cos120 = -0.5
        sin120 = self._sqrt3 / 2.0
        return (
            x * cos120 - y * sin120,
            x * sin120 + y * cos120,
        )

    def _rotate_240(self, x: float, y: float) -> Tuple[float, float]:
        """Rotate (x, y) by +240° (≡ −120°) around Z."""
        cos240 = -0.5
        sin240 = -self._sqrt3 / 2.0
        return (
            x * cos240 - y * sin240,
            x * sin240 + y * cos240,
        )


if __name__ == "__main__":
    dk = DeltaKinematics(upper_arm=150, lower_arm=268, Fd=82.5, Ed=27.3)
    for x, y, z in [(0.0, 0.0, 250.0), (50.0, 0.0, 250.0)]:
        angles = dk.inverse(x, y, z)
        print(f"IK ({x}, {y}, {z}) → ({angles[0]:.4f}, {angles[1]:.4f}, {angles[2]:.4f})°")
    for a in [(-15.0, -15.0, -15.0), (24.6, 24.6, 24.6)]:
        pos = dk.forward(*a)
        print(f"FK ({a[0]:.1f}, {a[1]:.1f}, {a[2]:.1f})° → ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
