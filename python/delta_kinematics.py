"""
Delta Robot Inverse Kinematics

Two methods for computing inverse kinematics (Cartesian → joint angles):

  1. Classic circle-intersection approach (quadratic solve)
  2. Two-triangle approach (simpler trigonometry)

Both produce identical results; method 2 is shorter and avoids the
quadratic-formula bookkeeping.

Reference frame:
  - Origin at the centre of the base platform.
  - Z axis points downward (end-effector workspace is at negative z).

Robot geometry parameters (all in the same unit, e.g. mm):
  upper_arm — shoulder-to-elbow length  (L)
  lower_arm — elbow-to-effector-joint length  (l)
  Fd        — base joint offset from centre
  Ed        — effector joint offset from centre

Usage:
    from delta_kinematics import DeltaKinematics

    dk = DeltaKinematics(upper_arm=150, lower_arm=271, Fd=36.7, Ed=80)
    angles = dk.inverse(x=0, y=0, z=-250)
    print(angles)  # (θ1, θ2, θ3) in degrees
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class DeltaKinematics:
    """Inverse kinematics solver for a symmetric delta robot."""

    upper_arm: float   # L  — shoulder-to-elbow
    lower_arm: float   # l  — elbow-to-effector-joint
    Fd: float          # base joint offset from centre
    Ed: float          # effector joint offset from centre

    def __post_init__(self) -> None:
        self._sqrt3 = math.sqrt(3.0)

    # ------------------------------------------------------------------ #
    #  Public API
    # ------------------------------------------------------------------ #

    def inverse(
        self,
        x: float,
        y: float,
        z: float,
        method: str = "triangles",
    ) -> Tuple[float, float, float]:
        """
        Compute the three joint angles for a desired end-effector position.

        Args:
            x, y, z: Target position in the robot's reference frame.
            method:  ``"triangles"`` (default, recommended) or ``"classic"``.

        Returns:
            (theta1, theta2, theta3) in **degrees**.

        Raises:
            ValueError: If the target is unreachable.
        """
        if method == "classic":
            solve = self._solve_arm_classic
        elif method == "triangles":
            solve = self._solve_arm_triangles
        else:
            raise ValueError(f"Unknown method {method!r}; use 'classic' or 'triangles'")

        # Arm 1 — no rotation
        t1 = solve(x, y, z)

        # Arm 2 — rotate (x, y) by +120°
        x2, y2 = self._rotate_120(x, y)
        t2 = solve(x2, y2, z)

        # Arm 3 — rotate (x, y) by +240°
        x3, y3 = self._rotate_240(x, y)
        t3 = solve(x3, y3, z)

        return (t1, t2, t3)

    # ------------------------------------------------------------------ #
    #  Method 1 — Classic circle-intersection (quadratic)
    # ------------------------------------------------------------------ #

    def _solve_arm_classic(self, x: float, y: float, z: float) -> float:
        """
        Solve one arm's angle via circle-circle intersection.

        Projects the geometry onto the YZ plane, sets up two circle
        equations, reduces them to a quadratic in Py, and picks the
        valid (outer) root.
        """
        L = self.upper_arm
        l = self.lower_arm
        Fd = self.Fd
        Ed = self.Ed

        y_hat = y - Ed

        if abs(z) < 1e-12:
            raise ValueError("z ≈ 0 causes a division by zero in the classic method")

        # Linear relation  Pz = a + b·Py
        a = (x * x + y_hat * y_hat + z * z + L * L - l * l - Fd * Fd) / (2.0 * z)
        b = -(Fd + y_hat) / z

        # Quadratic  A·Py² + B·Py + C = 0
        A = b * b + 1.0
        B = 2.0 * (b * (a - z) - y_hat)
        C = y_hat * y_hat + (a - z) ** 2 - (l * l - x * x)

        discriminant = B * B - 4.0 * A * C
        if discriminant < 0.0:
            raise ValueError(
                f"Target ({x}, {y}, {z}) is unreachable "
                f"(discriminant = {discriminant:.6f})"
            )

        sqrt_d = math.sqrt(discriminant)
        Py = (-B - sqrt_d) / (2.0 * A)  # outer (valid) solution
        Pz = a + b * Py

        theta_deg = math.degrees(math.atan2(-Pz, -(Fd + Py)))
        return theta_deg

    # ------------------------------------------------------------------ #
    #  Method 2 — Two-triangle approach
    # ------------------------------------------------------------------ #

    def _solve_arm_triangles(self, x: float, y: float, z: float) -> float:
        """
        Solve one arm's angle via the two-triangle decomposition.

        Derives α from the left triangle (arcsin) and ω from the law
        of cosines on the right triangle, then combines them.
        """
        L = self.upper_arm
        l = self.lower_arm
        Fd = self.Fd
        Ed = self.Ed

        # Left triangle → α
        d = y - Ed + Fd
        W2 = z * z + d * d
        W = math.sqrt(W2) if W2 > 1e-12 else 1e-6
        alpha_deg = math.degrees(math.asin(max(-1.0, min(1.0, d / W))))

        # Right triangle → ω  (law of cosines)
        A2 = l * l - x * x
        cos_omega = (W2 + L * L - A2) / (2.0 * L * W)
        cos_omega = max(-1.0, min(1.0, cos_omega))
        omega_deg = math.degrees(math.acos(cos_omega))

        theta_deg = 90.0 + alpha_deg - omega_deg
        return theta_deg

    # ------------------------------------------------------------------ #
    #  Rotation helpers (120° / 240° around Z)
    # ------------------------------------------------------------------ #

    def _rotate_120(self, x: float, y: float) -> Tuple[float, float]:
        """Rotate (x, y) by +120° around the Z axis."""
        cos120 = -0.5
        sin120 = self._sqrt3 / 2.0
        return (
            x * cos120 - y * sin120,
            x * sin120 + y * cos120,
        )

    def _rotate_240(self, x: float, y: float) -> Tuple[float, float]:
        """Rotate (x, y) by +240° around the Z axis."""
        cos240 = -0.5
        sin240 = -self._sqrt3 / 2.0
        return (
            x * cos240 - y * sin240,
            x * sin240 + y * cos240,
        )


# ====================================================================== #
#  Quick sanity check
# ====================================================================== #

if __name__ == "__main__":
    dk = DeltaKinematics(upper_arm=150, lower_arm=271, Fd=36.7, Ed=80)

    test_points = [
        (0.0, 0.0, -250.0),
        (50.0, 0.0, -250.0),
        (0.0, 50.0, -300.0),
        (30.0, -30.0, -200.0),
    ]

    print(f"{'Point':>24s}   {'Classic':>36s}   {'Triangles':>36s}   {'Max Δ':>8s}")
    print("-" * 114)

    for x, y, z in test_points:
        try:
            classic = dk.inverse(x, y, z, method="classic")
            triangles = dk.inverse(x, y, z, method="triangles")
            max_diff = max(abs(a - b) for a, b in zip(classic, triangles))
            print(
                f"({x:6.2f}, {y:6.2f}, {z:6.2f})   "
                f"({classic[0]:10.4f}, {classic[1]:10.4f}, {classic[2]:10.4f})   "
                f"({triangles[0]:10.4f}, {triangles[1]:10.4f}, {triangles[2]:10.4f})   "
                f"{max_diff:.2e}"
            )
        except ValueError as e:
            print(f"({x:6.2f}, {y:6.2f}, {z:6.2f})   {e}")
