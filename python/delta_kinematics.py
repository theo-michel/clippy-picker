"""
Delta Robot Inverse Kinematics

Classic circle-intersection approach: project arm geometry onto the YZ plane,
solve circle-circle intersection (quadratic in elbow position), then compute
joint angle from elbow.

Reference frame:
  - Origin at the centre of the base platform.
  - Z axis points downward (end-effector workspace is at negative z).

Joint angle convention (kinematic space):
  - θ = 0° means the upper arm is **horizontal** (elbow in the base plane, z = 0).
  - θ > 0° means the arm is tilted **down** from horizontal; θ < 0° means **up**.
  So cos(θ) and sin(θ) in the formulas depend on this numeric value; the kinematics
  does care what number you pass. The mechanical stop is 21.8° above horizontal,
  which corresponds to θ = -21.8° in this convention. The firmware reports 0 at
  that pose (after ZERO); use the offset in coordinates.DELTA_KINEMATIC_AT_FIRMWARE_ZERO
  when converting between firmware and kinematic angles.

Robot geometry parameters (all in the same unit, e.g. mm):
  upper_arm — shoulder-to-elbow length  (L)
  lower_arm — elbow-to-effector-joint length  (l)
  Fd        — base joint offset from centre
  Ed        — effector joint offset from centre

Usage:
    from delta_kinematics import DeltaKinematics

    dk = DeltaKinematics(upper_arm=150, lower_arm=268, Fd=82.5, Ed=27.3)
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

    def inverse(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        Compute the three joint angles for a desired end-effector position.

        Args:
            x, y, z: Target position in the robot's reference frame.

        Returns:
            (theta1, theta2, theta3) in **degrees**.

        Raises:
            ValueError: If the target is unreachable.
        """
        t1 = self._solve_arm_classic(x, y, z)

        x2, y2 = self._rotate_120(x, y)
        t2 = self._solve_arm_classic(x2, y2, z)
        x3, y3 = self._rotate_240(x, y)
        t3 = self._solve_arm_classic(x3, y3, z)

        return (t1, t2, t3)

    def forward(
        self,
        theta1_deg: float,
        theta2_deg: float,
        theta3_deg: float,
    ) -> Tuple[float, float, float]:
        """
        Compute end-effector (x, y, z) from joint angles (forward kinematics).

        Uses the same frame as inverse(): origin at base centre, Z downward.
        The solution with z < 0 (below the base) is returned.

        Args:
            theta1_deg, theta2_deg, theta3_deg: Joint angles in **degrees**.

        Returns:
            (x, y, z) in the same units as the robot geometry (e.g. mm).

        Raises:
            ValueError: If the configuration is singular or unreachable.
        """
        L = self.upper_arm
        l = self.lower_arm
        Fd = self.Fd
        sqrt3 = self._sqrt3

        def rad(d: float) -> float:
            return math.radians(d)

        t1, t2, t3 = rad(theta1_deg), rad(theta2_deg), rad(theta3_deg)

        # Elbow positions consistent with IK: for arm 1, elbow = (0, Py, Pz) with
        # Py = -Fd - L*cos(θ), Pz = -L*sin(θ) (classic method convention)
        E1 = (
            0.0,
            -Fd - L * math.cos(t1),
            -L * math.sin(t1),
        )
        # Arm 2 and 3: same (y, z) as arm 1 in their local frames; local y is at 120° / 240°
        # So E2 = ( (Fd+L*cos(t2))*sqrt3/2, -(Fd+L*cos(t2))/2, -L*sin(t2) ), E3 = ( -(Fd+L*cos(t3))*sqrt3/2, -(Fd+L*cos(t3))/2, -L*sin(t3) )
        c2, s2 = math.cos(t2), math.sin(t2)
        c3, s3 = math.cos(t3), math.sin(t3)
        E2 = (
            (Fd + L * c2) * sqrt3 / 2.0,
            -(Fd + L * c2) / 2.0,
            -L * s2,
        )
        E3 = (
            -(Fd + L * c3) * sqrt3 / 2.0,
            -(Fd + L * c3) / 2.0,
            -L * s3,
        )

        # P is the intersection of three spheres |P - Ei|^2 = l^2
        # Subtract sphere 1 from 2 and 3 to get two linear equations in P
        def dot(a: tuple, b: tuple) -> float:
            return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

        def sq_norm(a: tuple) -> float:
            return dot(a, a)

        A = (E1[0] - E2[0], E1[1] - E2[1], E1[2] - E2[2])
        B = (E1[0] - E3[0], E1[1] - E3[1], E1[2] - E3[2])
        rhs_a = (sq_norm(E1) - sq_norm(E2)) / 2.0
        rhs_b = (sq_norm(E1) - sq_norm(E3)) / 2.0

        # Solve P.A = rhs_a, P.B = rhs_b. Express Px, Py in terms of Pz using
        # the two linear equations (2x2 system in Px, Py when A and B have
        # nonzero XY components).
        ax, ay, az = A
        bx, by, bz = B
        det = ax * by - ay * bx
        if abs(det) < 1e-12:
            raise ValueError(
                "Forward kinematics singular (elbows aligned or degenerate)"
            )
        # Px = (rhs_a*by - rhs_b*ay - Pz*(az*by - ay*bz)) / det
        # Py = (rhs_b*ax - rhs_a*bx - Pz*(ax*bz - az*bx)) / det
        kx = (az * by - ay * bz) / det
        ky = (ax * bz - az * bx) / det
        cx = (rhs_a * by - rhs_b * ay) / det
        cy = (rhs_b * ax - rhs_a * bx) / det
        # P = (cx - kx*Pz, cy - ky*Pz, Pz)
        # |P - E1|^2 = l^2  =>  quadratic in Pz
        ex, ey, ez = E1
        px0 = cx - ex
        py0 = cy - ey
        qa = kx * kx + ky * ky + 1.0
        qb = 2.0 * (px0 * kx + py0 * ky - ez)
        qc = px0 * px0 + py0 * py0 + ez * ez - l * l
        disc = qb * qb - 4.0 * qa * qc
        if disc < 0.0:
            raise ValueError(
                "Forward kinematics: no intersection (invalid joint angles)"
            )
        sqrt_d = math.sqrt(disc)
        pz1 = (-qb + sqrt_d) / (2.0 * qa)
        pz2 = (-qb - sqrt_d) / (2.0 * qa)
        # Choose the solution below the base (negative z)
        if pz1 <= 0 and pz2 <= 0:
            pz = max(pz1, pz2)
        elif pz1 <= 0:
            pz = pz1
        elif pz2 <= 0:
            pz = pz2
        else:
            pz = min(pz1, pz2)
        px = cx - kx * pz
        py = cy - ky * pz
        return (px, py, pz)

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


if __name__ == "__main__":
    dk = DeltaKinematics(upper_arm=150, lower_arm=268, Fd=82.5, Ed=27.3)
    for x, y, z in [(0.0, 0.0, -250.0), (50.0, 0.0, -250.0)]:
        angles = dk.inverse(x, y, z)
        print(f"({x}, {y}, {z}) -> ({angles[0]:.4f}, {angles[1]:.4f}, {angles[2]:.4f})°")
