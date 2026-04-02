#!/usr/bin/env python3
"""
Delta Robot Workspace Analysis

Computes the reachable workspace envelope by scanning a 3D volume and
testing inverse kinematics at each point.  Produces terminal output
(safe operating parameters at each Z height) and optional matplotlib plots.

At shallow Z heights the workspace can become annular (donut-shaped) where
the centre is unreachable but a ring around it is.  The script detects and
reports this correctly.

Outputs per Z-slice:
  - Inscribed radius  : largest circle at origin fully inside the workspace
  - Max square side    : largest axis-aligned square at origin fully inside
  - Outer reach        : furthest reachable point in any direction
  - Inner hole radius  : (annular only) radius of the central dead zone

Usage:
    uv run python scripts/workspace_analysis.py
    uv run python scripts/workspace_analysis.py --plot
    uv run python scripts/workspace_analysis.py --z-min -350 --z-max -100 --z-step 10
    uv run python scripts/workspace_analysis.py --plot --save workspace.png
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent))

from delta_kinematics import DeltaKinematics


# ── Reachability test ────────────────────────────────────────────────────────


def is_reachable(
    dk: DeltaKinematics,
    x: float,
    y: float,
    z: float,
    angle_min: float,
    angle_max: float,
) -> tuple[bool, tuple[float, float, float] | None]:
    try:
        angles = dk.inverse(x, y, z)
    except ValueError:
        return False, None
    for a in angles:
        if a < angle_min or a > angle_max:
            return False, None
    return True, angles


def _binary_search_boundary(
    dk: DeltaKinematics,
    z: float,
    cos_t: float,
    sin_t: float,
    lo: float,
    hi: float,
    angle_min: float,
    angle_max: float,
    iters: int = 30,
) -> float:
    """Binary search for the transition point along a ray from (lo, hi)."""
    for _ in range(iters):
        mid = (lo + hi) / 2
        ok, _ = is_reachable(dk, mid * cos_t, mid * sin_t, z, angle_min, angle_max)
        if ok:
            lo = mid
        else:
            hi = mid
    return lo


# ── Workspace metrics ────────────────────────────────────────────────────────


def origin_reachable(
    dk: DeltaKinematics, z: float, angle_min: float, angle_max: float
) -> bool:
    ok, _ = is_reachable(dk, 0.0, 0.0, z, angle_min, angle_max)
    return ok


def outer_reach_per_ray(
    dk: DeltaKinematics,
    z: float,
    angle_min: float,
    angle_max: float,
    n_rays: int = 72,
) -> list[float]:
    """For each ray, find the outermost reachable distance from the origin."""
    radii = []
    for i in range(n_rays):
        theta = 2 * math.pi * i / n_rays
        cos_t, sin_t = math.cos(theta), math.sin(theta)

        # First find any reachable point on this ray (scan outward)
        probe = 0.0
        found = False
        for r_probe in np.arange(0, 500, 5):
            ok, _ = is_reachable(
                dk, r_probe * cos_t, r_probe * sin_t, z, angle_min, angle_max
            )
            if ok:
                probe = r_probe
                found = True

        if not found:
            radii.append(0.0)
            continue

        # Binary search from the last known reachable point outward
        outer = _binary_search_boundary(
            dk, z, cos_t, sin_t, probe, probe + 100.0, angle_min, angle_max
        )
        radii.append(outer)

    return radii


def inner_hole_per_ray(
    dk: DeltaKinematics,
    z: float,
    angle_min: float,
    angle_max: float,
    n_rays: int = 72,
) -> list[float]:
    """
    For an annular workspace, find the inner boundary (closest reachable
    distance from origin) along each ray.
    """
    radii = []
    for i in range(n_rays):
        theta = 2 * math.pi * i / n_rays
        cos_t, sin_t = math.cos(theta), math.sin(theta)

        # Find the first reachable point going outward
        inner = 0.0
        for r_probe in np.arange(0, 500, 2):
            ok, _ = is_reachable(
                dk, r_probe * cos_t, r_probe * sin_t, z, angle_min, angle_max
            )
            if ok:
                # Binary search between previous (unreachable) and this (reachable)
                lo_search = max(0, r_probe - 2)
                hi_search = r_probe
                # Invert: search for unreachable→reachable boundary
                for _ in range(30):
                    mid = (lo_search + hi_search) / 2
                    ok2, _ = is_reachable(
                        dk, mid * cos_t, mid * sin_t, z, angle_min, angle_max
                    )
                    if ok2:
                        hi_search = mid
                    else:
                        lo_search = mid
                inner = hi_search
                break
        else:
            inner = float("inf")

        radii.append(inner)

    return radii


def inscribed_radius(
    dk: DeltaKinematics,
    z: float,
    angle_min: float,
    angle_max: float,
    n_rays: int = 72,
) -> float:
    """Largest circle centered at origin fully inside the workspace."""
    if not origin_reachable(dk, z, angle_min, angle_max):
        return 0.0

    outer = outer_reach_per_ray(dk, z, angle_min, angle_max, n_rays)
    return min(outer) if outer else 0.0


def inscribed_square_half(
    dk: DeltaKinematics,
    z: float,
    angle_min: float,
    angle_max: float,
) -> float:
    """Half-side of the largest axis-aligned square at origin inside workspace."""
    if not origin_reachable(dk, z, angle_min, angle_max):
        return 0.0

    lo, hi = 0.0, 500.0
    for _ in range(30):
        h = (lo + hi) / 2
        pts = [
            (h, h),
            (h, -h),
            (-h, h),
            (-h, -h),
            (h, 0),
            (-h, 0),
            (0, h),
            (0, -h),
        ]
        if all(is_reachable(dk, px, py, z, angle_min, angle_max)[0] for px, py in pts):
            lo = h
        else:
            hi = h
    return lo


def scan_z_slice(
    dk: DeltaKinematics,
    z: float,
    scan_radius: float,
    resolution: float,
    angle_min: float,
    angle_max: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Scan a square XY grid at height z, return coordinate axis + 2D bool grid."""
    coords = np.arange(-scan_radius, scan_radius + resolution, resolution)
    grid = np.zeros((len(coords), len(coords)), dtype=bool)
    for ix, x in enumerate(coords):
        for iy, y in enumerate(coords):
            ok, _ = is_reachable(dk, x, y, z, angle_min, angle_max)
            grid[iy, ix] = ok
    return coords, grid


# ── Data structures ──────────────────────────────────────────────────────────


@dataclass
class ZSliceResult:
    z: float
    is_annular: bool
    inscribed_r: float
    max_square_side: float
    outer_reach: float
    inner_hole_r: float  # > 0 only for annular workspaces
    angle_range: tuple[float, float] | None


# ── Analysis ─────────────────────────────────────────────────────────────────


def analyze_workspace(
    dk: DeltaKinematics,
    z_min: float,
    z_max: float,
    z_step: float,
    angle_min: float,
    angle_max: float,
    n_rays: int = 72,
) -> list[ZSliceResult]:
    results = []
    z_values = np.arange(z_min, z_max + z_step / 2, z_step)

    for z in z_values:
        z = float(z)
        annular = not origin_reachable(dk, z, angle_min, angle_max)

        outer_rays = outer_reach_per_ray(dk, z, angle_min, angle_max, n_rays)
        outer_max = max(outer_rays) if outer_rays else 0.0
        outer_min = min(outer_rays) if outer_rays else 0.0

        if annular:
            inner_rays = inner_hole_per_ray(dk, z, angle_min, angle_max, n_rays)
            inner_max = (
                max(r for r in inner_rays if r < float("inf")) if inner_rays else 0.0
            )
            insc_r = 0.0
            sq_half = 0.0
        else:
            inner_max = 0.0
            insc_r = outer_min
            sq_half = inscribed_square_half(dk, z, angle_min, angle_max)

        # Collect angle extremes across the outer boundary
        angle_lo, angle_hi = 180.0, -180.0
        for i in range(n_rays):
            theta = 2 * math.pi * i / n_rays
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            r_test = outer_rays[i] * 0.5 if outer_rays[i] > 0 else 0
            if r_test > 0:
                ok, angles = is_reachable(
                    dk, r_test * cos_t, r_test * sin_t, z, angle_min, angle_max
                )
                if ok and angles:
                    for a in angles:
                        angle_lo = min(angle_lo, a)
                        angle_hi = max(angle_hi, a)

        ar = (angle_lo, angle_hi) if angle_hi >= angle_lo else None

        any_reachable = outer_max > 1.0
        if not any_reachable:
            results.append(ZSliceResult(z, False, 0, 0, 0, 0, None))
        else:
            results.append(
                ZSliceResult(z, annular, insc_r, sq_half * 2, outer_max, inner_max, ar)
            )

    return results


# ── Reporting ────────────────────────────────────────────────────────────────


def print_table(results: list[ZSliceResult], dk: DeltaKinematics) -> None:
    print(
        f"\nDelta geometry: L={dk.upper_arm}mm, l={dk.lower_arm}mm, "
        f"Fd={dk.Fd}mm, Ed={dk.Ed}mm"
    )

    print(
        f"\n{'Z':>7}  {'Type':>7}  {'Inscr. R':>9}  {'Max Sq':>9}  "
        f"{'Outer R':>8}  {'Hole R':>7}  {'Angle Range':>18}"
    )
    print("-" * 78)

    for r in results:
        if r.outer_reach < 1.0:
            print(f"{r.z:7.0f}  {'---':>7}  {'unreachable':>52}")
            continue

        shape = "annulus" if r.is_annular else "disk"
        insc = f"{r.inscribed_r:.0f}mm" if r.inscribed_r > 0 else "—"
        sq = f"{r.max_square_side:.0f}mm" if r.max_square_side > 0 else "—"
        hole = f"{r.inner_hole_r:.0f}mm" if r.inner_hole_r > 0 else "—"
        ar = (
            f"{r.angle_range[0]:+.1f}° to {r.angle_range[1]:+.1f}°"
            if r.angle_range
            else "n/a"
        )

        print(
            f"{r.z:7.0f}  {shape:>7}  {insc:>9}  {sq:>9}  "
            f"{r.outer_reach:6.0f}mm  {hole:>7}  {ar}"
        )


def find_optimal_z(results: list[ZSliceResult]) -> ZSliceResult | None:
    reachable = [r for r in results if r.inscribed_r > 1.0]
    if not reachable:
        return None
    return max(reachable, key=lambda r: r.inscribed_r)


def print_recommendations(results: list[ZSliceResult]) -> None:
    opt = find_optimal_z(results)
    if opt is None:
        print("\nNo working height with a usable inscribed workspace found.")
        print("Consider widening the joint angle limits or checking geometry.")
        return

    print(f"\n{'='*78}")
    print("RECOMMENDATIONS")
    print(f"{'='*78}")
    print(f"  Best working height    : Z = {opt.z:.0f} mm")
    print(
        f"  Inscribed circle there : R = {opt.inscribed_r:.1f} mm  (diameter {opt.inscribed_r*2:.0f} mm)"
    )
    print(
        f"  Max safe square there  : {opt.max_square_side:.0f} x {opt.max_square_side:.0f} mm"
    )
    print(f"  Max reach there        : {opt.outer_reach:.1f} mm")
    if opt.angle_range:
        print(
            f"  Joint angles used      : {opt.angle_range[0]:+.1f}° to {opt.angle_range[1]:+.1f}°"
        )

    # Report at the user's pick zone (Z = -200)
    pick_z = next((r for r in results if abs(r.z - (-200.0)) < 1.0), None)
    if pick_z and pick_z.inscribed_r > 1.0:
        print(f"\n  At Z = -200 mm (approximate pick height):")
        print(f"    Inscribed circle : R = {pick_z.inscribed_r:.1f} mm")
        print(
            f"    Max safe square  : {pick_z.max_square_side:.0f} x {pick_z.max_square_side:.0f} mm"
        )

    # Find Z range where workspace is a disk (not annular)
    disk_range = [r for r in results if not r.is_annular and r.inscribed_r > 1.0]
    if disk_range:
        z_top = max(r.z for r in disk_range)
        z_bot = min(r.z for r in disk_range)
        print(
            f"\n  Disk-shaped workspace (origin reachable): Z ∈ [{z_bot:.0f}, {z_top:.0f}] mm"
        )

    # Find annular range
    annular_range = [r for r in results if r.is_annular and r.outer_reach > 1.0]
    if annular_range:
        z_top = max(r.z for r in annular_range)
        z_bot = min(r.z for r in annular_range)
        print(
            f"  Annular workspace (origin unreachable)   : Z ∈ [{z_bot:.0f}, {z_top:.0f}] mm"
        )
        print(
            f"  (At these heights the centre is a dead zone — usable only as a ring.)"
        )


# ── Plotting ─────────────────────────────────────────────────────────────────


def plot_workspace(
    dk: DeltaKinematics,
    results: list[ZSliceResult],
    detail_z_levels: list[float],
    angle_min: float,
    angle_max: float,
    save_path: str | None = None,
) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    n_detail = len(detail_z_levels)
    fig = plt.figure(figsize=(6 * (n_detail + 1), 6))

    # Left panel: workspace envelope profile (Z vs radius)
    ax_profile = fig.add_subplot(1, n_detail + 1, 1)

    z_disk = [r.z for r in results if not r.is_annular and r.inscribed_r > 1.0]
    r_insc_disk = [
        r.inscribed_r for r in results if not r.is_annular and r.inscribed_r > 1.0
    ]
    r_outer_disk = [
        r.outer_reach for r in results if not r.is_annular and r.inscribed_r > 1.0
    ]

    z_ann = [r.z for r in results if r.is_annular and r.outer_reach > 1.0]
    r_outer_ann = [
        r.outer_reach for r in results if r.is_annular and r.outer_reach > 1.0
    ]
    r_inner_ann = [
        r.inner_hole_r for r in results if r.is_annular and r.outer_reach > 1.0
    ]

    if z_disk:
        ax_profile.fill_betweenx(z_disk, 0, r_outer_disk, alpha=0.12, color="steelblue")
        ax_profile.fill_betweenx(z_disk, 0, r_insc_disk, alpha=0.25, color="green")
        ax_profile.plot(r_outer_disk, z_disk, "steelblue", lw=1.5, label="Outer reach")
        ax_profile.plot(r_insc_disk, z_disk, "green", lw=2, label="Inscribed R")

    if z_ann:
        ax_profile.fill_betweenx(
            z_ann, r_inner_ann, r_outer_ann, alpha=0.15, color="orange"
        )
        ax_profile.plot(r_outer_ann, z_ann, "orange", lw=1.5, label="Annular outer")
        ax_profile.plot(
            r_inner_ann, z_ann, "red", lw=1.5, ls="--", label="Annular hole"
        )

    for zl in detail_z_levels:
        ax_profile.axhline(y=zl, color="gray", ls="--", alpha=0.4, lw=0.8)

    ax_profile.set_xlabel("Radius (mm)")
    ax_profile.set_ylabel("Z height (mm)")
    ax_profile.set_title("Workspace Envelope (side view)")
    ax_profile.legend(loc="lower right", fontsize=7)
    ax_profile.grid(True, alpha=0.3)
    ax_profile.set_xlim(0, None)

    # Right panels: XY cross-sections at selected Z heights
    for idx, z_level in enumerate(detail_z_levels):
        ax = fig.add_subplot(1, n_detail + 1, idx + 2)

        scan_r = 300.0
        res = 3.0
        coords, grid = scan_z_slice(dk, z_level, scan_r, res, angle_min, angle_max)

        ax.contourf(
            coords,
            coords,
            grid.astype(float),
            levels=[0.5, 1.5],
            colors=["#4a90d9"],
            alpha=0.3,
        )
        ax.contour(
            coords,
            coords,
            grid.astype(float),
            levels=[0.5],
            colors=["steelblue"],
            linewidths=1.5,
        )

        result = next((r for r in results if abs(r.z - z_level) < 1.0), None)
        if result:
            if result.inscribed_r > 1.0:
                circ = Circle(
                    (0, 0),
                    result.inscribed_r,
                    fill=False,
                    edgecolor="green",
                    lw=2,
                    label=f"R={result.inscribed_r:.0f}mm",
                )
                ax.add_patch(circ)
            if result.max_square_side > 1.0:
                half = result.max_square_side / 2
                rect = Rectangle(
                    (-half, -half),
                    result.max_square_side,
                    result.max_square_side,
                    fill=False,
                    edgecolor="orange",
                    lw=2,
                    ls="--",
                    label=f"Sq={result.max_square_side:.0f}mm",
                )
                ax.add_patch(rect)
            if result.is_annular and result.inner_hole_r > 1.0:
                hole = Circle(
                    (0, 0),
                    result.inner_hole_r,
                    fill=True,
                    facecolor="white",
                    edgecolor="red",
                    lw=1.5,
                    ls="--",
                    label=f"Hole R={result.inner_hole_r:.0f}mm",
                )
                ax.add_patch(hole)

        shape = "annulus" if result and result.is_annular else "disk"
        ax.set_xlim(-scan_r, scan_r)
        ax.set_ylim(-scan_r, scan_r)
        ax.set_aspect("equal")
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_title(f"Z = {z_level:.0f} mm  ({shape})")
        ax.legend(loc="upper right", fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="gray", lw=0.5)
        ax.axvline(0, color="gray", lw=0.5)

    fig.suptitle("Delta Robot Workspace Analysis", fontsize=14, fontweight="bold")
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"\nPlot saved to {save_path}")
    else:
        plt.show()


# ── Main ─────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute and visualize the delta robot's reachable workspace.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    g = parser.add_argument_group("Robot geometry (mm)")
    g.add_argument("--upper-arm", type=float, default=150.0, help="Shoulder-to-elbow L")
    g.add_argument("--lower-arm", type=float, default=268.0, help="Elbow-to-effector l")
    g.add_argument("--fd", type=float, default=82.5, help="Base joint offset Fd")
    g.add_argument("--ed", type=float, default=27.3, help="Effector joint offset Ed")

    g = parser.add_argument_group("Scan parameters")
    g.add_argument("--z-min", type=float, default=-380.0, help="Lowest Z to scan (mm)")
    g.add_argument("--z-max", type=float, default=-80.0, help="Highest Z to scan (mm)")
    g.add_argument("--z-step", type=float, default=10.0, help="Z step size (mm)")
    g.add_argument(
        "--angle-min", type=float, default=-70.0, help="Min joint angle limit (deg)"
    )
    g.add_argument(
        "--angle-max", type=float, default=70.0, help="Max joint angle limit (deg)"
    )

    g = parser.add_argument_group("Output")
    g.add_argument("--plot", action="store_true", help="Show matplotlib plots")
    g.add_argument(
        "--save", type=str, default=None, help="Save plot to file (implies --plot)"
    )
    g.add_argument(
        "--detail-z",
        type=float,
        nargs="+",
        default=[-150.0, -200.0, -250.0, -300.0],
        help="Z heights for detailed XY cross-section plots",
    )

    args = parser.parse_args()

    dk = DeltaKinematics(
        upper_arm=args.upper_arm,
        lower_arm=args.lower_arm,
        Fd=args.fd,
        Ed=args.ed,
    )

    print(f"Scanning Z = [{args.z_min}, {args.z_max}] mm  (step {args.z_step} mm)")
    print(f"Joint limits: [{args.angle_min}°, {args.angle_max}°]")
    print("Computing workspace envelope ...")

    results = analyze_workspace(
        dk,
        args.z_min,
        args.z_max,
        args.z_step,
        args.angle_min,
        args.angle_max,
    )

    print_table(results, dk)
    print_recommendations(results)

    if args.plot or args.save:
        print("\nGenerating cross-section plots (this may take a moment) ...")
        valid_detail = [z for z in args.detail_z if args.z_min <= z <= args.z_max]
        plot_workspace(
            dk, results, valid_detail, args.angle_min, args.angle_max, args.save
        )


if __name__ == "__main__":
    main()
