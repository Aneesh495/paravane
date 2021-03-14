#!/usr/bin/env python3
"""
tools/bathymetry_generator.py
Fractal seabed terrain and bathymetry generator for subsea robotics simulations.

Generates realistic multibeam bathymetric digital elevation models (DEM)
featuring continental shelves, seamounts, fracture trenches, and hydrothermal
vent chimneys using multi-scale spectral noise and geomorphological primitives.
"""

import sys
import math
import random
import json
import argparse
from typing import List, Dict, Any, Tuple


class BathymetryField:
    """Represents a 2.5D bathymetric elevation grid in local coordinates."""

    def __init__(self, nx: int, ny: int, resolution: float, base_depth: float = 1200.0):
        self.nx = nx
        self.ny = ny
        self.resolution = resolution
        self.base_depth = base_depth
        self.grid = [[base_depth for _ in range(nx)] for _ in range(ny)]

    def add_fractal_noise(self, octaves: int = 5, persistence: float = 0.5, amplitude: float = 120.0):
        """Adds multi-scale harmonic noise to simulate natural seafloor roughness."""
        for oct_idx in range(octaves):
            freq = (2.0 ** oct_idx) * 0.005
            amp = amplitude * (persistence ** oct_idx)
            phase_x = random.uniform(0.0, 2.0 * math.pi)
            phase_y = random.uniform(0.0, 2.0 * math.pi)

            for r in range(self.ny):
                y = r * self.resolution
                for c in range(self.nx):
                    x = c * self.resolution
                    val = (
                        math.sin(x * freq + phase_x) * math.cos(y * freq + phase_y) +
                        0.5 * math.sin((x + y) * freq * 0.7 + phase_x * 0.5)
                    )
                    self.grid[r][c] += val * amp

    def add_seamount(self, cx: float, cy: float, height: float, radius: float):
        """Adds a conical seamount with Gaussian profile."""
        r2 = radius * radius
        for r in range(self.ny):
            y = r * self.resolution
            dy = y - cy
            for c in range(self.nx):
                x = c * self.resolution
                dx = x - cx
                dist2 = dx * dx + dy * dy
                if dist2 < 4.0 * r2:
                    delta = height * math.exp(-dist2 / (2.0 * (radius * 0.4) ** 2))
                    self.grid[r][c] -= delta

    def add_trench(self, x0: float, y0: float, x1: float, y1: float, depth: float, width: float):
        """Adds a fault trench or rift fracture line."""
        dx = x1 - x0
        dy = y1 - y0
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-6:
            return

        w2 = (width * 0.5) ** 2
        for r in range(self.ny):
            y = r * self.resolution
            for c in range(self.nx):
                x = c * self.resolution
                t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / seg_len2))
                proj_x = x0 + t * dx
                proj_y = y0 + t * dy
                dist2 = (x - proj_x) ** 2 + (y - proj_y) ** 2
                if dist2 < 4.0 * w2:
                    profile = math.exp(-dist2 / (2.0 * (width * 0.3) ** 2))
                    self.grid[r][c] += depth * profile

    def add_hydrothermal_vents(self, count: int = 4, cluster_x: float = 250.0, cluster_y: float = 250.0):
        """Adds high-aspect black smoker hydrothermal vent chimneys."""
        for _ in range(count):
            vx = cluster_x + random.gauss(0.0, 30.0)
            vy = cluster_y + random.gauss(0.0, 30.0)
            height = random.uniform(8.0, 22.0)
            radius = random.uniform(2.5, 5.0)
            self.add_seamount(vx, vy, height, radius)

    def to_dict(self) -> Dict[str, Any]:
        """Serializes bathymetric field to structured dictionary."""
        min_depth = min(min(row) for row in self.grid)
        max_depth = max(max(row) for row in self.grid)
        return {
            "nx": self.nx,
            "ny": self.ny,
            "resolution_m": self.resolution,
            "base_depth_m": self.base_depth,
            "min_depth_m": round(min_depth, 2),
            "max_depth_m": round(max_depth, 2),
            "grid": [[round(val, 2) for val in row] for row in self.grid]
        }

    def save_json(self, path: str):
        """Saves bathymetric grid as JSON."""
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, indent=2)

    def sample_depth(self, x: float, y: float) -> float:
        """Bilinear interpolation of seabed depth at continuous (x, y)."""
        gx = x / self.resolution
        gy = y / self.resolution
        c0 = int(math.floor(gx))
        r0 = int(math.floor(gy))

        if c0 < 0 or c0 >= self.nx - 1 or r0 < 0 or r0 >= self.ny - 1:
            return self.base_depth

        tx = gx - c0
        ty = gy - r0
        d00 = self.grid[r0][c0]
        d10 = self.grid[r0][c0 + 1]
        d01 = self.grid[r0 + 1][c0]
        d11 = self.grid[r0 + 1][c0 + 1]

        top = (1.0 - tx) * d00 + tx * d10
        bot = (1.0 - tx) * d01 + tx * d11
        return (1.0 - ty) * top + ty * bot


def main():
    parser = argparse.ArgumentParser(description="Paravane Fractal Bathymetry Field Generator")
    parser.add_argument("--width", type=int, default=100, help="Grid points in X axis (default 100)")
    parser.add_argument("--height", type=int, default=100, help="Grid points in Y axis (default 100)")
    parser.add_argument("--resolution", type=float, default=5.0, help="Cell spacing in meters (default 5.0m)")
    parser.add_argument("--base-depth", type=float, default=1200.0, help="Mean water depth in meters (default 1200m)")
    parser.add_argument("--out", type=str, default="bathymetry.json", help="Output JSON path")
    parser.add_argument("--seed", type=int, default=42, help="Random number generator seed")
    args = parser.parse_args()

    random.seed(args.seed)
    field = BathymetryField(args.width, args.height, args.resolution, args.base_depth)
    field.add_fractal_noise(octaves=4, persistence=0.5, amplitude=45.0)

    # Add geological features
    field.add_seamount(cx=args.width * args.resolution * 0.35,
                       cy=args.height * args.resolution * 0.4,
                       height=110.0, radius=80.0)

    field.add_trench(x0=args.width * args.resolution * 0.1,
                     y0=args.height * args.resolution * 0.8,
                     x1=args.width * args.resolution * 0.9,
                     y1=args.height * args.resolution * 0.7,
                     depth=65.0, width=45.0)

    field.add_hydrothermal_vents(count=6,
                                 cluster_x=args.width * args.resolution * 0.65,
                                 cluster_y=args.height * args.resolution * 0.35)

    field.save_json(args.out)
    info = field.to_dict()
    print(f"Generated bathymetric field {args.width}x{args.height} at {args.resolution}m resolution.")
    print(f"Depth envelope: [{info['min_depth_m']}m, {info['max_depth_m']}m]. Saved to {args.out}")


if __name__ == "__main__":
    main()
