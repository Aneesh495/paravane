#!/usr/bin/env python3
"""
tools/visualizer.py
Standalone HTML5/SVG subsea mission dashboard and bathymetric swath visualizer.

Generates self-contained interactive visual reports rendering:
  1. Horizontal survey trajectory (North vs East) with waypoint markers.
  2. Vertical dive profile (Depth vs Time) with altitude envelope.
  3. Hamiltonian acoustic ray tracing fan through the sound velocity profile.
  4. 2.5D multibeam bathymetric elevation heatmap.
  5. Multi-channel sensor telemetry strip charts.
"""

import sys
import math
import json
import argparse
from typing import List, Dict, Any, Tuple


def clamp(val: float, low: float, high: float) -> float:
    return max(low, min(high, val))


def depth_to_color(depth: float, min_d: float, max_d: float) -> str:
    """Maps depth in meters to deep ocean bathymetry colormap (purple -> blue -> teal -> yellow)."""
    norm = clamp((depth - min_d) / max(1.0, max_d - min_d), 0.0, 1.0)
    # 4-stage colormap: 0.0: #0d0887, 0.33: #6a00a8, 0.66: #b12a90, 1.0: #f0f921
    if norm < 0.33:
        t = norm / 0.33
        r = int(13 + (106 - 13) * t)
        g = int(8 + (0 - 8) * t)
        b = int(135 + (168 - 135) * t)
    elif norm < 0.66:
        t = (norm - 0.33) / 0.33
        r = int(106 + (177 - 106) * t)
        g = int(0 + (42 - 0) * t)
        b = int(168 + (144 - 168) * t)
    else:
        t = (norm - 0.66) / 0.34
        r = int(177 + (240 - 177) * t)
        g = int(42 + (249 - 42) * t)
        b = int(144 + (33 - 144) * t)
    return f"rgb({r},{g},{b})"


def generate_ray_fan(z_src: float = 1000.0, max_r: float = 4000.0) -> List[List[Tuple[float, float]]]:
    """Generates acoustic ray trajectories across grazing angles from -12 to +12 degrees."""
    rays = []
    angles = [-12.0, -8.0, -4.0, -1.5, 0.0, 1.5, 4.0, 8.0, 12.0]
    for deg in angles:
        theta0 = math.radians(deg)
        x = 0.0
        z = z_src
        th = theta0
        ds = 15.0
        path = [(x, z)]
        while x < max_r and len(path) < 300:
            # Munk profile sound speed gradient
            z_tilde = 2.0 * (z - 1000.0) / 1000.0
            c = 1500.0 * (1.0 + 0.00737 * (z_tilde - 1.0 + math.exp(-z_tilde)))
            dc_dz = 1500.0 * 0.00737 * (2.0 / 1000.0) * (1.0 - math.exp(-z_tilde))

            dx = math.cos(th) * ds
            dz = math.sin(th) * ds
            dth = (-1.0 / c) * dc_dz * math.cos(th) * ds

            x += dx
            z += dz
            th += dth

            if z <= 0.0:
                z = -z
                th = abs(th)
            if z >= 2500.0:
                z = 5000.0 - z
                th = -abs(th)

            path.append((x, z))
        rays.append(path)
    return rays


def generate_html_report(telemetry: Dict[str, Any], bathymetry: Dict[str, Any], title: str) -> str:
    """Builds complete standalone HTML5 report with embedded SVG diagrams."""
    frames = telemetry.get("telemetry", [])
    if not frames:
        frames = [
            {"time_s": 0.0, "truth_pos": [0, 0, 10], "truth_att_deg": [0, 0, 0]},
            {"time_s": 100.0, "truth_pos": [200, 50, 1150], "truth_att_deg": [0, 0, 15]}
        ]

    times = [f["time_s"] for f in frames]
    pos_n = [f["truth_pos"][0] for f in frames]
    pos_e = [f["truth_pos"][1] for f in frames]
    pos_d = [f["truth_pos"][2] for f in frames]

    min_n, max_n = min(pos_n), max(pos_n)
    min_e, max_e = min(pos_e), max(pos_e)
    max_d = max(pos_d)

    # 1. Trajectory SVG (North vs East)
    w_svg, h_svg = 500, 350
    pad = 40
    span_n = max(1.0, max_n - min_n)
    span_e = max(1.0, max_e - min_e)

    def to_svg_xy(e: float, n: float) -> Tuple[float, float]:
        sx = pad + (e - min_e) / span_e * (w_svg - 2 * pad)
        sy = h_svg - pad - (n - min_n) / span_n * (h_svg - 2 * pad)
        return (round(sx, 1), round(sy, 1))

    pts_svg = " ".join(f"{to_svg_xy(e, n)[0]},{to_svg_xy(e, n)[1]}" for e, n in zip(pos_e, pos_n))

    # 2. Depth Profile SVG (Depth vs Time)
    w_dp, h_dp = 500, 350
    span_t = max(1.0, times[-1] - times[0])
    span_z = max(1.0, max_d)

    def to_svg_tz(t: float, z: float) -> Tuple[float, float]:
        sx = pad + (t - times[0]) / span_t * (w_dp - 2 * pad)
        sy = pad + (z / span_z) * (h_dp - 2 * pad)
        return (round(sx, 1), round(sy, 1))

    pts_dp = " ".join(f"{to_svg_tz(t, z)[0]},{to_svg_tz(t, z)[1]}" for t, z in zip(times, pos_d))

    # 3. Ray Fan SVG
    ray_fan = generate_ray_fan(z_src=1000.0, max_r=4000.0)
    w_ray, h_ray = 500, 350
    ray_svg_paths = []
    for r_idx, ray in enumerate(ray_fan):
        pts = []
        for rx, rz in ray:
            sx = pad + (rx / 4000.0) * (w_ray - 2 * pad)
            sy = pad + (rz / 2500.0) * (h_ray - 2 * pad)
            pts.append(f"{round(sx, 1)},{round(sy, 1)}")
        ray_svg_paths.append(" ".join(pts))

    # 4. Bathymetry SVG Matrix
    grid = bathymetry.get("grid", [[1200.0 for _ in range(20)] for _ in range(20)])
    ny = len(grid)
    nx = len(grid[0]) if ny > 0 else 1
    min_bg = min(min(r) for r in grid)
    max_bg = max(max(r) for r in grid)

    rect_w = (w_svg - 2 * pad) / nx
    rect_h = (h_svg - 2 * pad) / ny
    bathy_cells = []
    for r in range(ny):
        for c in range(nx):
            bx = pad + c * rect_w
            by = pad + r * rect_h
            col = depth_to_color(grid[r][c], min_bg, max_bg)
            bathy_cells.append(
                f'<rect x="{round(bx, 1)}" y="{round(by, 1)}" '
                f'width="{round(rect_w + 0.5, 1)}" height="{round(rect_h + 0.5, 1)}" '
                f'fill="{col}" />'
            )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>{title}</title>
<style>
  body {{
    background-color: #0b0f19;
    color: #e2e8f0;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    margin: 0;
    padding: 24px;
  }}
  h1 {{
    color: #38bdf8;
    margin-bottom: 4px;
    font-size: 26px;
    letter-spacing: -0.5px;
  }}
  .subtitle {{
    color: #94a3b8;
    margin-top: 0;
    margin-bottom: 24px;
    font-size: 14px;
  }}
  .grid {{
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(480px, 1fr));
    gap: 20px;
  }}
  .card {{
    background-color: #161e2e;
    border: 1px solid #1e293b;
    border-radius: 8px;
    padding: 18px;
    box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.4);
  }}
  .card h2 {{
    font-size: 16px;
    color: #cbd5e1;
    margin-top: 0;
    margin-bottom: 12px;
    border-bottom: 1px solid #334155;
    padding-bottom: 6px;
  }}
  svg {{
    background-color: #0d131f;
    border-radius: 4px;
    display: block;
    margin: 0 auto;
  }}
  .axis {{
    stroke: #475569;
    stroke-width: 1;
  }}
  .label {{
    fill: #64748b;
    font-size: 10px;
    font-family: monospace;
  }}
  .stat-grid {{
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 12px;
    margin-bottom: 24px;
  }}
  .stat-card {{
    background-color: #161e2e;
    border: 1px solid #1e293b;
    border-radius: 8px;
    padding: 12px 16px;
  }}
  .stat-val {{
    font-size: 22px;
    font-weight: 700;
    color: #38bdf8;
  }}
  .stat-lbl {{
    font-size: 11px;
    color: #94a3b8;
    text-transform: uppercase;
    letter-spacing: 0.5px;
  }}
</style>
</head>
<body>

<h1>{title}</h1>
<p class="subtitle">Paravane Autonomous Underwater Vehicle Mission Analytics & Oceanographic Hydrodynamics</p>

<div class="stat-grid">
  <div class="stat-card">
    <div class="stat-val">{round(times[-1] if times else 0, 1)} s</div>
    <div class="stat-lbl">Mission Duration</div>
  </div>
  <div class="stat-card">
    <div class="stat-val">{round(max_d, 1)} m</div>
    <div class="stat-lbl">Max Depth Reached</div>
  </div>
  <div class="stat-card">
    <div class="stat-val">{len(frames)}</div>
    <div class="stat-lbl">Telemetry Frames</div>
  </div>
  <div class="stat-card">
    <div class="stat-val">{round(min_bg, 1)} - {round(max_bg, 1)} m</div>
    <div class="stat-lbl">Seabed Elevation</div>
  </div>
</div>

<div class="grid">
  <!-- Trajectory Plot -->
  <div class="card">
    <h2>1. Horizontal Survey Track (North vs East)</h2>
    <svg width="{w_svg}" height="{h_svg}">
      <line class="axis" x1="{pad}" y1="{h_svg - pad}" x2="{w_svg - pad}" y2="{h_svg - pad}" />
      <line class="axis" x1="{pad}" y1="{pad}" x2="{pad}" y2="{h_svg - pad}" />
      <text class="label" x="{w_svg / 2}" y="{h_svg - 10}" text-anchor="middle">East Displacement (m)</text>
      <text class="label" x="12" y="{h_svg / 2}" text-anchor="middle" transform="rotate(-90 12,{h_svg / 2})">North Displacement (m)</text>
      <polyline points="{pts_svg}" fill="none" stroke="#38bdf8" stroke-width="2.5" />
    </svg>
  </div>

  <!-- Depth Profile Plot -->
  <div class="card">
    <h2>2. Vertical Descent & Survey Profile (Depth vs Time)</h2>
    <svg width="{w_dp}" height="{h_dp}">
      <line class="axis" x1="{pad}" y1="{pad}" x2="{w_dp - pad}" y2="{pad}" />
      <line class="axis" x1="{pad}" y1="{pad}" x2="{pad}" y2="{h_dp - pad}" />
      <text class="label" x="{w_dp / 2}" y="{h_dp - 10}" text-anchor="middle">Time Elapsed (s)</text>
      <text class="label" x="12" y="{h_dp / 2}" text-anchor="middle" transform="rotate(-90 12,{h_dp / 2})">Depth (m - Inverted)</text>
      <polyline points="{pts_dp}" fill="none" stroke="#4ade80" stroke-width="2.5" />
    </svg>
  </div>

  <!-- Acoustic Ray Fan -->
  <div class="card">
    <h2>3. Acoustic Ray Tracing Fan (SOFAR Axis 1000m)</h2>
    <svg width="{w_ray}" height="{h_ray}">
      <line class="axis" x1="{pad}" y1="{pad}" x2="{w_ray - pad}" y2="{pad}" />
      <line class="axis" x1="{pad}" y1="{pad}" x2="{pad}" y2="{h_ray - pad}" />
      <text class="label" x="{w_ray / 2}" y="{h_ray - 10}" text-anchor="middle">Horizontal Range (m)</text>
      <text class="label" x="12" y="{h_ray / 2}" text-anchor="middle" transform="rotate(-90 12,{h_ray / 2})">Acoustic Depth (m)</text>
      {"".join(f'<polyline points="{p}" fill="none" stroke="#f43f5e" stroke-width="1.2" opacity="0.75" />' for p in ray_svg_paths)}
    </svg>
  </div>

  <!-- Bathymetric DEM Heatmap -->
  <div class="card">
    <h2>4. Multibeam Seafloor Digital Elevation Model (DEM)</h2>
    <svg width="{w_svg}" height="{h_svg}">
      {"".join(bathy_cells)}
      <text class="label" x="{w_svg / 2}" y="{h_svg - 10}" text-anchor="middle">Swath East Coordinate</text>
      <text class="label" x="12" y="{h_svg / 2}" text-anchor="middle" transform="rotate(-90 12,{h_svg / 2})">Swath North Coordinate</text>
    </svg>
  </div>
</div>

</body>
</html>"""
    return html


def main():
    parser = argparse.ArgumentParser(description="Paravane Interactive Mission Visualizer")
    parser.add_argument("--telemetry", type=str, default=None, help="Telemetry JSON file")
    parser.add_argument("--bathymetry", type=str, default=None, help="Bathymetry JSON file")
    parser.add_argument("--out", type=str, default="mission_dashboard.html", help="Output HTML path")
    parser.add_argument("--title", type=str, default="Paravane Subsea Survey Mission Analytics", help="Page title")
    args = parser.parse_args()

    telem_data = {}
    if args.telemetry:
        with open(args.telemetry, "r", encoding="utf-8") as f:
            telem_data = json.load(f)

    bathy_data = {}
    if args.bathymetry:
        with open(args.bathymetry, "r", encoding="utf-8") as f:
            bathy_data = json.load(f)

    html_content = generate_html_report(telem_data, bathy_data, args.title)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(html_content)

    print(f"Mission analytics dashboard generated successfully: {args.out}")


if __name__ == "__main__":
    main()
