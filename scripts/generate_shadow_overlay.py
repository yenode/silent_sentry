#!/usr/bin/env python3
"""
generate_shadow_overlay.py
==========================
Procedurally generates a Gazebo SDF overlay for the "shadow region" on a DEM.

Outputs:
  shadow_overlay.sdf  — 150 vegetation <include> tags + boundary-wall <model>

Usage:
  python3 generate_shadow_overlay.py                     # defaults
  python3 generate_shadow_overlay.py --dem output.tif    # custom DEM path

Requires:  rasterio  pyproj  shapely  numpy
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import textwrap
from pathlib import Path

import numpy as np
import rasterio
from pyproj import Transformer
from rasterio.transform import rowcol
from shapely.geometry import Polygon, Point, LineString

# ──────────────────────────────────────────────────────────────────────
# Configuration constants
# ──────────────────────────────────────────────────────────────────────
NUM_VEGETATION = 150
WALL_HEIGHT    = 2.0          # metres above terrain
WALL_THICKNESS = 0.15         # metres
WALL_SEGMENTS  = 30           # segments per edge (more = smoother)
VEGETATION_URI = "model://desert_scrub_bush"
SEED           = 42           # reproducible random placement

# Shadow-region corner coordinates  (lat, lon)
SHADOW_CORNERS_LATLON = [
    (27.0145120, 69.5175878),  # P1  (south-west-ish)
    (27.0131034, 69.5223673),  # P2  (south-east-ish)
    (27.0488652, 69.5371675),  # P4  (north-east-ish)  ← swap P3/P4 for
    (27.0501774, 69.5330125),  # P3  (north-west-ish)    CCW winding
]

# Gazebo heightmap <size> that the DEM is stretched to (must match SDF)
GZ_HEIGHTMAP_SIZE_X = 1735.3   # east-west  metres
GZ_HEIGHTMAP_SIZE_Y = 4143.6   # north-south metres
GZ_HEIGHTMAP_SIZE_Z = 67.0     # vertical   metres


# ──────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────
def open_dem(dem_path: str) -> rasterio.DatasetReader:
    """Open the DEM and return the rasterio dataset."""
    if not os.path.isfile(dem_path):
        sys.exit(f"[ERROR] DEM not found: {dem_path}")
    return rasterio.open(dem_path)


def build_latlon_to_gazebo(ds: rasterio.DatasetReader):
    """
    Return a function  f(lat, lon) -> (gx, gy)  that converts WGS-84
    geographic coordinates into the Gazebo local frame.

    Strategy:
      1. Project lat/lon to a local UTM zone (metres).
      2. Compute the UTM position of the DEM center.
      3. Subtract the center so the DEM center ≡ Gazebo (0, 0).
      4. Scale by the ratio  (Gazebo <size>) / (DEM metric extent)
         because Gazebo stretches the raster to the declared <size>.

    Notes:
      - Gazebo X = East,  Y = North  (ENU frame).
      - The raster's geographic center is computed from its bounds.
    """
    bounds = ds.bounds                              # left, bottom, right, top  (lon, lat order)
    center_lon = (bounds.left  + bounds.right) / 2.0
    center_lat = (bounds.bottom + bounds.top)  / 2.0

    # Determine the UTM zone from the center longitude
    utm_zone = int((center_lon + 180) / 6) + 1
    utm_epsg = 32600 + utm_zone   # northern hemisphere
    if center_lat < 0:
        utm_epsg = 32700 + utm_zone

    to_utm = Transformer.from_crs("EPSG:4326", f"EPSG:{utm_epsg}", always_xy=False)
    # pyproj with always_xy=False:  input is (lat, lon) -> (easting, northing)

    # DEM corners in UTM
    e_min, n_min = to_utm.transform(bounds.bottom, bounds.left)
    e_max, n_max = to_utm.transform(bounds.top,    bounds.right)
    e_ctr, n_ctr = to_utm.transform(center_lat,    center_lon)

    # Metric span of the raster in UTM
    dem_span_x = e_max - e_min   # east-west
    dem_span_y = n_max - n_min   # north-south

    # Scale factors: Gazebo stretches the raster to <size>
    sx = GZ_HEIGHTMAP_SIZE_X / dem_span_x
    sy = GZ_HEIGHTMAP_SIZE_Y / dem_span_y

    def latlon_to_gz(lat: float, lon: float) -> tuple[float, float]:
        e, n = to_utm.transform(lat, lon)
        gx = (e - e_ctr) * sx
        gy = (n - n_ctr) * sy
        return gx, gy

    print(f"  DEM center  (lat, lon)   : ({center_lat:.7f}, {center_lon:.7f})")
    print(f"  UTM zone                 : {utm_epsg}")
    print(f"  DEM metric span (UTM)    : {dem_span_x:.1f} x {dem_span_y:.1f} m")
    print(f"  Gazebo <size>            : {GZ_HEIGHTMAP_SIZE_X} x {GZ_HEIGHTMAP_SIZE_Y} m")
    print(f"  Scale factors            : sx={sx:.6f}  sy={sy:.6f}")

    return latlon_to_gz


def sample_elevation(ds: rasterio.DatasetReader, lat: float, lon: float) -> float:
    """
    Sample the DEM elevation at the given (lat, lon).

    Returns the elevation in Gazebo Z, accounting for the heightmap
    vertical scaling.  Gazebo maps [raster_min .. raster_max] to
    [0 .. <size_z>], with the heightmap base at z = 0.
    """
    band = ds.read(1)
    nodata = ds.nodata

    # Row/col in the raster (lon, lat order for rasterio transform)
    try:
        r, c = rowcol(ds.transform, lon, lat)
    except Exception:
        return 0.0

    # Clamp to valid pixel range
    r = max(0, min(r, ds.height - 1))
    c = max(0, min(c, ds.width  - 1))

    raw = float(band[r, c])
    if nodata is not None and raw == nodata:
        return 0.0

    # Gazebo vertical mapping:  z = (raw - raster_min) / (raster_max - raster_min) * size_z
    valid = band[band != nodata] if nodata is not None else band.ravel()
    rmin  = float(valid.min())
    rmax  = float(valid.max())

    if rmax == rmin:
        return 0.0

    gz_z = (raw - rmin) / (rmax - rmin) * GZ_HEIGHTMAP_SIZE_Z
    return gz_z


def sample_elevation_at_gz(
    ds: rasterio.DatasetReader,
    gx: float, gy: float,
    gz_to_latlon,
) -> float:
    """Convenience: sample elevation from Gazebo (x, y) coordinates."""
    lat, lon = gz_to_latlon(gx, gy)
    return sample_elevation(ds, lat, lon)


def build_gz_to_latlon(ds: rasterio.DatasetReader):
    """Inverse of latlon_to_gz: Gazebo (x,y) -> (lat, lon)."""
    bounds = ds.bounds
    center_lon = (bounds.left  + bounds.right) / 2.0
    center_lat = (bounds.bottom + bounds.top)  / 2.0

    utm_zone = int((center_lon + 180) / 6) + 1
    utm_epsg = 32600 + utm_zone
    if center_lat < 0:
        utm_epsg = 32700 + utm_zone

    to_utm   = Transformer.from_crs("EPSG:4326", f"EPSG:{utm_epsg}", always_xy=False)
    from_utm = Transformer.from_crs(f"EPSG:{utm_epsg}", "EPSG:4326", always_xy=False)

    e_min, n_min = to_utm.transform(bounds.bottom, bounds.left)
    e_max, n_max = to_utm.transform(bounds.top,    bounds.right)
    e_ctr, n_ctr = to_utm.transform(center_lat,    center_lon)

    dem_span_x = e_max - e_min
    dem_span_y = n_max - n_min
    sx = GZ_HEIGHTMAP_SIZE_X / dem_span_x
    sy = GZ_HEIGHTMAP_SIZE_Y / dem_span_y

    def gz_to_latlon(gx: float, gy: float) -> tuple[float, float]:
        e = gx / sx + e_ctr
        n = gy / sy + n_ctr
        lat, lon = from_utm.transform(e, n)
        return lat, lon

    return gz_to_latlon


# ──────────────────────────────────────────────────────────────────────
# SDF generation
# ──────────────────────────────────────────────────────────────────────
def generate_vegetation_sdf(
    points: list[tuple[float, float, float]],
    rng: np.random.Generator,
) -> str:
    """Return SDF snippet for 150 vegetation <include> tags."""
    lines: list[str] = []
    lines.append("    <!-- ═══ SHADOW-REGION VEGETATION (procedural) ═══ -->")
    for i, (x, y, z) in enumerate(points):
        yaw = rng.uniform(0, 2 * math.pi)
        scale_factor = rng.uniform(0.6, 1.4)
        lines.append(
            f"    <include>\n"
            f"      <uri>{VEGETATION_URI}</uri>\n"
            f"      <name>shadow_bush_{i:03d}</name>\n"
            f"      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose>\n"
            f"      <static>true</static>\n"
            f"    </include>"
        )
    return "\n\n".join(lines)


def generate_wall_sdf(
    edges: list[tuple[tuple[float, float], tuple[float, float]]],
    ds: rasterio.DatasetReader,
    gz_to_latlon,
) -> str:
    """
    Return SDF for a boundary wall model made of box segments.

    For each of the 4 polygon edges, we subdivide into WALL_SEGMENTS
    sub-segments. Each sub-segment becomes a <link> with a <box> visual
    and collision whose bottom sits on the terrain.
    """
    lines: list[str] = []
    lines.append("    <!-- ═══ SHADOW-REGION BOUNDARY WALL ═══ -->")
    lines.append("    <model name=\"shadow_boundary_wall\">")
    lines.append("      <static>true</static>")
    lines.append("      <pose>0 0 0 0 0 0</pose>")

    seg_id = 0
    for edge_idx, ((x0, y0), (x1, y1)) in enumerate(edges):
        edge_len = math.hypot(x1 - x0, y1 - y0)
        edge_yaw = math.atan2(y1 - y0, x1 - x0)

        for s in range(WALL_SEGMENTS):
            t0 = s       / WALL_SEGMENTS
            t1 = (s + 1) / WALL_SEGMENTS
            tc = (t0 + t1) / 2.0

            # Midpoint of this sub-segment
            cx = x0 + tc * (x1 - x0)
            cy = y0 + tc * (y1 - y0)

            # Start and end for length calculation
            sx = x0 + t0 * (x1 - x0)
            sy = y0 + t0 * (y1 - y0)
            ex = x0 + t1 * (x1 - x0)
            ey = y0 + t1 * (y1 - y0)
            seg_len = math.hypot(ex - sx, ey - sy) + 0.02  # tiny overlap

            # Sample terrain elevation at segment center
            z_terrain = sample_elevation_at_gz(ds, cx, cy, gz_to_latlon)

            # Also sample at start/end to compute pitch for terrain slope
            z_start = sample_elevation_at_gz(ds, sx, sy, gz_to_latlon)
            z_end   = sample_elevation_at_gz(ds, ex, ey, gz_to_latlon)
            pitch   = math.atan2(z_end - z_start, seg_len)

            # Wall center z: terrain + half wall height
            cz = z_terrain + WALL_HEIGHT / 2.0

            link_name = f"wall_e{edge_idx}_s{s:02d}"
            lines.append(f"      <link name=\"{link_name}\">")
            lines.append(f"        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 {pitch:.4f} {edge_yaw:.4f}</pose>")

            # Visual
            lines.append(f"        <visual name=\"v\">")
            lines.append(f"          <geometry>")
            lines.append(f"            <box><size>{seg_len:.3f} {WALL_THICKNESS} {WALL_HEIGHT}</size></box>")
            lines.append(f"          </geometry>")
            lines.append(f"          <material>")
            lines.append(f"            <ambient>0.55 0.45 0.35 1</ambient>")
            lines.append(f"            <diffuse>0.55 0.45 0.35 1</diffuse>")
            lines.append(f"            <specular>0.1 0.1 0.1 1</specular>")
            lines.append(f"          </material>")
            lines.append(f"        </visual>")

            # Collision
            lines.append(f"        <collision name=\"c\">")
            lines.append(f"          <geometry>")
            lines.append(f"            <box><size>{seg_len:.3f} {WALL_THICKNESS} {WALL_HEIGHT}</size></box>")
            lines.append(f"          </geometry>")
            lines.append(f"        </collision>")

            lines.append(f"      </link>")
            seg_id += 1

    lines.append("    </model>")
    return "\n".join(lines)


def write_sdf(vegetation_sdf: str, wall_sdf: str, out_path: str) -> None:
    """Wrap everything in a complete SDF world fragment file."""
    sdf = (
        '<?xml version="1.0" ?>\n'
        '<!--\n'
        '  shadow_overlay.sdf\n'
        '  Auto-generated procedural overlay for the shadow region.\n'
        '  Paste the contents inside your main world SDF.\n'
        '-->\n'
        '<sdf version="1.9">\n'
        '  <world name="shadow_overlay">\n\n'
        f'{vegetation_sdf}\n\n'
        f'{wall_sdf}\n\n'
        '  </world>\n'
        '</sdf>\n'
    )
    with open(out_path, "w") as f:
        f.write(sdf)
    print(f"\n  [OK] Written {out_path}  ({os.path.getsize(out_path)} bytes)")


# ──────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Generate shadow-region SDF overlay from a DEM."
    )
    parser.add_argument(
        "--dem", default="src/bot_description/worlds/dem/output_be.tif",
        help="Path to the GeoTIFF DEM (default: world/output.tif)",
    )
    parser.add_argument(
        "--out", default="src/bot_description/worlds/shadow_overlay.sdf",
        help="Output SDF file path (default: shadow_overlay.sdf)",
    )
    parser.add_argument(
        "--num-vegetation", type=int, default=NUM_VEGETATION,
        help=f"Number of vegetation points (default: {NUM_VEGETATION})",
    )
    parser.add_argument(
        "--wall-segments", type=int, default=WALL_SEGMENTS,
        help=f"Wall segments per edge (default: {WALL_SEGMENTS})",
    )
    parser.add_argument(
        "--seed", type=int, default=SEED,
        help=f"Random seed (default: {SEED})",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  Shadow-Region SDF Generator")
    print("=" * 60)

    # ── 1. Open DEM ──────────────────────────────────────────────
    print(f"\n[1/5] Opening DEM: {args.dem}")
    ds = open_dem(args.dem)
    print(f"  Raster size              : {ds.width} x {ds.height}")
    print(f"  CRS                      : {ds.crs}")
    print(f"  Bounds (lon)             : {ds.bounds.left:.7f} .. {ds.bounds.right:.7f}")
    print(f"  Bounds (lat)             : {ds.bounds.bottom:.7f} .. {ds.bounds.top:.7f}")

    # ── 2. Build coordinate transforms ───────────────────────────
    print("\n[2/5] Building coordinate transforms...")
    latlon_to_gz = build_latlon_to_gazebo(ds)
    gz_to_latlon = build_gz_to_latlon(ds)

    # Convert the 4 shadow corners to Gazebo local coords
    gz_corners = []
    print("\n  Shadow-region corners (Gazebo local):")
    for i, (lat, lon) in enumerate(SHADOW_CORNERS_LATLON):
        gx, gy = latlon_to_gz(lat, lon)
        gz_corners.append((gx, gy))
        z = sample_elevation(ds, lat, lon)
        print(f"    P{i+1}: lat={lat:.7f} lon={lon:.7f}  ->  gx={gx:+9.2f}  gy={gy:+9.2f}  z={z:.2f}")

    shadow_poly = Polygon(gz_corners)
    if not shadow_poly.is_valid:
        shadow_poly = shadow_poly.buffer(0)
    print(f"\n  Polygon area             : {shadow_poly.area:.1f} m^2")
    print(f"  Polygon perimeter        : {shadow_poly.length:.1f} m")
    cx, cy = shadow_poly.centroid.x, shadow_poly.centroid.y
    print(f"  Polygon centroid (gz)    : ({cx:.1f}, {cy:.1f})")

    # ── 3. Generate vegetation points ────────────────────────────
    print(f"\n[3/5] Generating {args.num_vegetation} vegetation points...")
    rng = np.random.default_rng(args.seed)

    minx, miny, maxx, maxy = shadow_poly.bounds
    veg_points: list[tuple[float, float, float]] = []

    attempts = 0
    max_attempts = args.num_vegetation * 50
    while len(veg_points) < args.num_vegetation and attempts < max_attempts:
        rx = rng.uniform(minx, maxx)
        ry = rng.uniform(miny, maxy)
        if shadow_poly.contains(Point(rx, ry)):
            lat, lon = gz_to_latlon(rx, ry)
            z = sample_elevation(ds, lat, lon)
            veg_points.append((rx, ry, z))
        attempts += 1

    print(f"  Generated {len(veg_points)} points in {attempts} attempts")
    if len(veg_points) < args.num_vegetation:
        print(f"  [WARN] Only {len(veg_points)}/{args.num_vegetation} points found!")

    # ── 4. Generate boundary wall ────────────────────────────────
    print(f"\n[4/5] Generating boundary walls ({args.wall_segments} segments/edge)...")

    # Build edge list from the polygon exterior ring (closed)
    coords = list(shadow_poly.exterior.coords)  # last == first
    edges = []
    for i in range(len(coords) - 1):
        edges.append((coords[i], coords[i + 1]))
    print(f"  Edges: {len(edges)}")
    for i, ((x0, y0), (x1, y1)) in enumerate(edges):
        length = math.hypot(x1 - x0, y1 - y0)
        print(f"    Edge {i}: ({x0:+.1f},{y0:+.1f}) -> ({x1:+.1f},{y1:+.1f})  len={length:.1f} m")

    # ── 5. Build SDF ─────────────────────────────────────────────
    print(f"\n[5/5] Writing SDF to {args.out}...")
    veg_sdf  = generate_vegetation_sdf(veg_points, rng)
    wall_sdf = generate_wall_sdf(edges, ds, gz_to_latlon)
    write_sdf(veg_sdf, wall_sdf, args.out)

    total_models = len(veg_points) + len(edges) * args.wall_segments
    print(f"  Total SDF entities       : {total_models}")
    print(f"    Vegetation includes    : {len(veg_points)}")
    print(f"    Wall segments          : {len(edges) * args.wall_segments}")
    print("\nDone!")

    ds.close()


if __name__ == "__main__":
    main()
