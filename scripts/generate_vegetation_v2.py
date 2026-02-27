#!/usr/bin/env python3
"""
Generate EVEN vegetation distribution across the entire shadow region.
Uses a grid-based approach with jitter to ensure uniform coverage — no sparse gaps.
All 13 model types used, with realistic proportions relative to the robot.
Robot: 1.37m L × 0.37m W × 0.59m H (with wheels).
"""

import numpy as np
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '.venv', 'lib', 'python3.12', 'site-packages'))
import rasterio

np.random.seed(2026)

# ─── DEM ──────────────────────────────────────────────────────────────
DEM_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
    'src/bot_description/worlds/dem/output_be_sq.tif'))
HM_SIZE_X = 1735.3
HM_SIZE_Y = 4143.6

# Shadow polygon (clipped to DEM bounds)
SHADOW_POLY = np.array([
    [-867.65, -1981.47],
    [-564.0,  -2071.8],
    [491.5,    2045.2],
    [867.65,   1899.1],
])

SPAWN_X, SPAWN_Y = -50.5, -23.2
SPAWN_CLEAR = 25.0  # clearance around spawn

# ─── Models with CORRECT sizes (after orientation + scale fix) ────────
# (model_uri, category, real_height_m)
# Categories: bush, shrub, rock, small_tree, medium_tree, large_tree
MODELS = [
    ("model://desert_scrub_bush",              "bush",        0.8),
    ("model://stylized_bush_low_poly",         "bush",        1.0),
    ("model://bush",                           "bush",        1.2),
    ("model://low_poly_bush_buxus",            "shrub",       0.8),
    ("model://low_poly_shrub",                 "shrub",       1.5),
    ("model://low_poly_shrub_-_small_texture", "shrub",       1.5),
    ("model://low_poly_stylized_mossy_rocks",  "rock",        0.6),
    ("model://lowpoly_tree",                   "small_tree",  3.0),
    ("model://low_poly_tree",                  "small_tree",  3.5),
    ("model://hill_top_tree",                  "medium_tree", 5.0),
    ("model://palm_tree_realistic",            "large_tree",  6.0),
    ("model://Oak_tree",                       "large_tree",  8.0),
    ("model://Pine_Tree",                      "large_tree",  8.0),
]

# Weight distribution for desert realism:
# 40% bushes, 20% shrubs, 10% rocks, 15% small trees, 10% medium, 5% large
CAT_WEIGHTS = {
    "bush": 0.40, "shrub": 0.20, "rock": 0.10,
    "small_tree": 0.15, "medium_tree": 0.10, "large_tree": 0.05
}

def load_dem():
    with rasterio.open(DEM_PATH) as ds:
        return ds.read(1).astype(float)

def sample_z(dem, x, y):
    nr, nc = dem.shape
    px = (x + HM_SIZE_X/2) / HM_SIZE_X * (nc - 1)
    py = (HM_SIZE_Y/2 - y) / HM_SIZE_Y * (nr - 1)
    px = np.clip(px, 0, nc - 1)
    py = np.clip(py, 0, nr - 1)
    x0, y0 = int(np.floor(px)), int(np.floor(py))
    x1, y1 = min(x0+1, nc-1), min(y0+1, nr-1)
    fx, fy = px - x0, py - y0
    return float(dem[y0,x0]*(1-fx)*(1-fy) + dem[y0,x1]*fx*(1-fy) +
                 dem[y1,x0]*(1-fx)*fy + dem[y1,x1]*fx*fy)

def point_in_poly(x, y, poly):
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]; xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj-xi)*(y-yi)/(yj-yi)+xi):
            inside = not inside
        j = i
    return inside

def pick_model():
    """Weighted random model selection."""
    r = np.random.random()
    cum = 0
    for cat, w in CAT_WEIGHTS.items():
        cum += w
        if r <= cum:
            candidates = [m for m in MODELS if m[1] == cat]
            return candidates[np.random.randint(len(candidates))]
    return MODELS[0]

def main():
    dem = load_dem()
    print(f"DEM: {dem.shape}, z={dem.min():.1f}–{dem.max():.1f}")

    # Bounding box of shadow polygon
    xmin, xmax = SHADOW_POLY[:,0].min(), SHADOW_POLY[:,0].max()
    ymin, ymax = SHADOW_POLY[:,1].min(), SHADOW_POLY[:,1].max()

    # ─── GRID-BASED EVEN PLACEMENT ───────────────────────────────────
    # Use a grid with ~20m spacing + random jitter of ±8m
    # This ensures EVEN dense coverage with no large gaps
    GRID_SPACING = 20.0   # meters between grid points
    JITTER = 8.0          # random offset from grid point

    # Generate grid points across the bounding box
    grid_xs = np.arange(xmin + GRID_SPACING/2, xmax, GRID_SPACING)
    grid_ys = np.arange(ymin + GRID_SPACING/2, ymax, GRID_SPACING)

    plants = []
    for gx in grid_xs:
        for gy in grid_ys:
            # Add random jitter
            x = gx + np.random.uniform(-JITTER, JITTER)
            y = gy + np.random.uniform(-JITTER, JITTER)

            # Check inside shadow polygon
            if not point_in_poly(x, y, SHADOW_POLY):
                continue

            # Skip spawn clearance
            if np.sqrt((x - SPAWN_X)**2 + (y - SPAWN_Y)**2) < SPAWN_CLEAR:
                continue

            # 95% chance to place (high fill for dense coverage)
            if np.random.random() > 0.95:
                continue

            uri, cat, height = pick_model()
            yaw = np.random.uniform(0, 2*np.pi)
            z = sample_z(dem, x, y)
            plants.append((x, y, z, uri, yaw))

    print(f"Generated {len(plants)} vegetation instances")
    print(f"Grid: {len(grid_xs)}×{len(grid_ys)} = {len(grid_xs)*len(grid_ys)} grid points")

    # Count types
    counts = {}
    for _, _, _, uri, _ in plants:
        name = uri.split("://")[1]
        counts[name] = counts.get(name, 0) + 1
    print("\nDistribution:")
    for name, c in sorted(counts.items(), key=lambda x: -x[1]):
        print(f"  {name:42s} {c:4d}")

    # Generate SDF
    lines = []
    lines.append("    <!-- ═══ VEGETATION — grid-based dense even placement (v3) ═══ -->")
    lines.append(f"    <!-- {len(plants)} instances, ~20m grid + 8m jitter, 95% fill, rotation FIXED -->")
    lines.append("")

    for i, (x, y, z, uri, yaw) in enumerate(plants):
        lines.append("    <include>")
        lines.append(f"      <uri>{uri}</uri>")
        lines.append(f"      <name>veg_{i:04d}</name>")
        lines.append(f"      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose>")
        lines.append("      <static>true</static>")
        lines.append("    </include>")

    out = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'bot_description', 'worlds', 'vegetation_includes.sdf'))
    with open(out, 'w') as f:
        f.write("\n".join(lines))
    print(f"\nWritten to: {out}")

if __name__ == "__main__":
    main()
