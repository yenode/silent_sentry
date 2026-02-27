#!/usr/bin/env python3
"""
Generate realistic desert vegetation placement for the Thar Desert world.

Satellite image analysis (30m resolution):
- Dark grey patches = vegetation clusters (bushes, scrub, small trees)
- Covers roughly 15-20% of the terrain area in scattered clumps
- Vegetation is NOT uniform — it clusters in patches of 3-15 plants
- Patches are roughly 30-100m across with 50-200m of bare sand between them
- A thin road/track runs through the center (keep clear)
- Vegetation density is higher toward edges, sparser in center

Strategy:
1. Generate cluster centers using Poisson disk sampling
2. Around each cluster center, scatter 3-12 plants
3. Mix model types: 60% small bushes/shrubs, 25% medium trees, 15% large trees/palms
4. Sample Z from DEM at each point
5. Add random yaw rotation + slight scale variation
"""

import numpy as np
import sys
import os

# Add venv support
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '.venv', 'lib', 'python3.12', 'site-packages'))

import rasterio

np.random.seed(42)

# ─── DEM Configuration ────────────────────────────────────────────────
DEM_PATH = os.path.join(os.path.dirname(__file__), '..', 
                        'src/bot_description/worlds/dem/output_be_sq.tif')
DEM_PATH = os.path.abspath(DEM_PATH)

# Heightmap extent in Gazebo coordinates (from SDF)
HM_SIZE_X = 1735.3   # meters (East-West)
HM_SIZE_Y = 4143.6   # meters (North-South)
HM_SIZE_Z = 51.0     # meters (elevation range)
HM_Z_MIN = 42.0      # minimum elevation in DEM

# Shadow region polygon (clipped to DEM bounds) — the operational area
# From previous computation: P1(-1038,-1925) P2(-564,-2082) P3(491.5,2045) P4(903.5,1899)
# Clipped to DEM rect: x∈[-867.65, 867.65], y∈[-2071.8, 2071.8]
SHADOW_POLY = np.array([
    [-867.65, -1981.47],  # P1 clipped
    [-564.0,  -2071.8],   # P2 clipped  
    [491.5,    2045.2],   # P3
    [867.65,   1899.1],   # P4 clipped
])

# Robot spawn point — keep a 30m clearance radius
SPAWN_X, SPAWN_Y = -50.5, -23.2

# Road/track corridor — keep 8m clearance on each side of the center line
# From the image, a thin track runs roughly N-S through the middle
ROAD_CENTER_X = 0.0  # approximate center of the track
ROAD_CLEARANCE = 10.0

# ─── Model Categories ─────────────────────────────────────────────────
# Category: (model_uri, realistic_height_m, is_large)
SMALL_BUSHES = [
    ("model://desert_scrub_bush",                  0.8,  False),
    ("model://stylized_bush_low_poly",             1.44, False),
    ("model://low_poly_bush_buxus",                1.16, False),
    ("model://bush",                               1.3,  False),
    ("model://low_poly_shrub",                     1.46, False),
    ("model://low_poly_shrub_-_small_texture",     1.46, False),
]

MEDIUM_TREES = [
    ("model://lowpoly_tree",                       1.84, False),
    ("model://low_poly_tree",                      1.46, False),
    ("model://low_poly_stylized_mossy_rocks",      0.5,  False),  # rocks
]

LARGE_TREES = [
    ("model://Oak_tree",                           8.0,  True),
    ("model://Pine_Tree",                          8.0,  True),
    ("model://hill_top_tree",                      5.5,  True),
    ("model://palm_tree_realistic",                7.16, True),
]

# ─── DEM Sampling ─────────────────────────────────────────────────────
def load_dem():
    """Load DEM and return the data array."""
    with rasterio.open(DEM_PATH) as ds:
        data = ds.read(1).astype(float)
    return data

def sample_z(dem_data, x, y):
    """Sample terrain elevation at Gazebo (x,y) coordinate."""
    nrows, ncols = dem_data.shape
    # Gazebo coords: x∈[-HM_SIZE_X/2, HM_SIZE_X/2], y∈[-HM_SIZE_Y/2, HM_SIZE_Y/2]
    # Map to pixel coords
    px = (x + HM_SIZE_X / 2) / HM_SIZE_X * (ncols - 1)
    py = (HM_SIZE_Y / 2 - y) / HM_SIZE_Y * (nrows - 1)  # y is flipped
    
    px = np.clip(px, 0, ncols - 1)
    py = np.clip(py, 0, nrows - 1)
    
    # Bilinear interpolation
    px0, py0 = int(np.floor(px)), int(np.floor(py))
    px1, py1 = min(px0 + 1, ncols - 1), min(py0 + 1, nrows - 1)
    fx, fy = px - px0, py - py0
    
    z00 = dem_data[py0, px0]
    z01 = dem_data[py0, px1]
    z10 = dem_data[py1, px0]
    z11 = dem_data[py1, px1]
    
    z = z00 * (1-fx)*(1-fy) + z01 * fx*(1-fy) + z10 * (1-fx)*fy + z11 * fx*fy
    return float(z)

# ─── Point-in-polygon test ────────────────────────────────────────────
def point_in_polygon(x, y, poly):
    """Ray casting algorithm for point-in-polygon."""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside

# ─── Cluster-based vegetation placement ───────────────────────────────
def generate_cluster_centers(num_clusters, min_dist=60.0):
    """Generate cluster centers within the shadow polygon using Poisson-like sampling."""
    # Bounding box of shadow region
    xmin, xmax = SHADOW_POLY[:, 0].min(), SHADOW_POLY[:, 0].max()
    ymin, ymax = SHADOW_POLY[:, 1].min(), SHADOW_POLY[:, 1].max()
    
    centers = []
    attempts = 0
    max_attempts = num_clusters * 50
    
    while len(centers) < num_clusters and attempts < max_attempts:
        x = np.random.uniform(xmin, xmax)
        y = np.random.uniform(ymin, ymax)
        attempts += 1
        
        if not point_in_polygon(x, y, SHADOW_POLY):
            continue
        
        # Check minimum distance to existing centers
        too_close = False
        for cx, cy in centers:
            if np.sqrt((x - cx)**2 + (y - cy)**2) < min_dist:
                too_close = True
                break
        
        if too_close:
            continue
        
        # Skip road corridor
        if abs(x - ROAD_CENTER_X) < ROAD_CLEARANCE:
            continue
        
        # Skip spawn area
        if np.sqrt((x - SPAWN_X)**2 + (y - SPAWN_Y)**2) < 30.0:
            continue
        
        centers.append((x, y))
    
    return centers

def scatter_around_center(cx, cy, num_plants, radius=25.0):
    """Scatter plants around a cluster center with Gaussian distribution."""
    points = []
    for _ in range(num_plants):
        dx = np.random.normal(0, radius / 2.5)
        dy = np.random.normal(0, radius / 2.5)
        x = cx + dx
        y = cy + dy
        
        # Ensure still in shadow polygon
        if point_in_polygon(x, y, SHADOW_POLY):
            # Skip road and spawn
            if abs(x - ROAD_CENTER_X) >= ROAD_CLEARANCE:
                if np.sqrt((x - SPAWN_X)**2 + (y - SPAWN_Y)**2) >= 20.0:
                    points.append((x, y))
    return points

def pick_model(rng_val):
    """Pick a model based on weighted probability."""
    # 55% small bushes, 25% medium, 20% large trees
    if rng_val < 0.55:
        return SMALL_BUSHES[np.random.randint(len(SMALL_BUSHES))]
    elif rng_val < 0.80:
        return MEDIUM_TREES[np.random.randint(len(MEDIUM_TREES))]
    else:
        return LARGE_TREES[np.random.randint(len(LARGE_TREES))]

def generate_sdf_includes(plants, dem_data):
    """Generate SDF <include> blocks for all plants."""
    lines = []
    lines.append("    <!-- ═══ VEGETATION — satellite-density placement ═══ -->")
    lines.append(f"    <!-- {len(plants)} vegetation instances across shadow region -->")
    lines.append(f"    <!-- Density based on 30m-resolution satellite analysis: ~15-20% cover -->")
    lines.append("")
    
    for i, (x, y, model_uri, height, is_large) in enumerate(plants):
        z = sample_z(dem_data, x, y)
        yaw = np.random.uniform(0, 2 * np.pi)
        
        lines.append("    <include>")
        lines.append(f"      <uri>{model_uri}</uri>")
        lines.append(f"      <name>veg_{i:04d}</name>")
        lines.append(f"      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose>")
        lines.append("      <static>true</static>")
        lines.append("    </include>")
    
    return "\n".join(lines)

def main():
    print("Loading DEM...")
    dem_data = load_dem()
    print(f"  DEM shape: {dem_data.shape}, range: {dem_data.min():.1f} – {dem_data.max():.1f}")
    
    # ─── Satellite image analysis ─────────────────────────────────────
    # The 30m image shows:
    # - Vegetation covers ~15-20% of ground area
    # - Organized in clusters/patches of 30-100m diameter
    # - Spacing between clusters: 50-200m
    # - Our shadow region is roughly 1735 × 4000m ≈ 7 km²
    # - At 15% cover with avg cluster area 50×50m = 2500m², 
    #   that's ~420 clusters, with ~5-10 plants each → ~2000-4000 plants
    # - But for sim performance, we'll aim for ~500-600 plants max
    #   using fewer but larger clusters
    
    # Generate cluster centers
    print("Generating vegetation clusters...")
    NUM_CLUSTERS = 80  # Dense enough but manageable
    MIN_CLUSTER_DIST = 80.0  # meters between cluster centers
    
    centers = generate_cluster_centers(NUM_CLUSTERS, MIN_CLUSTER_DIST)
    print(f"  Generated {len(centers)} cluster centers")
    
    # Scatter plants around each cluster
    all_plants = []
    for cx, cy in centers:
        # Cluster size varies: some dense (8-15), some sparse (3-6)
        density = np.random.choice(['sparse', 'medium', 'dense'], p=[0.3, 0.45, 0.25])
        if density == 'sparse':
            n_plants = np.random.randint(2, 5)
            radius = 15.0
        elif density == 'medium':
            n_plants = np.random.randint(5, 9)
            radius = 25.0
        else:
            n_plants = np.random.randint(8, 14)
            radius = 35.0
        
        points = scatter_around_center(cx, cy, n_plants, radius)
        
        for x, y in points:
            model_uri, height, is_large = pick_model(np.random.random())
            all_plants.append((x, y, model_uri, height, is_large))
    
    # Also scatter some isolated plants (not in clusters) — 15% of total
    print("Adding isolated scattered vegetation...")
    xmin, xmax = SHADOW_POLY[:, 0].min(), SHADOW_POLY[:, 0].max()
    ymin, ymax = SHADOW_POLY[:, 1].min(), SHADOW_POLY[:, 1].max()
    
    n_isolated = int(len(all_plants) * 0.15)
    isolated_added = 0
    for _ in range(n_isolated * 10):
        if isolated_added >= n_isolated:
            break
        x = np.random.uniform(xmin, xmax)
        y = np.random.uniform(ymin, ymax)
        if not point_in_polygon(x, y, SHADOW_POLY):
            continue
        if abs(x - ROAD_CENTER_X) < ROAD_CLEARANCE:
            continue
        if np.sqrt((x - SPAWN_X)**2 + (y - SPAWN_Y)**2) < 20.0:
            continue
        
        # Isolated plants are mostly small bushes
        model_uri, height, is_large = pick_model(np.random.random() * 0.7)  # bias toward bushes
        all_plants.append((x, y, model_uri, height, is_large))
        isolated_added += 1
    
    print(f"  Total vegetation: {len(all_plants)} instances")
    
    # Count by type
    type_counts = {}
    for _, _, uri, _, _ in all_plants:
        name = uri.split("://")[1]
        type_counts[name] = type_counts.get(name, 0) + 1
    
    print("\n  Model distribution:")
    for name, count in sorted(type_counts.items(), key=lambda x: -x[1]):
        print(f"    {name:40s}  {count:4d}")
    
    # Generate SDF
    print("\nGenerating SDF includes...")
    sdf_text = generate_sdf_includes(all_plants, dem_data)
    
    # Write to output file
    out_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'bot_description', 'worlds', 'vegetation_includes.sdf'))
    with open(out_path, 'w') as f:
        f.write(sdf_text)
    
    print(f"\nWritten to: {out_path}")
    print(f"Total vegetation: {len(all_plants)}")
    print("\nPaste the contents into thar_desert.sdf between the terrain model and the shadow boundary model.")

if __name__ == "__main__":
    main()
