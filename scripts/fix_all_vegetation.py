#!/usr/bin/env python3
"""
COMPREHENSIVE vegetation fix:
1. Convert all 10 OBJ files from Y-up to Z-up (eliminates rotation ambiguity)
2. Shift models so Z_min=0 (base sits on ground plane)
3. Rewrite all 10 model.sdf files (no rotation needed, clean geometry)
4. Generate dense vegetation placement (~2000 instances, 20m grid)
"""

import os, sys, shutil, re
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '.venv', 'lib', 'python3.12', 'site-packages'))
import rasterio

np.random.seed(42)

BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
MODELS_DIR = os.path.join(BASE, 'src/bot_description/models')
DEM_PATH = os.path.join(BASE, 'src/bot_description/worlds/dem/output_be_sq.tif')

HM_SIZE_X = 1735.3
HM_SIZE_Y = 4143.6

# Shadow polygon vertices (Gazebo coordinates)
SHADOW_POLY = np.array([
    [-867.65, -1981.47],
    [-564.0,  -2071.8],
    [491.5,    2045.2],
    [867.65,   1899.1],
])

SPAWN_X, SPAWN_Y = -50.5, -23.2
SPAWN_CLEAR = 20.0

# 10 OBJ models with their TARGET real-world heights (meters)
OBJ_MODELS = {
    'bush':                           1.2,
    'hill_top_tree':                  5.0,
    'low_poly_bush_buxus':            0.8,
    'low_poly_shrub':                 1.5,
    'low_poly_shrub_-_small_texture': 1.5,
    'low_poly_stylized_mossy_rocks':  0.6,
    'low_poly_tree':                  3.5,
    'lowpoly_tree':                   3.0,
    'palm_tree_realistic':            6.0,
    'stylized_bush_low_poly':         1.0,
}

# Categories for vegetation distribution
MODEL_CATEGORIES = {
    'desert_scrub_bush':              ('bush',        0.8),
    'stylized_bush_low_poly':         ('bush',        1.0),
    'bush':                           ('bush',        1.2),
    'low_poly_bush_buxus':            ('shrub',       0.8),
    'low_poly_shrub':                 ('shrub',       1.5),
    'low_poly_shrub_-_small_texture': ('shrub',       1.5),
    'low_poly_stylized_mossy_rocks':  ('rock',        0.6),
    'lowpoly_tree':                   ('small_tree',  3.0),
    'low_poly_tree':                  ('small_tree',  3.5),
    'hill_top_tree':                  ('medium_tree', 5.0),
    'palm_tree_realistic':            ('large_tree',  6.0),
    'Oak_tree':                       ('large_tree',  8.0),
    'Pine_Tree':                      ('large_tree',  8.0),
}

CAT_WEIGHTS = {
    'bush': 0.38, 'shrub': 0.22, 'rock': 0.10,
    'small_tree': 0.15, 'medium_tree': 0.10, 'large_tree': 0.05
}


# ═══════════════════════════════════════════════════════════════════════
# STEP 1: Convert OBJ files from Y-up to Z-up
# ═══════════════════════════════════════════════════════════════════════

def convert_obj_to_zup(model_name):
    """
    Convert OBJ from Y-up to Z-up:
      vertex:  (x, y, z) → (x, -z, y)       [+90° rotation around X]
      normal:  (nx, ny, nz) → (nx, -nz, ny)
    Then shift so Z_min = 0 (model base at ground plane).
    Returns (z_span_after_shift, x_span, y_span) in OBJ units.
    """
    obj_path = os.path.join(MODELS_DIR, model_name, 'Untitled.obj')
    bak_path = obj_path + '.yup_backup'
    
    # Backup original if not already backed up
    if not os.path.exists(bak_path):
        shutil.copy2(obj_path, bak_path)
    
    lines = open(bak_path).readlines()  # Always read from original backup
    
    # First pass: convert coordinates, collect Z values for shift
    new_lines = []
    z_values = []
    
    for line in lines:
        if line.startswith('v ') and not line.startswith('vt') and not line.startswith('vn'):
            parts = line.split()
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            # Y-up to Z-up: (x, y, z) → (x, -z, y)
            nx, ny, nz = x, -z, y
            z_values.append(nz)
            new_lines.append(f'v {nx} {ny} {nz}\n')
        elif line.startswith('vn '):
            parts = line.split()
            nx, ny, nz = float(parts[1]), float(parts[2]), float(parts[3])
            # Same rotation for normals
            new_lines.append(f'vn {nx} {-nz} {ny}\n')
        else:
            new_lines.append(line)
    
    # Second pass: shift Z so Z_min = 0
    z_min = min(z_values)
    z_max = max(z_values)
    
    final_lines = []
    for line in new_lines:
        if line.startswith('v ') and not line.startswith('vt') and not line.startswith('vn'):
            parts = line.split()
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            z_shifted = z - z_min
            final_lines.append(f'v {x} {y} {z_shifted}\n')
        else:
            final_lines.append(line)
    
    with open(obj_path, 'w') as f:
        f.writelines(final_lines)
    
    # Compute bounding box of converted+shifted model
    xs, ys, zs = [], [], []
    for line in final_lines:
        if line.startswith('v ') and not line.startswith('vt') and not line.startswith('vn'):
            parts = line.split()
            xs.append(float(parts[1]))
            ys.append(float(parts[2]))
            zs.append(float(parts[3]))
    
    x_span = max(xs) - min(xs)
    y_span = max(ys) - min(ys)
    z_span = max(zs) - min(zs)  # This equals z_max - z_min (height)
    
    return z_span, x_span, y_span


# ═══════════════════════════════════════════════════════════════════════
# STEP 2: Rewrite model.sdf files (no rotation, clean geometry)
# ═══════════════════════════════════════════════════════════════════════

def write_model_sdf(model_name, target_height, z_span_raw, x_span_raw, y_span_raw):
    """Write clean model.sdf with no rotation. Model is already Z-up."""
    scale = target_height / z_span_raw
    
    # Collision dimensions in real meters
    coll_height = target_height
    coll_radius = max(x_span_raw, y_span_raw) * scale / 2.0
    coll_z = coll_height / 2.0
    
    sdf_content = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://{model_name}/Untitled.obj</uri>
            <scale>{scale:.6f} {scale:.6f} {scale:.6f}</scale>
          </mesh>
        </geometry>
      </visual>
      <collision name="collision">
        <pose>0 0 {coll_z:.3f} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>{coll_radius:.3f}</radius>
            <length>{coll_height:.3f}</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
'''
    sdf_path = os.path.join(MODELS_DIR, model_name, 'model.sdf')
    with open(sdf_path, 'w') as f:
        f.write(sdf_content)
    
    return scale


# ═══════════════════════════════════════════════════════════════════════
# STEP 3: Generate dense vegetation
# ═══════════════════════════════════════════════════════════════════════

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
    r = np.random.random()
    cum = 0
    all_models = list(MODEL_CATEGORIES.items())
    for cat, w in CAT_WEIGHTS.items():
        cum += w
        if r <= cum:
            candidates = [(name, info) for name, info in all_models if info[0] == cat]
            idx = np.random.randint(len(candidates))
            return candidates[idx][0]
    return all_models[0][0]

def generate_vegetation(dem):
    xmin, xmax = SHADOW_POLY[:,0].min(), SHADOW_POLY[:,0].max()
    ymin, ymax = SHADOW_POLY[:,1].min(), SHADOW_POLY[:,1].max()
    
    # Dense grid: 20m spacing, ±8m jitter, 92% fill
    GRID = 20.0
    JITTER = 8.0
    FILL = 0.92
    Z_LIFT = 0.0  # models have Z_min=0 (base at origin), so no lift needed
    
    grid_xs = np.arange(xmin + GRID/2, xmax, GRID)
    grid_ys = np.arange(ymin + GRID/2, ymax, GRID)
    
    plants = []
    for gx in grid_xs:
        for gy in grid_ys:
            x = gx + np.random.uniform(-JITTER, JITTER)
            y = gy + np.random.uniform(-JITTER, JITTER)
            
            if not point_in_poly(x, y, SHADOW_POLY):
                continue
            if np.sqrt((x - SPAWN_X)**2 + (y - SPAWN_Y)**2) < SPAWN_CLEAR:
                continue
            if np.random.random() > FILL:
                continue
            
            model_name = pick_model()
            yaw = np.random.uniform(0, 2*np.pi)
            z = sample_z(dem, x, y) + Z_LIFT
            plants.append((x, y, z, model_name, yaw))
    
    return plants


# ═══════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════

def main():
    print("=" * 70)
    print("STEP 1: Converting OBJ files from Y-up to Z-up")
    print("=" * 70)
    
    model_data = {}
    for model_name, target_h in OBJ_MODELS.items():
        z_span, x_span, y_span = convert_obj_to_zup(model_name)
        model_data[model_name] = (z_span, x_span, y_span)
        scale = target_h / z_span
        print(f"  {model_name:40s}  Z-span={z_span:>10.2f}  scale={scale:.6f}  → {target_h}m")
    
    print(f"\n{'=' * 70}")
    print("STEP 2: Rewriting model.sdf files (no rotation)")
    print("=" * 70)
    
    for model_name, target_h in OBJ_MODELS.items():
        z_span, x_span, y_span = model_data[model_name]
        scale = write_model_sdf(model_name, target_h, z_span, x_span, y_span)
        print(f"  {model_name:40s}  scale={scale:.6f}")
    
    print(f"\n{'=' * 70}")
    print("STEP 3: Generating dense vegetation")
    print("=" * 70)
    
    dem = load_dem()
    print(f"  DEM: {dem.shape}, z={dem.min():.1f}–{dem.max():.1f}")
    
    plants = generate_vegetation(dem)
    print(f"  Generated {len(plants)} vegetation instances")
    
    # Count per model
    counts = {}
    for _, _, _, name, _ in plants:
        counts[name] = counts.get(name, 0) + 1
    print("\n  Distribution:")
    for name, c in sorted(counts.items(), key=lambda x: -x[1]):
        print(f"    {name:42s} {c:4d}")
    
    # Write SDF includes
    lines = []
    lines.append("    <!-- ═══ VEGETATION — Z-up OBJ, 20m grid, dense (v3) ═══ -->")
    lines.append(f"    <!-- {len(plants)} instances, 20m grid + 8m jitter, 92% fill, +0.2m Z-lift -->")
    lines.append("")
    
    for i, (x, y, z, name, yaw) in enumerate(plants):
        lines.append("    <include>")
        lines.append(f"      <uri>model://{name}</uri>")
        lines.append(f"      <name>veg_{i:04d}</name>")
        lines.append(f"      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 {yaw:.4f}</pose>")
        lines.append("      <static>true</static>")
        lines.append("    </include>")
    
    out_path = os.path.join(BASE, 'src', 'bot_description', 'worlds', 'vegetation_includes.sdf')
    with open(out_path, 'w') as f:
        f.write("\n".join(lines))
    print(f"\n  Written to: {out_path}")
    
    print(f"\n{'=' * 70}")
    print("ALL DONE. Next: splice into world SDF, build, launch.")
    print("=" * 70)

if __name__ == "__main__":
    main()
