#!/usr/bin/env python3
"""
trim_vegetation.py
==================
Generates a vegetation-trimmed copy of thar_desert.sdf and prints
ONLY the resulting file path to stdout (used by $(command ...) in
gazebo.launch.xml).

Usage:
    python3 trim_vegetation.py <num_vegetation> <world_sdf_path>
"""
import hashlib
import os
import random
import re
import sys
import tempfile

MAX_VEGETATION = 1790


def main():
    if len(sys.argv) != 3:
        sys.exit(1)

    try:
        n_veg = int(sys.argv[1])
    except ValueError:
        n_veg = MAX_VEGETATION

    n_veg = max(0, min(n_veg, MAX_VEGETATION))
    world_path = sys.argv[2]

    # Full vegetation requested — return the original unchanged (no disk write)
    if n_veg >= MAX_VEGETATION:
        print(world_path, end="")
        return

    # Read original SDF
    with open(world_path, "r") as f:
        original = f.read()

    # ── Locate all <include>…</include> blocks ────────────────────────────
    # Find the position of the first <include> — everything before it is
    # the world header (physics, lights, terrain model).
    first_inc = original.index("<include>")
    header = original[:first_inc]

    # Everything after the last </include> is the world closing tags.
    last_end = original.rfind("</include>") + len("</include>")
    closing_tags = original[last_end:]

    # Extract and split the vegetation section into individual blocks.
    veg_section = original[first_inc:last_end]
    vegetation_blocks = re.findall(
        r"<include>.*?</include>", veg_section, re.DOTALL
    )

    # Keep only the requested number of blocks.
    # Use a fixed seed derived from n_veg so the sample is reproducible
    # across launches but uniformly distributed across the whole terrain
    # (the SDF is ordered sequentially, so taking first-N would cluster
    # all instances at one edge of the shadow region).
    rng = random.Random(n_veg)
    sampled = rng.sample(vegetation_blocks, n_veg)

    # Reconstruct with the original 4-space indent that the header ends with.
    indent = "    "
    kept = (indent + b + "\n" for b in sampled)
    trimmed_sdf = header + "".join(kept) + closing_tags

    # ── Deterministic temp path (source mtime in hash → auto-invalidates) ──
    mtime = str(os.path.getmtime(world_path))
    uid = hashlib.md5(f"{world_path}_{n_veg}_{mtime}".encode()).hexdigest()[:8]
    out_path = os.path.join(
        tempfile.gettempdir(), f"thar_desert_veg{n_veg}_{uid}.sdf"
    )

    with open(out_path, "w") as f:
        f.write(trimmed_sdf)

    print(out_path, end="")


if __name__ == "__main__":
    main()
