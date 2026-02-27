# Silent Sentry — Autonomous UGV Navigation in Contested Environments

---

## Abstract

Silent Sentry presents a multi-layer autonomous navigation stack for Unmanned Ground Vehicles (UGVs) operating in unstructured, GPS-degraded desert corridors. The system integrates:

- **EMCON Controller** — Emission-Control-aware command arbitration that suppresses unnecessary RF/sensor emissions during adversarial exposure windows
- **SBLP Planner** — Scenario-Based Local Planner that switches trajectory primitives based on terrain class (sand dune, rock field, open corridor)
- **VLM Costmap** — Vision-Language Model costmap layer that annotates traversability using zero-shot scene understanding (no fine tuning required)

Experiments are conducted in a photorealistic Gazebo Harmonic simulation of the Thar Desert corridor (1735 × 4144 m heightmap, 187 vegetation models).

---

## Architecture

```
/cmd_vel (Twist)
      │
      ▼
┌─────────────┐     terrain class     ┌──────────────┐
│ VLM Costmap │ ──────────────────▶  │ SBLP Planner │
│  (vlm_costmap)│                    │ (sblp_planner)│
└─────────────┘                      └──────┬───────┘
                                            │ waypoints
                                            ▼
                                   ┌─────────────────┐
                                   │ EMCON Controller │
                                   │ (emcon_controller)│
                                   └────────┬────────┘
                                            │ /forward_velocity_controller/commands
                                            ▼
                                        [autobot]
```

---

## Repository Layout

```
silent_sentry_ros2/
├── README.md
├── .gitignore
├── docker/
│   ├── Dockerfile            # ROS 2 Jazzy + Gazebo Harmonic base image
│   └── docker-compose.yml    # Multi-agent network emulation (3 UGVs)
├── paper_data/
│   ├── plotting_scripts/     # matplotlib scripts to reproduce all paper figures
│   └── final_graphs/         # PNGs used in the IEEE submission
├── src/
│   ├── emcon_controller/     # EMCON-aware command arbitration node
│   ├── sblp_planner/         # Scenario-based local planner
│   └── vlm_costmap/          # VLM traversability costmap layer
└── world/
    └── thar_corridor.sdf     # Gazebo world (references local .tif heightmap)
```

---

## Dependencies

| Package | Version |
|---------|---------|
| ROS 2 | Jazzy |
| Gazebo | Harmonic 8.10+ |
| Python | 3.12 |
| OpenCV | 4.x |
| Transformers (HuggingFace) | ≥ 4.40 |
| robot_localization | Jazzy |

---

## Build & Run

```bash
# 1. Clone
git clone https://github.com/YOUR_ORG/silent_sentry_ros2.git
cd silent_sentry_ros2

# 2. Build
colcon build --symlink-install
source install/setup.bash

# 3. Launch full stack
ros2 launch emcon_controller silent_sentry.launch.xml
```

### Docker (multi-agent)
```bash
cd docker
docker-compose up --build
```

---

## Reproducing Paper Figures

```bash
cd paper_data/plotting_scripts
python3 plot_traversability.py
python3 plot_emcon_timeline.py
python3 plot_sblp_scenario_switch.py
```

Output PNGs will appear in `paper_data/final_graphs/`.

---

