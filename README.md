# 🛸 UAV-X Swarm Studio — Autonomous BVLOS Swarm Ground Control Station

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![ArduPilot SITL](https://img.shields.io/badge/ArduPilot-SITL%20Copter-orange.svg)](https://ardupilot.org/)
[![Microsoft AirSim](https://img.shields.io/badge/AirSim-Unreal%20Engine-brightgreen.svg)](https://github.com/microsoft/AirSim)
[![Three.js](https://img.shields.io/badge/Three.js-WebGL%203D-black.svg)](https://threejs.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

> **UAV-X Swarm Studio** is an industrial-grade, dark-glassmorphic Ground Control Station (GCS) and autonomous multi-agent flight controller designed for resilient **BVLOS Swarm Operations**, **3D LiDAR SLAM Solidification**, and **Decentralized Search & Rescue (SAR)**.

---

## 🌟 Key Capabilities

1. **🌐 Real-Time 3D Digital Twin (Three.js WebGL)**
   - 60 FPS real-time 3D tracking with trailing ribbons, drop lines, orientation quaternions, and pitch/roll flight attitudes.
   - Dynamic 3D Geofence cylindrical boundary with automatic radius ($40\text{m}$–$150\text{m}$) and ceiling expansion.

2. **🧊 3D LiDAR SLAM & Obstacle Solidification**
   - 360° Polar Horizon rangefinding integrated with Microsoft AirSim LiDAR sensors.
   - Ground reflection and dynamic drone-body exclusion filters.
   - High-performance voxel instantiation ($0.35\text{m}$ voxels) with export capabilities in **.XYZ (RGB)**, **.PCD (ROS)**, and **.JSON**.

3. **🧭 Point-Free Autonomous Frontier Exploration**
   - 36-sector polar horizon collision avoidance with strict obstacle standoff ($4.0\text{m}$) and inter-drone repulsion ($16.0\text{m}$).
   - Elastic boundary deflection preventing geofence breaches while maintaining forward momentum.

4. **🚨 Fault Injection & Resilient Dynamic SAR Handover**
   - Simulated motor cut / link loss fault injection on any swarm drone.
   - Dynamic real-time rescuer evaluation: automatically calculates nearest active healthy drone and dispatches it to the accident coordinates, dynamically handing over if a closer UAV becomes available.

5. **💾 Persistent Flight Parameters & Live Simulation Sync**
   - Automatically saves and loads flight tuning parameters (`swarm_settings.json` + `localStorage`).
   - Dynamically broadcasts live MAVLink `PARAM_SET` commands (`FENCE_RADIUS`, `FENCE_ALT_MAX`, `AVOID_MARGIN`) to ArduPilot SITL instances in real-time.

---

## 🏗️ System Architecture

```
┌────────────────────────────────────────────────────────────────────────┐
│                   UAV-X Swarm Studio Web GCS                           │
│  (Three.js 3D Viewport | Swarm Command Deck | Object Inspector | Wave Chart) │
└──────────────────┬─────────────────────────────────▲───────────────────┘
                   │ HTTP / WebSocket (Port 8765)    │ Telemetry (25 Hz)
┌──────────────────▼─────────────────────────────────┴───────────────────┐
│                      Swarm Studio Backend Engine                       │
│                        (server.py & Fast MAVLink)                      │
│   - Guided Reactive Survey Controller      - Dynamic SAR Dispatcher   │
│   - Persistent Settings Manager             - Point Cloud Solidifier   │
└───────────┬───────────────────────────────────────────────▲────────────┘
            │ MAVLink UDP (Ports 5762-5802)                 │ AirSim RPC
┌───────────▼───────────────┐                  ┌────────────┴────────────┐
│   ArduPilot SITL Swarm    │                  │  Microsoft AirSim Sim   │
│  (UAV 1 ... UAV 5 Copter) │◄─UDP Physics────►│ (Unreal Engine Blocks)  │
└───────────────────────────┘                  └─────────────────────────┘
```

---

## 📁 Repository Structure

```
├── .gitignore
├── LICENSE
├── README.md
├── requirements.txt
├── start_sitl.sh              # Master launcher for AirSim + SITL + Web GCS
├── restart_simulation.sh      # Clean process killer & relauncher
├── check_all_drones_params.py # MAVLink parameter verification script
├── swarm_params.parm          # Clean default ArduPilot parameter set
├── swarm_planner.py           # Multi-drone formation & trajectory planner
├── docs/                      # Stage 1 Technical Proposal Deliverables
│   ├── UAV_X_Stage1_Technical_Proposal.docx
│   ├── UAV_X_Stage1_Technical_Proposal.pdf
│   ├── UAV_X_Stage1_Technical_Proposal.md
│   └── UAV_X_CHALLENGE_CHECKLIST.md
└── swarm_studio/              # GCS Backend & Frontend
    ├── server.py              # WebSocket telemetry server & survey controller
    ├── swarm_settings.json    # Local configuration file
    └── static/
        ├── index.html         # Modern GCS HTML5 UI
        ├── style.css          # Industrial dark glassmorphism stylesheet
        └── app.js             # Three.js 3D engine & WebSocket client
```

---

## 🚀 Quickstart Guide

### 1. Prerequisites
- **Ubuntu 20.04 / 22.04 / 24.04 LTS**
- **Python 3.10+**
- **ArduPilot SITL** (`sim_vehicle.py`)
- **Microsoft AirSim** (e.g. `LinuxBlocks1.8.1`)
- **Google Chrome** / **Chromium** (Optional, for standalone GCS app window)

### 2. Installation
```bash
# Clone the repository
git clone https://github.com/your-username/uav-x-swarm-studio.git
cd uav-x-swarm-studio

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Running the Simulation & GCS
```bash
# Launch a 5-drone simulation with AirSim and Swarm Studio
./start_sitl.sh 5
```

Once running:
- Open your browser at **`http://localhost:8080`**
- WebSocket telemetry connects automatically at **`ws://localhost:8765`**

### 4. Standalone Web GCS Server Only
If you are already running SITL or real UAV hardware over MAVLink:
```bash
python3 swarm_studio/server.py
```

---

## 🎮 GCS Controls & Flight Modes

| Deck Command | Description |
| :--- | :--- |
| **ARM ALL** | Arms motors across all online swarm drones simultaneously |
| **DISARM** | Immediately disarms motors for safety |
| **TAKEOFF** | Ascends swarm to target altitude ($5.0\text{m}$ default) |
| **AUTO SURVEY** | Launches autonomous 360° frontier exploration & reactive SLAM |
| **SWARM RTL** | Commands all drones to Return to Launch coordinates |
| **LAND ALL** | Commands immediate vertical descent and landing |
| **💀 Kill Drone** | Injects motor cutoff fault on target UAV to test SAR rescue handover |
| **💾 Save & Sync** | Persists flight tuning parameters locally and pushes live MAVLink limits |

---

## 📄 Stage 1 Technical Proposal Deliverables
Detailed design documentation, architecture breakdown, and PUSHPAK challenge compliance matrices are located in the [`docs/`](docs/) directory:
- [Technical Proposal (PDF)](docs/UAV_X_Stage1_Technical_Proposal.pdf)
- [Technical Proposal (DOCX)](docs/UAV_X_Stage1_Technical_Proposal.docx)
- [Technical Proposal (Markdown)](docs/UAV_X_Stage1_Technical_Proposal.md)
- [Challenge Compliance Checklist](docs/UAV_X_CHALLENGE_CHECKLIST.md)

---

## 📜 License
This project is licensed under the [MIT License](LICENSE).
