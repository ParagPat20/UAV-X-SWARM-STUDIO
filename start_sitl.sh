#!/bin/bash
# ==============================================================================
# UAV-X Swarm Studio — Multi-UAV SITL + AirSim Simulation Launcher
# ==============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Ensure MAVProxy and ArduPilot tools are in PATH
export PATH="$HOME/venv-ardupilot/bin:$HOME/ardupilot/Tools/autotest:$PATH"
if [ -d "$HOME/venv-ardupilot" ]; then
    source "$HOME/venv-ardupilot/bin/activate"
fi

# Simulation & Binary paths (can be overridden via environment variables)
AIRSIM_BIN="${AIRSIM_BIN:-$HOME/Downloads/Blocks/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh}"
SITL_CMD="${SITL_CMD:-$HOME/ardupilot/Tools/autotest/sim_vehicle.py}"

cleanup() {
    trap - INT TERM EXIT
    stty sane
    echo ""
    echo "Shutting down SITL, Swarm Studio, Browser and Blocks..."
    pkill -9 -f "arducopter" 2>/dev/null || true
    pkill -9 -f "mavproxy" 2>/dev/null || true
    pkill -9 -f "sim_vehicle" 2>/dev/null || true
    pkill -9 -f "server.py" 2>/dev/null || true
    [ -n "$BROWSER_PID" ] && kill -9 "$BROWSER_PID" 2>/dev/null || true
    pkill -9 -f "localhost:8080" 2>/dev/null || true
    pkill -9 -f "UAV-X Swarm Studio" 2>/dev/null || true
    [ -n "$AIRSIM_PID" ] && kill -9 "$AIRSIM_PID" 2>/dev/null || true
    pkill -9 -f "Blocks|AirSimNH|Africa_001" 2>/dev/null || true
    fuser -k 8080/tcp 8765/tcp 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM EXIT

if [ -n "$1" ]; then
    num_drones="$1"
else
    echo -n "Enter the number of drones to spawn [5]: "
    read num_drones
    num_drones="${num_drones:-5}"
fi

if ! [[ "$num_drones" =~ ^[0-9]+$ ]] || [ "$num_drones" -lt 1 ]; then
    echo "Error: Please enter a valid number of drones (>= 1)."
    exit 1
fi

WIPE_ARG="-w"
vehicle="ArduCopter"
frame="airsim-copter"
FRAME_ARG="-f $frame"

# 1. Dynamically configure ~/Documents/AirSim/settings.json
echo "Configuring AirSim settings for $num_drones drone(s)..."
python3 - <<EOF
import json, os

num = int("$num_drones")
settings_path = os.path.expanduser("~/Documents/AirSim/settings.json")
os.makedirs(os.path.dirname(settings_path), exist_ok=True)

vehicles = {}
for i in range(num):
    vehicles[f"UAV{i+1}"] = {
        "VehicleType": "ArduCopter",
        "UseSerial": False,
        "LocalHostIp": "127.0.0.1",
        "UdpIp": "127.0.0.1",
        "UdpPort": 9003 + (i * 10),
        "ControlPort": 9002 + (i * 10),
        "X": 0,
        "Y": i * 5.0,
        "Z": 0.0,
        "Sensors": {
            "lidar": {
                "SensorType": 6,
                "Enabled": True,
                "NumberOfChannels": 1,
                "RotationsPerSecond": 10,
                "PointsPerSecond": 10000,
                "Range": 60,
                "VerticalFOVUpper": 0,
                "VerticalFOVLower": 0,
                "X": 0, "Y": 0, "Z": -0.1,
                "Roll": 0, "Pitch": 0, "Yaw": 0,
                "DrawDebugPoints": False,
                "DataFrame": "SensorLocalFrame"
            },
            "rng": {
                "SensorType": 5,
                "Enabled": True,
                "MinDistance": 0.2,
                "MaxDistance": 60,
                "X": 0, "Y": 0, "Z": -1,
                "Yaw": 0, "Pitch": 0, "Roll": 0,
                "DrawDebugPoints": False
            }
        }
    }

subwindows = []
if num == 1:
    subwindows.append({
        "WindowID": 0,
        "CameraName": "0",
        "ImageType": 0,
        "VehicleName": "UAV1",
        "Visible": True
    })

settings = {
    "SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockSpeed": 1.0,
    "ViewMode": "SpringArmChase",
    "CameraDirector": { "FollowDistance": -6 if num <= 2 else -10, "X": -3 if num <= 2 else -5, "Y": 0, "Z": -2 if num <= 2 else -4 },
    "SubWindows": subwindows,
    "Vehicles": vehicles
}

with open(settings_path, "w") as f:
    json.dump(settings, f, indent=2)

print(f"Saved settings.json with {num} vehicle(s) and {len(subwindows)} subwindow camera(s).")
