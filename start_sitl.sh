#!/bin/bash
# ==============================================================================
# UAV-X Swarm Studio — Multi-UAV SITL + Multi-Environment Simulation Launcher
# Supports: Blocks | AirSimNH (Neighborhood) | LandscapeMountains (Terrain)
# ==============================================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Ensure MAVProxy and ArduPilot tools are in PATH
export PATH="$HOME/venv-ardupilot/bin:$HOME/ardupilot/Tools/autotest:$PATH"
if [ -d "$HOME/venv-ardupilot" ]; then
    source "$HOME/venv-ardupilot/bin/activate"
fi

SITL_CMD="${SITL_CMD:-$HOME/ardupilot/Tools/autotest/sim_vehicle.py}"

cleanup() {
    trap - INT TERM EXIT
    stty sane
    echo ""
    echo "Shutting down SITL, Swarm Studio, Browser and AirSim environments..."
    pkill -9 -f "arducopter" 2>/dev/null || true
    pkill -9 -f "mavproxy" 2>/dev/null || true
    pkill -9 -f "sim_vehicle" 2>/dev/null || true
    pkill -9 -f "server.py" 2>/dev/null || true
    [ -n "$BROWSER_PID" ] && kill -9 "$BROWSER_PID" 2>/dev/null || true
    pkill -9 -f "localhost:8080" 2>/dev/null || true
    pkill -9 -f "UAV-X Swarm Studio" 2>/dev/null || true
    [ -n "$AIRSIM_PID" ] && kill -9 "$AIRSIM_PID" 2>/dev/null || true
    pkill -9 -f "Blocks|AirSimNH|LandscapeMountains|Africa_001" 2>/dev/null || true
    fuser -k 8080/tcp 8765/tcp 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM EXIT

# --- 1. Parse Arguments: Number of Drones & AirSim Environment ---
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

ENV_CHOICE="${2:-${AIRSIM_ENV:-Blocks}}"

# Auto-detect Environment and Executable Path
case "$ENV_CHOICE" in
    [Aa]ir[Ss]im[Nn][Hh]|nh|NH|neighborhood|2)
        ENV_NAME="AirSimNH (Urban Neighborhood)"
        POSSIBLE_PATHS=(
            "$AIRSIM_NH_BIN"
            "$HOME/Downloads/AirSimNH/LinuxNoEditor/AirSimNH.sh"
            "$HOME/Downloads/AirSimNH/AirSimNH/LinuxNoEditor/AirSimNH.sh"
            "$HOME/Downloads/AirSimNH/LinuxAirSimNH1.8.1/LinuxNoEditor/AirSimNH.sh"
            "$HOME/AirSimNH/LinuxNoEditor/AirSimNH.sh"
            "/opt/AirSimNH/LinuxNoEditor/AirSimNH.sh"
        )
        ;;
    [Ll]andscape*|[Mm]ountain*|mountains|3)
        ENV_NAME="LandscapeMountains (Mountain BVLOS)"
        POSSIBLE_PATHS=(
            "$AIRSIM_MOUNTAINS_BIN"
            "$HOME/Downloads/LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh"
            "$HOME/Downloads/LandscapeMountains/LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh"
            "$HOME/Downloads/LandscapeMountains/LinuxLandscapeMountains1.8.1/LinuxNoEditor/LandscapeMountains.sh"
            "$HOME/LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh"
            "/opt/LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh"
        )
        ;;
    *)
        ENV_NAME="Blocks (Obstacle City Grid)"
        POSSIBLE_PATHS=(
            "$AIRSIM_BIN"
            "$HOME/Downloads/Blocks/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh"
            "$HOME/Downloads/Blocks/LinuxNoEditor/Blocks.sh"
            "$HOME/Blocks/LinuxNoEditor/Blocks.sh"
            "/opt/Blocks/LinuxNoEditor/Blocks.sh"
        )
        ;;
esac

AIRSIM_EXECUTABLE=""
for p in "${POSSIBLE_PATHS[@]}"; do
    if [ -n "$p" ] && [ -f "$p" ]; then
        AIRSIM_EXECUTABLE="$p"
        break
    fi
done

echo "================================================================================"
echo "★ UAV-X SWARM SIMULATION CONFIGURATION"
echo "  Fleet Size    : $num_drones Multi-Rotor Drones (UAV 1 -> UAV $num_drones)"
echo "  Environment   : $ENV_NAME"
if [ -n "$AIRSIM_EXECUTABLE" ]; then
    echo "  Binary Path   : $AIRSIM_EXECUTABLE"
else
    echo "  Binary Path   : Auto-launch skipped (no pre-built binary found at standard path)"
fi
echo "================================================================================"

WIPE_ARG="-w"
vehicle="ArduCopter"
frame="airsim-copter"
FRAME_ARG="-f $frame"

# --- 2. Dynamically configure ~/Documents/AirSim/settings.json ---
echo "Generating multi-UAV sensor & physics settings..."
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

print(f"AirSim multi-drone configuration generated for {num} vehicle(s).")
EOF

# --- 3. Clean previous instances and launch AirSim ---
echo "Cleaning up any old simulation and GCS processes..."
pkill -9 -f "Blocks|AirSimNH|LandscapeMountains|Africa_001|arducopter|mavproxy" 2>/dev/null || true
pkill -9 -f "server.py" 2>/dev/null || true
rm -f "$SCRIPT_DIR"/eeprom*.bin 2>/dev/null || true
fuser -k 8080/tcp 8765/tcp 2>/dev/null || true
sleep 1

if [ -n "$AIRSIM_EXECUTABLE" ]; then
    echo "Launching $ENV_NAME environment..."
    setsid nice -n 5 "$AIRSIM_EXECUTABLE" -windowed -ResX=640 -ResY=480 -FPS=60 \
        -ExecCmds="r.Streaming.PoolSize 3000,sg.ShadowQuality 0,sg.PostProcessQuality 0,sg.TextureQuality 1,sg.EffectsQuality 0,sg.FoliageQuality 0,r.Shadow.CSM.MaxCascades 0,t.maxFPS 60,r.VSync 0" \
        > /tmp/airsim.log 2>&1 &
    AIRSIM_PID=$!
    echo "Waiting 6 seconds for AirSim to initialize map and physics..."
    sleep 6
else
    echo "AirSim executable not found. Running SITL in standalone mode."
fi

# --- 4. Launch UAV-X Swarm Studio Server & App Window ---
echo "Starting UAV-X Swarm Studio Backend Server..."
python3 "$SCRIPT_DIR/swarm_studio/server.py" > /tmp/swarm_studio.log 2>&1 &
sleep 1

echo "Launching UAV-X Swarm Studio Desktop Window..."
if command -v google-chrome &> /dev/null; then
    google-chrome --app=http://localhost:8080 --window-size=1366,820 --window-position=100,60 --class="UAV-X Swarm Studio" > /dev/null 2>&1 &
    BROWSER_PID=$!
elif command -v chromium &> /dev/null; then
    chromium --app=http://localhost:8080 --window-size=1366,820 > /dev/null 2>&1 &
    BROWSER_PID=$!
else
    xdg-open http://localhost:8080 > /dev/null 2>&1 &
    BROWSER_PID=$!
fi

# --- 5. Automatic Parameter Verification Watcher ---
PARAM_FILE=""
if [ -f "$SCRIPT_DIR/swarm_params.parm" ]; then
    PARAM_FILE="--add-param-file=$SCRIPT_DIR/swarm_params.parm"
elif [ -f "$HOME/swarm_params.parm" ]; then
    PARAM_FILE="--add-param-file=$HOME/swarm_params.parm"
fi

if [ -f "$SCRIPT_DIR/check_all_drones_params.py" ]; then
    (
        sleep 9
        echo ""
        echo "================================================================================"
        python3 "$SCRIPT_DIR/check_all_drones_params.py" "$num_drones"
    ) &
fi

# --- 6. Start SITL in foreground (MAVProxy needs stdin) ---
echo "Starting $num_drones $vehicle(s) in SITL with clean default parameters..."
$SITL_CMD -v $vehicle $FRAME_ARG $WIPE_ARG $PARAM_FILE -N --count $num_drones --auto-sysid --console "$@"
