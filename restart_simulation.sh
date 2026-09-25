#!/bin/bash
# ==============================================================================
# UAV-X Swarm Studio — Clean Simulation Restart Script
# ==============================================================================
NUM_DRONES="${1:-5}"
ENV_CHOICE="${2:-Blocks}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[RESTART] Initiating full clean restart for $NUM_DRONES drone(s) in $ENV_CHOICE environment..."

# 1. Terminate running instances
pkill -9 -f "arducopter" 2>/dev/null || true
pkill -9 -f "mavproxy" 2>/dev/null || true
pkill -9 -f "sim_vehicle" 2>/dev/null || true
pkill -9 -f "server.py" 2>/dev/null || true
pkill -9 -f "LinuxNoEditor/.*\.sh|Binaries/Linux/.*-Linux|Blocks\.sh|AirSimNH\.sh|LandscapeMountains\.sh" 2>/dev/null || true
fuser -k 8080/tcp 8765/tcp 2>/dev/null || true

sleep 1.5

# 2. Launch start_sitl.sh in fresh session
cd "$SCRIPT_DIR"
nohup bash "$SCRIPT_DIR/start_sitl.sh" "$NUM_DRONES" "$ENV_CHOICE" > /tmp/start_sitl.log 2>&1 &
echo "[RESTART] start_sitl.sh launched with PID $!"
