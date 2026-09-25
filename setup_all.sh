#!/bin/bash
# ==============================================================================
# UAV-X Swarm Studio — 1-Click Complete System & Simulation Setup
# Installs: System Packages + Python Venv + ArduPilot SITL + AirSim Binaries
# Compatible with: Ubuntu 20.04 / 22.04 / 24.04 LTS
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "================================================================================"
echo "🛸 UAV-X SWARM STUDIO — 1-CLICK COMPLETE SETUP"
echo "================================================================================"
echo "This script will configure your blank machine from scratch:"
echo "  1. System & build tools (apt)"
echo "  2. Python 3 Virtual Environment & Python dependencies"
echo "  3. ArduPilot Multi-UAV SITL Simulator"
echo "  4. Microsoft AirSim Unreal Engine Binary Simulation"
echo "================================================================================"
echo ""

# --- 1. System Packages ---
echo "▶ [1/4] Installing system prerequisites..."
sudo apt update
sudo apt install -y \
    git \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    unzip \
    wget \
    curl \
    psmisc \
    libxml2-dev \
    libxslt-dev

# --- 2. Python Virtual Environment ---
echo ""
echo "▶ [2/4] Setting up Python virtual environment..."
VENV_DIR="$HOME/venv-ardupilot"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
fi

source "$VENV_DIR/bin/activate"
pip install --upgrade pip
if [ -f "$SCRIPT_DIR/requirements.txt" ]; then
    pip install -r "$SCRIPT_DIR/requirements.txt"
fi

# --- 3. ArduPilot SITL Multi-Rotor Setup ---
echo ""
echo "▶ [3/4] Checking ArduPilot SITL installation..."
ARDUPILOT_DIR="$HOME/ardupilot"
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "Cloning ArduPilot repository..."
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
    cd "$ARDUPILOT_DIR"
    echo "Running ArduPilot Ubuntu prerequisites installer..."
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    source "$HOME/.profile" 2>/dev/null || true
    echo "Configuring and building ArduCopter SITL binary..."
    ./waf configure --board sitl
    ./waf copter
    cd "$SCRIPT_DIR"
else
    echo "✔ ArduPilot already installed in $ARDUPILOT_DIR."
fi

# --- 4. AirSim Environments Download ---
echo ""
echo "▶ [4/4] Setting up Microsoft AirSim Binary Simulation..."
chmod +x "$SCRIPT_DIR/download_airsim.sh"
bash "$SCRIPT_DIR/download_airsim.sh" "${1:-1}"

# Set permissions for all helper scripts
chmod +x "$SCRIPT_DIR"/*.sh 2>/dev/null || true

echo ""
echo "================================================================================"
echo "🎉 SETUP 100% COMPLETE! ALL DEPENDENCIES & SIMULATORS ARE READY."
echo "================================================================================"
echo "To launch your 5-drone autonomous swarm simulation:"
echo ""
echo "  cd $SCRIPT_DIR"
echo "  ./start_sitl.sh 5"
echo ""
echo "Or choose specific environments:"
echo "  ./start_sitl.sh 5 Blocks"
echo "  ./start_sitl.sh 5 AirSimNH"
echo "  ./start_sitl.sh 5 LandscapeMountains"
echo "================================================================================"
