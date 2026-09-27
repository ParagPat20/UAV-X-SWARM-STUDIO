#!/bin/bash
# ==============================================================================
# UAV-X Swarm Studio — Environment Activation Script
# Usage:
#   source ./activate.sh
#   or:
#   . ./activate.sh
# ==============================================================================

# Check if script is sourced or executed directly
if [ "${BASH_SOURCE[0]}" -ef "$0" ]; then
    echo "⚠️  NOTE: Please SOURCE this script so the virtualenv remains active in your shell:"
    echo "   source ./activate.sh"
    echo "   OR"
    echo "   . ./activate.sh"
    echo ""
fi

# 1. Locate and activate ArduPilot virtualenv
VENV_DIR="$HOME/venv-ardupilot"

if [ -d "$VENV_DIR" ] && [ -f "$VENV_DIR/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
    echo "✔ Activated Python Virtual Environment: $VENV_DIR"
else
    echo "⚠️  Warning: $VENV_DIR not found. Checking system python..."
fi

# 2. Add ArduPilot SITL tools, MAVProxy and scripts to PATH
export PATH="$HOME/venv-ardupilot/bin:$HOME/ardupilot/Tools/autotest:$HOME/.local/bin:$PATH"

# 3. Export default SITL Command
if [ -f "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" ]; then
    export SITL_CMD="$HOME/ardupilot/Tools/autotest/sim_vehicle.py"
fi

# 4. Print Environment Status
echo "🛸 UAV-X Swarm Studio Environment Active!"
echo "   Python  : $(which python3) ($(python3 --version 2>&1))"
echo "   AirSim  : $(python3 -c "import airsim; print('Available (v' + getattr(airsim, '__version__', '1.8.1') + ')') " 2>/dev/null || echo 'Not installed in current Python')"
echo "   MAVLink : $(python3 -c "import pymavlink; print('Available') " 2>/dev/null || echo 'Not installed in current Python')"
echo "================================================================================"
