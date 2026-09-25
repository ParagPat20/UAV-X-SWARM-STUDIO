#!/bin/bash
# ==============================================================================
# UAV-X Swarm Studio — Automated Microsoft AirSim Binary Downloader & Installer
# Downloads and extracts: Blocks | AirSimNH | LandscapeMountains
# ==============================================================================
set -e

DOWNLOAD_DIR="$HOME/Downloads"
mkdir -p "$DOWNLOAD_DIR"

echo "================================================================================"
echo "🛸 UAV-X Swarm Studio — Microsoft AirSim Environments Setup"
echo "================================================================================"
echo "Choose which AirSim binary environment(s) to download & configure:"
echo "  1) Blocks (City & Obstacle Grid — 560 MB) [RECOMMENDED DEFAULT]"
echo "  2) AirSimNH (Urban Residential Neighborhood — 1.1 GB)"
echo "  3) LandscapeMountains (Rugged Mountains & BVLOS Terrain — 580 MB)"
echo "  4) ALL Environments (Blocks + AirSimNH + LandscapeMountains — ~2.2 GB)"
echo "================================================================================"

if [ -n "$1" ]; then
    CHOICE="$1"
else
    echo -n "Enter selection [1-4, default 1]: "
    read CHOICE
    CHOICE="${CHOICE:-1}"
fi

download_blocks() {
    echo ""
    echo "▶ [1/3] Downloading Microsoft AirSim: Blocks..."
    TARGET_DIR="$DOWNLOAD_DIR/Blocks"
    mkdir -p "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    if [ -f "LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh" ] || [ -f "LinuxNoEditor/Blocks.sh" ]; then
        echo "✔ Blocks already exists in $TARGET_DIR. Skipping download."
    else
        wget -c --show-progress "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux/Blocks.zip" -O Blocks.zip
        echo "Extracting Blocks.zip..."
        unzip -q -o Blocks.zip -d LinuxBlocks1.8.1
        rm -f Blocks.zip
    fi
    
    chmod +x "$TARGET_DIR"/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh 2>/dev/null || true
    chmod +x "$TARGET_DIR"/LinuxNoEditor/Blocks.sh 2>/dev/null || true
    echo "✔ Blocks environment installed successfully!"
}

download_nh() {
    echo ""
    echo "▶ [2/3] Downloading Microsoft AirSim: AirSimNH (Neighborhood)..."
    TARGET_DIR="$DOWNLOAD_DIR/AirSimNH"
    mkdir -p "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    if [ -f "LinuxNoEditor/AirSimNH.sh" ] || [ -f "AirSimNH/LinuxNoEditor/AirSimNH.sh" ]; then
        echo "✔ AirSimNH already exists in $TARGET_DIR. Skipping download."
    else
        wget -c --show-progress "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux/AirSimNH.zip" -O AirSimNH.zip
        echo "Extracting AirSimNH.zip..."
        unzip -q -o AirSimNH.zip
        rm -f AirSimNH.zip
    fi
    
    chmod +x "$TARGET_DIR"/LinuxNoEditor/AirSimNH.sh 2>/dev/null || true
    chmod +x "$TARGET_DIR"/AirSimNH/LinuxNoEditor/AirSimNH.sh 2>/dev/null || true
    echo "✔ AirSimNH environment installed successfully!"
}

download_mountains() {
    echo ""
    echo "▶ [3/3] Downloading Microsoft AirSim: LandscapeMountains..."
    TARGET_DIR="$DOWNLOAD_DIR/LandscapeMountains"
    mkdir -p "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    if [ -f "LinuxNoEditor/LandscapeMountains.sh" ] || [ -f "LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh" ]; then
        echo "✔ LandscapeMountains already exists in $TARGET_DIR. Skipping download."
    else
        wget -c --show-progress "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux/LandscapeMountains.zip" -O LandscapeMountains.zip
        echo "Extracting LandscapeMountains.zip..."
        unzip -q -o LandscapeMountains.zip
        rm -f LandscapeMountains.zip
    fi
    
    chmod +x "$TARGET_DIR"/LinuxNoEditor/LandscapeMountains.sh 2>/dev/null || true
    chmod +x "$TARGET_DIR"/LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh 2>/dev/null || true
    echo "✔ LandscapeMountains environment installed successfully!"
}

case "$CHOICE" in
    1|[Bb]locks)
        download_blocks
        ;;
    2|[Nn][Hh]|[Aa]ir[Ss]im[Nn][Hh])
        download_nh
        ;;
    3|[Mm]ountains|[Ll]andscape*)
        download_mountains
        ;;
    4|[Aa]ll)
        download_blocks
        download_nh
        download_mountains
        ;;
    *)
        echo "Invalid selection. Defaulting to Blocks."
        download_blocks
        ;;
esac

echo ""
echo "================================================================================"
echo "🎉 AirSim Environment setup completed!"
echo "You can now run:"
echo "  ./start_sitl.sh 5 Blocks"
echo "  ./start_sitl.sh 5 AirSimNH"
echo "  ./start_sitl.sh 5 LandscapeMountains"
echo "================================================================================"
