#!/bin/bash
# ==============================================================================
# UAV-X Swarm Studio — Automated Microsoft AirSim Binary Downloader & Installer
# Downloads and extracts directly inside the project's 'environments/' directory
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOWNLOAD_DIR="$SCRIPT_DIR/environments"
mkdir -p "$DOWNLOAD_DIR"

echo "================================================================================"
echo "🛸 UAV-X Swarm Studio — Microsoft AirSim Environments Setup"
echo "  Target Location: $DOWNLOAD_DIR"
echo "================================================================================"
echo "Choose which AirSim binary environment(s) to download & configure:"
echo "  1) Blocks (City & Obstacle Grid — 560 MB) [RECOMMENDED DEFAULT]"
echo "  2) AirSimNH (Urban Residential Neighborhood — 1.1 GB)"
echo "  3) LandscapeMountains (Rugged Mountains & BVLOS Terrain — 580 MB)"
echo "  4) ZhangJiajie (Avatar Mountain Pillars & Gorges — 560 MB)"
echo "  5) ALL Environments (Blocks + AirSimNH + LandscapeMountains + ZhangJiajie)"
echo "================================================================================"

if [ -n "$1" ]; then
    CHOICE="$1"
else
    echo -n "Enter selection [1-5, default 1]: "
    read CHOICE
    CHOICE="${CHOICE:-1}"
fi

download_blocks() {
    echo ""
    echo "▶ [1/3] Setting up Microsoft AirSim: Blocks..."
    TARGET_DIR="$DOWNLOAD_DIR/Blocks"
    mkdir -p "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    if [ -f "LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh" ] || [ -f "LinuxNoEditor/Blocks.sh" ]; then
        echo "✔ Blocks already exists in $TARGET_DIR. Ready."
    elif [ -d "$HOME/Downloads/Blocks/LinuxBlocks1.8.1" ]; then
        echo "✔ Found existing Blocks in ~/Downloads. Linking to local environments/ folder..."
        cp -rs "$HOME/Downloads/Blocks/LinuxBlocks1.8.1" "$TARGET_DIR/" 2>/dev/null || cp -r "$HOME/Downloads/Blocks/LinuxBlocks1.8.1" "$TARGET_DIR/"
    else
        echo "Downloading official Microsoft AirSim Blocks.zip..."
        wget -c --show-progress "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux/Blocks.zip" -O Blocks.zip
        echo "Extracting Blocks.zip..."
        unzip -q -o Blocks.zip -d LinuxBlocks1.8.1
        rm -f Blocks.zip
    fi
    
    chmod +x "$TARGET_DIR"/LinuxBlocks1.8.1/LinuxNoEditor/Blocks.sh 2>/dev/null || true
    chmod +x "$TARGET_DIR"/LinuxNoEditor/Blocks.sh 2>/dev/null || true
    echo "✔ Blocks environment configured successfully at $TARGET_DIR!"
}

download_nh() {
    echo ""
    echo "▶ [2/3] Setting up Microsoft AirSim: AirSimNH (Neighborhood)..."
    TARGET_DIR="$DOWNLOAD_DIR/AirSimNH"
    mkdir -p "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    if [ -f "LinuxNoEditor/AirSimNH.sh" ] || [ -f "AirSimNH/LinuxNoEditor/AirSimNH.sh" ]; then
        echo "✔ AirSimNH already exists in $TARGET_DIR. Ready."
    elif [ -d "$HOME/Downloads/AirSimNH" ] && [ -f "$HOME/Downloads/AirSimNH/LinuxNoEditor/AirSimNH.sh" ]; then
        echo "✔ Found existing AirSimNH in ~/Downloads. Linking to local environments/ folder..."
        cp -rs "$HOME/Downloads/AirSimNH/"* "$TARGET_DIR/" 2>/dev/null || cp -r "$HOME/Downloads/AirSimNH/"* "$TARGET_DIR/"
    else
        echo "Downloading official Microsoft AirSim AirSimNH.zip..."
        wget -c --show-progress "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux/AirSimNH.zip" -O AirSimNH.zip
        echo "Extracting AirSimNH.zip..."
        unzip -q -o AirSimNH.zip
        rm -f AirSimNH.zip
    fi
    
    chmod +x "$TARGET_DIR"/LinuxNoEditor/AirSimNH.sh 2>/dev/null || true
    chmod +x "$TARGET_DIR"/AirSimNH/LinuxNoEditor/AirSimNH.sh 2>/dev/null || true
    echo "✔ AirSimNH environment configured successfully at $TARGET_DIR!"
}

download_mountains() {
    echo ""
    echo "▶ [3/3] Setting up Microsoft AirSim: LandscapeMountains..."
    TARGET_DIR="$DOWNLOAD_DIR/LandscapeMountains"
    mkdir -p "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    if [ -f "LinuxNoEditor/LandscapeMountains.sh" ] || [ -f "LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh" ]; then
        echo "✔ LandscapeMountains already exists in $TARGET_DIR. Ready."
    elif [ -d "$HOME/Downloads/LandscapeMountains" ] && [ -f "$HOME/Downloads/LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh" ]; then
        echo "✔ Found existing LandscapeMountains in ~/Downloads. Linking to local environments/ folder..."
        cp -rs "$HOME/Downloads/LandscapeMountains/"* "$TARGET_DIR/" 2>/dev/null || cp -r "$HOME/Downloads/LandscapeMountains/"* "$TARGET_DIR/"
    else
        echo "Downloading official Microsoft AirSim LandscapeMountains.zip..."
        wget -c --show-progress "https://github.com/microsoft/AirSim/releases/download/v1.8.1-linux/LandscapeMountains.zip" -O LandscapeMountains.zip
        echo "Extracting LandscapeMountains.zip..."
        unzip -q -o LandscapeMountains.zip
        rm -f LandscapeMountains.zip
    fi
    
    chmod +x "$TARGET_DIR"/LinuxNoEditor/LandscapeMountains.sh 2>/dev/null || true
    chmod +x "$TARGET_DIR"/LandscapeMountains/LinuxNoEditor/LandscapeMountains.sh 2>/dev/null || true
    echo "✔ LandscapeMountains environment configured successfully at $TARGET_DIR!"
}

download_zhangjiajie() {
    echo ""
    echo "▶ [4/4] Setting up Microsoft AirSim: ZhangJiajie..."
    TARGET_DIR="$DOWNLOAD_DIR/ZhangJiajie"
    mkdir -p "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    if [ -f "ZhangJiajie/LinuxNoEditor/ZhangJiajie.sh" ] || [ -f "LinuxNoEditor/ZhangJiajie.sh" ]; then
        echo "✔ ZhangJiajie already exists in $TARGET_DIR. Ready."
    elif [ -f "$HOME/ZhangJiajie.zip" ]; then
        echo "✔ Found existing ZhangJiajie.zip in $HOME. Extracting to local environments/ folder..."
        unzip -q -o "$HOME/ZhangJiajie.zip" -d "$TARGET_DIR"
    else
        echo "Downloading official Microsoft AirSim ZhangJiajie.zip..."
        wget -c --show-progress "https://github.com/microsoft/AirSim/releases/download/v1.8.0-linux/ZhangJiajie.zip" -O ZhangJiajie.zip
        echo "Extracting ZhangJiajie.zip..."
        unzip -q -o ZhangJiajie.zip
        rm -f ZhangJiajie.zip
    fi
    
    chmod +x "$TARGET_DIR"/ZhangJiajie/LinuxNoEditor/ZhangJiajie.sh 2>/dev/null || true
    chmod +x "$TARGET_DIR"/LinuxNoEditor/ZhangJiajie.sh 2>/dev/null || true
    echo "✔ ZhangJiajie environment configured successfully at $TARGET_DIR!"
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
    4|[Zz]hang*|[Aa]vatar*)
        download_zhangjiajie
        ;;
    5|[Aa]ll)
        download_blocks
        download_nh
        download_mountains
        download_zhangjiajie
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
echo "  ./start_sitl.sh 5 ZhangJiajie"
echo "================================================================================"
