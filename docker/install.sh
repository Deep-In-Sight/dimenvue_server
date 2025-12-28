#!/bin/bash
# DimenvuePro Installation Script
# This script builds the Docker image and installs all necessary files

set -e

# Change to script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Paths
BIN_DIR="$HOME/bin"
ICON_DIR="$HOME/.local/share/icons"
DESKTOP_DIR="$HOME/Desktop"
LOG_DIR="$HOME/.local/log/dimenvue"

echo "========================================"
echo "  DimenvuePro Installation"
echo "========================================"
echo ""

# Parse command line arguments
NO_CACHE=""
SKIP_DOCKER=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --no-cache)
            NO_CACHE="--no-cache"
            shift
            ;;
        --skip-docker)
            SKIP_DOCKER=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --no-cache     Build Docker image without cache"
            echo "  --skip-docker  Skip Docker image build"
            echo "  -h, --help     Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Step 1: Build Docker image
if [ "$SKIP_DOCKER" = false ]; then
    echo "[1/5] Building Docker image..."
    docker build --network=host $NO_CACHE -f Dockerfile.jetson -t dimenvue_server:jetson .
    echo "      Docker image built successfully."
else
    echo "[1/5] Skipping Docker image build."
fi

# Step 2: Create directories
echo "[2/5] Creating directories..."
mkdir -p "$BIN_DIR"
mkdir -p "$ICON_DIR"
mkdir -p "$DESKTOP_DIR"
mkdir -p "$LOG_DIR"
echo "      Directories created."

# Step 3: Install launcher script
echo "[3/5] Installing launcher script..."
cat > "$BIN_DIR/dimenvue-launcher.sh" << 'EOF'
#!/bin/bash
# DimenvuePro Launcher - starts backend server and frontend application

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$HOME/.local/log/dimenvue"
mkdir -p "$LOG_DIR"

# Start backend server
echo "Starting DimenvuePro backend server..."
"$HOME/dimenvue_server/docker/start_server.sh" > "$LOG_DIR/backend.log" 2>&1 &
BACKEND_PID=$!

# Wait a moment for the server to initialize
sleep 3

# Start frontend application
echo "Starting DimenvuePro frontend..."
"$HOME/DimenvuePro/release/linux-arm64-unpacked/dimenvuepro" --force-device-scale-factor=2 > "$LOG_DIR/frontend.log" 2>&1

# When frontend closes, optionally stop backend
echo "Frontend closed. Backend server may still be running in Docker."
EOF
chmod +x "$BIN_DIR/dimenvue-launcher.sh"
echo "      Launcher script installed to $BIN_DIR/dimenvue-launcher.sh"

# Step 4: Install icon
echo "[4/5] Installing application icon..."
if [ -f "$SCRIPT_DIR/icon.png" ]; then
    cp "$SCRIPT_DIR/icon.png" "$ICON_DIR/dimenvuepro.png"
    echo "      Icon installed to $ICON_DIR/dimenvuepro.png"
else
    echo "      Warning: icon.png not found in $SCRIPT_DIR"
fi

# Step 5: Create desktop entry
echo "[5/5] Creating desktop entry..."
cat > "$DESKTOP_DIR/dimenvuepro.desktop" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=DimenvuePro
Comment=Deep Insight DimenvuePro Application
Exec=$BIN_DIR/dimenvue-launcher.sh
Icon=$ICON_DIR/dimenvuepro.png
Terminal=true
Categories=Development;Science;
StartupNotify=true
StartupWMClass=dimenvuepro
EOF
chmod +x "$DESKTOP_DIR/dimenvuepro.desktop"
echo "      Desktop entry created at $DESKTOP_DIR/dimenvuepro.desktop"

echo ""
echo "========================================"
echo "  Installation Complete!"
echo "========================================"
echo ""
echo "You can now launch DimenvuePro by:"
echo "  - Double-clicking the desktop icon"
echo "  - Running: ~/bin/dimenvue-launcher.sh"
echo ""
echo "Logs are stored in: $LOG_DIR"
echo ""
