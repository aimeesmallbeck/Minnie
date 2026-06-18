#!/bin/bash
# Aimee Robot - New Board Bootstrap Script
# Run this on a fresh Arduino UNO Q to provision the entire ROS2 stack.
#
# Usage:
#   curl -fsSL https://raw.githubusercontent.com/aimeesmallbeck/Minnie/main/deploy/bootstrap.sh | bash
#   # Or, if cloned locally:
#   bash deploy/bootstrap.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
REPO_URL="https://github.com/aimeesmallbeck/Minnie.git"
# Set these to rsync large/binary files from your first board (Ron)
MODELS_SOURCE=""   # e.g. "arduino@10.0.0.156:/home/arduino/aimee-robot-ws/models"
VOSK_SOURCE=""     # e.g. "arduino@10.0.0.156:/home/arduino/aimee-robot-ws/vosk-models"
CONFIG_SOURCE=""   # e.g. "arduino@10.0.0.156:/home/arduino/aimee-robot-ws/config"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# ─────────────────────────────── Step 1: System Prep ───────────────────────────────
log_info "Aimee Robot Bootstrap starting..."

ARCH=$(uname -m)
if [ "$ARCH" != "aarch64" ]; then
    log_warn "Architecture is $ARCH, expected aarch64 (Arduino UNO Q)"
fi

# Install Docker if missing
if ! command -v docker &> /dev/null; then
    log_info "Installing Docker..."
    sudo apt-get update
    sudo apt-get install -y docker.io docker-compose-plugin
    sudo usermod -aG docker "$USER"
    log_warn "Docker installed. You may need to log out and back in for group changes."
    log_warn "After re-login, re-run this script."
    exit 0
else
    log_info "Docker already installed."
fi

# Ensure user is in docker group
if ! groups "$USER" | grep -q '\bdocker\b'; then
    sudo usermod -aG docker "$USER"
    log_warn "Added $USER to docker group. Please log out and back in, then re-run."
    exit 0
fi

# ─────────────────────────────── Step 2: Clone Workspace ───────────────────────────────
if [ ! -d "$HOME/aimee-robot-ws" ]; then
    log_info "Cloning Aimee workspace to ~/aimee-robot-ws..."
    git clone "$REPO_URL" "$HOME/aimee-robot-ws"
    cd "$HOME/aimee-robot-ws"
else
    log_info "Workspace exists at ~/aimee-robot-ws. Pulling latest changes..."
    cd "$HOME/aimee-robot-ws"
    git pull origin main || true
fi

# ─────────────────────────────── Step 3: Environment Configuration ───────────────────────────────
if [ ! -f ".env" ]; then
    log_info "Creating .env from template..."
    cp .env.example .env
    log_warn "Please edit ~/aimee-robot-ws/.env and add your API keys before starting the robot."
fi

# ─────────────────────────────── Step 4: Audio Setup ───────────────────────────────
if [ ! -f "$HOME/.asoundrc" ]; then
    log_info "Creating default ALSA configuration..."
    cat > "$HOME/.asoundrc" << 'EOF'
pcm.!default {
    type plug
    slave.pcm "plughw:0,0"
}
ctl.!default {
    type hw
    card 0
}
EOF
fi

# ─────────────────────────────── Step 5: Asset Sync (Optional) ───────────────────────────────
# Large binary assets (models, configs) are NOT in git. Sync them from board #1.

if [ -n "$MODELS_SOURCE" ]; then
    log_info "Syncing ML models from source board..."
    mkdir -p models
    rsync -avz --progress "$MODELS_SOURCE/" models/ || log_warn "Model sync failed."
else
    log_warn "No MODELS_SOURCE set. Skipping ML model sync."
fi

if [ -n "$VOSK_SOURCE" ]; then
    log_info "Syncing Vosk speech models from source board..."
    mkdir -p vosk-models
    rsync -avz --progress "$VOSK_SOURCE/" vosk-models/ || log_warn "Vosk model sync failed."
else
    log_warn "No VOSK_SOURCE set. Skipping Vosk model sync."
fi

if [ -n "$CONFIG_SOURCE" ]; then
    log_info "Syncing robot configs from source board..."
    mkdir -p config
    rsync -avz --progress "$CONFIG_SOURCE/" config/ || log_warn "Config sync failed."
else
    log_warn "No CONFIG_SOURCE set. Skipping config sync."
    log_warn "Tip: Copy config/robots/<hostname>.yaml from Ron and set ROBOT_CONFIG in .env"
fi

# ─────────────────────────────── Step 6: Build Docker Image ───────────────────────────────
log_info "Building Aimee Docker image (this may take 10-20 minutes on first run)..."
docker compose build

# ─────────────────────────────── Step 7: Build ROS2 Workspace ───────────────────────────────
log_info "Building ROS2 workspace inside container..."
docker compose run --rm aimee-robot bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /workspace &&
    rosdep install --from-paths src --ignore-src -y 2>/dev/null || true &&
    colcon build --symlink-install
"

# ─────────────────────────────── Step 8: Install Systemd Service ───────────────────────────────
log_info "Installing systemd service for auto-start on boot..."
sudo cp systemd/aimee-robot.service /etc/systemd/system/aimee-robot.service
sudo sed -i "s|/home/arduino|$HOME|g" /etc/systemd/system/aimee-robot.service
sudo systemctl daemon-reload
sudo systemctl enable aimee-robot.service

# ─────────────────────────────── Done ───────────────────────────────
echo ""
log_info "========================================"
log_info "Aimee Robot Bootstrap Complete!"
log_info "========================================"
echo ""
echo "Next steps:"
echo "  1. Edit ~/aimee-robot-ws/.env with your API keys and robot name"
echo "  2. Create robot config: cp src/aimee_bringup/config/robots/default.yaml config/robots/$(hostname).yaml"
echo "     (or sync from Ron:   rsync -avz arduino@ron-ip:~/aimee-robot-ws/config/ config/)"
echo "  3. Start the robot:     docker compose up -d"
echo "  4. View logs:           docker compose logs -f"
echo "  5. Open monitor:        http://$(hostname -I | awk '{print $1}'):8081"
echo "  6. Launch manually:     docker compose exec aimee-robot bash -c 'source install/setup.bash && ros2 launch aimee_bringup robot.launch.py'"
echo "  7. Auto-start on boot:  sudo systemctl start aimee-robot"
echo ""
