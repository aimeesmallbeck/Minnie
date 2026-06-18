#!/bin/bash
# Aimee Robot - Push main branch to the primary GitHub repo
#
# Usage:
#   bash deploy/push-public.sh

set -e

cd "$HOME/aimee-robot-ws"

echo "Pushing main branch to origin (Minnie)..."

# Ensure we're on main
git checkout main

# Push to origin remote
git push origin main

echo ""
echo "Push complete!"
echo "View at: https://github.com/aimeesmallbeck/Minnie"
