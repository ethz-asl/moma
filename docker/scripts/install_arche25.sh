#!/bin/bash
set -e  # Exit on error
set -o pipefail  # Ensure pipeline failures are propagated
export DEBIAN_FRONTEND=noninteractive  # Suppress interactive prompts

echo "Updating ARCHE 2025 system packages..."
# apt-get -qq update && apt-get -qq upgrade -y

# BASICS
echo "Installing basic dependencies..."
apt-get update && apt-get install -y nano vim usbutils
echo "Installing basic dependencies...done"

# Install controller
echo "Installing controller dependencies..."
# cd /root/moma_ws && git clone https://github.com/matthias-mayr/Cartesian-Impedance-Controller.git
# Clear cache to reduce Docker image size
echo "Cleaning up unnecessary files..."
rm -rf /var/lib/apt/lists/* ~/.cache/pip
