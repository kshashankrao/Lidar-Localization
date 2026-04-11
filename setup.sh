#!/bin/bash

# 1. Exit immediately if a command exits with a non-zero status
set -e

echo "Starting C++ Environment Setup..."

# 2. Define your libraries in a list for easy reading and editing
LIBRARIES=(
    build-essential
    cmake
    git
    libpcl-dev
    libeigen3-dev
    libboost-all-dev
    libopenmpi-dev openmpi-bin
    nlohmann-json3-dev
)

# 3. Update the package list first (only need to do this once!)
echo "Updating package lists..."
sudo apt-get update

# 4. Install all libraries at once. 
# The magic flags here are critical for automation.
echo "Installing libraries..."
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "${LIBRARIES[@]}"

echo "Setup Complete!"