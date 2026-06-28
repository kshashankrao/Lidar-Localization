#!/bin/bash
# Script to run hyperparameter tuning for ICP LiDAR Localization using Optuna.

# Ensure we are in the correct directory
cd "$(dirname "$0")"

# Activate the virtual environment
if [ -d "venv" ]; then
    echo "Activating virtual environment..."
    source venv/bin/activate
else
    echo "Creating virtual environment..."
    python3 -m venv venv
    source venv/bin/activate
fi

# Install dependencies
echo "Installing dependencies..."
pip install optuna plotly numpy scipy open3d-cpu 2>/dev/null || pip install optuna plotly numpy scipy

# Compile C++ project if not built
if [ ! -f "build/bin/lidar_localization" ]; then
    echo "Building C++ project..."
    mkdir -p build
    cd build
    cmake ..
    make -j$(nproc)
    cd ..
fi

# Run the Optuna tuning script
echo "Starting Optuna hyperparameter tuning..."
python3 python/optuna_tuner.py --n-trials 30 --frames 200 --n-jobs 2

