#! /bin/bash

# Checkout to test branch
git -C $HOME/VirtualMoCap checkout test > /dev/null 2>&1 || echo "[ERROR] Checkout failed"

# Get updates, if any, from the remote to local repo
git -C $HOME/VirtualMoCap pull > /dev/null 2>&1 || echo -e "\n[ERROR] Git pull failed"

# Start pigpiod daemon for advanced GPIO control
sudo pigpiod > /dev/null 2>&1 || echo "[ERROR] Pigpio daemon failed"

# Activate python virtual environment
source $HOME/.venv/bin/activate > /dev/null 2>&1 || echo "[ERROR] Venv activation failed"

# Change for development directory for convenience
cd $HOME/VirtualMoCap/environments/mocaprasp/ > /dev/null 2>&1 || echo "[ERROR] Test directory not found"

# Kill all processes using the camera
sudo fuser -k /dev/video0 > /dev/null 2>&1 || echo "[INFO] Could not kill camera process"

# Run capture script
python -m capture