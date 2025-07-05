#! /bin/bash

# Wait for rasp to connect to the internet
echo "[INFO] Waiting for internet connection..."
while ! ping -q -c 1 -W 1 192.168.0.1 >/dev/null; do sleep 1; done
echo "[INFO] Connected to local network"

# Checkout to test branch
git -C $HOME/VirtualMoCap checkout test > /dev/null 2>&1 || echo "[ERROR] Checkout failed"

# Get updates, if any, from the remote to local repo
git -C $HOME/VirtualMoCap reset --hard > /dev/null 2>&1 || echo "[ERROR] Hard reset failed"
git -C $HOME/VirtualMoCap pull > /dev/null 2>&1 || echo "[ERROR] Pull failed"

# Start pigpiod daemon for advanced GPIO control
sudo pigpiod > /dev/null 2>&1 || echo "[ERROR] Pigpio daemon failed"

# Activate python virtual environment
source $HOME/.venv/bin/activate > /dev/null 2>&1 || echo "[ERROR] Venv activation failed"

# Change for development directory for convenience
cd $HOME/VirtualMoCap/environments/mocaprasp/ > /dev/null 2>&1 || echo "[ERROR] Test directory not found"

# Kill all video processes
sudo fuser -k /dev/video0 > /dev/null 2>&1

# Run capture script
python -m capture