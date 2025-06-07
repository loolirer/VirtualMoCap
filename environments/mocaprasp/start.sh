#! /bin/bash

# Checkout to test branch
git -C $HOME/VirtualMoCap checkout test > /dev/null 2>&1 || echo "[ERROR] Checkout Failed"

# Get updates, if any, from the remote to local repo
git -C $HOME/VirtualMoCap pull > /dev/null 2>&1 || echo -e "\n[ERROR] Git Pull Failed"

# Start pigpiod daemon for advanced GPIO control
sudo pigpiod > /dev/null 2>&1 || echo "[ERROR] Pigpio Daemon Failed"

# Activate python virtual environment
source $HOME/.venv/bin/activate > /dev/null 2>&1 || echo "[ERROR] Venv Activation Failed"

# Change for development directory for convenience
cd $HOME/VirtualMoCap/environments/mocaprasp/ > /dev/null 2>&1 || echo "[ERROR] Test Directory Not Found"