#!/bin/bash

echo "Enabling all interfaces..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_serial 0
sudo raspi-config nonint do_onewire 0
sudo raspi-config nonint do_ssh 0
sudo raspi-config nonint do_vnc 0
sudo raspi-config nonint do_camera 0

echo "start_x=1" | sudo tee -a /boot/config.txt
echo "gpu_mem=128" | sudo tee -a /boot/config.txt

echo "[INFO] Setting Python development environment..."
echo "[INFO] Installing Picamera2 Dependencies..."
sudo apt update
sudo apt install -y python3-picamera2
echo "[INFO] Creating Python Virtual Environment..."
python -m venv $HOME/.venv --system-site-packages
echo "[INFO] Activating Python Virtual Environment..."
source $HOME/.venv/bin/activate
echo "[INFO] Upgrading pip, setuptools and wheel..."
pip install --upgrade pip setuptools wheel > /dev/null
echo "[INFO] Installing dependencies..."
pip install $HOME/VirtualMoCap
pip install -r $HOME/VirtualMoCap/environments/mocaprasp/requirements.txt 

# Get current system hostname
CURRENT_HOSTNAME=$(hostname)

# Target config file
AVAHI_CONF="/etc/avahi/avahi-daemon.conf"

# Ensure the [server] section exists
sudo grep -q "^\[server\]" "$AVAHI_CONF" || echo "[server]" | sudo tee -a "$AVAHI_CONF" > /dev/null

# Insert or update host-name=... under [server] section
sudo sed -i "/^\[server\]/,/^\[.*\]/ { 
    s/^host-name=.*/host-name=${CURRENT_HOSTNAME}/; 
    t; 
    /host-name=/! a host-name=${CURRENT_HOSTNAME}
}" "$AVAHI_CONF"

echo "[INFO] Set static hostname to ${CURRENT_HOSTNAME}"

# Add Avahi restart to root crontab if not already present
CRON_ENTRY='@reboot sleep 10 && systemctl restart avahi-daemon'

# Check if it's already there to avoid duplicates
if ! sudo crontab -l | grep -Fxq "$CRON_ENTRY"; then
    (sudo crontab -l 2>/dev/null; echo "$CRON_ENTRY") | sudo crontab -
    echo "[INFO] Avahi reboot line added to crontab."
else
    echo "[INFO] Crontab entry already exists."
fi

echo "[INFO] Adding start script to .bashrc if not already present..."
START_LINE="source $HOME/VirtualMoCap/environments/mocaprasp/start.sh"
if ! grep -Fxq "$START_LINE" $HOME/.bashrc; then
    echo "$START_LINE" >> $HOME/.bashrc
    echo "[INFO] Added start.sh to .bashrc"
else
    echo "[INFO] start.sh already configured in .bashrc"
fi

echo "[INFO] Adding autostart script on desktop login..."
AUTOSTART_DIR="$HOME/.config/lxsession/LXDE-pi"
AUTOSTART_FILE="$AUTOSTART_DIR/autostart"
AUTOSTART_CMD="lxterminal -e \"$HOME/VirtualMoCap/environments/mocaprasp/start.sh\""

# Create directory if it doesn't exist and add autostart command
mkdir -p "$AUTOSTART_DIR"

if ! grep -Fxq "$AUTOSTART_CMD" "$AUTOSTART_FILE" 2>/dev/null; then 
    echo "$AUTOSTART_CMD" >> "$AUTOSTART_FILE"
    echo "[INFO] Added script to desktop autostart."
else    
    echo "[INFO] Desktop autostart already exists."
fi

echo "[INFO] Installation Finished"
echo "[INFO] Reboot for changes to take effect"