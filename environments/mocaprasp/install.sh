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

CURRENT_HOSTNAME=$(hostname)
AVAHI_CONF="/etc/avahi/avahi-daemon.conf"

# Ensure the [server] section exists
sudo grep -q "^\[server\]" "$AVAHI_CONF" || echo -e "\n[server]" | sudo tee -a "$AVAHI_CONF" > /dev/null

# Remove any existing host-name= lines under [server]
sudo sed -i "/^\[server\]/,/^\[.*\]/ {/^\s*host-name=.*/d}" "$AVAHI_CONF"

# Add host-name under [server] (only if it's not already there)
sudo sed -i "/^\[server\]/a host-name=${CURRENT_HOSTNAME}" "$AVAHI_CONF"

echo "[INFO] Set static hostname to ${CURRENT_HOSTNAME}"

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

# Enable pigpio daemon
sudo systemctl enable pigpiod

echo "[INFO] Installation Finished"
echo "[INFO] Reboot for changes to take effect"