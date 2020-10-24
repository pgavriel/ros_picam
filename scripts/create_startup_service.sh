#!/bin/bash
#TO BE RUN ON PI, NOT MASTER MACHINE

# Copy service file to /lib/systemd/system
sudo cp picam_autostart.service /lib/systemd/system/picam_autostart.service
# Reload system control daemon
sudo systemctl daemon-reload
# Enable service on boot
sudo systemctl enable picam_autostart.service

echo "Picam startup service created. System reboot required."
