#!/bin/bash

# Apply the pinmux settings for CAN
echo "Applying pinmux settings for CAN..."
busybox devmem 0x0c303018 w 0x458
busybox devmem 0x0c303010 w 0x400

# Load CAN kernel modules
echo "Loading CAN kernel drivers..."
modprobe can
modprobe can_raw
modprobe mttcan  # Assuming mttcan is used for CAN on your device

# Bring up the CAN interface
echo "Setting up CAN interface..."
ip link set can0 up type can bitrate 250000

# Verify the CAN setup
echo "CAN interface setup complete. Verifying..."
ip -details link show can0

echo "Done."

