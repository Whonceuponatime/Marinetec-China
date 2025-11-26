#!/bin/bash

# MarineTec-China Raspberry Pi Setup Script
# This script sets up everything needed to run the EICAR button controller

set -e  # Exit on error

echo "=========================================="
echo "MarineTec-China Pi Setup Script"
echo "=========================================="
echo ""

# Check if running on Raspberry Pi
if [ ! -f /proc/device-tree/model ] || ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "WARNING: This doesn't appear to be a Raspberry Pi."
    echo "GPIO functionality may not work correctly."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "[1/6] Updating package list..."
sudo apt update

echo ""
echo "[2/6] Installing system packages..."
sudo apt install -y python3-pip python3-venv python3-dev python3-rpi.gpio

echo ""
echo "[3/6] Creating virtual environment..."
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo "Virtual environment created."
else
    echo "Virtual environment already exists."
fi

echo ""
echo "[4/6] Activating virtual environment and installing Python packages..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo ""
echo "[5/6] Setting up project directory..."
sudo mkdir -p /opt/marinetec
sudo chown -R $USER:$USER /opt/marinetec

echo ""
echo "[6/6] Setup complete! Script is ready in project directory."
echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "To run the script from project directory:"
echo "  cd $SCRIPT_DIR"
echo "  sudo ./venv/bin/python3 marinetec_eicar_button.py"
echo ""
echo "Or create a symlink to /opt/marinetec if preferred:"
echo "  sudo ln -s $SCRIPT_DIR/marinetec_eicar_button.py /opt/marinetec/"
echo "  cd /opt/marinetec"
echo "  sudo $SCRIPT_DIR/venv/bin/python3 marinetec_eicar_button.py"
echo ""
echo "Make sure your EICAR pcap file is at:"
echo "  /opt/marinetec/EICAR_test.pcap"
echo ""
echo "Or the script will generate packets on-the-fly."
echo ""
echo "Hardware connections:"
echo "  Button: GPIO 27 (pin 13) to GND"
echo "  LED: GPIO 22 (pin 15) to GND (through resistor)"
echo ""

