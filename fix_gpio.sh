#!/bin/bash

# Quick fix script to enable GPIO access in venv

echo "Fixing GPIO access in virtual environment..."
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Install RPi.GPIO system-wide if not already installed
echo "[1/3] Installing RPi.GPIO system package..."
sudo apt install -y python3-rpi.gpio

# Remove old venv
echo ""
echo "[2/3] Removing old virtual environment..."
if [ -d "venv" ]; then
    rm -rf venv
    echo "Old venv removed."
else
    echo "No existing venv found."
fi

# Create new venv with system site packages
echo ""
echo "[3/3] Creating new virtual environment with system site packages..."
python3 -m venv --system-site-packages venv

# Activate and install requirements
echo ""
echo "Installing Python packages..."
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

echo ""
echo "=========================================="
echo "GPIO Fix Complete!"
echo "=========================================="
echo ""
echo "Now run:"
echo "  sudo ./venv/bin/python3 marinetec_eicar_button.py"
echo ""

