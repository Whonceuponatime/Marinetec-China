#!/bin/bash

# GPIO cleanup script - unexports GPIO pins that may be in use

echo "Cleaning up GPIO pins..."

# GPIO 27 (button)
if [ -d "/sys/class/gpio/gpio27" ]; then
    echo "27" > /sys/class/gpio/unexport 2>/dev/null || echo "GPIO 27 already unexported or in use"
fi

# GPIO 22 (LED)
if [ -d "/sys/class/gpio/gpio22" ]; then
    echo "22" > /sys/class/gpio/unexport 2>/dev/null || echo "GPIO 22 already unexported or in use"
fi

echo "GPIO cleanup complete."
echo ""
echo "Note: If pins are still in use, you may need to:"
echo "  1. Stop any running instances of the script"
echo "  2. Check for other processes using GPIO: lsof | grep gpio"
echo "  3. Reboot if necessary"

