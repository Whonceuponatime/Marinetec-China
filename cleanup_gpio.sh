#!/bin/bash
for pin in 17 22 27; do
    if [ -d "/sys/class/gpio/gpio$pin" ]; then
        echo $pin | sudo tee /sys/class/gpio/unexport > /dev/null 2>&1
    fi
done
echo "GPIO cleanup complete"

