# Quick Start Guide - Raspberry Pi Setup

## One-Command Setup

After pulling the code from git, run:

```bash
chmod +x setup_pi.sh
./setup_pi.sh
```

This script will:
1. Install all system dependencies (python3-rpi.gpio, etc.)
2. Create a virtual environment
3. Install Python packages (scapy, gpiozero)
4. Set up the project directory

## Run the Script

After setup completes:

```bash
cd ~/Desktop/Marinetec-China  # or your project directory
sudo ./venv/bin/python3 marinetec_eicar_button.py
```

## Hardware Connections

- **Button**: Connect one side to GPIO 17 (physical pin 11), other side to GND
- **LED**: Connect anode (through 220Ω-1kΩ resistor) to GPIO 22 (physical pin 15), cathode to GND

## Configuration

The script will:
- Auto-detect your Pi's IP address (or use static 192.168.127.25 if configured)
- Send EICAR packets to 192.168.127.10:80
- Resolve MAC address automatically via ping/ARP

To change settings, edit `marinetec_eicar_button.py`:
- `TARGET_IP` - destination IP
- `SOURCE_IP` - set to your static IP or leave as `None` for auto-detect
- `NETWORK_INTERFACE` - network interface name (usually "eth0" or "wlan0")
- `BUTTON_PIN` - GPIO pin for button (currently 27)
- `LED_PIN` - GPIO pin for LED (currently 22)

## Troubleshooting

**GPIO errors?**
- Make sure you ran `setup_pi.sh` which installs `python3-rpi.gpio`
- Run with `sudo` for GPIO access

**Module not found errors?**
- Use the venv's Python: `sudo ./venv/bin/python3` (not system python3)
- Make sure you activated venv or use full path to venv's Python

**Network errors?**
- Check interface name: `ip link show`
- Test connectivity: `ping 192.168.127.10`
- Update `NETWORK_INTERFACE` in the script if needed

