# MarineTec-China EICAR Button Controller

A Raspberry Pi-based system for replaying EICAR test packets via physical GPIO button control. This MVP allows you to trigger packet replay with a simple button press, with visual feedback via an LED indicator.

## ⚠️ Safety Warning

**This tool is designed for LAB and controlled network environments only. Only use this system in networks where you have explicit permission to send test traffic. Unauthorized use may violate network policies and laws.**

## Hardware Requirements

- Raspberry Pi (any model with GPIO pins)
- Physical push button (momentary switch)
- LED with appropriate resistor (220Ω-1kΩ recommended)
- Jumper wires for connections

## Hardware Setup

### GPIO Pin Connections (BCM numbering)

**Button:**
- One side → GND
- Other side → GPIO 17 (Physical pin 11)

**LED:**
- Anode (through resistor) → GPIO 22 (Physical pin 15)
- Cathode → GND

### Physical Pin Reference (40-pin header)
```
GPIO 17 = Physical Pin 11
GPIO 22 = Physical Pin 15
```

## One-Time Setup on Raspberry Pi

### 1. Update and Install Dependencies

```bash
# Update package list
sudo apt update

# Install required tools
sudo apt install -y tcpreplay python3-gpiozero
```

### 2. Create Project Directory

```bash
# Create project directory
sudo mkdir -p /opt/marinetec
sudo chown -R $USER:$USER /opt/marinetec
```

### 3. Place EICAR PCAP File

Copy your EICAR test pcap file to the project directory:

```bash
# Example (adjust source path as needed):
cp ~/Downloads/EICAR_test.pcap /opt/marinetec/EICAR_test.pcap
```

Verify the file exists:

```bash
ls -lh /opt/marinetec/EICAR_test.pcap
```

### 4. Install Python Dependencies

If you're running from this repository:

```bash
cd /opt/marinetec
pip3 install -r requirements.txt
```

Or install directly:

```bash
pip3 install gpiozero
```

## Usage

### Manual Execution

Run the script with sudo privileges (required for GPIO and tcpreplay):

```bash
cd /opt/marinetec
sudo python3 marinetec_eicar_button.py
```

**Expected Behavior:**
- Script initializes and prints configuration information
- Pressing the button will:
  1. Turn on the LED
  2. Execute `tcpreplay` to send EICAR packets on the configured interface
  3. Turn off the LED when complete
  4. Print logs to the terminal

Press `Ctrl+C` to exit.

### Running as a Systemd Service (Headless Operation)

To run the service automatically at boot:

1. Copy the service file:

```bash
sudo cp systemd/marinetec-eicar.service /etc/systemd/system/
```

2. Reload systemd and enable the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now marinetec-eicar.service
```

3. Check service status:

```bash
sudo systemctl status marinetec-eicar.service
```

4. View logs:

```bash
sudo journalctl -u marinetec-eicar.service -f
```

5. Stop/disable the service:

```bash
sudo systemctl stop marinetec-eicar.service
sudo systemctl disable marinetec-eicar.service
```

## Configuration

Edit `marinetec_eicar_button.py` to modify these constants:

```python
EICAR_PCAP_PATH = "/opt/marinetec/EICAR_test.pcap"  # Path to your pcap file
NETWORK_INTERFACE = "eth0"                           # Network interface name
BUTTON_PIN = 17                                      # GPIO pin for button (BCM)
LED_PIN = 22                                         # GPIO pin for LED (BCM)
DEBOUNCE_TIME = 0.2                                  # Button debounce in seconds
```

### Finding Your Network Interface

To list available network interfaces:

```bash
ip link show
```

Common interface names:
- `eth0` - Ethernet (wired)
- `wlan0` - WiFi
- `usb0` - USB network adapter

## Troubleshooting

### Button Not Responding
- Verify GPIO connections are secure
- Check that button is connected between GPIO 17 and GND
- Ensure script is running with sudo privileges
- Check GPIO permissions: `groups $USER` should include `gpio` or `dialout`

### LED Not Working
- Verify LED polarity (anode through resistor to GPIO 22, cathode to GND)
- Check resistor value (220Ω-1kΩ recommended)
- Test LED with a simple script to verify hardware

### tcpreplay Fails
- Verify pcap file exists at the configured path
- Check network interface name is correct: `ip link show`
- Ensure interface is up: `sudo ip link set eth0 up`
- Verify tcpreplay is installed: `which tcpreplay`
- Check permissions (script must run as root/sudo)

### Service Won't Start
- Check service file path and permissions
- Verify Python path: `which python3`
- Check service logs: `sudo journalctl -u marinetec-eicar.service`
- Ensure user in service file has GPIO access

## Project Structure

```
Marinetec-China/
├── marinetec_eicar_button.py    # Main Python script
├── requirements.txt              # Python dependencies
├── README.md                     # This file
├── config/                       # Configuration directory
└── systemd/                      # Systemd service files
    └── marinetec-eicar.service  # Service definition
```

## Features

- **GPIO Button Control**: Physical button press triggers packet replay
- **LED Status Indicator**: Visual feedback during packet transmission
- **Thread-Safe Operation**: Prevents overlapping replays
- **Comprehensive Logging**: Detailed output for debugging
- **Debounced Input**: Prevents false triggers from button bounce
- **Graceful Shutdown**: Clean exit on Ctrl+C or service stop

## Technical Details

- **GPIO Library**: Uses `gpiozero` for hardware abstraction
- **Packet Replay**: Uses `tcpreplay` command-line tool
- **Threading**: Non-blocking packet replay via threading
- **Pin Numbering**: BCM (Broadcom) numbering convention

## License

This project is for educational and authorized testing purposes only.

## Next Steps

Potential enhancements:
- Add second button for SNMP up/down toggle
- Web interface for remote control
- Multiple pcap file selection
- Configuration via config file
- Network interface auto-detection

