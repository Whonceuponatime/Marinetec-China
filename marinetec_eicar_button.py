#!/usr/bin/env python3

"""

MarineTec-China MVP (v1) — EICAR packet generation and send button

Behavior:

- When the physical button is pressed:

    -> Generate EICAR TCP packet with auto-detected source IP and resolved MAC

    -> Send the packet on the configured interface using scapy.

    -> Turn the LED ON while sending, then OFF when done.

Hardware assumptions (BCM pin numbering):

- Button:

    - One side -> GND

    - Other side -> GPIO 17

- LED (optional, but recommended):

    - Anode (through resistor / driver) -> GPIO 22

    - Cathode -> GND

Notes:

- Run this script with sufficient privileges (sudo) so scapy and GPIO work.

- Only use this in a LAB / controlled network where you have permission!

"""

from gpiozero import Button, LED

import subprocess

import threading

import time

import socket

import struct

from scapy.all import get_if_addr, get_if_hwaddr, ARP, Ether, IP, TCP, sendp, srp

# --------- CONFIGURABLE CONSTANTS ---------

# Network interface to send packets out of

NETWORK_INTERFACE = "eth0"

# Target IP address

TARGET_IP = "192.168.127.10"

# Source IP (will auto-detect from interface, or use static if set)

# Set to None for auto-detection, or specify like "192.168.127.25"

SOURCE_IP = None  # None = auto-detect, or set to "192.168.127.25"

# TCP port for EICAR packet

TCP_PORT = 80

# GPIO pins (BCM numbering)

BUTTON_PIN = 17   # physical: pin 11 - EICAR packet button

LED_PIN = 22      # physical: pin 15 - LED indicator (optional)

# SNMP button pins

SNMP_DOWN_PIN = 27   # physical: pin 13 - SNMP port DOWN button

SNMP_UP_PIN = 22     # physical: pin 15 - SNMP port UP button (shares with LED)

# Debounce time (seconds)

DEBOUNCE_TIME = 0.2

# SNMP Configuration

SNMP_TARGET = "192.168.127.10"

SNMP_PORT = 161  # Standard SNMP port

SNMP_COMMUNITY = "private"

SNMP_OID_BASE = "1.3.6.1.2.1.2.2.1.7"  # ifAdminStatus OID

SNMP_IFINDEX = 8  # Interface index (port 8)

# EICAR test string (standard antivirus test pattern)

EICAR_STRING = b"X5O!P%@AP[4\\PZX54(P^)7CC)7}$EICAR-STANDARD-ANTIVIRUS-TEST-FILE!$H+H*"

# ------------------------------------------

class MarineTecController:

    def __init__(self, eicar_button_pin: int, snmp_down_pin: int, snmp_up_pin: int, led_pin: int = None):

        # Try to use RPi.GPIO factory if available (better for Raspberry Pi)
        # Otherwise fall back to default
        self._gpio_cleanup_needed = False
        try:
            # First check if RPi module is available
            import RPi
            import RPi.GPIO as GPIO
            from gpiozero.pins.rpigpio import RPiGPIOFactory
            from gpiozero import Device
            
            # Clean up any existing GPIO state first
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Force cleanup of specific pins (all pins we'll use)
            pins_to_cleanup = [eicar_button_pin, snmp_down_pin, snmp_up_pin]
            if led_pin and led_pin != snmp_up_pin:
                pins_to_cleanup.append(led_pin)
            
            for pin in pins_to_cleanup:
                try:
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
                    GPIO.cleanup(pin)
                except:
                    pass
            
            # Full cleanup
            try:
                GPIO.cleanup()
            except:
                pass
            
            # Small delay to let GPIO settle
            time.sleep(0.2)
            
            # Now set the pin factory
            Device.pin_factory = RPiGPIOFactory()
            self._gpio_cleanup_needed = True
            print("[INFO] Using RPi.GPIO pin factory")
        except (ImportError, ModuleNotFoundError):
            try:
                import pigpio
                from gpiozero.pins.pigpio import PiGPIOFactory
                from gpiozero import Device
                Device.pin_factory = PiGPIOFactory()
                print("[INFO] Using pigpio pin factory")
            except (ImportError, ModuleNotFoundError):
                print("[WARN] RPi.GPIO not available, using default pin factory")
                print("[WARN] Installing RPi.GPIO in venv: pip install RPi.GPIO")
                print("[WARN] Or install system package: sudo apt install python3-rpi.gpio")
                print("[WARN] Then recreate venv with: python3 -m venv --system-site-packages venv")

        # Initialize LED if provided (optional)
        # Skip LED if GPIO 22 is used for SNMP UP button
        if led_pin and led_pin != snmp_up_pin:
            try:
                self.led = LED(led_pin)
                print(f"[INFO] LED initialized on GPIO {led_pin}")
            except Exception as e:
                print(f"[WARN] Failed to initialize LED on GPIO {led_pin}: {e}")
                print("[WARN] Continuing without LED...")
                self.led = None
        else:
            if led_pin == snmp_up_pin:
                print(f"[INFO] Skipping LED on GPIO {led_pin} (used for SNMP UP button)")
            self.led = None

        # Store pins FIRST (before any polling threads start)
        self.eicar_button_pin = eicar_button_pin
        self.snmp_down_pin = snmp_down_pin
        self.snmp_up_pin = snmp_up_pin

        # Avoid overlapping sends
        self._busy = False

        # Network configuration
        self.source_ip = self._get_source_ip()
        self.source_mac = get_if_hwaddr(NETWORK_INTERFACE)

        # Initialize EICAR button
        print("[INFO] Initializing EICAR button...")
        self.button = self._init_button(eicar_button_pin, "EICAR")
        if self.button and hasattr(self.button, 'when_pressed'):
            self.button.when_pressed = self._on_button_pressed
        elif self.button and isinstance(self.button, dict) and self.button.get('polling'):
            # Polling mode for EICAR button
            self.eicar_button_polling = True
            self.eicar_button_state = None
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            self.eicar_button_state = GPIO.input(eicar_button_pin)
            print("[INFO] Starting EICAR button polling thread...")
            self._eicar_polling_thread = threading.Thread(target=self._poll_eicar_button, daemon=True)
            self._eicar_polling_thread.start()
        else:
            print("[WARN] EICAR button initialization failed, continuing without it...")
            self.button = None
        
        # Initialize SNMP buttons
        self._init_snmp_buttons()
        
        print(f"[INFO] MarineTec controller initialized.")
        print(f"[INFO] EICAR button: GPIO {eicar_button_pin}")
        print(f"[INFO] SNMP DOWN button: GPIO {snmp_down_pin}")
        print(f"[INFO] SNMP UP button: GPIO {snmp_up_pin}")
        print(f"[INFO] Interface: {NETWORK_INTERFACE}")
        print(f"[INFO] Source IP: {self.source_ip}")
        print(f"[INFO] Source MAC: {self.source_mac}")
        print(f"[INFO] EICAR Target IP: {TARGET_IP}")
        print(f"[INFO] SNMP Target: {SNMP_TARGET}:{SNMP_PORT}, Interface Index: {SNMP_IFINDEX}")

    def _poll_eicar_button(self):
        """Poll EICAR button state when edge detection is not available."""
        import RPi.GPIO as GPIO
        last_state = self.eicar_button_state
        last_press_time = 0
        
        while True:
            try:
                current_state = GPIO.input(self.eicar_button_pin)
                
                # Detect button press (transition from HIGH to LOW)
                if last_state == GPIO.HIGH and current_state == GPIO.LOW:
                    current_time = time.time()
                    # Debounce: ignore presses within DEBOUNCE_TIME of last press
                    if current_time - last_press_time > DEBOUNCE_TIME:
                        if not self._busy:
                            print("[EVENT] EICAR button press detected (polling mode)")
                            self._on_button_pressed()
                            last_press_time = current_time
                
                self.eicar_button_state = current_state
                last_state = current_state
                time.sleep(0.05)  # Poll every 50ms
                
            except Exception as e:
                print(f"[ERROR] Error in EICAR button polling: {e}")
                time.sleep(0.1)
    
    def _init_snmp_buttons(self):
        """Initialize SNMP port up/down buttons."""
        print("[INFO] Initializing SNMP buttons...")
        
        # Initialize SNMP DOWN button (GPIO 27)
        self.snmp_down_button = self._init_button(self.snmp_down_pin, "SNMP DOWN")
        if self.snmp_down_button:
            if hasattr(self.snmp_down_button, 'when_pressed'):
                self.snmp_down_button.when_pressed = self._on_snmp_down_pressed
            elif isinstance(self.snmp_down_button, dict) and self.snmp_down_button.get('polling'):
                # Polling mode for SNMP DOWN button
                print("[INFO] Starting SNMP DOWN button polling thread...")
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                self._snmp_down_polling_thread = threading.Thread(target=self._poll_snmp_down_button, daemon=True)
                self._snmp_down_polling_thread.start()
            else:
                print("[WARN] SNMP DOWN button initialized but callback not set")
        
        # Initialize SNMP UP button (GPIO 22)
        self.snmp_up_button = self._init_button(self.snmp_up_pin, "SNMP UP")
        if self.snmp_up_button:
            if hasattr(self.snmp_up_button, 'when_pressed'):
                self.snmp_up_button.when_pressed = self._on_snmp_up_pressed
            elif isinstance(self.snmp_up_button, dict) and self.snmp_up_button.get('polling'):
                # Polling mode for SNMP UP button
                print("[INFO] Starting SNMP UP button polling thread...")
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                self._snmp_up_polling_thread = threading.Thread(target=self._poll_snmp_up_button, daemon=True)
                self._snmp_up_polling_thread.start()
            else:
                print("[WARN] SNMP UP button initialized but callback not set")
    
    def _poll_snmp_down_button(self):
        """Poll SNMP DOWN button state when edge detection is not available."""
        import RPi.GPIO as GPIO
        last_state = GPIO.HIGH
        last_press_time = 0
        
        while True:
            try:
                current_state = GPIO.input(self.snmp_down_pin)
                
                # Detect button press (transition from HIGH to LOW)
                if last_state == GPIO.HIGH and current_state == GPIO.LOW:
                    current_time = time.time()
                    if current_time - last_press_time > DEBOUNCE_TIME:
                        if not self._busy:
                            print("[EVENT] SNMP DOWN button press detected (polling mode)")
                            self._on_snmp_down_pressed()
                            last_press_time = current_time
                
                last_state = current_state
                time.sleep(0.05)  # Poll every 50ms
                
            except Exception as e:
                print(f"[ERROR] Error in SNMP DOWN button polling: {e}")
                time.sleep(0.1)
    
    def _poll_snmp_up_button(self):
        """Poll SNMP UP button state when edge detection is not available."""
        import RPi.GPIO as GPIO
        last_state = GPIO.HIGH
        last_press_time = 0
        
        while True:
            try:
                current_state = GPIO.input(self.snmp_up_pin)
                
                # Detect button press (transition from HIGH to LOW)
                if last_state == GPIO.HIGH and current_state == GPIO.LOW:
                    current_time = time.time()
                    if current_time - last_press_time > DEBOUNCE_TIME:
                        if not self._busy:
                            print("[EVENT] SNMP UP button press detected (polling mode)")
                            self._on_snmp_up_pressed()
                            last_press_time = current_time
                
                last_state = current_state
                time.sleep(0.05)  # Poll every 50ms
                
            except Exception as e:
                print(f"[ERROR] Error in SNMP UP button polling: {e}")
                time.sleep(0.1)
    
    def _init_button(self, pin: int, name: str):
        """Helper to initialize a button with retry logic."""
        max_retries = 3
        button_initialized = False
        use_polling = False
        
        for attempt in range(max_retries):
            try:
                print(f"[INFO] Initializing {name} button on GPIO {pin} (attempt {attempt + 1}/{max_retries})...")
                
                if self._gpio_cleanup_needed:
                    try:
                        import RPi.GPIO as GPIO
                        GPIO.setmode(GPIO.BCM)
                        GPIO.setwarnings(False)
                        try:
                            GPIO.remove_event_detect(pin)
                        except:
                            pass
                        try:
                            GPIO.cleanup(pin)
                        except:
                            pass
                        time.sleep(0.2)
                        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                        time.sleep(0.1)
                    except Exception as gpio_err:
                        print(f"[WARN] GPIO setup warning for {name}: {gpio_err}")
                
                if attempt == max_retries - 1:
                    button = Button(pin, pull_up=True)
                else:
                    button = Button(pin, pull_up=True, bounce_time=DEBOUNCE_TIME)
                
                print(f"[INFO] {name} button initialized successfully on GPIO {pin}")
                button_initialized = True
                return button
                
            except RuntimeError as e:
                if "Failed to add edge detection" in str(e):
                    if attempt < max_retries - 1:
                        try:
                            import RPi.GPIO as GPIO
                            GPIO.cleanup(pin)
                            time.sleep(0.3)
                        except:
                            pass
                        continue
                    else:
                        print(f"[WARN] Edge detection failed for {name} button, using polling mode")
                        use_polling = True
                        break
                else:
                    print(f"[WARN] Failed to initialize {name} button: {e}")
                    return None
            except Exception as e:
                print(f"[WARN] Unexpected error initializing {name} button: {e}")
                return None
        
        # Polling mode fallback
        if use_polling:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                print(f"[INFO] {name} button initialized in polling mode on GPIO {pin}")
                # Return a special marker for polling mode
                return {"pin": pin, "name": name, "polling": True}
            except Exception as e:
                print(f"[WARN] Failed to initialize {name} button even in polling mode: {e}")
                return None
        
        return None
    
    def _on_snmp_down_pressed(self):
        """Handle SNMP port DOWN button press."""
        if self._busy:
            print("[WARN] SNMP DOWN button pressed while operation in progress — ignoring.")
            return
        print("[EVENT] SNMP DOWN button press detected.")
        t = threading.Thread(target=self._send_snmp_port_down, daemon=True)
        t.start()
    
    def _on_snmp_up_pressed(self):
        """Handle SNMP port UP button press."""
        if self._busy:
            print("[WARN] SNMP UP button pressed while operation in progress — ignoring.")
            return
        print("[EVENT] SNMP UP button press detected.")
        t = threading.Thread(target=self._send_snmp_port_up, daemon=True)
        t.start()
    
    def _send_snmp_port_down(self):
        """Send SNMP command to bring port DOWN."""
        self._busy = True
        if self.led:
            self.led.on()
        
        try:
            oid = f"{SNMP_OID_BASE}.{SNMP_IFINDEX}"
            cmd = [
                "snmpset",
                "-v2c",
                "-c", SNMP_COMMUNITY,
                SNMP_TARGET,
                oid,
                "i", "2"  # 2 = down
            ]
            
            print(f"[INFO] Sending SNMP port DOWN command...")
            print(f"[CMD] {' '.join(cmd)}")
            
            start_time = time.time()
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            duration = time.time() - start_time
            
            if result.returncode == 0:
                print(f"[INFO] SNMP port DOWN command sent successfully in {duration:.2f}s")
                if result.stdout:
                    print(f"[STDOUT] {result.stdout.strip()}")
            else:
                print(f"[ERROR] SNMP command failed with return code {result.returncode}")
                if result.stderr:
                    print(f"[STDERR] {result.stderr.strip()}")
                if result.stdout:
                    print(f"[STDOUT] {result.stdout.strip()}")
                    
        except subprocess.TimeoutExpired:
            print("[ERROR] SNMP command timed out")
        except FileNotFoundError:
            print("[ERROR] snmpset command not found. Install with: sudo apt install snmp")
        except Exception as e:
            print(f"[ERROR] Failed to send SNMP port DOWN command: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if self.led:
                self.led.off()
            self._busy = False
            print("[INFO] Ready for next button press.")
    
    def _send_snmp_port_up(self):
        """Send SNMP command to bring port UP."""
        self._busy = True
        if self.led:
            self.led.on()
        
        try:
            oid = f"{SNMP_OID_BASE}.{SNMP_IFINDEX}"
            cmd = [
                "snmpset",
                "-v2c",
                "-c", SNMP_COMMUNITY,
                SNMP_TARGET,
                oid,
                "i", "1"  # 1 = up
            ]
            
            print(f"[INFO] Sending SNMP port UP command...")
            print(f"[CMD] {' '.join(cmd)}")
            
            start_time = time.time()
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            duration = time.time() - start_time
            
            if result.returncode == 0:
                print(f"[INFO] SNMP port UP command sent successfully in {duration:.2f}s")
                if result.stdout:
                    print(f"[STDOUT] {result.stdout.strip()}")
            else:
                print(f"[ERROR] SNMP command failed with return code {result.returncode}")
                if result.stderr:
                    print(f"[STDERR] {result.stderr.strip()}")
                if result.stdout:
                    print(f"[STDOUT] {result.stdout.strip()}")
                    
        except subprocess.TimeoutExpired:
            print("[ERROR] SNMP command timed out")
        except FileNotFoundError:
            print("[ERROR] snmpset command not found. Install with: sudo apt install snmp")
        except Exception as e:
            print(f"[ERROR] Failed to send SNMP port UP command: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if self.led:
                self.led.off()
            self._busy = False
            print("[INFO] Ready for next button press.")
    
    def _get_source_ip(self):

        """Get source IP address from interface or use configured static IP."""

        if SOURCE_IP:

            print(f"[INFO] Using configured source IP: {SOURCE_IP}")

            return SOURCE_IP

        try:

            # Try scapy's get_if_addr first

            ip = get_if_addr(NETWORK_INTERFACE)

            if ip and ip != "0.0.0.0":

                print(f"[INFO] Auto-detected source IP: {ip}")

                return ip

        except Exception as e:

            print(f"[WARN] Could not get IP from scapy: {e}")

        # Fallback: use socket to get IP that would be used to reach target

        try:

            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            s.connect((TARGET_IP, 80))

            ip = s.getsockname()[0]

            s.close()

            print(f"[INFO] Auto-detected source IP (via socket): {ip}")

            return ip

        except Exception as e:

            print(f"[ERROR] Failed to get source IP: {e}")

            raise

    def _resolve_target_mac(self, target_ip: str):

        """Resolve target MAC address via ARP. Ping first to populate ARP cache if needed."""

        try:

            # First, check ARP cache

            print(f"[INFO] Checking ARP cache for {target_ip}...")

            try:

                with open("/proc/net/arp", "r") as f:

                    for line in f:

                        parts = line.split()

                        if len(parts) >= 4 and parts[0] == target_ip:

                            mac = parts[3]

                            if mac != "00:00:00:00:00:00" and mac != "<incomplete>":

                                print(f"[INFO] Found MAC in ARP cache: {mac}")

                                return mac

            except Exception as e:

                print(f"[WARN] Could not read ARP cache: {e}")

            # If not in cache, ping to populate it

            print(f"[INFO] Pinging {target_ip} to populate ARP cache...")

            result = subprocess.run(

                ["ping", "-c", "1", "-W", "1", target_ip],

                capture_output=True,

                timeout=3

            )

            # Small delay to ensure ARP entry is populated

            time.sleep(0.5)

            # Check ARP cache again

            try:

                with open("/proc/net/arp", "r") as f:

                    for line in f:

                        parts = line.split()

                        if len(parts) >= 4 and parts[0] == target_ip:

                            mac = parts[3]

                            if mac != "00:00:00:00:00:00" and mac != "<incomplete>":

                                print(f"[INFO] Resolved MAC after ping: {mac}")

                                return mac

            except Exception as e:

                print(f"[WARN] Could not read ARP cache after ping: {e}")

            # If still not found, use scapy ARP request

            print(f"[INFO] Sending ARP request for {target_ip}...")

            arp_request = ARP(pdst=target_ip)

            broadcast = Ether(dst="ff:ff:ff:ff:ff:ff")

            arp_request_broadcast = broadcast / arp_request

            result, unanswered = srp(arp_request_broadcast, iface=NETWORK_INTERFACE, timeout=2, verbose=0)

            if result:

                for sent, received in result:

                    mac = received.hwsrc

                    print(f"[INFO] Resolved MAC via ARP request: {mac}")

                    return mac

            # Fallback: use broadcast MAC if resolution fails

            print(f"[WARN] Could not resolve MAC for {target_ip}, using broadcast")

            return "ff:ff:ff:ff:ff:ff"

        except Exception as e:

            print(f"[WARN] MAC resolution failed: {e}, using broadcast")

            return "ff:ff:ff:ff:ff:ff"

    def _on_button_pressed(self):

        if self._busy:

            print("[WARN] Button pressed while send already in progress — ignoring.")

            return

        print("[EVENT] Button press detected, generating and sending EICAR packet.")

        t = threading.Thread(target=self._send_eicar_packet, daemon=True)

        t.start()

    def _send_eicar_packet(self):

        self._busy = True

        if self.led:
            self.led.on()

        try:

            # Resolve target MAC

            target_mac = self._resolve_target_mac(TARGET_IP)

            # Build the EICAR TCP packet

            print(f"[INFO] Building EICAR TCP packet...")

            print(f"[INFO] Source: {self.source_ip}:{TCP_PORT} -> {TARGET_IP}:{TCP_PORT}")

            # Create Ethernet frame

            ether = Ether(src=self.source_mac, dst=target_mac)

            # Create IP packet

            ip_pkt = IP(src=self.source_ip, dst=TARGET_IP)

            # Create TCP segment with EICAR payload

            tcp_pkt = TCP(sport=TCP_PORT, dport=TCP_PORT, flags="PA") / EICAR_STRING

            # Assemble full packet

            packet = ether / ip_pkt / tcp_pkt

            print(f"[INFO] Packet built. Sending on {NETWORK_INTERFACE}...")

            start_time = time.time()

            # Send the packet

            sendp(packet, iface=NETWORK_INTERFACE, verbose=1)

            duration = time.time() - start_time

            print(f"[INFO] EICAR packet sent successfully in {duration:.2f}s")

            print(f"[INFO] EICAR string: {EICAR_STRING.decode('utf-8', errors='ignore')}")

        except Exception as e:

            print(f"[ERROR] Failed to send EICAR packet: {e}")

            import traceback

            traceback.print_exc()

        finally:

            if self.led:
                self.led.off()

            self._busy = False

            print("[INFO] Ready for next button press.")

def main():

    controller = MarineTecController(

        eicar_button_pin=BUTTON_PIN,
        snmp_down_pin=SNMP_DOWN_PIN,
        snmp_up_pin=SNMP_UP_PIN,
        led_pin=LED_PIN,

    )

    print("[INFO] MarineTec-China Controller running. Press Ctrl+C to exit.")
    print("[INFO] Buttons:")
    print(f"[INFO]   GPIO {BUTTON_PIN} - EICAR packet")
    print(f"[INFO]   GPIO {SNMP_DOWN_PIN} - SNMP port DOWN")
    print(f"[INFO]   GPIO {SNMP_UP_PIN} - SNMP port UP")

    try:

        # Just keep the main thread alive

        while True:

            time.sleep(1)

    except KeyboardInterrupt:

        print("\n[INFO] Exiting on user request.")
        
    finally:
        # Cleanup GPIO on exit
        if hasattr(controller, '_gpio_cleanup_needed') and controller._gpio_cleanup_needed:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)  # Set mode before cleanup
                # Cleanup pins if in polling mode
                if hasattr(controller, 'eicar_button_polling') and controller.eicar_button_polling:
                    try:
                        GPIO.cleanup(controller.eicar_button_pin)
                    except:
                        pass
                if hasattr(controller, 'snmp_down_button') and isinstance(controller.snmp_down_button, dict):
                    try:
                        GPIO.cleanup(controller.snmp_down_pin)
                    except:
                        pass
                if hasattr(controller, 'snmp_up_button') and isinstance(controller.snmp_up_button, dict):
                    try:
                        GPIO.cleanup(controller.snmp_up_pin)
                    except:
                        pass
                GPIO.cleanup()
                print("[INFO] GPIO cleaned up.")
            except Exception as e:
                print(f"[WARN] Error during GPIO cleanup: {e}")
                pass

if __name__ == "__main__":

    main()


