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

    - Other side -> GPIO 27

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

BUTTON_PIN = 27   # physical: pin 13

LED_PIN = 22      # physical: pin 15

# Debounce time (seconds)

DEBOUNCE_TIME = 0.2

# EICAR test string (standard antivirus test pattern)

EICAR_STRING = b"X5O!P%@AP[4\\PZX54(P^)7CC)7}$EICAR-STANDARD-ANTIVIRUS-TEST-FILE!$H+H*"

# ------------------------------------------

class EicarButtonController:

    def __init__(self, button_pin: int, led_pin: int):

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
            
            # Force cleanup of specific pins
            try:
                GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
                GPIO.cleanup(button_pin)
            except:
                pass
            try:
                GPIO.setup(led_pin, GPIO.OUT)
                GPIO.cleanup(led_pin)
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

        # Initialize LED first (simpler, less likely to conflict)
        try:
            self.led = LED(led_pin)
            print(f"[INFO] LED initialized on GPIO {led_pin}")
        except Exception as e:
            print(f"[WARN] Failed to initialize LED on GPIO {led_pin}: {e}")
            print("[WARN] Continuing without LED...")
            self.led = None

        # Button: pulled up internally, active on press (to GND)
        # Add small delay to ensure GPIO is ready
        time.sleep(0.2)
        
        # Try to initialize button with retry logic
        max_retries = 3
        button_initialized = False
        use_polling = False
        
        for attempt in range(max_retries):
            try:
                print(f"[INFO] Attempting to initialize button on GPIO {button_pin} (attempt {attempt + 1}/{max_retries})...")
                
                # Try with explicit pin factory reset
                if self._gpio_cleanup_needed:
                    try:
                        import RPi.GPIO as GPIO
                        GPIO.setmode(GPIO.BCM)
                        GPIO.setwarnings(False)
                        # Remove any existing event detection
                        try:
                            GPIO.remove_event_detect(button_pin)
                        except:
                            pass
                        # Cleanup the pin
                        try:
                            GPIO.cleanup(button_pin)
                        except:
                            pass
                        time.sleep(0.2)
                        # Ensure pin is in input mode with pull-up before Button tries to use it
                        GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                        time.sleep(0.1)
                    except Exception as gpio_err:
                        print(f"[WARN] GPIO setup warning: {gpio_err}")
                
                # Try creating button - on last attempt, try without bounce_time
                if attempt == max_retries - 1:
                    print(f"[INFO] Final attempt: trying without bounce_time...")
                    self.button = Button(button_pin, pull_up=True)
                else:
                    self.button = Button(button_pin, pull_up=True, bounce_time=DEBOUNCE_TIME)
                
                print(f"[INFO] Button initialized successfully on GPIO {button_pin}")
                button_initialized = True
                break
                
            except RuntimeError as e:
                if "Failed to add edge detection" in str(e):
                    print(f"[WARN] Edge detection failed on attempt {attempt + 1}")
                    if attempt < max_retries - 1:
                        # Try cleanup and retry
                        try:
                            import RPi.GPIO as GPIO
                            GPIO.cleanup(button_pin)
                            time.sleep(0.3)
                        except:
                            pass
                        continue
                    else:
                        # All attempts failed - use polling fallback
                        print(f"[WARN] Edge detection failed after {max_retries} attempts")
                        print(f"[INFO] Falling back to polling mode (less efficient but should work)")
                        use_polling = True
                        break
                else:
                    print(f"[ERROR] Failed to initialize button: {e}")
                    raise
            except Exception as e:
                print(f"[ERROR] Unexpected error initializing button: {e}")
                raise
        
        # If edge detection failed, use polling fallback
        if use_polling:
            print(f"[INFO] Initializing button in polling mode...")
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                self.button_pin = button_pin
                self.button_state = GPIO.input(button_pin)
                self.button = None  # No gpiozero Button object
                print(f"[INFO] Button initialized in polling mode on GPIO {button_pin}")
                print(f"[INFO] Current button state: {'PRESSED' if not self.button_state else 'RELEASED'}")
            except Exception as e:
                print(f"[ERROR] Failed to initialize button even in polling mode: {e}")
                raise
        elif not button_initialized:
            raise RuntimeError("Failed to initialize button after all retry attempts")

        # Avoid overlapping sends

        self._busy = False
        
        # Set up button callback or polling thread
        if self.button is not None:
            # Normal mode: use gpiozero Button with edge detection
            self.button.when_pressed = self._on_button_pressed
        else:
            # Polling mode: start polling thread
            print("[INFO] Starting button polling thread...")
            self._polling_thread = threading.Thread(target=self._poll_button, daemon=True)
            self._polling_thread.start()

        # Network configuration

        self.source_ip = self._get_source_ip()

        self.source_mac = get_if_hwaddr(NETWORK_INTERFACE)

        print(f"[INFO] EICAR button controller initialized.")

        print(f"[INFO] Watching GPIO {button_pin} for presses.")

        print(f"[INFO] Interface: {NETWORK_INTERFACE}")

        print(f"[INFO] Source IP: {self.source_ip}")

        print(f"[INFO] Source MAC: {self.source_mac}")

        print(f"[INFO] Target IP: {TARGET_IP}")

    def _poll_button(self):
        """Poll button state when edge detection is not available."""
        import RPi.GPIO as GPIO
        last_state = self.button_state
        last_press_time = 0
        
        while True:
            try:
                current_state = GPIO.input(self.button_pin)
                
                # Detect button press (transition from HIGH to LOW)
                if last_state == GPIO.HIGH and current_state == GPIO.LOW:
                    current_time = time.time()
                    # Debounce: ignore presses within DEBOUNCE_TIME of last press
                    if current_time - last_press_time > DEBOUNCE_TIME:
                        if not self._busy:
                            print("[EVENT] Button press detected (polling mode)")
                            self._on_button_pressed()
                            last_press_time = current_time
                
                self.button_state = current_state
                last_state = current_state
                time.sleep(0.05)  # Poll every 50ms
                
            except Exception as e:
                print(f"[ERROR] Error in button polling: {e}")
                time.sleep(0.1)
    
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

    controller = EicarButtonController(

        button_pin=BUTTON_PIN,

        led_pin=LED_PIN,

    )

    print("[INFO] MarineTec-China EICAR MVP running. Press Ctrl+C to exit.")

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
                # Cleanup button pin if in polling mode
                if hasattr(controller, 'button_pin') and controller.button is None:
                    try:
                        GPIO.cleanup(controller.button_pin)
                    except:
                        pass
                GPIO.cleanup()
                print("[INFO] GPIO cleaned up.")
            except:
                pass

if __name__ == "__main__":

    main()


