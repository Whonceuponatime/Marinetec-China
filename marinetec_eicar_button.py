#!/usr/bin/env python3

import os
import sys

if os.geteuid() != 0:
    print("[ERROR] This script must be run with root privileges (sudo)")
    print("[ERROR] Raw packet sending requires root access")
    print("[ERROR] Please run: sudo python3 marinetec_eicar_button.py")
    sys.exit(1)

from gpiozero import Button, LED
import subprocess
import threading
import time
import socket
import struct
from scapy.all import get_if_addr, get_if_hwaddr, ARP, Ether, IP, TCP, Raw, sendp, srp, conf, TCPOptions

NETWORK_INTERFACE = "eth0"
TARGET_IP = "192.168.127.15"
SOURCE_IP = "192.168.127.25"
TCP_PORT = 80
SCAN_PORTS = True
COMMON_PORTS = [80, 8080, 8000, 3000, 5000, 21, 23, 25, 53, 110, 143, 993, 995, 3389, 5900]
BUTTON_PIN = 17
LED_PIN = 22
SNMP_DOWN_PIN = 27
SNMP_UP_PIN = 22
DEBOUNCE_TIME = 0.2
SNMP_TARGET = "192.168.127.10"
SNMP_PORT = 161
SNMP_COMMUNITY = "private"
SNMP_OID_BASE = "1.3.6.1.2.1.2.2.1.7"
SNMP_IFINDEX = 8
EICAR_STRING = b"X5O!P%@AP[4\\PZX54(P^)7CC)7}$EICAR-STANDARD-ANTIVIRUS-TEST-FILE!$H+H*"

class MarineTecController:
    def __init__(self, eicar_button_pin: int, snmp_down_pin: int, snmp_up_pin: int, led_pin: int = None):
        self._gpio_cleanup_needed = False
        try:
            import RPi
            import RPi.GPIO as GPIO
            from gpiozero.pins.rpigpio import RPiGPIOFactory
            from gpiozero import Device
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            pins_to_cleanup = [eicar_button_pin, snmp_down_pin, snmp_up_pin]
            if led_pin and led_pin != snmp_up_pin:
                pins_to_cleanup.append(led_pin)
            for pin in pins_to_cleanup:
                try:
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
                    GPIO.cleanup(pin)
                except:
                    pass
            try:
                GPIO.cleanup()
            except:
                pass
            time.sleep(0.2)
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
        self.eicar_button_pin = eicar_button_pin
        self.snmp_down_pin = snmp_down_pin
        self.snmp_up_pin = snmp_up_pin
        self._busy = False
        self.source_ip = self._get_source_ip()
        self.source_mac = get_if_hwaddr(NETWORK_INTERFACE)
        print("[INFO] Initializing EICAR button...")
        self.button = self._init_button(eicar_button_pin, "EICAR")
        if self.button and hasattr(self.button, 'when_pressed'):
            self.button.when_pressed = self._on_button_pressed
        elif self.button and isinstance(self.button, dict) and self.button.get('polling'):
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
        import RPi.GPIO as GPIO
        last_state = self.eicar_button_state
        last_press_time = 0
        while True:
            try:
                current_state = GPIO.input(self.eicar_button_pin)
                if last_state == GPIO.HIGH and current_state == GPIO.LOW:
                    current_time = time.time()
                    if current_time - last_press_time > DEBOUNCE_TIME:
                        if not self._busy:
                            print("[EVENT] EICAR button press detected (polling mode)")
                            self._on_button_pressed()
                            last_press_time = current_time
                self.eicar_button_state = current_state
                last_state = current_state
                time.sleep(0.05)
            except Exception as e:
                print(f"[ERROR] Error in EICAR button polling: {e}")
                time.sleep(0.1)
    
    def _init_snmp_buttons(self):
        print("[INFO] Initializing SNMP buttons...")
        self.snmp_down_button = self._init_button(self.snmp_down_pin, "SNMP DOWN")
        if self.snmp_down_button:
            if hasattr(self.snmp_down_button, 'when_pressed'):
                self.snmp_down_button.when_pressed = self._on_snmp_down_pressed
            elif isinstance(self.snmp_down_button, dict) and self.snmp_down_button.get('polling'):
                print("[INFO] Starting SNMP DOWN button polling thread...")
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                self._snmp_down_polling_thread = threading.Thread(target=self._poll_snmp_down_button, daemon=True)
                self._snmp_down_polling_thread.start()
            else:
                print("[WARN] SNMP DOWN button initialized but callback not set")
        self.snmp_up_button = self._init_button(self.snmp_up_pin, "SNMP UP")
        if self.snmp_up_button:
            if hasattr(self.snmp_up_button, 'when_pressed'):
                self.snmp_up_button.when_pressed = self._on_snmp_up_pressed
            elif isinstance(self.snmp_up_button, dict) and self.snmp_up_button.get('polling'):
                print("[INFO] Starting SNMP UP button polling thread...")
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                self._snmp_up_polling_thread = threading.Thread(target=self._poll_snmp_up_button, daemon=True)
                self._snmp_up_polling_thread.start()
            else:
                print("[WARN] SNMP UP button initialized but callback not set")
    
    def _poll_snmp_down_button(self):
        import RPi.GPIO as GPIO
        last_state = GPIO.HIGH
        last_press_time = 0
        while True:
            try:
                current_state = GPIO.input(self.snmp_down_pin)
                if last_state == GPIO.HIGH and current_state == GPIO.LOW:
                    current_time = time.time()
                    if current_time - last_press_time > DEBOUNCE_TIME:
                        if not self._busy:
                            print("[EVENT] SNMP DOWN button press detected (polling mode)")
                            self._on_snmp_down_pressed()
                            last_press_time = current_time
                last_state = current_state
                time.sleep(0.05)
            except Exception as e:
                print(f"[ERROR] Error in SNMP DOWN button polling: {e}")
                time.sleep(0.1)
    
    def _poll_snmp_up_button(self):
        import RPi.GPIO as GPIO
        last_state = GPIO.HIGH
        last_press_time = 0
        while True:
            try:
                current_state = GPIO.input(self.snmp_up_pin)
                if last_state == GPIO.HIGH and current_state == GPIO.LOW:
                    current_time = time.time()
                    if current_time - last_press_time > DEBOUNCE_TIME:
                        if not self._busy:
                            print("[EVENT] SNMP UP button press detected (polling mode)")
                            self._on_snmp_up_pressed()
                            last_press_time = current_time
                last_state = current_state
                time.sleep(0.05)
            except Exception as e:
                print(f"[ERROR] Error in SNMP UP button polling: {e}")
                time.sleep(0.1)
    
    def _init_button(self, pin: int, name: str):
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
        if use_polling:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                print(f"[INFO] {name} button initialized in polling mode on GPIO {pin}")
                return {"pin": pin, "name": name, "polling": True}
            except Exception as e:
                print(f"[WARN] Failed to initialize {name} button even in polling mode: {e}")
                return None
        return None
    
    def _on_snmp_down_pressed(self):
        if self._busy:
            print("[WARN] SNMP DOWN button pressed while operation in progress — ignoring.")
            return
        print("[EVENT] SNMP DOWN button press detected.")
        t = threading.Thread(target=self._send_snmp_port_down, daemon=True)
        t.start()
    
    def _on_snmp_up_pressed(self):
        if self._busy:
            print("[WARN] SNMP UP button pressed while operation in progress — ignoring.")
            return
        print("[EVENT] SNMP UP button press detected.")
        t = threading.Thread(target=self._send_snmp_port_up, daemon=True)
        t.start()
    
    def _send_snmp_port_down(self):
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
                "i", "2"
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
                "i", "1"
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
        if SOURCE_IP:
            print(f"[INFO] Using configured source IP: {SOURCE_IP}")
            return SOURCE_IP
        try:
            ip = get_if_addr(NETWORK_INTERFACE)
            if ip and ip != "0.0.0.0":
                print(f"[INFO] Auto-detected source IP: {ip}")
                return ip
        except Exception as e:
            print(f"[WARN] Could not get IP from scapy: {e}")
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

    def _scan_open_port(self, target_ip: str, timeout: float = 0.3):
        if not SCAN_PORTS:
            print(f"[INFO] Port scanning disabled, using default port {TCP_PORT}")
            if TCP_PORT == 22:
                print(f"[ERROR] Default port is 22 (SSH) - this should never happen!")
            return TCP_PORT
        print(f"[INFO] Scanning for open TCP ports on {target_ip} (excluding encrypted ports 22, 443, 445)...")
        import socket
        encrypted_ports = [22, 443, 445]
        if 22 in COMMON_PORTS:
            print(f"[ERROR] Port 22 found in COMMON_PORTS - removing it!")
            COMMON_PORTS.remove(22)
        scanned_count = 0
        for port in COMMON_PORTS:
            if port in encrypted_ports:
                print(f"[DEBUG] Skipping encrypted/SMB port: {port}")
                continue
            if port == 22:
                print(f"[ERROR] Port 22 detected in scan loop - skipping!")
                continue
            scanned_count += 1
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(timeout)
                result = sock.connect_ex((target_ip, port))
                sock.close()
                if result == 0:
                    if port == 22:
                        print(f"[ERROR] Port 22 was found as open - this should never happen!")
                        continue
                    print(f"[INFO] Found open port: {port} (plain TCP) - will use this port")
                    print(f"[INFO] VERIFIED: Port {port} is NOT 22, 443, or 445")
                    return port
                else:
                    print(f"[DEBUG] Port {port} is closed or filtered (Result: {result})")
            except Exception as e:
                print(f"[DEBUG] Error scanning port {port}: {e}")
                continue
        print(f"[INFO] Scanned {scanned_count} ports, none were open")
        print(f"[WARN] No open plain TCP ports found, using default port {TCP_PORT}")
        if TCP_PORT == 22:
            print(f"[ERROR] Default port is 22 (SSH) - changing to 80!")
            return 80
        return TCP_PORT
    
    def _resolve_target_mac(self, target_ip: str):
        try:
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
            print(f"[INFO] Pinging {target_ip} to populate ARP cache...")
            result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", target_ip],
                capture_output=True,
                timeout=3
            )
            time.sleep(0.5)
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
            target_port = self._scan_open_port(TARGET_IP)
            if target_port in [22, 443, 445]:
                print(f"[ERROR] Scanned port {target_port} is encrypted - this should never happen!")
                print(f"[ERROR] Falling back to port 80")
                target_port = 80
            print(f"[INFO] Using open TCP port {target_port} for EICAR packet (NOT encrypted)")
            import random
            source_port = random.randint(49152, 65535)
            while source_port in [22, 443, 445]:
                source_port = random.randint(49152, 65535)
            print(f"[INFO] Building plain TCP EICAR packet...")
            print(f"[INFO] Source: {self.source_ip}:{source_port} → Destination: {TARGET_IP}:{target_port}")
            print(f"[INFO] TCP Flags: FIN, PSH, ACK | Seq=1, Ack=1, Win=509")
            print(f"[INFO] Port {target_port} is confirmed open and NOT encrypted (not 22, 443, or 445)")
            target_mac = self._resolve_target_mac(TARGET_IP)
            payload_bytes = EICAR_STRING
            print(f"[INFO] EICAR payload: {len(payload_bytes)} bytes")
            print(f"[INFO] Payload (plain text): {payload_bytes.decode('utf-8', errors='ignore')[:50]}...")
            ether = Ether(src=self.source_mac, dst=target_mac)
            ip_pkt = IP(src=self.source_ip, dst=TARGET_IP, ttl=64, flags="DF")
            tcp_options = [
                (1, None),
                (1, None),
                (8, (3267583673, 4272320793))
            ]
            tcp_pkt = TCP(
                sport=source_port,
                dport=target_port,
                flags="FPA",
                seq=1,
                ack=1,
                window=509,
                options=tcp_options
            ) / Raw(load=payload_bytes)
            print(f"[INFO] TCP packet length: {len(tcp_pkt)} bytes")
            packet = ether / ip_pkt / tcp_pkt
            print(f"[INFO] Packet structure: Ethernet -> IP -> TCP -> Raw Payload (PLAIN TEXT)")
            print(f"[INFO] TCP Timestamp: TSval=3267583673, TSecr=4272320793")
            print(f"[INFO] Configuring scapy for raw Layer 2 sending (bypasses OS TCP stack)...")
            conf.iface = NETWORK_INTERFACE
            conf.checkIPaddr = False
            conf.verb = 0
            conf.use_pcap = False
            print(f"[INFO] Sending raw Layer 2 packet (Ethernet frame) on interface {NETWORK_INTERFACE}...")
            print(f"[INFO] This bypasses the OS TCP/IP stack completely - packet is sent as-is")
            start_time = time.time()
            sendp(packet, iface=NETWORK_INTERFACE, verbose=0, realtime=False)
            duration = time.time() - start_time
            print(f"[INFO] EICAR packet sent successfully in {duration:.2f}s")
            print(f"[INFO] Plain TCP packet: {source_port} → {target_port} [FIN, PSH, ACK] Seq=1 Ack=1 Win=509")
            print(f"[INFO] Wireshark filter: tcp.port == {target_port} and ip.addr == {TARGET_IP}")
            print(f"[INFO] Expected: Plain TCP packet with EICAR payload visible in Wireshark")
        except PermissionError as e:
            print(f"[ERROR] Permission denied: Raw packet sending requires root privileges")
            print(f"[ERROR] Please run the script with: sudo python3 marinetec_eicar_button.py")
            print(f"[ERROR] Details: {e}")
        except OSError as e:
            if "Permission denied" in str(e) or "Operation not permitted" in str(e):
                print(f"[ERROR] Permission denied: Raw packet sending requires root privileges")
                print(f"[ERROR] Please run the script with: sudo python3 marinetec_eicar_button.py")
            else:
                print(f"[ERROR] Network error: {e}")
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
    print("                           _______                   __    __                               ")
    print("                          |       \\                 |  \\  |  \\                              ")
    print("  ______    ______        | $$$$$$$\\  ______        | $$  | $$  ______   _______    ______  ")
    print(" /      \\  /      \\       | $$  | $$ /      \\       | $$__| $$ /      \\ |       \\  /      \\ ")
    print("|  $$$$$$\\|  $$$$$$\\      | $$  | $$|  $$$$$$\\      | $$    $$|  $$$$$$\\| $$$$$$$\\|  $$$$$$\\")
    print("| $$  | $$| $$  | $$      | $$  | $$| $$   \\$$      | $$$$$$$$| $$  | $$| $$  | $$| $$  | $$")
    print("| $$__| $$| $$__/ $$      | $$__/ $$| $$            | $$  | $$| $$__/ $$| $$  | $$| $$__| $$")
    print(" \\$$    $$ \\$$    $$      | $$    $$| $$            | $$  | $$ \\$$    $$| $$  | $$ \\$$    $$")
    print(" _\\$$$$$$$  \\$$$$$$        \\$$$$$$$  \\$$             \\$$   \\$$  \\$$$$$$  \\$$   \\$$ _\\$$$$$$$")
    print("|  \\__| $$                                                                        |  \\__| $$")
    print(" \\$$    $$                                                                         \\$$    $$")
    print("  \\$$$$$$                                                                           \\$$$$$$ ")
    print("")
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
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Exiting on user request.")
    finally:
        if hasattr(controller, '_gpio_cleanup_needed') and controller._gpio_cleanup_needed:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
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
