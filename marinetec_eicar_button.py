#!/usr/bin/env python3

"""

MarineTec-China MVP (v1) — EICAR packet replay button

Behavior:

- When the physical button is pressed:

    -> Replay the EICAR pcap on the configured interface using tcpreplay.

    -> Turn the LED ON while replaying, then OFF when done.

Hardware assumptions (BCM pin numbering):

- Button:

    - One side -> GND

    - Other side -> GPIO 27

- LED (optional, but recommended):

    - Anode (through resistor / driver) -> GPIO 22

    - Cathode -> GND

Notes:

- Run this script with sufficient privileges (sudo) so tcpreplay and GPIO work.

- Only use this in a LAB / controlled network where you have permission!

"""

from gpiozero import Button, LED

import subprocess

import threading

import time

# --------- CONFIGURABLE CONSTANTS ---------

# Path to your EICAR pcap file

EICAR_PCAP_PATH = "/opt/marinetec/EICAR_test.pcap"

# Network interface to send packets out of

NETWORK_INTERFACE = "eth0"

# GPIO pins (BCM numbering)

BUTTON_PIN = 27   # physical: pin 13

LED_PIN = 22      # physical: pin 15

# Debounce time (seconds)

DEBOUNCE_TIME = 0.2

# ------------------------------------------

class EicarButtonController:

    def __init__(self, button_pin: int, led_pin: int):

        # Button: pulled up internally, active on press (to GND)

        self.button = Button(button_pin, pull_up=True, bounce_time=DEBOUNCE_TIME)

        self.led = LED(led_pin)

        # Avoid overlapping replays

        self._busy = False

        self.button.when_pressed = self._on_button_pressed

        print(f"[INFO] EICAR button controller initialized.")

        print(f"[INFO] Watching GPIO {button_pin} for presses.")

        print(f"[INFO] EICAR pcap: {EICAR_PCAP_PATH}")

        print(f"[INFO] Interface: {NETWORK_INTERFACE}")

    def _on_button_pressed(self):

        if self._busy:

            print("[WARN] Button pressed while replay already in progress — ignoring.")

            return

        print("[EVENT] Button press detected, starting EICAR replay.")

        t = threading.Thread(target=self._run_eicar_replay, daemon=True)

        t.start()

    def _run_eicar_replay(self):

        self._busy = True

        self.led.on()

        try:

            cmd = [

                "tcpreplay",

                "--intf1",

                NETWORK_INTERFACE,

                EICAR_PCAP_PATH,

            ]

            print(f"[CMD] {' '.join(cmd)}")

            start_time = time.time()

            result = subprocess.run(cmd, capture_output=True, text=True)

            duration = time.time() - start_time

            print(f"[INFO] tcpreplay finished in {duration:.2f}s, return code={result.returncode}")

            if result.stdout:

                print("[STDOUT]")

                print(result.stdout)

            if result.stderr:

                print("[STDERR]")

                print(result.stderr)

        except Exception as e:

            print(f"[ERROR] Failed to run tcpreplay: {e}")

        finally:

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

if __name__ == "__main__":

    main()


