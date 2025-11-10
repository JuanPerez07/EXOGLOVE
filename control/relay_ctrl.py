#!/usr/bin/env python3
"""
Relay control using gpiozero on Raspberry Pi.
Supports active-low relay modules (most common).
"""

from gpiozero import OutputDevice
from signal import pause
import sys
import time

# === CONFIGURATION ===
RELAY_PIN = 17  # BCM pin number (change to your wiring)
ACTIVE_HIGH = False  # Most relay boards are active-low

try:
    # Create relay object
    relay = OutputDevice(RELAY_PIN, active_high=ACTIVE_HIGH, initial_value=False)
except Exception as e:
    print(f"Error initializing relay on GPIO {RELAY_PIN}: {e}")
    sys.exit(1)

def turn_on():
    """Turn the relay ON."""
    relay.on()
    print("Relay ON")

def turn_off():
    """Turn the relay OFF."""
    relay.off()
    print("Relay OFF")

def toggle():
    """Toggle relay state."""
    if relay.value == 0:
        turn_on()
    else:
        turn_off()

if __name__ == "__main__":
    try:
        print("Relay control started. Commands:")
        print("  1 - ON")
        print("  0 - OFF")
        print("  t - TOGGLE")
        print("  q - QUIT")

        while True:
            cmd = input("Enter command: ").strip().lower()
            if cmd == "1":
                turn_on()
            elif cmd == "0":
                turn_off()
            elif cmd == "t":
                toggle()
            elif cmd == "q":
                break
            else:
                print("Invalid command. Use 1, 0, t, or q.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Ensure relay is OFF before exit
        turn_off()
        print("Relay control stopped.")
