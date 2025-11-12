#!/usr/bin/env python3
"""
Relay control using gpiozero on Raspberry Pi.
Supports active-low relay modules (most common).
"""
from gpiozero import Button, OutputDevice 
from signal import pause
import sys
import time

# PINOUT CONFIG
BUTTON_PIN = 3
RELAY_PIN = 4
# global vars
is_ON = False # status 
ACTIVE_HIGH = False # relay is low-active

# toggle relay
def update_relay(st):
    global relay
    relay.on() if st else relay.off()

# behavoiur when button pressed
def button_pressed():
    global is_ON
    #print("Button was pressed!")
    is_ON = not is_ON
    # update relay status
    update_relay(is_ON)
    

# MAIN EXECUTION BLOCK
if __name__ == "__main__":
    global relay
    try:
        # create both relay and button objects
        relay = OutputDevice(RELAY_PIN, active_high=ACTIVE_HIGH, initial_value=True)
        button = Button(BUTTON_PIN, pull_up=True) # button wired to GND
        ## attach event handlers
        button.when_pressed = button_pressed
        
        # keep the program running for events
        pause()
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

