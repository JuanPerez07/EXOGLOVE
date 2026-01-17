#!/usr/bin/env python3
"""
Relay class to control using gpiozero on Raspberry Pi.
Supports active-low relay modules (most common).
"""
from gpiozero import Button, OutputDevice 
from signal import pause

class RelayControl:
    # PINOUT CONFIG
    BUTTON_PIN = 3
    RELAY_PIN = 4
    
    # constructor: relay works low-active and initially is OFF
    def __init__ (self, active_high=False, initial_st=False):
        self.is_ON = not initial_st
        self.active_high = active_high
        
        # create the relay and button obj
        self.relay = OutputDevice(self.RELAY_PIN, active_high=self.active_high, initial_value=initial_st)
        self.button = Button(self.BUTTON_PIN, pull_up=True)

        # event assignment
        self.button.when_pressed = self.button_pressed
        # update status of relay
        self.toggle()
    
    def toggle(self):
        self.relay.on() if self.is_ON else self.relay.off()

    def button_pressed(self):
        self.is_ON = not self.is_ON
        self.toggle() # update relay 
    
    def set(self, st):
        self.is_ON = st
        self.toggle()    
    
    # main loop event
    def run (self):
        pause()