import time
import json
import threading
import queue
import tkinter as tk
from tkinter import ttk

import odrive
from odrive.enums import *
from bluezero import peripheral, adapter, device

# --- Configuration Constants ---
MASTER_NAME = 'NanoESP32_BLE'
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

CALIB_TIME = 32 # secs
FREQ = 0.25  # secs
WATCHDOG_THRES = FREQ + 0.5

INTERFACE_SIZE = "1600x1000"
SP_BAR_LENGTH = 800
# --- NEW: Global variable for font sizes ---
FONT_SIZE = {
    "status": 35,
    "title": 50,
    "sp_value": 40
}
# -----------------------------------------------------------------------------
# Main Application Class
# -----------------------------------------------------------------------------
class ExogloveApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Exoglove ODrive Controller")
        self.root.geometry(INTERFACE_SIZE)

        # --- Shared State Variables ---
        self.m1 = None  # ODrive axis object
        self.last_msg = None
        self.last_msg_time = time.time()
        self.dynamic_setpoint = tk.DoubleVar(value=0.95)
        self.setpoint_lock = threading.Lock()  # Lock for thread-safe access to the setpoint

        # --- Threading and Communication ---
        self.shutdown_event = threading.Event()
        self.message_queue = queue.Queue() # For communication from background thread to GUI
        
        # --- Build the GUI ---
        self._build_gui()

        # --- Start background tasks ---
        self.background_thread = threading.Thread(target=self.background_worker)
        self.background_thread.daemon = True
        self.background_thread.start()

        # --- Start the GUI queue processor ---
        self.process_gui_queue()
        
        # --- Handle window closing ---
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _build_gui(self):
        """Creates all the Tkinter widgets."""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Status Label 
        self.status_label = ttk.Label(main_frame, text="Status: Initializing...", font=("Arial", FONT_SIZE["status"]), wraplength=380)
        self.status_label.pack(pady=10, anchor='w')

        # Setpoint Slider Title 
        ttk.Label(main_frame, text="Dynamic velocity setpoint for fingers {0 - 1.5} rev/s:", font=("Arial", FONT_SIZE["title"])).pack(pady=(10, 0))
        slider = ttk.Scale(
            main_frame,
            from_=0.0,
            to=1.5,
            orient="horizontal",
            length=SP_BAR_LENGTH,
            variable=self.dynamic_setpoint,
            command=self.update_setpoint_label
        )
        slider.pack(pady=100)

        # Setpoint Value Label 
        self.value_label = ttk.Label(main_frame, text=f"Current value = {self.dynamic_setpoint.get():.2f}", font=("Arial", FONT_SIZE["sp_value"]))
        self.value_label.pack(pady=5)
    
    def update_setpoint_label(self, val):
        """Updates the text label for the slider, called by the slider's command."""
        with self.setpoint_lock:
            current_value = self.dynamic_setpoint.get()
        self.value_label.config(text=f"Current value = {current_value:.2f}")

    def process_gui_queue(self):
        """Processes messages from the background thread to update the GUI safely."""
        try:
            message = self.message_queue.get_nowait()
            if 'status' in message:
                self.status_label.config(text=f"Status: {message['status']}")
        except queue.Empty:
            pass
        finally:
            self.root.after(100, self.process_gui_queue)

    def background_worker(self):
        """
        Runs all blocking operations: ODrive setup, watchdog, and BLE server.
        """
        try:
            self.update_gui_status("Searching for ODrive board...")
            my_drive = odrive.find_any()
            self.m1 = self.configure_odrive(my_drive)
            self.update_gui_status("ODrive connected & calibrated")
        except Exception as e:
            self.update_gui_status(f"Error ODrive: {e}")
            return

        # Start the watchdog
        watchdog_thread = threading.Thread(target=self.watchdog_task, daemon=True)
        watchdog_thread.start()

        try:
            adapter_address = list(adapter.Adapter.available())[0].address
            ble_device = peripheral.Peripheral(adapter_address, local_name=MASTER_NAME)
            
            ble_device.add_service(srv_id=1, uuid=UART_SERVICE_UUID, primary=True)
            ble_device.add_characteristic(
                srv_id=1, chr_id=1, uuid=RX_CHAR_UUID,
                value=[], notifying=False, flags=['write'],
                write_callback=self.rx_handler
            )
            ble_device.add_characteristic(
                srv_id=1, chr_id=2, uuid=TX_CHAR_UUID,
                value=[], notifying=False, flags=['notify']
            )
            
            self.update_gui_status("Awaiting master device pairing...")
            ble_device.publish()
        except IndexError:
            self.update_gui_status("Error: No Bluetooth adapter found.")
        except Exception as e:
            self.update_gui_status(f"Error BLE: {e}")

    def update_gui_status(self, text):
        """Helper to send a status update to the GUI thread."""
        self.message_queue.put({'status': text})

    def configure_odrive(self, my_drive):
        """Loads config, calibrates, and sets up the ODrive axis."""
        axis = my_drive.axis0
        with open("odrive_config.json", "r") as json_file:
            config = json.load(json_file)

        
        # Apply motor config
        motor_cfg = config["axis0"]["motor"]["config"]
        for key, value in motor_cfg.items():
            setattr(axis.motor.config, key, value)
        
        # Apply encoder config
        enc_cfg = config["axis0"]["encoder"]["config"]
        for key, value in enc_cfg.items():
            setattr(axis.encoder.config, key, value)
        axis.encoder.config.pre_calibrated = True

        # Apply controller config
        ctrl_cfg = config["axis0"]["controller"]["config"]
        axis.controller.config.pos_gain = ctrl_cfg["kp"]
        axis.controller.config.vel_gain = ctrl_cfg["kv"]
        axis.controller.config.vel_integrator_gain = ctrl_cfg["ki"]
        axis.controller.config.vel_limit = ctrl_cfg["vel_limit"]
        axis.controller.config.control_mode = ctrl_cfg["control_mode"]
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        
        my_drive.config.dc_max_negative_current = -2.0
        my_drive.config.enable_brake_resistor = False
        
        self.update_gui_status("Performing full motor calibration sequence...")
        axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        # Adjust calibration duration
        time.sleep(CALIB_TIME) 
        
        if axis.motor.error != 0 or axis.encoder.error != 0 or axis.controller.error != 0:
            raise RuntimeError(f"Calibration error! Motor_axis_err: {axis.motor.error}, Encoder_err: {axis.encoder.error}, Controller_err: {axis.controller.error}")

        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.update_gui_status("Closed loop control activated")
        return axis

    def rx_handler(self, value, options):
        """Callback for BLE messages. Runs in the BLE thread."""
        msg = value.decode("utf-8").strip()
        #print(f"--> Received: {msg}") # Translated from Spanish

        with self.setpoint_lock:
            setpoint = self.dynamic_setpoint.get()

        new_vel = 0.0
        if msg == "0001":
            new_vel = setpoint
            print(f'>>> Supination cmd sent! (vel = {setpoint:.2f})')
        elif msg == "0010":
            new_vel = -setpoint
            print(f'>>> Pronation cmd sent! (vel = {-setpoint:.2f})')
        else:
            print('>>> Motor stopped!')

        if self.m1 and self.last_msg != msg:
            self.m1.controller.input_vel = new_vel

        self.last_msg = msg
        self.last_msg_time = time.time()

    def watchdog_task(self):
        """Background task to stop the motor if no BLE messages are received."""
        while not self.shutdown_event.is_set():
            if self.last_msg_time is not None:
                elapsed = time.time() - self.last_msg_time
                if elapsed > WATCHDOG_THRES:
                    if self.m1 and self.m1.controller.input_vel != 0.0:
                        self.m1.controller.input_vel = 0.0
                        print(f"⏱️ WATCHDOG activated -> Motor stop")
                    self.last_msg_time = None 
            time.sleep(0.1)

    def on_closing(self):
        """Handles graceful shutdown when the window is closed."""
        print("Closing application...")
        self.shutdown_event.set()
        if self.m1:
            try:
                self.m1.requested_state = AXIS_STATE_IDLE
            except Exception as e:
                print(f"Could not set ODrive to idle: {e}")
        self.root.destroy()

# -----------------------------------------------------------------------------
# Main execution block
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = ExogloveApp(root)
    root.mainloop()
