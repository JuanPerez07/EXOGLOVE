import time
import json
import threading
import queue
import tkinter as tk
from tkinter import ttk

import odrive
from odrive.enums import *
from bluezero import peripheral, adapter, device
from relay_ctrl import RelayControl # to supply power to the ODrive board
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
SCALE = 0.40
# --- NEW: Global variable for font sizes ---
FONT_SIZE = {
    "status": 12, #*SCALE),
    "title": 25, #*SCALE),
    "sp_value": 15
}

# Motor status constants
MOTOR_STATUS = {
    "INITIALIZING": "yellow",
    "CALIBRATING": "yellow",
    "ERROR": "red",
    "OK": "green"
}
# -----------------------------------------------------------------------------
# Main Application Class
# -----------------------------------------------------------------------------
class ExogloveApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Exoglove ODrive Controller")
        self.root.geometry(INTERFACE_SIZE)
        
        # --- Threading and Communication (Initialize FIRST) ---
        self.shutdown_event = threading.Event()
        self.message_queue = queue.Queue() # For communication from background thread to GUI
        
        # --- Relay control obj ---
        try:
            self.relay = RelayControl()
        except Exception as e:
            print(f"Relay not working due to exception {e}")
            self.update_gui_status("WARNING: RELAY not avalaible -> power supply may be off")
            self.relay = None
        # --- Shared State Variables ---
        self.m1 = None  # ODrive axis0 object (wrist actuation)
        self.m2 = None  # ODrive axis1 object (hand actuation)

        self.last_msg = None
        self.last_msg_time = time.time()
        
        # Dynamic setpoints for each motor
        self.dynamic_setpoint_axis0 = tk.DoubleVar(value=15.0)  # Wrist actuation
        self.dynamic_setpoint_axis1 = tk.DoubleVar(value=1.0)   # Hand actuation
        self.setpoint_lock = threading.Lock()  # Lock for thread-safe access to setpoints
        
        # Motor status indicators
        self.axis0_status = "INITIALIZING"
        self.axis1_status = "INITIALIZING"
        self.axis0_indicator = None
        self.axis1_indicator = None
        
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
        self.status_label = ttk.Label(main_frame, text="Status: Initializing...", font=("Arial", FONT_SIZE["status"]), wraplength=500)
        self.status_label.pack(pady=10, anchor='w')

        # Motor indicators frame
        indicators_frame = ttk.Frame(main_frame)
        indicators_frame.pack(pady=10, anchor='w')
        
        # Axis0 (Wrist) indicator
        ttk.Label(indicators_frame, text="Axis0 (Wrist):", font=("Arial", 10)).pack(side=tk.LEFT, padx=5)
        self.axis0_indicator = tk.Canvas(indicators_frame, width=30, height=30, bg="gray", highlightthickness=0)
        self.axis0_indicator.pack(side=tk.LEFT, padx=5)
        self.axis0_indicator.create_oval(5, 5, 25, 25, fill=MOTOR_STATUS["INITIALIZING"], outline="black", width=2)
        
        # Axis1 (Hand) indicator
        ttk.Label(indicators_frame, text="Axis1 (Hand):", font=("Arial", 10)).pack(side=tk.LEFT, padx=20)
        self.axis1_indicator = tk.Canvas(indicators_frame, width=30, height=30, bg="gray", highlightthickness=0)
        self.axis1_indicator.pack(side=tk.LEFT, padx=5)
        self.axis1_indicator.create_oval(5, 5, 25, 25, fill=MOTOR_STATUS["INITIALIZING"], outline="black", width=2)

        # ========== AXIS0 (Wrist) Setpoint Slider ==========
        ttk.Label(main_frame, text="Axis0 (Wrist) - Dynamic velocity setpoint {0 - 20} rev/s:", font=("Arial", FONT_SIZE["title"])).pack(pady=(20, 0))
        slider_axis0 = ttk.Scale(
            main_frame,
            from_=0.0,
            to=20,
            orient="horizontal",
            length=SP_BAR_LENGTH,
            variable=self.dynamic_setpoint_axis0,
            command=self.update_setpoint_label_axis0
        )
        slider_axis0.pack(pady=10)

        # Setpoint Value Label for Axis0
        self.value_label_axis0 = ttk.Label(main_frame, text=f"Current value = {self.dynamic_setpoint_axis0.get():.2f}", font=("Arial", FONT_SIZE["sp_value"]))
        self.value_label_axis0.pack(pady=5)

        # ========== AXIS1 (Hand) Setpoint Slider ==========
        ttk.Label(main_frame, text="Axis1 (Hand) - Dynamic velocity setpoint {0 - 5} rev/s:", font=("Arial", FONT_SIZE["title"])).pack(pady=(20, 0))
        slider_axis1 = ttk.Scale(
            main_frame,
            from_=0.0,
            to=5,
            orient="horizontal",
            length=SP_BAR_LENGTH,
            variable=self.dynamic_setpoint_axis1,
            command=self.update_setpoint_label_axis1
        )
        slider_axis1.pack(pady=10)

        # Setpoint Value Label for Axis1
        self.value_label_axis1 = ttk.Label(main_frame, text=f"Current value = {self.dynamic_setpoint_axis1.get():.2f}", font=("Arial", FONT_SIZE["sp_value"]))
        self.value_label_axis1.pack(pady=5)
    
    def update_setpoint_label_axis0(self, val):
        """Updates the text label for axis0 slider."""
        with self.setpoint_lock:
            current_value = self.dynamic_setpoint_axis0.get()
        self.value_label_axis0.config(text=f"Current value = {current_value:.2f}")
    
    def update_setpoint_label_axis1(self, val):
        """Updates the text label for axis1 slider."""
        with self.setpoint_lock:
            current_value = self.dynamic_setpoint_axis1.get()
        self.value_label_axis1.config(text=f"Current value = {current_value:.2f}")
    
    def update_motor_indicator(self, axis_num, status):
        """Updates the motor status indicator color."""
        color = MOTOR_STATUS.get(status, "gray")
        if axis_num == 0:
            self.axis0_status = status
            self.axis0_indicator.delete("all")
            self.axis0_indicator.create_oval(5, 5, 25, 25, fill=color, outline="black", width=2)
        elif axis_num == 1:
            self.axis1_status = status
            self.axis1_indicator.delete("all")
            self.axis1_indicator.create_oval(5, 5, 25, 25, fill=color, outline="black", width=2)

    def process_gui_queue(self):
        """Processes messages from the background thread to update the GUI safely."""
        try:
            message = self.message_queue.get_nowait()
            if 'status' in message:
                self.status_label.config(text=f"Status: {message['status']}")
            if 'axis0_status' in message:
                self.update_motor_indicator(0, message['axis0_status'])
            if 'axis1_status' in message:
                self.update_motor_indicator(1, message['axis1_status'])
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
            self.configure_odrive(my_drive)
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
    
    def update_motor_status_msg(self, axis_num, status):
        """Helper to send a motor status update to the GUI thread."""
        if axis_num == 0:
            self.message_queue.put({'axis0_status': status})
        elif axis_num == 1:
            self.message_queue.put({'axis1_status': status})

    def configure_odrive(self, my_drive):
        """Loads config, calibrates, and sets up both ODrive axes sequentially."""
        with open("odrive_config.json", "r") as json_file:
            config = json.load(json_file)
        
        my_drive.config.dc_max_negative_current = -2.0
        my_drive.config.enable_brake_resistor = False
        
        # ===== AXIS0 (Wrist Actuation) =====
        try:
            self.update_motor_status_msg(0, "INITIALIZING")
            axis0 = my_drive.axis0  # Motor linked to wrist actuation
            self.m1 = axis0
            
            # Apply motor config
            motor_cfg = config["axis0"]["motor"]["config"]
            for key, value in motor_cfg.items():
                setattr(axis0.motor.config, key, value)
            
            # Apply encoder config
            enc_cfg = config["axis0"]["encoder"]["config"]
            for key, value in enc_cfg.items():
                setattr(axis0.encoder.config, key, value)
            axis0.encoder.config.pre_calibrated = True

            # Apply controller config
            ctrl_cfg = config["axis0"]["controller"]["config"]
            axis0.controller.config.pos_gain = ctrl_cfg["kp"]
            axis0.controller.config.vel_gain = ctrl_cfg["kv"]
            axis0.controller.config.vel_integrator_gain = ctrl_cfg["ki"]
            axis0.controller.config.vel_limit = ctrl_cfg["vel_limit"]
            axis0.controller.config.control_mode = ctrl_cfg["control_mode"]
            axis0.controller.config.input_mode = ctrl_cfg["input_mode"]
            if ctrl_cfg["control_mode"] == 2:  # Velocity control mode
                axis0.controller.config.vel_ramp_rate = ctrl_cfg["vel_ramp_rate"]
            
            # Calibrate axis0
            self.update_gui_status("Calibrating Axis0 (Wrist)...")
            self.update_motor_status_msg(0, "CALIBRATING")
            axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            time.sleep(CALIB_TIME)
            
            if axis0.motor.error != 0 or axis0.encoder.error != 0 or axis0.controller.error != 0:
                raise RuntimeError(f"Axis0 calibration error! Motor: {axis0.motor.error}, Encoder: {axis0.encoder.error}, Controller: {axis0.controller.error}")
            
            axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.update_motor_status_msg(0, "OK")
            self.update_gui_status("Axis0 (Wrist) calibrated successfully")
            
        except Exception as e:
            self.update_motor_status_msg(0, "ERROR")
            self.update_gui_status(f"Error Axis0 (Wrist): {e}")
            self.m1 = None
        
        # ===== AXIS1 (Hand Actuation) =====
        try:
            self.update_motor_status_msg(1, "INITIALIZING")
            axis1 = my_drive.axis1  # Motor linked to hand actuation
            self.m2 = axis1
            
            # Apply motor config
            motor_cfg = config["axis1"]["motor"]["config"]
            for key, value in motor_cfg.items():
                setattr(axis1.motor.config, key, value)
            
            # Apply encoder config
            enc_cfg = config["axis1"]["encoder"]["config"]
            for key, value in enc_cfg.items():
                setattr(axis1.encoder.config, key, value)
            axis1.encoder.config.pre_calibrated = True

            # Apply controller config
            ctrl_cfg = config["axis1"]["controller"]["config"]
            axis1.controller.config.pos_gain = ctrl_cfg["kp"]
            axis1.controller.config.vel_gain = ctrl_cfg["kv"]
            axis1.controller.config.vel_integrator_gain = ctrl_cfg["ki"]
            axis1.controller.config.vel_limit = ctrl_cfg["vel_limit"]
            axis1.controller.config.control_mode = ctrl_cfg["control_mode"]
            axis1.controller.config.input_mode = ctrl_cfg["input_mode"]
            if ctrl_cfg["control_mode"] == 2:  # Velocity control mode
                axis1.controller.config.vel_ramp_rate = ctrl_cfg["vel_ramp_rate"]
            
            # Calibrate axis1
            self.update_gui_status("Calibrating Axis1 (Hand)...")
            self.update_motor_status_msg(1, "CALIBRATING")
            axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            time.sleep(CALIB_TIME)
            
            if axis1.motor.error != 0 or axis1.encoder.error != 0 or axis1.controller.error != 0:
                raise RuntimeError(f"Axis1 calibration error! Motor: {axis1.motor.error}, Encoder: {axis1.encoder.error}, Controller: {axis1.controller.error}")
            
            axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.update_motor_status_msg(1, "OK")
            self.update_gui_status("Axis1 (Hand) calibrated successfully")
            
        except Exception as e:
            self.update_motor_status_msg(1, "ERROR")
            self.update_gui_status(f"Error Axis1 (Hand): {e}")
            self.m2 = None

    def rx_handler(self, value, options):
        """Callback for BLE messages. Runs in the BLE thread."""
        msg = value.decode("utf-8").strip()
        #print(f"--> Received: {msg}") # Translated from Spanish

        with self.setpoint_lock:
            setpoint_axis0 = self.dynamic_setpoint_axis0.get()
            setpoint_axis1 = self.dynamic_setpoint_axis1.get()

        new_vel_axis0 = 0.0
        new_vel_axis1 = 0.0
        
        if msg == "0001":
            new_vel_axis0 = setpoint_axis0
            print(f'>>> Supination cmd sent! (vel = {setpoint_axis0:.2f})')
        elif msg == "0010":
            new_vel_axis0 = -setpoint_axis0
            print(f'>>> Pronation cmd sent! (vel = {-setpoint_axis0:.2f})')
        elif msg == "0100":
            new_vel_axis1 = setpoint_axis1
            print(f'>>> Flexion cmd sent! (vel = {setpoint_axis1:.2f})')
        elif msg == "1000":
            new_vel_axis1 = -setpoint_axis1
            print(f'>>> Extension cmd sent! (vel = {-setpoint_axis1:.2f})')
        else:
            print('>>> Motor stopped!')
            new_vel_axis0 = 0.0
            new_vel_axis1 = 0.0

        if self.m1 and self.last_msg != msg:
            self.m1.controller.input_vel = new_vel_axis0
        
        if self.m2 and self.last_msg != msg:
            self.m2.controller.input_vel = new_vel_axis1

        self.last_msg = msg
        self.last_msg_time = time.time()

    def watchdog_task(self):
        """Background task to stop the motors if no BLE messages are received."""
        while not self.shutdown_event.is_set():
            if self.last_msg_time is not None:
                elapsed = time.time() - self.last_msg_time
                if elapsed > WATCHDOG_THRES:
                    if self.m1 and self.m1.controller.input_vel != 0.0:
                        self.m1.controller.input_vel = 0.0
                        print(f"⏱️ WATCHDOG activated -> Axis0 (Wrist) stopped")
                    if self.m2 and self.m2.controller.input_vel != 0.0:
                        self.m2.controller.input_vel = 0.0
                        print(f"⏱️ WATCHDOG activated -> Axis1 (Hand) stopped")
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
                print(f"Could not set Axis0 to idle: {e}")
        if self.m2:
            try:
                self.m2.requested_state = AXIS_STATE_IDLE
            except Exception as e:
                print(f"Could not set Axis1 to idle: {e}")
        if self.relay is not None:
            self.relay.set(False) # turn off power        
        self.root.destroy()

# -----------------------------------------------------------------------------
# Main execution block
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = ExogloveApp(root)
    root.mainloop()
