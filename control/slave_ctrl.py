from bluezero import peripheral, adapter, device
import odrive
from odrive.enums import *
import json
import time
import threading
import tkinter as tk
from tkinter import ttk

# Master device name
MASTER_NAME = 'NanoESP32_BLE'

# UUIDs del servicio y características UART (NUS)
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

# global variable for motor1
global m1
global last_msg
global last_msg_time

FREQ = 0.25  # secs
WATCHDOG_THRES = FREQ + 0.5

# -----------------------------------------------------------------------------
# Clase para manejar el valor dinámico del setpoint y su interfaz
class MotorControllerInterface:
    def __init__(self):
        self.dynamic_setpoint = 0.95  # valor inicial

        # Crear ventana con Tkinter
        self.root = tk.Tk()
        self.root.title("Control de Dynamic Setpoint")

        ttk.Label(self.root, text="Setpoint dinámico (0 - 1.5):", font=("Arial", 12)).pack(pady=10)

        # Slider
        self.slider = ttk.Scale(
            self.root,
            from_=0.0,
            to=1.5,
            orient="horizontal",
            length=300,
            command=self.update_setpoint
        )
        self.slider.set(self.dynamic_setpoint)
        self.slider.pack(pady=10)

        # Etiqueta de valor
        self.value_label = ttk.Label(self.root, text=f"Valor actual: {self.dynamic_setpoint:.2f}", font=("Arial", 11))
        self.value_label.pack(pady=5)

        # Ejecutar la GUI en un hilo aparte
        threading.Thread(target=self.root.mainloop, daemon=True).start()

    def update_setpoint(self, val):
        """Callback del slider: actualiza el valor dinámico"""
        self.dynamic_setpoint = float(val)
        self.value_label.config(text=f"Valor actual: {self.dynamic_setpoint:.2f}")

# Crear instancia global
interface = MotorControllerInterface()

# -----------------------------------------------------------------------------
class UARTDevice:
    tx_obj = None

    @classmethod
    def on_connect(cls, ble_device: device.Device):
        print("Connected to " + str(ble_device.address))

    @classmethod
    def on_disconnect(cls, adapter_address, device_address):
        print("Disconnected from " + device_address)

    @classmethod
    def uart_notify(cls, notifying, characteristic):
        if notifying:
            cls.tx_obj = characteristic
        else:
            cls.tx_obj = None

    @classmethod
    def update_tx(cls, value):
        if cls.tx_obj:
            print("Sending")
            cls.tx_obj.set_value(value)

    @classmethod
    def uart_write(cls, value, options):
        print('raw bytes:', value)
        print('With options:', options)
        print('Text value:', bytes(value).decode('utf-8'))
        cls.update_tx(value)

# -----------------------------------------------------------------------------
# odrive motors calib and config
def doMotorCalib(my_drive):
    print("🔍 Buscando ODrive...")
    while my_drive is None:
        my_drive = odrive.find_any()
    print("✅ ODrive conectado")

    axis = my_drive.axis0

    with open("odrive_config.json", "r") as json_file:
        config = json.load(json_file)
        print("✅ Configuración JSON cargada")

    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)

    # Configuración del motor
    motor_cfg = config["axis0"]["motor"]["config"]
    axis.motor.config.motor_type = motor_cfg["motor_type"]
    axis.motor.config.current_lim = motor_cfg["current_lim"]
    axis.motor.config.pole_pairs = motor_cfg["pole_pairs"]
    axis.motor.config.resistance_calib_max_voltage = motor_cfg["resistance_calib_max_voltage"]
    axis.motor.config.requested_current_range = motor_cfg["requested_current_range"]
    axis.motor.config.current_control_bandwidth = motor_cfg["current_control_bandwidth"]
    axis.motor.config.torque_constant = motor_cfg["torque_constant"]

    # Encoder
    enc_cfg = config["axis0"]["encoder"]["config"]
    axis.encoder.config.cpr = enc_cfg["cpr"]
    axis.encoder.config.mode = enc_cfg["mode"]
    axis.encoder.config.calib_scan_distance = enc_cfg["calib_scan_distance"]
    axis.encoder.config.bandwidth = enc_cfg["bandwidth"]
    axis.encoder.config.use_index = enc_cfg["use_index"]
    axis.encoder.config.pre_calibrated = True

    # Controlador
    ctrl_cfg = config["axis0"]["controller"]["config"]
    axis.controller.config.pos_gain = ctrl_cfg["kp"]
    axis.controller.config.vel_gain = ctrl_cfg["kv"]
    axis.controller.config.vel_integrator_gain = ctrl_cfg["ki"]
    axis.controller.config.vel_limit = ctrl_cfg["vel_limit"]
    axis.controller.config.control_mode = ctrl_cfg["control_mode"]
    axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

    my_drive.config.dc_max_negative_current = -2.0
    my_drive.config.enable_brake_resistor = False

    print("⚙️ Iniciando calibración...")
    axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    time.sleep(32)
    print("✅ Calibración completada")

    if axis.motor.error != 0:
        print(f"❌ Error durante calibración del motor. Código: {axis.motor.error}")
        quit()

    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    print("🟢 Lazo cerrado activado")

    return my_drive.axis0

# -----------------------------------------------------------------------------
# Callback BLE
def rx_handler(value, options):
    global m1, last_msg, last_msg_time, interface
    msg = value.decode("utf-8").strip()
    print(f"--> Recibido: {msg}")

    setpoint = interface.dynamic_setpoint  # obtener valor dinámico

    if msg == "0001":  # supination
        if m1 is not None and last_msg != msg:
            m1.controller.input_vel = setpoint
            print(f'>>> Supination cmd sent! (vel = {setpoint:.2f})')
    elif msg == "0010":  # pronation
        if m1 is not None and last_msg != msg:
            m1.controller.input_vel = -setpoint
            print(f'>>> Pronation cmd sent! (vel = {-setpoint:.2f})')
    else:
        if m1 is not None and last_msg != msg:
            m1.controller.input_vel = 0.0
            print('>>> Motor stopped!')

    last_msg = msg
    last_msg_time = time.time()

# -----------------------------------------------------------------------------
# Watchdog
def watchdog_thread():
    global m1, last_msg_time
    while True:
        if m1 is not None and last_msg_time is not None:
            elapsed = time.time() - last_msg_time
            if elapsed > WATCHDOG_THRES:
                m1.controller.input_vel = 0.0
                print(f"⏱️ WATCHDOG activated -> Motor stop")
        time.sleep(0.1)

# -----------------------------------------------------------------------------
# Crear periférico BLE
adapter_address = list(adapter.Adapter.available())[0].address
device = peripheral.Peripheral(adapter_address, local_name=MASTER_NAME)

device.add_service(srv_id=1, uuid=UART_SERVICE_UUID, primary=True)
device.add_characteristic(srv_id=1, chr_id=1, uuid=RX_CHAR_UUID,
                          value=[], notifying=False, flags=['write'], write_callback=rx_handler)
device.add_characteristic(srv_id=1, chr_id=2, uuid=TX_CHAR_UUID,
                          value=[], notifying=False, flags=['notify'])

# -----------------------------------------------------------------------------
# Inicialización ODrive
my_drive = None
m1 = doMotorCalib(my_drive)
last_msg = None
last_msg_time = None

print("------ Slave BLE server ready, awaiting master pairing -------------")
device.on_connect = UARTDevice.on_connect

# Hilo del watchdog
t = threading.Thread(target=watchdog_thread, daemon=True)
t.start()

device.publish()
