from bluezero import peripheral, adapter, device
# webgraphy: https://bluezero.readthedocs.io/en/stable/examples.html#peripheral-nordic-uart-service
#import odrive
#from odrive.enums import *
import json
import time
# Master device name
MASTER_NAME = 'NanoESP32_BLE'
# UUIDs del servicio y características UART (NUS)
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
# global variable for motor1
global m1
global last_msg
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

## ----------------------------------------------------------------------------
## CLASS ODRIVE to emulate config and speed acquisition.
## ----------------------------------------------------------------------------
class ODrive:
    def __init__(self, calibrate=False):
        self.motor_type = "DC"
        self.current_lim = 2.0
        self.pole_pairs = 3
        self.resistance_calib_max_voltage = 0.0
        self.requested_current_range = 0.0
        self.current_control_bandwidth = 0.0
        self.torque_constant = 0.0
   
        if calibrate:
            # Leer configuración JSON
            with open("odrive_config.json", "r") as json_file:
                config = json.load(json_file)
                print("✅ Configuración JSON cargada")
            
            # Configuración del motor
            motor_cfg = config["axis0"]["motor"]["config"]
            self.motor_type = motor_cfg["motor_type"]
            self.current_lim = motor_cfg["current_lim"]
            self.pole_pairs = motor_cfg["pole_pairs"]
            self.resistance_calib_max_voltage = motor_cfg["resistance_calib_max_voltage"]
            self.requested_current_range = motor_cfg["requested_current_range"]
            self.current_control_bandwidth = motor_cfg["current_control_bandwidth"]
            self.torque_constant = motor_cfg["torque_constant"]

        # Velocidad inicial
        self.controller_input_vel = 0.0

    def setInputVel(self, vel):
        print("Modificando velocidad ...")
        self.controller_input_vel = vel
        time.sleep(3)

    def getVelocity(self):
        return float(self.controller_input_vel) 

# odrive motors calib and config
def doMotorCalib(my_drive):
    print("🔍 Buscando ODrive...")
    while my_drive is None:
        my_drive = ODrive(calibrate=True)
    print("✅ ODrive conectado")
    print(f"Velocidad actual del motor: {my_drive.getVelocity()}")
    return my_drive

# Callback cuando llega un mensaje por RX
def rx_handler(value, options):
    global m1, last_msg
    msg = value.decode("utf-8").strip()   
    print(f"--> Recibido: {msg}")
    target = 0.85 # const value
    # decode the message received 
    if (msg == "0001"): # supination
        if (m1 is not None and last_msg != msg): 
            m1.setInputVel(target)
            print('>>> Supination cmd sent!')
        else: print(' xxxxx Supination cmd not sent to motor')
    elif (msg == "0010"): # pronation
        if (m1 is not None and last_msg != msg): 
            m1.setInputVel(-target)
            print('>>> Pronation cmd sent!')
        else: print('xxxxx Pronation cmd not sent to the motor')
    else: # default case we send null speed
        if (m1 is not None and last_msg != msg): m1.setInputVel(0.0)
    # update last_msg
    last_msg = msg
    # Check velocity
    print(f"Current Motor Speed is : {m1.getVelocity()} rev/s") 


# Crear periférico
adapter_address = list(adapter.Adapter.available())[0].address
device = peripheral.Peripheral(adapter_address,local_name=MASTER_NAME)
print(f"🔑 Dirección del dispositivo BLE (adapter): {adapter_address}")

# Añadir servicio UART
device.add_service(srv_id=1, uuid=UART_SERVICE_UUID, primary=True)

# Añadir característica RX (escritura desde el maestro)
device.add_characteristic(
    srv_id=1,
    chr_id=1,
    uuid=RX_CHAR_UUID,
    value=[],
    notifying=False,
    flags=['write'],
    write_callback=rx_handler
)

# Añadir característica TX (notificación hacia el maestro)
device.add_characteristic(
    srv_id=1,
    chr_id=2,
    uuid=TX_CHAR_UUID,
    value=[],
    notifying=False,
    flags=['notify']
)
# odrive obj
my_drive = None
# Calibrate the motor
m1 = doMotorCalib(my_drive)
last_msg = None # global var to avoid sending same command many times
print("------ Slave BLE server ready, awaiting master pairing -------------")
print(f"📡 Nombre local: {MASTER_NAME}")
print(f"🔑 Dirección BLE: {adapter_address}")
device.on_connect = UARTDevice.on_connect
#device.on_disconnect = UARTDevice.on_disconnect

device.publish()
