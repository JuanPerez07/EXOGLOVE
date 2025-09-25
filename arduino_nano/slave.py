from bluezero import peripheral, adapter, device

# Master device name
MASTER_NAME = 'NanoESP32_BLE'
# UUIDs del servicio y características UART (NUS)
UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

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

# Callback cuando llega un mensaje por RX
def rx_handler(value, options):
    msg = value.decode("utf-8")
    print(f"📩 Recibido: {msg}")

# Crear periférico
adapter_address = list(adapter.Adapter.available())[0].address
device = peripheral.Peripheral(adapter_address,local_name=MASTER_NAME)

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

print("🔵 Servidor BLE listo, esperando conexión...")
device.on_connect = UARTDevice.on_connect
#device.on_disconnect = UARTDevice.on_disconnect

device.publish()
