#!/usr/bin/env python3
import time
import odrive
from odrive.enums import *
import json
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Conectar con ODrive
print("🔍 Buscando ODrive...")
my_drive = None
while my_drive is None:
    my_drive = odrive.find_any()
print("✅ ODrive conectado")

axis = my_drive.axis0

# Leer configuración JSON
with open("odrive_config.json", "r") as json_file:
    config = json.load(json_file)
    print("✅ Configuración JSON cargada")

# Establecer IDLE
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

# Otros ajustes
my_drive.config.dc_max_negative_current = -2.0
my_drive.config.enable_brake_resistor = False

# Calibración
print("⚙️ Iniciando calibración...")
axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
time.sleep(32)
print("✅ Calibración completada")

if axis.motor.error != 0:
    print(f"❌ Error durante calibración del motor. Código: {axis.motor.error}")
    quit()

# Activar lazo cerrado
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("🟢 Lazo cerrado activado")

# Parámetros de la gráfica en tiempo real
max_len = 400  # Cantidad de muestras visibles
data_pos = deque([0]*max_len, maxlen=max_len)
data_vel = deque([0]*max_len, maxlen=max_len)
time_data = deque([0]*max_len, maxlen=max_len)

start_time = time.time()

def update_data():
    while True:
        elapsed = time.time() - start_time
        pos = axis.encoder.pos_estimate
        vel = axis.encoder.vel_estimate
        time_data.append(elapsed)
        data_pos.append(pos)
        data_vel.append(vel)
        time.sleep(0.02)

# Hilo para recopilar datos continuamente
threading.Thread(target=update_data, daemon=True).start()

# Inicializar ventana de gráfica
fig, ax = plt.subplots()
line1, = ax.plot([], [], label="Posición (rev)")
line2, = ax.plot([], [], label="Velocidad (rev/s)")
ax.set_ylim(-5, 5)
ax.set_xlim(0, 5)
ax.set_xlabel("Tiempo (s)")
ax.set_title("Liveplotter: Posición y Velocidad")
ax.legend()
ax.grid(True)

def animate(i):
    ax.set_xlim(max(0, time_data[0]), time_data[-1])
    line1.set_data(time_data, data_pos)
    line2.set_data(time_data, data_vel)
    return line1, line2

axis.controller.input_vel = 0.5
time.sleep(10)
ani = animation.FuncAnimation(fig, animate, interval=50)
plt.show()
"""
# Enviar referencia tras retardo
duration = 20
delay_before_ref = 1.0
ref_enviada = False
target = 0.5

while time.time() - start_time < duration:
    elapsed = time.time() - start_time

    if not ref_enviada and elapsed >= delay_before_ref:
        axis.controller.input_vel = target
        print(f"📍 Referencia enviada: {target} vueltas/segundo")
        ref_enviada = True
        single_step = False

    time.sleep(0.05)

print("✅ Medición finalizada")
"""