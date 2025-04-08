#!/usr/bin/env python3
import time
import odrive
from odrive.enums import *
import json
import subprocess
from odrive.utils import dump_errors
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import csv
# Conectar con cualquier placa ODrive disponible
print("Buscando ODrive...")
found = False
my_drive = None
while not found:
    my_drive = odrive.find_any()
    if my_drive is not None:
        found = True
    
print("ODrive conectado")

# Usar eje M0
axis = my_drive.axis0

# Leer configuración desde archivo JSON
with open("odrive_config.json", "r") as json_file:
    config = json.load(json_file)
    print("✅ Configuración JSON cargada")

# Establecer estado inicial en IDLE
axis.requested_state = AXIS_STATE_IDLE
time.sleep(0.5)

#### Aplicar configuración básica al motor
ax_m0_config = config["axis0"]["motor"]["config"]


axis.motor.config.motor_type = ax_m0_config["motor_type"]
axis.motor.config.current_lim = ax_m0_config["current_lim"]
axis.motor.config.pole_pairs = ax_m0_config["pole_pairs"]
axis.motor.config.resistance_calib_max_voltage = ax_m0_config["resistance_calib_max_voltage"]
axis.motor.config.requested_current_range = ax_m0_config["requested_current_range"]
axis.motor.config.current_control_bandwidth = ax_m0_config["current_control_bandwidth"]
axis.motor.config.torque_constant = ax_m0_config["torque_constant"]

#### Config del encoder incremental
ax0_encoder_config = config["axis0"]["encoder"]["config"]

axis.encoder.config.cpr = ax0_encoder_config["cpr"]
axis.encoder.config.mode = ax0_encoder_config["mode"]
axis.encoder.config.calib_scan_distance = ax0_encoder_config["calib_scan_distance"]
axis.encoder.config.bandwidth = ax0_encoder_config["bandwidth"]

# Calibracion encoder con indice
axis.encoder.config.use_index = ax0_encoder_config["use_index"]
#axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

# check the axis0.error is zero

# save the config of the encoder 
axis.encoder.config.pre_calibrated = True
# to search index and the start up 
#axis.config.startup_encoder_index_search = True

#### Controller

# Ganancias para control de posición
ax0_ctrl_config = config["axis0"]["controller"]["config"]
axis.controller.config.pos_gain = ax0_ctrl_config["kp"]
axis.controller.config.vel_gain = ax0_ctrl_config["kv"] # * axis.motor.config.torque_constant * axis.encoder.config.cpr
axis.controller.config.vel_integrator_gain = ax0_ctrl_config["ki"] #0.1 * axis.motor.config.torque_constant * axis.encoder.config.cpr
axis.controller.config.vel_limit = ax0_ctrl_config["vel_limit"]
axis.controller.config.control_mode = 2 # ax0_ctrl_config["control_mode"] # Control en posicion

# para enviar directamente inputs de pos, vel o torque
axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

# Otros parámetros del sistema
my_drive.config.dc_max_negative_current = -2.0
my_drive.config.enable_brake_resistor = False

print("⚙️ Iniciando calibración del motor y del encoder...")
axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
time.sleep(32) # dar tiempo para calibrarse correctamente

print("✅ Motor calibrado")
print(f"Position gain: {axis.controller.config.pos_gain}")
print(f"Velocity gain: {axis.controller.config.vel_gain}")
print(f"Integrator gain: {axis.controller.config.vel_integrator_gain}")
print(f"Resultado de la calib (tiene que ser cero): {axis.motor.error}")
if axis.motor.error != 0:
    print("Error durante calibracion del motor")
    quit()
"""
Control en velocidad
Leer trayectoria del perfil de csv/velocity_profile.csv 
El .csv tiene columna de tiempo y de velocidad
Leer velocidad en cada instante y enviar con axis.controller.input_vel
"""
# Activar el eje en estado de control en bucle cerrado
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("Control en LAZO CERRADO ACTIVADO")
time.sleep(5)

# Cargar el perfil de velocidad desde el archivo CSV
csv_file_path = "csv/velocity_profile.csv"
vel_profile = []

with open(csv_file_path, newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
        # Guardamos como flotantes para precisión
        vel_profile.append((float(row['Time (s)']), float(row['Velocity (rev/s)'])))

print(f"📈 Se cargaron {len(vel_profile)} puntos del perfil de velocidad")

# Ejecutar el perfil de velocidad
print("🚀 Ejecutando perfil de velocidad...")
start_time = time.time()
for i in range(len(vel_profile) - 1):
    # Velocidad actual a enviar
    velocity = vel_profile[i][1]
    # Tiempo actual y el siguiente (para calcular cuánto esperar)
    current_time = vel_profile[i][0]
    next_time = vel_profile[i + 1][0]
    delta_t = next_time - current_time

    axis.controller.input_vel = velocity
    time.sleep(delta_t)

# Asegurarse de detener el motor al final
axis.controller.input_vel = 0.0
print("🛑 Perfil de velocidad completado.")

