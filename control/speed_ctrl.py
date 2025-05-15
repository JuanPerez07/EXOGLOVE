#!/usr/bin/env python3
import time
import odrive
from odrive.enums import *
import json
import csv
import keyboard

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

# Ganancias del controlador
kp = str(ctrl_cfg["kp"])
kv = str(ctrl_cfg["kv"])
ki = str(ctrl_cfg["ki"])
# Esperar input de usuario para comenzar la prueba
print("Pulse la tecla espacio para comenzar simulacion CONTROL VELOCIDAD")
while not keyboard.is_pressed('space'):
    time.sleep(0.1)

# Preparar CSV
CSV_DIR = "csv/"
filename = f"{CSV_DIR}_VEL_CMD_{kp}_{kv}_{ki}_motor_data.csv"
with open(filename, mode='w', newline='') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(['Time (s)', 'Position (rev)', 'Velocity (rev/s)'])

    # Comenzar a recopilar datos ANTES de enviar la referencia
    print("Comenzando registro de datos...")
    start_time = time.time()
    duration = 20  # segundos
    delay_before_ref = 1.0  # segundos antes de enviar primera referencia

    last_ref_time = 0
    #speed_command = [0, 0.5, 0, -0.5] objetivo
    speed_command = [0, 0.8, 0, -0.8]
    idx = 0
    num_cmds = len(speed_command)
    interval = 1 # comando cada 1s

    while time.time() - start_time < duration:
        elapsed = time.time() - start_time

        # Enviar referencias cada 1 s después del retardo inicial
        if elapsed >= delay_before_ref and (elapsed - last_ref_time >= interval):
            axis.controller.input_vel = speed_command[idx]
            print(f"📍 Referencia enviada: {speed_command[idx]} vueltas/s")
            idx = (idx + 1) % num_cmds
            last_ref_time = elapsed

        # Guardar datos
        pos = axis.encoder.pos_estimate
        vel = axis.encoder.vel_estimate
        writer.writerow([elapsed, pos, vel])
        time.sleep(0.05)

print(f"✅ Datos guardados en '{filename}'")


