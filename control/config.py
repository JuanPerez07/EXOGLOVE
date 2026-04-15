#!/usr/bin/env python3
import time
import odrive
from odrive.enums import *
import json
import csv
#import warnings
#warnings.filterwarnings("ignore", message=".*frames=None.*")

# Conectar con ODrive
print("🔍 Buscando ODrive...")
my_drive = None
while my_drive is None:
    my_drive = odrive.find_any()
print("✅ ODrive conectado")

# axis0 is the motor linked to the wrist 
axis = my_drive.axis0
# axis1 is the motor linked to the fingers
# axis1 = my_drive.axis1

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
axis.controller.config.input_mode = ctrl_cfg["input_mode"]

if(ctrl_cfg["control_mode"] == 2): # Control en velocidad
    axis.controller.config.vel_ramp_rate = ctrl_cfg["vel_ramp_rate"]

# Otros ajustes
my_drive.config.dc_max_negative_current = -2.0
my_drive.config.enable_brake_resistor = False

# Calibración
print("⚙️ Iniciando calibración...")
axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
time.sleep(32)
print("✅ Calibración completada")

if axis.motor.error != 0:
    print("❌ Error durante calibración del motor")
    quit()

# Activar lazo cerrado
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("🟢 Lazo cerrado activado")

# Preparar CSV
kp = str(ctrl_cfg["kp"])
kv = str(ctrl_cfg["kv"])
CSV_DIR = "csv/"
filename = f"{CSV_DIR}{kp}_{kv}_motor_data.csv"
with open(filename, mode='w', newline='') as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow(['Time (s)', 'Position (rev)', 'Velocity (rev/s)'])

    # Comenzar a recopilar datos ANTES de enviar la referencia
    print("Comenzando registro de datos...")
    start_time = time.time()
    duration = 20  # segundos
    delay_before_ref = 1.0  # segundos antes de enviar referencia

    ref_enviada = False
    target_position = 6
    single_step = True
    while time.time() - start_time < duration:
        elapsed = time.time() - start_time

        # Enviar referencia después del retardo inicial
        if not ref_enviada and elapsed >= delay_before_ref and single_step:
            axis.controller.input_pos = target_position
            print(f"📍 Referencia enviada: {target_position} vueltas")
            ref_enviada = True
            #single_step = False

        if ref_enviada and elapsed > 2.0:
            axis.controller.input_pos = -target_position
            ref_enviada = False

        # Guardar datos
        pos = axis.encoder.pos_estimate
        vel = axis.encoder.vel_estimate
        writer.writerow([elapsed, pos, vel])
        time.sleep(0.05)

print(f"✅ Datos guardados en '{filename}'")