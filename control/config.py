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
import warnings
warnings.filterwarnings("ignore", message=".*frames=None.*")

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
axis.controller.config.control_mode = ax0_ctrl_config["control_mode"] # Control en posicion

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

time.sleep(1)
print("Control lazo cerrado activado")
#axis.requested_state = 6 # encoder index calib
#axis.requested_state = 7 # encoder offeset calib
axis.requested_state = 8 # closed loop control
"""
Send position reference to the robot
"""
"""
value_int = 6 # probado que es suficiente para mover el exo wrist
print(f"Referencia posicion enviada: {value_int}")
time.sleep(1)
axis.controller.input_pos = value_int
"""
quit()

#### Abrir archivo CSV para guardar los datos
kp = str(ax0_ctrl_config["kp"])
kv = str(ax0_ctrl_config["kv"])  
CSV_DIR = 'csv/'

# Parámetros de la gráfica
tiempos = deque(maxlen=300)
posiciones = deque(maxlen=300)
velocidades = deque(maxlen=300)

fig, (ax1, ax2) = plt.subplots(2, 1)
linea_pos, = ax1.plot([], [], 'b-', label='Posición (rev)')
linea_vel, = ax2.plot([], [], 'r-', label='Velocidad (rev/s)')

ax1.set_title("Posición en tiempo real")
ax2.set_title("Velocidad en tiempo real")
ax2.set_xlabel("Tiempo (s)")
ax1.set_ylabel("Posición (rev)")
ax2.set_ylabel("Velocidad (rev/s)")
ax1.grid()
ax2.grid()
ax1.legend()
ax2.legend()

# Archivo CSV
filename = CSV_DIR + kp + '_' + kv + '_motor_data.csv'
csv_file = open(filename, mode='w', newline='')
writer = csv.writer(csv_file)
writer.writerow(['Time (s)', 'Position (revoluciones)', 'Velocity (rev/s)'])

start_time = time.time()

def actualizar(frame):
    tiempo = time.time() - start_time
    pos = axis.encoder.pos_estimate
    vel = axis.encoder.vel_estimate

    tiempos.append(tiempo)
    posiciones.append(pos)
    velocidades.append(vel)
    
    writer.writerow([tiempo, pos, vel])

    linea_pos.set_data(tiempos, posiciones)
    linea_vel.set_data(tiempos, velocidades)

    ax1.set_xlim(max(0, tiempo - 10), tiempo + 1)
    ax2.set_xlim(max(0, tiempo - 10), tiempo + 1)

    ax1.set_ylim(min(posiciones, default=-1)-0.1, max(posiciones, default=1)+0.1)
    ax2.set_ylim(min(velocidades, default=-2)-0.2, max(velocidades, default=2)+0.2)

    return linea_pos, linea_vel



# animate functions with time
ani = animation.FuncAnimation(fig, actualizar, interval=100, save_count=100, cache_frame_data=False)
plt.tight_layout()
plt.show()


csv_file.close()
print(f"✅ Datos guardados en '{filename}'")
#time.sleep(20)
#print(f"Error tras 20 seg con lazo cerrado activo: {axis.motor.error}")
#axis.controller.input_pos += 3 # tres vueltas
#time.sleep(10)
#print(f"Error tras 10 seg de enviar comando de input pos: {axis.motor.error}")

"""
# Activar modo de control de posición
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
print("🟢 Control de posición activo en axis0")

# Puedes probar enviar una posición (por ejemplo: 10 vueltas)
axis.controller.input_pos = 10.0
"""

