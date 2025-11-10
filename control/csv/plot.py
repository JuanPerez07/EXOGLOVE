import csv
import matplotlib.pyplot as plt
import sys
import numpy as np  # Para generar ticks del eje Y

DIR = 'with_relay/'
# --- Variable global añadida ---
SETPOINT = 10 
ARGS = 3 # arguments of the program
if len(sys.argv) < ARGS or len(sys.argv) > ARGS:
    print("Error | Usage: python3 plot.py file.csv enable_plot")
    quit()

# Leer nombre del archivo
file_str = DIR + str(sys.argv[1])
# Boolean for plotting
enable_plt = int(sys.argv[2])

# Leer los datos desde el archivo CSV
time_data = []
position_data = []
velocity_data = []

with open(file_str, mode='r') as file:
    reader = csv.reader(file)
    next(reader)  # Saltar la cabecera
    for row in reader:
        try:
            time_data.append(float(row[0]))      # Tiempo
            position_data.append(float(row[1]))  # Posición
            velocity_data.append(float(row[2]))  # Velocidad
        except ValueError:
            print(f"Advertencia: Saltando fila con datos no válidos: {row}")
            continue


# Convertir listas a arrays de NumPy para filtrar fácilmente
time_np = np.array(time_data)
velocity_np = np.array(velocity_data)

# Definir el intervalo de tiempo
t_inicio = 6.1
t_fin = 7.7

# 1. Filtrar los datos de velocidad dentro del intervalo [5.3, 7.75]
# Creamos una máscara booleana
interval_mask = (time_np >= t_inicio) & (time_np <= t_fin)
# Aplicamos la máscara a los datos de velocidad
velocity_interval = velocity_np[interval_mask]

# 2. Calcular las métricas solicitadas
if velocity_interval.size > 0:
    # Calcular media y máximo del intervalo
    media_intervalo = np.mean(velocity_interval)
    valor_max_intervalo = np.max(velocity_interval)

    # Calcular sobreoscilación (delta)
    if media_intervalo != 0:
        delta = 100 * ((valor_max_intervalo - media_intervalo) / media_intervalo)
        # Mostrar por terminal
        print(f"sobreoscilacion = {delta} %")
    else:
        print("Error: La media de la velocidad en el intervalo es 0, no se puede calcular delta.")

    # Calcular error en velocidad (ev)
    ev = abs(SETPOINT - media_intervalo)
    # Porcentaje de error en velocidad
    ev_percent = (ev/SETPOINT) * 100
    # Mostrar por terminal
    print(f"error en velocidad = {ev_percent} %")
    print(f"velocidad en estado estacionario = {media_intervalo} rev/s")

else:
    print(f"Advertencia: No se encontraron datos en el intervalo de tiempo [{t_inicio}, {t_fin}]")

if enable_plt == 1:
    # Crear la figura
    plt.figure(figsize=(10, 6))
    plt.suptitle(file_str)

    # Gráfica de la posición
    plt.subplot(2, 1, 1)
    plt.plot(time_data, position_data, label='Posición (revoluciones)', color='b', marker='^', linestyle='-')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Posición (revoluciones)')
    plt.title('Posición vs. Tiempo')
    plt.grid(True)
    plt.legend()

    # Mejorar resolución del eje Y
    # (Manejar caso de lista vacía o con un solo elemento)
    if position_data:
        min_pos = min(position_data)
        max_pos = max(position_data)
        if min_pos != max_pos:
            yticks_pos = np.linspace(min_pos, max_pos, num=20)
            plt.yticks(yticks_pos)

    # Gráfica de la velocidad
    plt.subplot(2, 1, 2)
    plt.plot(time_data, velocity_data, label='Velocidad (rev/s)', color='r', marker='^', linestyle='-')
    # Añadir línea de Setpoint a la gráfica de velocidad
    plt.axhline(y=SETPOINT, color='g', linestyle='--', label=f'Setpoint ({SETPOINT} rev/s)')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Velocidad (rev/s)')
    plt.title('Velocidad vs. Tiempo')
    plt.grid(True)
    plt.legend()

    # Mejorar resolución del eje Y
    # (Manejar caso de lista vacía o con un solo elemento)
    if velocity_data:
        min_vel = min(velocity_data)
        max_vel = max(velocity_data)
        # Ajustar límites para incluir el setpoint si está fuera del rango
        min_vel = min(min_vel, SETPOINT)
        max_vel = max(max_vel, SETPOINT)
        
        if min_vel != max_vel:
            yticks_vel = np.linspace(min_vel, max_vel, num=20)
            plt.yticks(yticks_vel)

    # Ajustar el layout y mostrar
    plt.tight_layout()
    plt.show()