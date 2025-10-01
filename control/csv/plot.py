import csv
import matplotlib.pyplot as plt
import sys
import numpy as np  # Para generar ticks del eje Y

if len(sys.argv) < 2 or len(sys.argv) > 2:
    print("Error | Usage: python3 plot.py file.csv")
    quit()

# Leer nombre del archivo
file_str = str(sys.argv[1])

# Leer los datos desde el archivo CSV
time_data = []
position_data = []
velocity_data = []

with open(file_str, mode='r') as file:
    reader = csv.reader(file)
    next(reader)  # Saltar la cabecera
    for row in reader:
        time_data.append(float(row[0]))      # Tiempo
        position_data.append(float(row[1]))  # Posición
        velocity_data.append(float(row[2]))  # Velocidad

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
min_pos = min(position_data)
max_pos = max(position_data)
yticks_pos = np.linspace(min_pos, max_pos, num=20)
plt.yticks(yticks_pos)

# Gráfica de la velocidad
plt.subplot(2, 1, 2)
plt.plot(time_data, velocity_data, label='Velocidad (rev/s)', color='r', marker='^', linestyle='-')
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad (rev/s)')
plt.title('Velocidad vs. Tiempo')
plt.grid(True)
plt.legend()

# Mejorar resolución del eje Y
min_vel = min(velocity_data)
max_vel = max(velocity_data)
yticks_vel = np.linspace(min_vel, max_vel, num=20)
plt.yticks(yticks_vel)

# Ajustar el layout y mostrar
plt.tight_layout()
plt.show()

