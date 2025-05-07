import csv
import matplotlib.pyplot as plt
import sys

if len(sys.argv) < 2 or len(sys.argv) > 2:
    print("Error | Usage python3 plot.py file.csv")
    quit()
# Read file name
file_str = str(sys.argv[1])
# Leer los datos desde el archivo CSV
time_data = []
position_data = []
velocity_data = []

with open(file_str, mode='r') as file:
    reader = csv.reader(file)
    next(reader)  # Saltar la cabecera
    for row in reader:
        time_data.append(float(row[0]))  # Tiempo
        position_data.append(float(row[1]))  # Posición
        velocity_data.append(float(row[2]))  # Velocidad

# Crear las gráficas
plt.figure(figsize=(10, 6))
plt.suptitle(file_str)
"""
"""
# Gráfica de la posición
plt.subplot(2, 1, 1)
plt.plot(time_data, position_data, label='Posición (revoluciones)', color='b')
plt.xlabel('Tiempo (s)')
plt.ylabel('Posición (revoluciones)')
plt.title('Posición vs. Tiempo')
plt.grid(True)
plt.legend()

# Gráfica de la velocidad
plt.subplot(2, 1, 2)

plt.plot(time_data, velocity_data, label='Velocidad (rev/s)', color='r')
plt.xlabel('Tiempo (s)')
plt.ylabel('Velocidad (rev/s)')
plt.title('Velocidad vs. Tiempo')
plt.grid(True)
plt.legend()

# Mostrar las gráficas
plt.tight_layout()
plt.show()
