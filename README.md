# Exoglove

Exoglove es un proyecto de prótesis/guante exoesquelético orientado a la rehabilitación y el control de movimiento de la muñeca y la mano. Este repositorio reúne el software, la lógica de control, los archivos de hardware/ensamblaje y los recursos de visión usados para desarrollar y validar el sistema.

## Alcance del proyecto

El proyecto Exoglove integra:
- control de actuadores mediante ODrive y comunicación BLE,
- firmware para microcontroladores Arduino Nano,
- procesamiento de visión para estimación de postura y movimiento de la mano,
- herramientas de validación, registro de datos y generación de gráficas,
- archivos CAD y recursos auxiliares para el desarrollo mecánico y eléctrico.

Además de su desarrollo técnico, Exoglove ha tenido un alcance significativo en el ámbito académico y en la transferencia de conocimiento hacia la comunidad científica en robótica y bioingeniería. Como parte de estas acciones de divulgación, el equipo presentó el artículo **_Desarrollo de un exoesqueleto para asistencia y rehabilitación de mano_** en el **Simposio del Comité Español de Automática (CEA)**, dentro de la sección de Bioingeniería del Vol. 2, Núm. 2 (2026). Esta publicación forma parte de las actas oficiales del simposio, un foro de referencia nacional para la presentación de avances en automatización, robótica y tecnologías aplicadas a la salud.

- Artículo relacionado con el software de Exoglove:  
  http://138.100.76.10/index.php/SimposiosCEA/article/view/187  
- Página del Comité Español de Automática (CEA):  
  https://www.ceautomatica.es/

## Estructura del repositorio

- [arduino_nano/](arduino_nano/): firmware para la comunicación entre nodos maestro/esclavo y los microcontroladores.
- [cad_files/](cad_files/): archivos de diseño mecánico y piezas asociadas al prototipo.
- [control/](control/): módulos de control del sistema, incluyendo:
  - interfaz gráfica para el control del esclavo,
  - control de posición y velocidad,
  - control de relés y comunicación con ODrive,
  - scripts de validación y análisis de datos.
- [vision/](vision/): pipeline de visión para detección de mano, calibración de cámara, estimación de ángulos y análisis de rangos de movimiento.
- [requirements.txt](requirements.txt): dependencias principales de Python para ejecutar el software del proyecto.

## Componentes principales

### 1. Control
La carpeta [control/](control/) contiene la lógica de supervisión y operación del sistema. Incluye:
- control de motores mediante ODrive,
- interfaz gráfica para monitorizar estados y ajustar setpoints,
- manejo de señales de relé y seguridad,
- scripts para pruebas, medición de desempeño y análisis de datos.

### 2. Firmware embedded
En [arduino_nano/](arduino_nano/) se almacenan los programas para los microcontroladores que participan en la comunicación del sistema, tanto en el lado maestro como en el esclavo.

### 3. Visión computacional
La carpeta [vision/](vision/) implementa soluciones basadas en OpenCV y MediaPipe para:
- detectar manos y puntos de referencia,
- calcular ángulos articulares,
- generar imágenes procesadas y datos de prueba para evaluación del sistema.

### 4. Diseño mecánico
En [cad_files/](cad_files/) se encuentran recursos CAD relacionados con la estructura física del dispositivo.

## Requisitos

El proyecto utiliza Python y varias librerías como:
- OpenCV
- NumPy
- ODrive
- Bluezero
- Matplotlib
- gpiozero

Se recomienda crear un entorno virtual e instalar las dependencias desde [requirements.txt](requirements.txt).

## Ejecución rápida

1. Crear un entorno virtual:
   ```bash
   python3 -m venv exoglove_env
   source exoglove_env/bin/activate
