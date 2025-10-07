"""
    Nombre archivo: Experimentacion.py
    Autores: Alejandro Orozco Romo
    Fecha de creación: 06/10/2025
    Fecha de edición: O6/10/2025

    Importaciónes: pyRobotics -> Gestiona la simulación y las trayectorias del modelo
                   numpy -> Empleado para la generación de vectores y otros calculos matemáticos.
                   matplotlib -> genera las gráficas para la visualizacion de la velocidades y la posición
                   pyESP32-> gestiona la comunicación wifi de python con esp32
                   time -> gestiona los tiempos de muestreo
    Descripción: Seguimiento de camino: Este código simula el movimiento de un robot tipo diferencial en base a su modelo cinemático.
                 Además, se conecta con el robot en físico, gestiona su movimiento y permite la visualizacion de
                 las graficas de velocidad, posición y control.
"""


#-----------------------------------LIBRERIAS-----------------------------------#
import matplotlib.pyplot as plt
from pyRobotics import *
from pyESP32 import *
import numpy as np
import time

#-----------------------------------TIEMPO-----------------------------------#

# Tiempo de muestreo (100 ms)
ts = 0.1
# Tiempo de simulación
tf = 60
# Vector de tiempo
t = np.arange(0, tf + ts, ts)
# Número de muestras
N = len(t)
# Distancia eje de las ruedas hacia el punto de interez
#a = 0.05 # en metros
a = 0.30
#-----------------------------------CONDICIONES INICIALES-----------------------------------#

# Asignar memoria para las variables
x1 = np.zeros(N + 1)
y1 = np.zeros(N + 1)
phi = np.zeros(N + 1)

# Punto de interez
hx = np.zeros(N + 1)
hy = np.zeros(N + 1)

x1[0] = 0
y1[0] = 0
phi[0] = 0 * (np.pi/180)

hx[0] = x1[0] + a * np.cos(phi[0])
hy[0] = y1[0] + a * np.sin(phi[0])

#-----------------------------------VELOCIDADES DE REFERENCIA-----------------------------------#

# Enviamos al robot velocidades de referencia globales, modificamos para movimiento
# Variable para la velocidad lineal
#uRef = 0.2 * np.sin(0.8 * t)
uRef = 0.8 * np.ones(N)
# Variable para la velocidad angular
wRef = 0.8 * np.ones(N)

#-----------------------------------VELOCIDADES MEDIDAS-----------------------------------#

# Recibimos del robot velocidades globales de medidas
#uMeas = np.zeros(N)
#wMeas = np.zeros(N)

#-----------------------------------VARIABLES COMUNICACIÓN WIFI-----------------------------------#
"""# IP y puerto del ESP32 maestro
#SERVER_IP = "192.168.100.10"
SERVER_IP = "192.168.100.16"
SERVER_PORT = 8084
esp = wifiESP32(SERVER_IP, SERVER_PORT, 2)
esp.readSerialStart()"""

#-----------------------------------BUCLE DE EJECUCION-----------------------------------#

for k in range(N):

    #start_time = time.time()

    # Enviar las velocidades de referencia a nuestra esp32
    #esp.sendData([uRef[k], wRef[k]])

    # Obtenemos cada una de las variables medidas
    #uMeas[k] = esp.rawData[0]
    #wMeas[k] = esp.rawData[0]

    # Metodo euler adelante
    #phi[k + 1] = phi[k] + ts * wMeas[k]
    phi[k + 1] = phi[k] + ts * wRef[k] # Solo para simulacion sin experimento

    # Velocidades en nuestro punto de control en x, y (en metros)
    #hxp = uMeas[k] * np.cos(phi[k + 1])
    #hyp = uMeas[k] * np.sin(phi[k + 1])

    x1p = uRef[k] * np.cos(phi[k + 1])
    y1p = uRef[k] * np.sin(phi[k + 1])
    # Posiciones
    x1[k + 1] = x1[k] + ts * x1p
    y1[k + 1] = y1[k] + ts * y1p

    # Aplicamos nuevamente la cinematica directa
    hx[k + 1] = x1[k + 1] + a * np.cos(phi[k + 1])
    hy[k + 1] = y1[k + 1] + a * np.sin(phi[k + 1])

    # Mantenemos tiempo de muestreo con el tiempo transcurrido
    #elapsed_time = time.time() - start_time
    # Aseguramos que el codigo se ejecute cada 100 ms
    #time.sleep(ts - elapsed_time)

#-----------------------------------COMUNICACION WIFI-----------------------------------#

"""esp.sendData([0, 0])
esp.close()"""

#-----------------------------------SIMULACIÓN-----------------------------------#

# Cargamos dirección de los archivos desde la carpeta
path = "stl_2"
#path = "stl"
# Asignamos un color
#color = ["yellow","black","gray","gray","white","blue"]
color = ["blue"]  # un solo color
# Inicializamos el objeto
diferencial = robotics(path, color)
# Configuramos nuestro entorno
xmin = -3
xmax = 3
ymin = -3
ymax = 3
zmin = 0
zmax = 2
# Lista de rangos de nuestro entorno
bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
# Configuramos la esena
diferencial.configureScene(bounds)
# Inicializamos nuestra trayectoria
diferencial.initTrajectory(hx, hy)
# Definimos la escala del robot
escala = 0.001
#escala = 5
#diferencial.initRobot(hx, hy, phi, escala)
diferencial.initRobot(x1, y1, phi, escala)
# Inicializamos la información
diferencial.startSimulation(1, ts)
# Mantener ventana abierta
diferencial.plotter.app.exec_()

#-----------------------------------MOSTRAR GÁFICAS-----------------------------------#

"""# Gráfica velocidad lineal
plt.figure()
plt.plot(t, uMeas, label="Velocidad lineal medida")
plt.plot(t, uRef, label="Velocidad lineal referencia")
plt.legend(loc="upper left")
plt.xlabel("Tiempo [S]")
plt.ylabel("Velocidad[m/s]")
plt.grid()

# Gráfica velocidad angular
plt.figure()
plt.plot(t, wMeas, label="Velocidad angular medida")
plt.plot(t, wRef, label="Velocidad angular referencia")
plt.legend(loc="upper left")
plt.xlabel("Tiempo [S]")
plt.ylabel("Velocidad[rad/s]")
plt.grid()

# Mostramos las gráficas
plt.show()
"""