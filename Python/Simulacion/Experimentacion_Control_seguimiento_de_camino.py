#-----------------------------------LIBRERIAS-----------------------------------#

from pyRobotics import *
import numpy as np

#-----------------------------------VARIABLES-----------------------------------#

# Tiempo de muestreo
ts = 0.1
# Tiempo de simulación
tf = 10
# Vector de tiempo
t = np.arange(0, tf + ts, ts)
# Número de muestras
N = len(t)
# Desplazamiento en x, y
hx = 0.5*np.cos(0.6 * t)
hy = 0.5*np.sin(0.6 * t)
# Angulo de inclinación phi (en radianes)
phi = 45*(np.pi/180.0) * np.ones(N)

#-----------------------------------SIMULACIÓN-----------------------------------#

# Cargamos dirección de los archivos desde la carpeta
path = "stl"
# Asignamos un color
color = ['yellow','black','gray','gray','white','blue']
# Inicializamos el objeto
diferencial = robotics(path, color)
# Configuramos nuestro entorno
xmin = -2
xmax = 2
ymin = -2
ymax = 2
zmin = 0
zmax = 2
# Lista de rangos de nuestro entorno
bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
# Configuramos la esena
diferencial.configureScene(bounds)
# Inicializamos nuestra trayectoria
diferencial.initTrajectory(hx, hy)
# Definimos la escala del robot
escala = 5
diferencial.initRobot(hx, hy, phi, escala)
# Inicializamos la información
diferencial.startSimulation(1, ts)


