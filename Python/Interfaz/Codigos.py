"""
CODIGO SOLO MUESTRA EL MODELO MATEMATICO DEL ROBOT DIFERECNAIL CON EL PUNTO YA DESPLAZADO

#-----------------------------------LIBRERIAS-----------------------------------#
from pyRobotics import *
import numpy as np

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
a = 0.05 # en metros
#a = 0.30
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

#-----------------------------------BUCLE DE EJECUCION-----------------------------------#

for k in range(N):

    # Metodo euler adelante
    #phi[k + 1] = phi[k] + ts * wMeas[k]
    phi[k + 1] = phi[k] + ts * wRef[k] # Solo para simulacion sin experimento

    x1p = uRef[k] * np.cos(phi[k + 1])
    y1p = uRef[k] * np.sin(phi[k + 1])
    # Posiciones
    x1[k + 1] = x1[k] + ts * x1p
    y1[k + 1] = y1[k] + ts * y1p

    # Aplicamos nuevamente la cinematica directa
    hx[k + 1] = x1[k + 1] + a * np.cos(phi[k + 1])
    hy[k + 1] = y1[k + 1] + a * np.sin(phi[k + 1])

#-----------------------------------SIMULACIÓN-----------------------------------#

# Cargamos dirección de los archivos desde la carpeta
#path = "stl_2"
path = "stl"
# Asignamos un color
color = ["yellow","black","gray","gray","white","blue"]
#color = ["blue"]  # un solo color
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
#escala = 0.001
escala = 5
#diferencial.initRobot(hx, hy, phi, escala)
diferencial.initRobot(x1, y1, phi, escala)
# Inicializamos la información
diferencial.startSimulation(1, ts)
# Mantener ventana abierta
diferencial.plotter.app.exec_()

"""

"""
#### CONTROL DE POSICION SIN ORIENTACION 
#-----------------------------------LIBRERIAS-----------------------------------#
import matplotlib.pyplot as plt
from pyRobotics import *
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
a = 0.05 # en metros
#a = 0.30
#-----------------------------------CONDICIONES INICIALES-----------------------------------#

# Asignar memoria para las variables
x1 = np.zeros(N + 1)
y1 = np.zeros(N + 1)
phi = np.zeros(N + 1)
# Punto de interez
hx = np.zeros(N + 1)
hy = np.zeros(N + 1)

# Posiciones iniciales eje central de las ruedas
x1[0] = 0
y1[0] = 0
# Posicion inicial orientación [rad]
phi[0] = 0 * (np.pi/180)

# CINEMARICA DIRECTA
# Posicion inicial en el eje x [m]
hx[0] = x1[0] + a * np.cos(phi[0])
# Posicion inicial en el eje y [m]
hy[0] = y1[0] + a * np.sin(phi[0])

#-----------------------------------POSICION DESEADA-----------------------------------#

hxd = 2
hyd = 2

#-----------------------------------VELOCIDADES DE REFERENCIA-----------------------------------#

# Inicializamos la variable para la velocidad lineal
uRef = np.zeros(N)
# Inicializamos la variable para la velocidad angular
wRef = np.zeros(N)
# El algoritmo de control se encargará de calcular esas velocidades, por eso nadie multiploca
# el np.zeros(N)

#-----------------------------------ERRORES-----------------------------------#

hxe = np.zeros(N)
hye = np.zeros(N)

#-----------------------------------BUCLE DE EJECUCION-----------------------------------#

for k in range(N):

    # -----------------------------------CONTROL-----------------------------------#

    # Errores
    hxe[k] = hxd - hx[k]
    hye[k] = hyd - hy[k]
    # Errores de forma matricial en vector columna 2 x 1
    he = np.array([[hxe[k]], [hye[k]]])
    # Ingresamos la matriz Jacobiana
    J = np.array([[np.cos(phi[k]), - a * np.sin(phi[k])],
                  [np.sin(phi[k]), - a * np.cos(phi[k])]])
    # Parámetros del control
    K = np.array([[0.1, 0],
                 [0, 0.1]])
    # Ley de control (inversa matriz jacobiana)
    qpRef = np.linalg.pinv(J)@K@he  # @ se usa para multiplicar matrices

    # -----------------------------------APLICAR ACCION DE CONTROL-----------------------------------#

    # Obtenemos la velocidad lineal de referencia de qpRef (matris 2x1) (derivada de qref)
    uRef[k] = qpRef[0][0]
    # Obtenemos la velocidad angular de referencia de qpRef (matris 2x1) (derivada de qref)
    wRef[k] = qpRef[1][0]

    # Orientación
    phi[k + 1] = phi[k] + ts * wRef[k]  # Solo para simulacion sin experimento
    # Modelo cinemático para encontrar velocidades en el centro del eje de las ruedas
    x1p = uRef[k] * np.cos(phi[k + 1])
    y1p = uRef[k] * np.sin(phi[k + 1])
    # Integral numerica para las posiciones
    x1[k + 1] = x1[k] + ts * x1p
    y1[k + 1] = y1[k] + ts * y1p
    # Cinematica directa
    hx[k + 1] = x1[k + 1] + a * np.cos(phi[k + 1]) # Posición inicial en eje x [m]
    hy[k + 1] = y1[k + 1] + a * np.sin(phi[k + 1]) # Posición inicial en eje y [m]

#-----------------------------------SIMULACIÓN-----------------------------------#

# Cargamos dirección de los archivos desde la carpeta
#path = "stl_2"
path = "stl"
# Asignamos un color
color = ["yellow","black","gray","gray","white","blue"]
#color = ["blue"]  # un solo color
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
#escala = 0.001
escala = 5
#diferencial.initRobot(hx, hy, phi, escala)
diferencial.initRobot(x1, y1, phi, escala)
# Inicializamos la simulación
diferencial.startSimulation(1, ts)
# Mantener ventana abierta

#-----------------------------------MOSTRAR GÁFICAS-----------------------------------#

# ERRORES
fig = plt.figure()
plt.plot(t, hxe, 'b', linewidth=2, label="hxe")
plt.plot(t, hye, 'r', linewidth=2, label="hye")
plt.legend(loc="upper right")
plt.xlabel("Tiempo [s]")
plt.ylabel("Error [m]")
plt.grid()

# ACCIONES DE CONTROL

# Velocidad lineal de referencia
fig = plt.figure()
plt.subplot(211)
plt.plot(t, uRef, 'b', linewidth=2, label="uRef")
plt.legend(loc="upper right")
plt.xlabel("Tiempo [s]")
plt.ylabel("Velocidad [m/s]")
plt.grid()

# Velocidad angular de referencia
plt.subplot(212)
plt.plot(t, wRef, 'b', linewidth=2, label="wRef")
plt.legend(loc="upper right")
plt.xlabel("Tiempo [s]")
plt.ylabel("Velocidad [rad/s]")
plt.grid()

plt.show()

diferencial.plotter.app.exec_()
"""
"""
## CAMINOS
#-----------------------------------LIBRERIAS-----------------------------------#
import matplotlib.pyplot as plt
from pyRobotics import *
import numpy as np

#-----------------------------------TIEMPO-----------------------------------#

# Tiempo de muestreo (100 ms)
ts = 0.1
# Tiempo de simulación
tf = 60
# Vector de tiempo
t = np.arange(0, tf + ts, ts)
# Número de muestras
N = len(t)

#-----------------------------------PARAMETROS DEL ROBOT-----------------------------------#

# Distancia eje de las ruedas hacia el punto de interez
a = 0.05  # en metros
#a = 0.30

#-----------------------------------CONDICIONES INICIALES-----------------------------------#

# Asignar memoria para las variables
x1 = np.zeros(N + 1)
y1 = np.zeros(N + 1)
phi = np.zeros(N + 1)
# Punto de interez
hx = np.zeros(N + 1)
hy = np.zeros(N + 1)

# Posiciones iniciales eje central de las ruedas
x1[0] = - 2
y1[0] = 0
# Posicion inicial orientación [rad]
phi[0] = 0 * (np.pi/180)

# CINEMARICA DIRECTA
# Posicion inicial en el eje x [m]
hx[0] = x1[0] + a * np.cos(phi[0])
# Posicion inicial en el eje y [m]
hy[0] = y1[0] + a * np.sin(phi[0])

#-----------------------------------CAMINO DESEADO-----------------------------------#

vMax = 0.15

# Camino 1 linea
pxd1 = 0
pxd2 = 1
pyd1 = 0
pyd2 = 0.8
# Cantidad de subiviciones
div = 250
# Subdividimos la recta
pxd = np.linspace(pxd1, pxd2, div)
pyd = np.linspace(pyd1, pyd2, div)

# Camino 2 lineas
pointX = [0, 1, 1, 2] # repetimos el valor final de la coordenada anterior y luego a donde queremos que valla
pointY = [0, 0.8, 0.8, 1.2]

px = []
py = []

# Concatenamos los valores de cada punto de la linea 1
px.append(np.linspace(pointX[0], pointX[1], div))
py.append(np.linspace(pointY[0], pointY[1], div))
# Concatenamos los valores de cada punto de la linea 2
px.append(np.linspace(pointX[2], pointX[3], div))
py.append(np.linspace(pointY[2], pointY[3], div))

# Concatenamos las lineas
pxd = np.hstack(px)
pyd = np.hstack(py)

# Camino N lineas
div = 250
px = []
py = []

pointX = [-2, -1, 1, 2, 2.1, 1, -1, -2, -2]
pointY = [0.5, 1, 1, 0.5, -0.5, -1, -1, -0.5, 0.5]

for p in range(len(pointX)-1):
    px.append(np.linspace(pointX[p], pointX[p + 1], div))
    py.append(np.linspace(pointY[p], pointY[p + 1], div))
    # Concatenamos las lineas
pxd = np.hstack(px)
pyd = np.hstack(py)

fig = plt.figure()
plt.plot(pxd, pyd, 'b', linewidth=2, label='camino deseado')
plt.legend(loc = 'upper right')
plt.xlabel("x[m]")
plt.ylabel("y[m]")
plt.grid()
plt.show()"""