"""
    Nombre archivo: Simulacion.py
    Autores: Alejandro Orozco Romo
    Fecha de creación: 06/10/2025
    Fecha de edición: O6/10/2025

    Importaciónes: pyRobotics -> Gestiona la simulación y las trayectorias del modelo
                   numpy -> Empleado para la generación de vectores y otros calculos matemáticos.
                   matplotlib -> genera las gráficas para la visualizacion de la velocidades y la posición
    Descripción: Seguimiento de camino: Este código simula el movimiento de un robot tipo diferencial en base a su modelo cinemático.
"""

#-----------------------------------LIBRERIAS-----------------------------------#
import matplotlib.pyplot as plt
from pyRobotics import *
import numpy as np

#-----------------------------------TIEMPO-----------------------------------#

# Tiempo de muestreo (100 ms)
ts = 0.1
# Tiempo de simulación
tf = 100
# Vector de tiempo
t = np.arange(0, tf + ts, ts)
# Número de muestras
N = len(t)

#-----------------------------------PARAMETROS DEL ROBOT-----------------------------------#

# Distancia eje de las ruedas hacia el punto de interez
#a = 0.07  # en metros
a = 0.30

#-----------------------------------CONDICIONES INICIALES-----------------------------------#

# Asignar memoria para las variables
x1 = np.zeros(N + 1)
y1 = np.zeros(N + 1)
phi = np.zeros(N + 1)
# Punto de interez
hx = np.zeros(N + 1)
hy = np.zeros(N + 1)

# Posiciones iniciales eje central de las ruedas

x1[0] = -4 # posicion inicial en el camino
y1[0] = 2
#x1[0] = -3 # posicion inicial Sigsag
#y1[0] = 1
#x1[0] = 0 # posicion inicial octagono
#y1[0] = 0

# Posicion inicial orientación [rad]
phi[0] = 0 * (np.pi/180)

# CINEMARICA DIRECTA
# Posicion inicial en el eje x [m]
hx[0] = x1[0] + a * np.cos(phi[0])
# Posicion inicial en el eje y [m]
hy[0] = y1[0] + a * np.sin(phi[0])

#-----------------------------------CAMINO DESEADO N LINEAS-----------------------------------#

vMax = 0.15
div = 700
px = []
py = []

pointX = [-4, 4, 4, -4, -4, 4, 4, -4, -4]
pointY = [2, 2, 1, 1, 0, 0, -1, -1, -2]


# SIGSAG
#pointX = [-3, 3, 3, -3, -3, 3, 3, -3, -3]
#pointY = [1, 1, 0.5, 0.5, 0, 0, -0.5, -0.5, -1]

# OCTAGONO El mas seguro y con menos error
#pointX = [-2, -1, 1, 2, 2.1, 1, -1, -2, -2]
#pointY = [0.5, 1, 1, 0.5, -0.5, -1, -1, -0.5, 0.5]

#pointX = [-2, 1.5, 2, 1.5, -2, -1.5, 2, 1.5]
#pointY = [-1, -0.8, -0.5, -0.2, 0, 0.2, 0.5, 0.8]


#theta = np.linspace(0, 2*np.pi, 40)
#pointX = list(np.sin(theta))
#pointY = list(np.sin(theta) * np.cos(theta))



for p in range(len(pointX)-1):
    px.append(np.linspace(pointX[p], pointX[p + 1], div))
    py.append(np.linspace(pointY[p], pointY[p + 1], div))

# Concatenamos las lineas
pxd = np.hstack(px)
pyd = np.hstack(py)

# Conocer la cantidad de puntos
sizePoint = len(pxd)

# Angulo beta
beta = np.zeros(sizePoint)

# Usamos la formula para calcular beta, la de la variacion
for p in range(sizePoint):
    #if p == 1:
    if p == 0:
        beta[p] = np.arctan2(pyd[p + 1] - pyd[p], pxd[p + 1] - pxd[p]);
    else:
        beta[p] = np.arctan2(pyd[p] - pyd[p - 1], pxd[p] - pxd[p - 1]);

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

    # CALCULAR PUNTO MAS CERCANO
    # Minima distancia del punto
    minimo = 100
    for p in range(sizePoint):
        # Usamos distancia euclidiana
        aux = np.sqrt((pxd[p] - hx[k]) ** 2 + (pyd[p] - hy[k]) ** 2)
        if aux < minimo:
            minimo = aux
            pos = p
            # --- ADELANTO DE PUNTO OBJETIVO (LOOKAHEAD) ---
            # En lugar de seguir el punto más cercano, el robot apunta unos pasos más adelante
            # sobre la trayectoria (lookahead). Esto evita que se detenga o se trabe en las esquinas,
            # generando un movimiento más suave y fluido, similar a "mirar hacia adelante" al conducir.
            lookahead = 10  # puedes ajustar entre 5 y 20 según el tipo de curva
            pos = min(pos + lookahead, sizePoint - 1)

    # ERRORES
    hxe[k] = pxd[pos] - hx[k]
    hye[k] = pyd[pos] - hy[k]
    # Errores de forma matricial en vector columna 2 x 1
    he = np.array([[hxe[k]], [hye[k]]])

    # MATRIZ JACOBIANA
    J = np.array([[np.cos(phi[k]), - a * np.sin(phi[k])],
                  [np.sin(phi[k]),  a * np.cos(phi[k])]])

    # PARAMETROS DE CONTROL
    K = np.array([[1, 0],
                 [0, 5]])

    # GENERAR VELOCIDADES DESEADAS ( derivada punto deseadopdp)
    pxdp = vMax * np.cos(beta[pos])
    pydp = vMax * np.sin(beta[pos])
    pdp = np.array([[pxdp], [pydp]])

    # LEY DE CONTROL (inversa matriz jacobiana)
    qpRef = np.linalg.pinv(J)@(pdp + K@he) # @ se usa para multiplicar matrices

    # -----------------------------------APLICAR ACCION DE CONTROL-----------------------------------#

    # Obtenemos la velocidad lineal de referencia de qpRef (matris 2x1) (derivada de qref)
    uRef[k] = qpRef[0][0]
    # Obtenemos la velocidad angular de referencia de qpRef (matris 2x1) (derivada de qref)
    wRef[k] = qpRef[1][0]

    # Orientación integral numerica
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
path = "stl_2"
#path = "stl"
# Asignamos un color
#color = ["yellow","black","gray","gray","white","blue"]
color = ["blue"]  # un solo color
# Inicializamos el objeto
diferencial = robotics(path, color)

# CONFIGURAMOS LA ESCENA
xmin = -3
xmax = 3
ymin = -3
ymax = 3
zmin = 0
zmax = 2
bounds = [xmin, xmax, ymin, ymax, zmin, zmax]  # Lista de rangos de nuestro entorno
diferencial.configureScene(bounds)  # Configuramos la esena

# MOSTRAR GRAFICAS
diferencial.plotDesiredTrajectory(pxd, pyd)  # Graficar trayectoria deseada
diferencial.initTrajectory(hx, hy)  # Graficar trayectoria hecha por el robot

# MOSTRAR ROBOTS
escala = 0.001
#escala = 5
#diferencial.initRobot(hx, hy, phi, escala)
diferencial.initRobot(x1, y1, phi, escala)

# INICIAR SIMULACION
diferencial.startSimulation(1, ts)
#diferencial.plotter.app.exec_() # Mantener ventana abierta

#-----------------------------------MOSTRAR GRAFICAS-----------------------------------#

# Errores
fig = plt.figure()
plt.plot(t,hxe,'b',linewidth = 2, label='hxe')
plt.plot(t,hye,'r',linewidth = 2,  label='hye')
plt.legend(loc='upper right')
plt.xlabel('Tiempo [s]')
plt.ylabel('Error [m]')
plt.grid()

# Acciones de control
fig = plt.figure()
plt.subplot(211)
plt.plot(t, uRef, linewidth=2, label='Velocidad lineal referencia')
plt.legend(loc='upper right')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad [m/s]')
plt.grid()

plt.subplot(212)
plt.plot(t, wRef, 'r', linewidth=2, label='Velocidad angular referencia')
plt.legend(loc='upper right')
plt.xlabel('Tiempo [s]')
plt.ylabel('Velocidad [rad/s]')
plt.grid()

plt.show()

