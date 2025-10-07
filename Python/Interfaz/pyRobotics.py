import glob
import numpy as np
import pyvista as pv
from pyvistaqt import BackgroundPlotter


class robotics:
    def __init__(self, path="", color=None):
        self.path = path
        self.color = color or []
        self.robot = []
        self.robotCopy = []
        self.isTrajectory = False

        # Cargar todos los STL
        filenames = glob.glob(self.path + "/*.stl")
        for filename in filenames:
            mesh = pv.read(filename)
            self.robot.append(mesh)
            self.robotCopy.append(mesh.copy())

    # --------------------------- Escena y Grid ---------------------------#
    def configureScene(self, bounds=[-2, 2, -2, 2, 0, 1], window_size=(1024, 768), title="Python Robotics"):
        """
        Configura la escena con grid y cámara fijos.
        bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
        """
        self.bounds = bounds
        self.plotter = BackgroundPlotter(window_size=window_size, title=title)
        self.plotter.set_background("white")

        # Fijar grid y límites
        self.plotter.show_bounds(grid="front", location="outer", color="#000000",
                                 xtitle="X[m]", ytitle="Y[m]", ztitle="Z[m]",
                                 bounds=self.bounds)

        # Configura cámara isométrica y evita auto-reset
        self.plotter.view_isometric()
        self.plotter.enable_parallel_projection()
        self.plotter.renderer.ResetCameraClippingRange()
        self.plotter.render()

    # --------------------------- Robot ---------------------------#
    def initRobot(self, x1, y1, phi, escala):
        """
        Inicializa el robot con posición y escala fijas.
        """
        self.x1 = np.array(x1)
        self.y1 = np.array(y1)
        self.phi = np.array(phi)
        self.escala = escala

        for i in range(len(self.robot)):
            self.robot[i].points *= self.escala
            self.robotCopy[i].points = self.robot[i].points.copy()

            mesh_color = self.color[i] if i < len(self.color) else "gray"
            self.plotter.add_mesh(self.robotCopy[i], color=mesh_color)

        # Asegura que la cámara no cambie automáticamente
        self.plotter.reset_camera()

    # --------------------------- Trayectoria ---------------------------#
    def initTrajectory(self, hx, hy):
        """
        Inicializa la trayectoria sin dibujar toda la línea.
        """
        self.isTrajectory = True
        self.hx = np.array(hx)
        self.hy = np.array(hy)

        # Primer punto de la trayectoria
        self.traj_points = np.column_stack(([hx[0]], [hy[0]], [0]))
        self.trajectory_line = pv.PolyData(self.traj_points)

        # Línea vacía inicial
        self.trajectory_line.lines = np.array([2, 0, 0])
        self.traj_actor = self.plotter.add_mesh(self.trajectory_line, color="red", line_width=4)

    def plotDesiredTrajectory(self, hxd, hyd):
        """
        Dibuja la trayectoria deseada completa (azul).
        """
        points = np.column_stack((hxd, hyd, np.zeros_like(hxd)))
        spline = pv.Spline(points, len(hxd))
        self.plotter.add_mesh(spline, color="blue", line_width=4)

    # --------------------------- Simulación ---------------------------#
    def startSimulation(self, step=1, ts=0.1):
        """
        Inicia la simulación con callback.
        """
        self.step = step
        self.ts = ts
        self.k = 0

        self.plotter.add_callback(self.simulation_step, int(self.ts * 1000))

    def simulation_step(self):
        if self.k >= len(self.x1):
            return

        # Actualiza posición del robot
        self.robotUniciclo(self.x1[self.k], self.y1[self.k], self.phi[self.k])

        # Dibuja la trayectoria progresivamente
        if self.isTrajectory and self.k > 0:
            new_point = np.array([[self.x1[self.k], self.y1[self.k], 0]])
            self.traj_points = np.vstack((self.traj_points, new_point))

            # Actualiza puntos y líneas sin remover actor
            self.trajectory_line.points = self.traj_points
            n = len(self.traj_points)
            lines = []
            for i in range(n - 1):
                lines.extend([2, i, i + 1])
            self.trajectory_line.lines = np.array(lines)

        self.k += self.step

    # --------------------------- Cinemática ---------------------------#
    def robotUniciclo(self, x, y, phi):
        """
        Actualiza la posición y orientación del robot tipo uniciclo.
        """
        Rz = np.array([[np.cos(phi), -np.sin(phi), 0],
                       [np.sin(phi), np.cos(phi), 0],
                       [0, 0, 1]])

        for i in range(len(self.robotCopy)):
            new_points = (Rz @ self.robot[i].points.T).T + np.array([x, y, 0])
            self.robotCopy[i].points = new_points
