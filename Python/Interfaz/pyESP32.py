from threading import Thread
import socket
import time
import sys

class wifiESP32:
    def __init__(self, host, port=12345, sizeData=1):
        """
        Clase para comunicarse con ESP32 vía Wi-Fi (TCP) usando nombres idénticos
        a la versión serial.
        """
        self.host = host
        self.port = port
        self.sizeData = sizeData

        self.isReceiving = False
        self.isRun = True
        self.thread = None

        self.rawData = [None] * self.sizeData
        print('Trying to connect to: ' + str(self.host))
        try:
            self.serialConnection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.serialConnection.connect((self.host, self.port))
            print('Connected to ' + str(self.host))
        except:
            sys.exit("Failed to connect with " + str(self.host))

    # ---------------- Lectura en segundo plano ----------------
    def readSerialStart(self):
        if self.thread is None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            while self.isReceiving != True:
                print("Starting receive data")
                time.sleep(0.1)
            print("Receiving Data")

    def backgroundThread(self):
        """Lee datos del ESP32 continuamente en un hilo."""
        buffer = ""
        while self.isRun:
            try:
                data = self.serialConnection.recv(1024).decode()  # recibir datos del ESP32
                if not data:
                    continue
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    values = line.strip().split(',')
                    if len(values) == self.sizeData:
                        try:
                            self.rawData = [float(v) for v in values]
                            self.isReceiving = True
                        except:
                            sys.exit("Error data receive")
            except Exception as e:
                print("Error en recepción:", e)
                self.isRun = False

    # ---------------- Envío de datos ----------------
    def sendData(self, dataToSend, separator=','):
        """Envía una lista de datos al ESP32"""
        stringData = ""
        sizeSendData = len(dataToSend)
        for k in range(sizeSendData):
            if k < sizeSendData - 1:
                stringData += str(dataToSend[k]) + ','
            else:
                stringData += str(dataToSend[k])

        try:
            self.serialConnection.sendall((stringData + '\n').encode())
        except Exception as e:
            print("Error sending data:", e)

    # ---------------- Cierre de comunicación ----------------
    def close(self):
        self.isRun = False
        if self.thread is not None:
            self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
