import asyncio
import websockets
import keyboard
import tkinter as tk
from tkinter import ttk
import time

# Variables globales
last_command = ""
connection_status = "Desconectado"
command_sent = "Ninguno"
ws_connection = None

# IP y puerto
SERVER_IP = "192.168.100.10"
SERVER_PORT = 8084

# Tkinter ventana
root = tk.Tk()
root.title("Control ESP32 por WebSocket")
root.geometry("400x250")
root.configure(bg="#1e1e2f")

# Fuentes y estilo
style = ttk.Style()
style.configure("TLabel", background="#1e1e2f", foreground="white", font=("Segoe UI", 14))
style.configure("Header.TLabel", foreground="#8aff80", font=("Segoe UI", 18, "bold"))
style.configure("Value.TLabel", foreground="#00ffff", font=("Segoe UI", 16))

# Widgets
ttk.Label(root, text="Estado de conexión:", style="Header.TLabel").pack(pady=10)
status_label = ttk.Label(root, text=connection_status, style="Value.TLabel")
status_label.pack()

ttk.Label(root, text="Último comando enviado:", style="Header.TLabel").pack(pady=10)
command_label = ttk.Label(root, text=command_sent, style="Value.TLabel")
command_label.pack()


def update_ui():
    status_label.config(text=connection_status)
    command_label.config(text=command_sent)
    root.after(100, update_ui)  # actualiza cada 100 ms


def checkKeys():
    if keyboard.is_pressed("up"):
        return "forward"
    elif keyboard.is_pressed("down"):
        return "stop"
    elif keyboard.is_pressed("left"):
        return "left"
    elif keyboard.is_pressed("right"):
        return "right"
    else:
        return "stop"


async def handle_ws(websocket):
    global last_command, command_sent, connection_status, ws_connection
    ws_connection = websocket
    connection_status = " Conectado"
    try:
        while True:
            command = checkKeys()
            if command != last_command:
                await websocket.send(command)
                print("Enviado:", command)
                command_sent = command
                last_command = command

            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=0.1)
                print("ESP32 →", response)
            except asyncio.TimeoutError:
                pass

            await asyncio.sleep(0.05)
    except websockets.ConnectionClosed:
        connection_status = " Desconectado"
        print("Conexión cerrada")
        command_sent = "Ninguno"
        await asyncio.sleep(1)


async def main_ws():
    global connection_status
    print("Esperando conexión en", f"{SERVER_IP}:{SERVER_PORT}")
    connection_status = "🟡 Esperando conexión..."
    async with websockets.serve(handle_ws, SERVER_IP, SERVER_PORT):
        await asyncio.Future()  # ejecuta indefinidamente


def iniciar_asyncio():
    loop = asyncio.get_event_loop()
    loop.create_task(main_ws())
    loop.run_forever()


# Lanzar UI
root.after(100, update_ui)
import threading

def run_loop():
    asyncio.run(main_ws())

threading.Thread(target=run_loop, daemon=True).start()
root.after(100, update_ui)
root.mainloop()

