import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk

def detectar_amarillo(frame):
    # Convierte el frame a espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define los límites inferior y superior del color amarillo en HSV
    lower_yellow = np.array([20, 150, 100])
    upper_yellow = np.array([30, 255, 255])

    # Crea una máscara para el color amarillo
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Encuentra los contornos de los objetos amarillos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Itera sobre cada contorno encontrado
    for contour in contours:
        # Calcula el área del contorno
        area = cv2.contourArea(contour)

        # Si el área es muy pequeña, ignora este contorno
        if area < 100:
            continue

        # Calcula los momentos del contorno para encontrar el centroide
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # Dibuja un punto rojo en el centroide
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)

        # Imprime las coordenadas del centroide
        print(f"Coordenadas del centroide amarillo: ({cX}, {cY})")

    return frame

def capturar_imagen():
    global cap, canvas, window

    ret, frame = cap.read()
    if ret:
        frame = detectar_amarillo(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame)
        photo = ImageTk.PhotoImage(image)

        canvas.create_image(0, 0, image=photo, anchor=tk.NW)
        canvas.image = photo
        window.update_idletasks()

def cerrar_programa():
    global cap, window

    cap.release()
    window.destroy()

cap = cv2.VideoCapture(0)

window = tk.Tk()
window.title("Detección de amarillo")

canvas = tk.Canvas(window, width=640, height=480)
canvas.pack()

boton_capturar = tk.Button(window, text="Capturar imagen", command=capturar_imagen)
boton_capturar.pack()

boton_salir = tk.Button(window, text="Salir", command=cerrar_programa)
boton_salir.pack()

window.mainloop()
