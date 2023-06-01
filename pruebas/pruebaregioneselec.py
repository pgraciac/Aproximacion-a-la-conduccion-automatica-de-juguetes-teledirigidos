import cv2
import numpy as np
import tkinter as tk
from tkinter import messagebox

# Inicializa variables globales
rois = []
current_roi = []

def mouse_callback(event, x, y, flags, param):
    global current_roi

    if event == cv2.EVENT_LBUTTONDOWN:
        current_roi = [(x, y)]
    elif event == cv2.EVENT_LBUTTONUP:
        current_roi.append((x, y))
        rois.append(current_roi)
        current_roi = []

def on_mark_regions_button_click():
    messagebox.showinfo("Marca las regiones", "Marca las regiones de interés en la ventana 'Detección de amarillo'")

def detectar_amarillo(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 150, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 100:
            continue
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"({cX}, {cY})", (cX - 50, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return frame

def capturar_imagen():
    ret, frame = cap.read()
    if ret:
        frame = detectar_amarillo(frame)
        cv2.imshow("Deteccion de amarillo", frame)

# Crea la ventana de la GUI y el botón
root = tk.Tk()
mark_regions_button = tk.Button(root, text="Marcar regiones", command=on_mark_regions_button_click)
mark_regions_button.pack()

cap = cv2.VideoCapture(1)
cv2.namedWindow("Deteccion de amarillo")
cv2.setMouseCallback("Deteccion de amarillo", mouse_callback)

while True:
    key = cv2.waitKey(1) & 0xFF

    if key == ord("r"):
        capturar_imagen()
    elif key == ord("q"):
        break

    # Actualiza la ventana de la GUI
    root.update()

cap.release()
cv2.destroyAllWindows()
root.destroy()
