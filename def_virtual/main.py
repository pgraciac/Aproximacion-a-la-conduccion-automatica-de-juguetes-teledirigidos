import time
import cv2
import tkinter as tk

import numpy as np
from gpio_virtual import move_point
import vision
from gpio_virtual import update_point

def mostrar_frame(window_name,frame):
    for i in range(1, len(vision.path)):
                cv2.line(frame, vision.path[i - 1], vision.path[i], (0, 255, 0), 2)
    for region in vision.rois:
                cv2.rectangle(frame, region[0], region[-1],(0,0,0),2)
    cv2.imshow(window_name,frame)

if __name__ == '__main__':
    window_name = "Deteccion de amarillo"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, vision.main_mouse_callback)
    # Crear la ventana de Tkinter
    root = tk.Tk()
    root.title("Menú")
    
    # Crear botones y asociarlos a sus funciones correspondientes
    btn_marcar_regiones = tk.Button(root, text="Marcar regiones", command=lambda: vision.set_current_mark("regions"))
    btn_dibujar_trayectoria = tk.Button(root, text="Dibujar trayectoria", command=lambda: vision.set_current_mark("path"))

    # Colocar los botones en la ventana
    btn_marcar_regiones.pack(fill=tk.BOTH, expand=True)
    btn_dibujar_trayectoria.pack(fill=tk.BOTH, expand=True)

    cap = cv2.VideoCapture(1)
    red_color = (0, 0, 255)
    cap.read()

    time.sleep(0.3)   
    frame = vision.capturar_imagen(cap)
    mostrar_frame(window_name, frame)
    point = (frame.shape[1] // 2, frame.shape[0] // 2)
    #regions_mask = np.zeros_like(frame)
    warning = False
    while True:
        root.update()

        if vision.current_mark=="path":
            mostrar_frame(window_name,frame)

        if vision.current_mark=="regions":
            mostrar_frame(window_name, frame)
        
        key = cv2.waitKey(1) & 0xFF
        #print("asdf")
        if key == ord("a"):  # Tecla de flecha izquierda
            point = move_point("left", point)
            frame = vision.capturar_imagen(cap)
            if vision.point_in_rois(point):
                cv2.putText(frame, "WARNING: POINT IS IN A INVALID PLACE", (140, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 1)
            update_point(frame, point)
            mostrar_frame(window_name, frame)
            
        elif key == ord("w"):  # Tecla de flecha arriba
            point = move_point("up", point)
            frame = vision.capturar_imagen(cap)
            if vision.point_in_rois(point):
                cv2.putText(frame, "WARNING: POINT IS IN A INVALID PLACE", (140, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 1)            
            update_point(frame, point)
            mostrar_frame(window_name, frame)
            
        elif key == ord("d"):  # Tecla de flecha derecha
            point = move_point("right", point)
            frame = vision.capturar_imagen(cap)
            if vision.point_in_rois(point):
                cv2.putText(frame, "WARNING: POINT IS IN A INVALID PLACE", (140, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 1)
            update_point(frame, point)
            mostrar_frame(window_name, frame)
            
        elif key == ord("s"):  # Tecla de flecha abajo
            point = move_point("down", point)            
            frame = vision.capturar_imagen(cap)
            if vision.point_in_rois(point):
                cv2.putText(frame, "WARNING: POINT IS IN A INVALID PLACE", (140, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 1)            
            update_point(frame, point)
            mostrar_frame(window_name, frame)
            
        if key == ord("q"):
            break
        

    cap.release()
    cv2.destroyAllWindows()