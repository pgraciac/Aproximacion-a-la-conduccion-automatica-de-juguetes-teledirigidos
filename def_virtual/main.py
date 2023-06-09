import time
import cv2
import tkinter as tk
import numpy as np
from pynput import keyboard

from gpio_virtual import move_point
import vision
from gpio_virtual import update_point

class KeyListener:
    def __init__(self, start_time):
        self.start_time = start_time
        self.keys_pressed = set()
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)

    def on_press(self, key):
        self.keys_pressed.add(key)
        print(self.keys_pressed)
        if key in [keyboard.Key.left, keyboard.Key.up, keyboard.Key.right, keyboard.Key.down]:
            if self.start_time is None:
                self.start_time = time.time()

    def on_release(self, key):
        if self.start_time is not None:
            elapsed_time = time.time() - self.start_time
            pixel_distance = int(elapsed_time * 100)
            
            if keyboard.Key.up in self.keys_pressed and keyboard.Key.right in self.keys_pressed:
                vision.point = move_point("up-right", vision.point, pixel_distance)
            elif keyboard.Key.up in self.keys_pressed and keyboard.Key.left in self.keys_pressed:
                vision.point = move_point("up-left", vision.point, pixel_distance)
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.right in self.keys_pressed:
                vision.point = move_point("down-right", vision.point, pixel_distance)
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.left in self.keys_pressed:
                vision.point = move_point("down-left", vision.point, pixel_distance)
            else:

                if key == keyboard.Key.left:
                    vision.point = move_point("left", vision.point, pixel_distance)
                elif key == keyboard.Key.up:
                    vision.point = move_point("up", vision.point, pixel_distance)
                elif key == keyboard.Key.right:
                    vision.point = move_point("right", vision.point, pixel_distance)
                elif key == keyboard.Key.down:
                    vision.point = move_point("down", vision.point, pixel_distance)

            self.start_time = None
        
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)
        
        if key == keyboard.KeyCode.from_char('q'):
            return False



    def start(self):
        self.listener.start()

    def running(self):
        return self.listener.running

def mostrar_frame(window_name,frame):
    update_point(frame, vision.point)
    for i in range(1, len(vision.path)):
        cv2.line(frame, vision.path[i - 1], vision.path[i], (0, 255, 0), 2)
    for region in vision.rois:
        cv2.rectangle(frame, region[0], region[-1],(0,0,0),2)
    cv2.imshow(window_name,frame)


if __name__ == '__main__':
    window_name = "Deteccion de amarillo"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, vision.main_mouse_callback)
    
    root = tk.Tk()
    root.title("Menú")
    
    btn_marcar_regiones = tk.Button(root, text="Marcar regiones", command=lambda: vision.set_current_mark("regions"))
    btn_dibujar_trayectoria = tk.Button(root, text="Dibujar trayectoria", command=lambda: vision.set_current_mark("path"))

    btn_marcar_regiones.pack(fill=tk.BOTH, expand=True)
    btn_dibujar_trayectoria.pack(fill=tk.BOTH, expand=True)

    cap = cv2.VideoCapture(1)
    red_color = (0, 0, 255)
    cap.read()

    time.sleep(0.3)
    frame = vision.capturar_imagen(cap)
    mostrar_frame(window_name, frame)
    vision.point = (frame.shape[1] // 2, frame.shape[0] // 2)
    warning = False
    listener = KeyListener(None)
    listener.start()
    
    last_point = None

    while listener.running():
        root.update()

        if vision.current_mark != None:
            mostrar_frame(window_name, frame)

        if vision.point != last_point:  # Actualizar el frame solo si el punto se movió
            last_point = vision.point
            frame = vision.capturar_imagen(cap)
            if vision.point_in_rois():
                cv2.putText(frame, "WARNING: POINT IS IN A INVALID PLACE", (140, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 1)
            update_point(frame, vision.point)
            mostrar_frame(window_name, frame)

    cv2.destroyAllWindows()

