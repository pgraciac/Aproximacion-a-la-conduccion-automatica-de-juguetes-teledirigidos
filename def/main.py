import cv2
import numpy as np
import os
from pynput import keyboard
import time
import vision
import gpio
import tkinter as tk

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
            
            if keyboard.Key.up in self.keys_pressed and keyboard.Key.right in self.keys_pressed:
                vision.point =gpio.move("up-right", elapsed_time)
            elif keyboard.Key.up in self.keys_pressed and keyboard.Key.left in self.keys_pressed:
                vision.point =gpio.move("up-left", elapsed_time)
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.right in self.keys_pressed:
                vision.point =gpio.move("down-right", elapsed_time)
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.left in self.keys_pressed:
                vision.point =gpio.move("down-left", elapsed_time)
            else:

                if key == keyboard.Key.left:
                    vision.point =gpio.move("left", elapsed_time)
                elif key == keyboard.Key.up:
                    vision.point =gpio.move("up", elapsed_time)
                elif key == keyboard.Key.right:
                    vision.point =gpio.move("right", elapsed_time)
                elif key == keyboard.Key.down:
                    vision.point =gpio.move("down", elapsed_time)

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
    frame = vision.detectar_verde(frame)
    for i in range(1, len(vision.path)):
        cv2.line(frame, vision.path[i - 1], vision.path[i], (0, 255, 0), 2)
    for region in vision.rois:
        cv2.rectangle(frame, region[0], region[-1],(0,0,0),2)
    cv2.imshow(window_name,frame)


if __name__ == '__main__':
    # os.environ['BLINKA_FT232H'] = 1
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
    

    while listener.running():
        root.update()

        if vision.current_mark != None:
            mostrar_frame(window_name, frame)

    cap.release()
    cv2.destroyAllWindows()
