import cv2
import numpy as np
import math
from pynput import keyboard
import time
import vision
import gpio
import tkinter as tk
import threading

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

def move_to_target(listener, target_point, threshold=100, kp=0.1, kd=0.1):
    if listener.running():
        if target_point is None or vision.point is None:
            print("Error: Target point or current position is None")
            return move_to_target(listener, target_point)

        # Get the initial position
        initial_position = vision.point

        # Move the robot slightly forward to get the second point
        vision.point = gpio.move("up", kp)
        time.sleep(0.5)

        # Get the position after moving
        next_position = vision.point

        # Calculate the robot's orientation
        delta_x_robot = next_position[0] - initial_position[0]
        delta_y_robot = next_position[1] - initial_position[1]
        robot_orientation = math.atan2(delta_x_robot, delta_y_robot)
        robot_orientation = math.degrees(robot_orientation)
        print(f"robot orientation: {robot_orientation}")
        # Calculate the distance to the target
        delta_x_point = target_point[0] - next_position[0]
        delta_y_point = target_point[1] - next_position[1]
        distance = math.sqrt(delta_x_point ** 2 + delta_y_point ** 2)

        # Check if the target is reached
        if distance < threshold:
            print("Target reached")
            return

        # Calculate the angle to the target
        angle_to_target = math.atan2(delta_x_point, delta_y_point)
        angle_to_target = math.degrees(angle_to_target)
        print(f"angle to target: {angle_to_target}")
        # Calculate the angular error
        error_angle = angle_to_target - robot_orientation

        # Normalize the error_angle to be between -pi and pi
        error_angle = (error_angle + math.pi) % (2 * math.pi) - math.pi

        # Decide on the robot's movement based on error_angle
        if abs(error_angle) < math.radians(30):
            # Move forward if the error is small
            vision.point = gpio.move("up", kp)
        elif error_angle > 0:
            # Turn left if the target is to the left
            vision.point = gpio.move("right", kd)
        else:
            # Turn right if the target is to the right
            vision.point = gpio.move("left", kd)

        # Give the robot time to move
        time.sleep(5)
        print("------------------------------------------------")
        # Call this function again to keep moving
        root.after(100, lambda: move_to_target(listener, target_point))

def mostrar_frame(window_name):
    global frame
    vision.detectar_verde(frame)
    
    if vision.point is not None:
        cv2.circle(frame, vision.point, 5, (0, 0, 255), -1)
        cv2.putText(frame, f"{vision.point}", (vision.point[0] - 50, vision.point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    if vision.target_point is not None:
        cv2.circle(frame, vision.target_point, 5, (0, 0, 255), -1)
        cv2.putText(frame, f"{vision.target_point}", (vision.target_point[0] - 50, vision.target_point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    for i in range(1, len(vision.path)):
        cv2.line(frame, vision.path[i - 1], vision.path[i], (0, 255, 0), 2)

    for region in vision.rois:
        cv2.rectangle(frame, region[0], region[-1],(0,0,0),2)

    if vision.point_in_rois():
        cv2.putText(frame, "WARNING: POINT IS IN A INVALID PLACE", (140, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 1)

    cv2.imshow(window_name, frame)


def update_point(cap, window_name, listener):
    global frame
    if listener.running():
        frame = vision.capturar_imagen(cap)
        mostrar_frame(window_name)
        threading.Timer(0.4, update_point, args=(cap, window_name, listener)).start()


if __name__ == '__main__':
    global frame
    # os.environ['BLINKA_FT232H'] = 1
    window_name = "Deteccion de verde"
    cv2.namedWindow(window_name)
    
    root = tk.Tk()
    root.title("Menú")
    
    btn_marcar_regiones = tk.Button(root, text="Marcar regiones", command=lambda: vision.set_current_mark("regions"))
    btn_dibujar_trayectoria = tk.Button(root, text="Dibujar trayectoria", command=lambda: vision.set_current_mark("path"))
    btn_marcar_meta = tk.Button(root, text="Marcar meta", command=lambda: vision.set_current_mark("target_point"))
    btn_llegar_meta = tk.Button(root, text="Llegar a meta", command=lambda: threading.Thread(target=move_to_target, args=(listener, vision.target_point)).start())

    btn_marcar_regiones.pack(fill=tk.BOTH, expand=True)
    btn_dibujar_trayectoria.pack(fill=tk.BOTH, expand=True)
    btn_marcar_meta.pack(fill=tk.BOTH, expand=True)

    cap = cv2.VideoCapture(1)
    red_color = (0, 0, 255)
    cap.read()

    time.sleep(0.3)
    frame = vision.capturar_imagen(cap)
    mostrar_frame(window_name)

    cv2.setMouseCallback(window_name, vision.set_color_range, param=frame)
    print("Haz clic en el robot para seleccionar su color")
    while vision.lower_green is None or vision.upper_green is None:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.setMouseCallback(window_name, vision.main_mouse_callback)

    warning = False
    listener = KeyListener(None)
    listener.start()
    
    btn_llegar_meta = tk.Button(root, text="Llegar a meta", command=lambda: move_to_target(listener, vision.target_point))
    btn_llegar_meta.pack(fill=tk.BOTH, expand=True)


    update_point(cap, window_name, listener)
    while listener.running():
        root.update()

        # if vision.current_mark != None:
        #     mostrar_frame(window_name)

    cap.release()
    cv2.destroyAllWindows()

frame = None