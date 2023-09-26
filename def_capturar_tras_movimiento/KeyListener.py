import math
from pynput import keyboard
import time
import vision
import gpio

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
            initial_position = vision.point
            if keyboard.Key.up in self.keys_pressed and keyboard.Key.right in self.keys_pressed:
                vision.point =gpio.move("up-right", elapsed_time, "manual")
            elif keyboard.Key.up in self.keys_pressed and keyboard.Key.left in self.keys_pressed:
                vision.point =gpio.move("up-left", elapsed_time, "manual")
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.right in self.keys_pressed:
                vision.point =gpio.move("down-right", elapsed_time, "manual")
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.left in self.keys_pressed:
                vision.point =gpio.move("down-left", elapsed_time, "manual")
            else:

                if key == keyboard.Key.left:
                    vision.point =gpio.move("left", elapsed_time, "manual")
                elif key == keyboard.Key.up:
                    vision.point =gpio.move("up", elapsed_time, "manual")
                elif key == keyboard.Key.right:
                    vision.point =gpio.move("right", elapsed_time, "manual")
                elif key == keyboard.Key.down:
                    vision.point =gpio.move("down", elapsed_time, "manual")
            vision.mostrar_frame()
            final_position = vision.point
            print("Final position: ", final_position)
        
            if key == keyboard.Key.up:
                delta_x_robot = final_position[0] - initial_position[0]
                delta_y_robot = final_position[1] - initial_position[1]
                robot_orientation = math.atan2(delta_x_robot, delta_y_robot)
                gpio.actual_orientation = math.degrees(robot_orientation)
                print("actual orientation after up: ", gpio.actual_orientation)
            elif key == keyboard.Key.down:
                delta_x_robot = final_position[0] - initial_position[0]
                delta_y_robot = final_position[1] - initial_position[1]
                robot_orientation = math.atan2(delta_x_robot, delta_y_robot)
                gpio.actual_orientation = (math.degrees(robot_orientation) + 180) % 360
                print("actual orientation after down: ", gpio.actual_orientation)
            # vision.root.update()
            self.start_time = None
        
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)
        
        if key == keyboard.KeyCode.from_char('q'):
            return False



    def start(self):
        self.listener.start()

    def running(self):
        return self.listener.running

