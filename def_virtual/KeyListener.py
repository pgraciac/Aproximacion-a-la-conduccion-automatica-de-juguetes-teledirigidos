import math
from pynput import keyboard
import time
import vision
import gpio_virtual

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
            pixel_distance = elapsed_time
            
            if keyboard.Key.up in self.keys_pressed and keyboard.Key.right in self.keys_pressed:
                gpio_virtual.move("up-right", pixel_distance, "manual")
            elif keyboard.Key.up in self.keys_pressed and keyboard.Key.left in self.keys_pressed:
                gpio_virtual.move("up-left", pixel_distance, "manual")
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.right in self.keys_pressed:
                gpio_virtual.move("down-right", pixel_distance, "manual")
            elif keyboard.Key.down in self.keys_pressed and keyboard.Key.left in self.keys_pressed:
                gpio_virtual.move("down-left", pixel_distance, "manual")
            else:

                if key == keyboard.Key.left:
                    gpio_virtual.move("left", pixel_distance, "manual")
                elif key == keyboard.Key.up:
                    gpio_virtual.move("up", pixel_distance, "manual")
                elif key == keyboard.Key.right:
                    gpio_virtual.move("right", pixel_distance, "manual")
                elif key == keyboard.Key.down:
                    gpio_virtual.move("down", pixel_distance, "manual")

            self.start_time = None
        
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)
        
        if key == keyboard.KeyCode.from_char('q'):
            return False



    def start(self):
        self.listener.start()

    def running(self):
        return self.listener.running
