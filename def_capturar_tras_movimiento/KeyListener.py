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

