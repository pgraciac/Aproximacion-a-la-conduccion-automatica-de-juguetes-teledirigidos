import math
from pynput import keyboard
import time
import gpio_virtual
import vision

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
        
        if key == keyboard.KeyCode.from_char('h'):
            print("Opciones disponibles:")
            print("1 - Marcar regiones")
            print("2 - Dibujar trayectoria")
            print("3 - Marcar meta")
            print("4 - Llegar a meta")
            print("5 - Calibrar giro")
            print("6 - Marcar limites")
            print("7 - Seguir trayectoria")
            print("8 - Experimento en linea recta")
            print("9 - Calibrar distancia")
            print("p - PWM_test")
            print("q - STOP")

        if key == keyboard.KeyCode.from_char('1'):
            vision.set_current_mark("regions")
        elif key == keyboard.KeyCode.from_char('2'):
            vision.set_current_mark("path")
        elif key == keyboard.KeyCode.from_char('3'):
            vision.set_current_mark("target_point")
        elif key == keyboard.KeyCode.from_char('4'):
            print("mueve al target")
            tiempo_inicio=time.time()
            gpio_virtual.move_to_target()
            tiempo_final = time.time() - tiempo_inicio
            print(tiempo_final)
        elif key == keyboard.KeyCode.from_char('5'):
            gpio_virtual.calibrate_rotation()
        elif key == keyboard.KeyCode.from_char('6'):
            vision.set_current_mark("limits")
        elif key == keyboard.KeyCode.from_char('7'):
            tiempo_inicio=time.time()
            gpio_virtual.follow_path()
            tiempo_final = time.time() - tiempo_inicio
            print(tiempo_final)
        elif key == keyboard.KeyCode.from_char('8'):
            gpio_virtual.mov_recta()
        elif key == keyboard.KeyCode.from_char('9'):
            gpio_virtual.calibrate_distance()
        elif key == keyboard.KeyCode.from_char('p'):
            gpio_virtual.pwm_test()
        elif key == keyboard.KeyCode.from_char('q'):
            self.listener.stop()
            vision.finish_vision()

        if key in self.keys_pressed and key not in [keyboard.Key.left, keyboard.Key.up, keyboard.Key.right, keyboard.Key.down]:
            self.keys_pressed.remove(key)
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
                    print("mueve up")
                    gpio_virtual.move("up", pixel_distance, "manual")
                elif key == keyboard.Key.right:
                    gpio_virtual.move("right", pixel_distance, "manual")
                elif key == keyboard.Key.down:
                    gpio_virtual.move("down", pixel_distance, "manual")

            self.start_time = None
        
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)
        
        if key == keyboard.KeyCode.from_char('q'):
            self.listener.stop()
            # vision.finish_vision()
            return False

    def check_listener(self):
    # Esta función se ejecutará en un hilo separado
        while self.listener.running:
            # Aquí se podría poner una pequeña pausa para no saturar el CPU
            time.sleep(0.1)
            
        print("se cierra el listener y vision")
    # Cuando listener ya no esté corriendo, ejecutar finish_vision
        vision.finish_vision()

    def start(self):
        self.listener.start()

    def running(self):
        return self.listener.running
