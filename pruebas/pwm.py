import sys
import os
import threading
import time
ruta_directorio_actual = os.path.dirname(__file__)
ruta_def_captura = os.path.join(ruta_directorio_actual, '../def_capturar_tras_movimiento')

sys.path.append(ruta_def_captura)

from gpio import up_button, right_button, left_button

def velocidad():
        global up_button, right_button, left_button, estado_robot, robot_pathing
        try:
            while robot_pathing == True:
                # up_button.value = not up_button.value
                # if estado_robot == GIRANDO_DERECHA:
                #     right_button.value = not right_button.value
                # elif estado_robot == GIRANDO_IZQUIERDA:
                #     left_button.value = not left_button.value
                left_button.value = not left_button.value
                if left_button.value == False:
                    time.sleep(0.05)
                else:
                    # time.sleep(0.1 - speed/1000)
                    time.sleep(0.3)
            left_button.value = True
            up_button.value = True
        except KeyboardInterrupt:
            up_button.value = True
            left_button.value = True

            sys.exit(1)
        

estado_robot = "yendorecto"
robot_pathing = True
up_button.value = False
hilo = threading.Thread(target=velocidad)
hilo.start()
# up_button.value = not up_button.value
# right_button.value = not right_button.value
time.sleep(3)
robot_pathing=False
