import random
import sys
import threading
import time
import board
import keyboard
import digitalio
import vision
import math
import numpy as np
import cv2

left_button = digitalio.DigitalInOut(board.C0)
left_button.direction = digitalio.Direction.OUTPUT
left_button.value = True

right_button = digitalio.DigitalInOut(board.C1)
right_button.direction = digitalio.Direction.OUTPUT
right_button.value = True

up_button = digitalio.DigitalInOut(board.C2)
up_button.direction = digitalio.Direction.OUTPUT
up_button.value = True

down_button = digitalio.DigitalInOut(board.C3)
down_button.direction = digitalio.Direction.OUTPUT
down_button.value = True





def calcular_angulo_tangente(initial_point, final_point):
    pendiente = (initial_point[0] - final_point[0]) / (initial_point[1] - final_point[1])
    angulo_pendiente = math.degrees(math.atan2(pendiente, 1))
    return (angulo_pendiente + 180) % 360 - 180

def move(direction, distance, mec="auto"):
    global total_distance, distance_second
    print(f"Moving {direction} for {distance} seconds")
    initial_position = vision.point
    print("Initial position: ", initial_position)
    if direction == "left":
        left_button.value = False
    elif direction == "up":
        up_button.value = False
    elif direction == "right":
        right_button.value = False
    elif direction == "down":
        down_button.value = False
    elif direction == "up-right":
        up_button.value = False
        right_button.value = False
    elif direction == "up-left":
        up_button.value = False
        left_button.value = False
    elif direction == "down-right":
        down_button.value = False
        right_button.value = False
    elif direction == "down-left":
        down_button.value = False
        left_button.value = False

    time.sleep(distance)
    left_button.value = True
    up_button.value = True
    right_button.value = True
    down_button.value = True

    time.sleep(1)
    vision.mostrar_frame(direction)
    final_position = vision.point

    if direction == "up" or direction == "down":
        pixels = abs(vision.distance_two_points(initial_position, final_position))
        if pixels > 3:
            distance_per_second = pixels * 0.1 / distance
            total_distance.append(distance_per_second)
            distance_second = sum(total_distance) / len(total_distance)
        
    # angles(init_point)

# def robot_orientation():
#     init_position = get_point()
#     move("up")
#     time.sleep(0.5)
#     final_position = get_point()

def mov_recta():
    global up_button, right_button, left_button, estado_robot, robot_pathing
    def velocidad():
        try:
            while robot_pathing == True:
                vision.mostrar_frame("up")
                print(up_button.value, left_button.value, right_button.value)
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
        
    initial_point = vision.point
    initial_orientation = vision.actual_orientation
    print(initial_point, initial_orientation)

    robot_pathing = True
    up_button.value = False
    time.sleep(0.05)
    hilo = threading.Thread(target=velocidad)
    hilo.start()
    # up_button.value = not up_button.value
    # right_button.value = not right_button.value
    time.sleep(3)
    robot_pathing=False
    # up_button.value = not up_button.value
    time.sleep(1)
    vision.mostrar_frame("up")
    final_point = vision.point
    print(final_point)
    final_orientation = vision.actual_orientation
    m = np.tan(np.radians(initial_orientation))
    distancia = abs(-m*final_point[0] + final_point[1] + (-initial_point[1] + m*initial_point[0])) / np.sqrt(m**2 + 1)

    dif_orientation = final_orientation - initial_orientation
    print(f"distancia deviado de la linea recta: {distancia}\norientación cambiada de la inicial: {dif_orientation}")


def follow_path():
    global spin_right_second, distance_second
    vision.target_point = vision.path[0]
    GIRANDO_DERECHA = 'girando_derecha'
    GIRANDO_IZQUIERDA = 'girando_izquierda'
    YENDO_RECTO = 'yendo_recto'
    PARADO = 'parado'
    f = open("pathinglog.txt", "w")
    f.truncate(0)
    mov_fuera = 0
    seg_fuera = 0
    mov_orient = 0
    seg_orient = 0
    mov_tot = 0
    fuera_de_linea = 0

    def get_closest_path_point(point):
        # Encuentra el punto más cercano en el camino al robot
        min_distance = float('inf')
        closest_index = -1

        for i, p in enumerate(vision.path):
            d = ((point[0] - p[0])**2 + (point[1] - p[1])**2)**0.5
            if d < min_distance:
                min_distance = d
                closest_index = i

        return vision.path[closest_index]
    
    def rotating(direction, sec):
        global spining, right_button, left_button
        spining = True
        if direction == "right":
            right_button.value = False
            time.sleep(sec)
            right_button.value = True
        elif direction == "left":
            left_button.value = False
            time.sleep(sec)
            left_button.value = True
        spining = False

    def velocidad():
        global up_button, right_button, left_button, speed, estado_robot, robot_pathing
        while robot_pathing == True:
            # up_button.value = not up_button.value
            # if estado_robot == GIRANDO_DERECHA:
            #     right_button.value = not right_button.value
            # elif estado_robot == GIRANDO_IZQUIERDA:
            #     left_button.value = not left_button.value
            # if estado_robot == YENDO_RECTO:
            #     up_button.value = not up_button.value
            if estado_robot == YENDO_RECTO:
                left_button.value = not left_button.value
            # estado_robot = estado_robot
            if left_button.value == False:
                time.sleep(0.05)
            else:
                # time.sleep(0.1 - speed/1000)
                time.sleep(0.3)

    def pathing(last_point, last_orientation, n_errores = 0):
        nonlocal f, mov_fuera, seg_fuera, mov_tot, fuera_de_linea
        global angulo_closest_point, closest_point, estado_robot, speed#,spining
        vision.mostrar_frame("up")
        print("pathing")
        f.write("pathing\n")
        # print(f"spining: {spining}")
        # f.write(f"spining: {spining}\n")
        #f.write(f"estado de robot: {estado_robot}\n")
        print(f"point: {vision.point}")
        f.write(f"point: {vision.point}\n")
        # if spining == True:
        #     vision.add_orientation()
        if vision.point != last_point: ##ver si cambia dos al menos
            print(f"actual orientation: {vision.actual_orientation}")
            f.write(f"actual orientation: {vision.actual_orientation}\n")
            closest_point = get_closest_path_point(vision.point)
            if closest_point == vision.path[0]: #Mirando si es el primero para que no coja el anterior como el ultimo de la lista
                closest_point = vision.path[1]
            elif closest_point == vision.path[-1]:
                closest_point = vision.path[-2]
            closest_point_index = vision.path.index(closest_point)
            long_path = len(vision.path)
            #f.write(f"len path: {long_path}")
            print(f"closest point real: {closest_point}")
            f.write((f"closest point real: {closest_point}\n"))
            point_before_index = closest_point_index -1
            point_after_index = closest_point_index +1
            print("el punto más cercano de la recta es: ", closest_point)
            f.write(f"el punto más cercano de la recta es: {closest_point}\n")
            # print(f"el punto más cercano es {closest_point} y la tangente la calculo con {vision.path[point_before_index]}, {vision.path[point_after_index]}")
            while vision.path[point_before_index] == vision.path[point_after_index]:
                point_after_index+=1
            angulo_closest_point = vision.orientation_two_points(vision.path[point_before_index], vision.path[point_after_index])
            print("angulo de la recta: ",angulo_closest_point)
            f.write(f"angulo de la recta: {angulo_closest_point}\n")

            orientation_robot_path = vision.orientation_two_points(vision.point, closest_point)
            print(f"angulo entre el robot y el punto mas cercano a la recta: {orientation_robot_path}")
            f.write(f"angulo entre el robot y el punto mas cercano a la recta: {orientation_robot_path}\n")
            last_distance = vision.distance_two_points(closest_point, last_point)
            actual_distance = vision.distance_two_points(closest_point, vision.point)
            print("la distancia actual es de: ", actual_distance)
            f.write(f"la distancia actual es de: {actual_distance}\n")
            
            if actual_distance > 80:
                print(RED+"muy lejos"+END)
                # f.write(RED+"muy lejos"+END)
                estado_robot = None
                left_button.value = True
                right_button.value = True
                up_button.value = True
                vision.target_point = closest_point
                fuera_de_linea+=1
                time.sleep(1)
                vision.mostrar_frame()
                cv2.waitKey(1)
                move("up", 0.05)   
                move_to_target(lambda cont = None, tiempo = None: on_target_reached(closest_point, cont = cont, tiempo = tiempo), 25, file=f, cont= mov_fuera,tiempo= time.time())
                return
            elif actual_distance < 1:
                good_orientation = angulo_closest_point
            else:
                factor = 1- (actual_distance / (vision.frame.shape[1] / 3))
                print("the factor is: ",factor)
                # f.write(f'the factor is: {factor}\n')
                next_point_path = vision.path[closest_point_index + 1]
                orientation_robot_next_point_path = vision.orientation_two_points(vision.point, next_point_path)
                print(f"orientacion al siguiente punto de la recta: {orientation_robot_next_point_path}")
                # f.write(f"orientacion al siguiente punto de la recta: {orientation_robot_next_point_path}\n")
                orientation_robot_path = orientation_robot_path + 360 if orientation_robot_path < 0 else orientation_robot_path
                angulo_closest_point = angulo_closest_point + 360 if angulo_closest_point < 0 else angulo_closest_point
                error_angle = angulo_closest_point - orientation_robot_path
                if abs(error_angle) > 180:
                    orientation_robot_path = (orientation_robot_path + 180) % 360 - 180
                    angulo_closest_point = (angulo_closest_point + 180) % 360 - 180
                good_orientation = (1-factor)*orientation_robot_path+factor*angulo_closest_point
                good_orientation = (good_orientation + 180) % 360 - 180

            print("The good orientation is: ",good_orientation)
            f.write(f'The good orientation is: {good_orientation}\n')

            error_orientation = vision.actual_orientation - good_orientation
            error_orientation = (error_orientation + 180) % 360 - 180
            print(f"error de orientacion entre actual orientation y good orientation: {error_orientation}")
            f.write(f"error de orientacion entre actual orientation y good orientation: {error_orientation}\n")

            if estado_robot == YENDO_RECTO:
                if abs(error_orientation) < 20:
                    print("seguir recto")
                    f.write("seguir recto\n")
                    left_button.value = True
                    right_button.value = True
                elif error_orientation < 0:
                    print("robot has to go left")
                    f.write("robot has to go left\n")
                    estado_robot = GIRANDO_IZQUIERDA
                    left_button.value = False
                    right_button.value = True
                    up_button.value = False
                elif error_orientation > 0:
                    print("robot has to go right")
                    f.write("robot has to go right\n")
                    estado_robot = GIRANDO_DERECHA
                    left_button.value = True
                    right_button.value = False
                    up_button.value = False
            else:
                if abs(error_orientation) < 20:
                    print("El robot tiene que ir recto")
                    f.write("El robot tiene que ir recto\n")
                    estado_robot = YENDO_RECTO
                    left_button.value = True
                    right_button.value = True
                elif error_orientation < 0:
                    print("robot has to go left")
                    f.write("robot has to go left\n")
                    estado_robot = GIRANDO_IZQUIERDA
                    left_button.value = False
                    right_button.value = True
                    up_button.value = False
                elif error_orientation > 0:
                    print("robot has to go right")
                    f.write("robot has to go right\n")
                    estado_robot = GIRANDO_DERECHA
                    left_button.value = True
                    right_button.value = False
                    up_button.value = False
            print(f"estado de robot: {estado_robot}")
            f.write(f"estado de robot: {estado_robot}\n")
            mov_tot+=1
            # Verificar si el robot ha alcanzado el final de la trayectoria
            if vision.distance_two_points(vision.point, vision.path[-1]) < 20 or closest_point == vision.path[-2]:  # Suponiendo que 20 es un umbral para estar "suficientemente cerca" del final
                estado_robot = PARADO
                robot_pathing = False
                up_button.value = True
                down_button.value = True
                left_button.value = True
                right_button.value = True
                
                closest_point = None
                angulo_closest_point = None
                print(f"fuera de linea: {fuera_de_linea}, de {mov_tot} veces")

            else:
                
                print("esta lejos todavia, rootafter\n------------------------------\n")
                f.write("esta lejos todavia, rootafter\n------------------------------\n")
                time.sleep(0.05)
                pathing(vision.point, vision.actual_orientation, n_errores)
        else:
            print("no se ha actualizado el frame\n------------------------------\n")
            f.write("no se ha actualizado el frame\n------------------------------\n")
            time.sleep(0.1)
            pathing(vision.point, vision.actual_orientation, n_errores)

    def begin_path():
        global up_button, right_button, left_button, robot_pathing, estado_robot, speed
        print(BLUE + "Empezando ruta" + END)
        orientation_robot_path = vision.orientation_two_points(vision.point, get_closest_path_point(vision.point))
        error_angle = (orientation_robot_path - vision.actual_orientation + 180) % 360 - 180
        time.sleep(1)
        if error_angle > -30 or error_angle < 30:
            up_button.value = False
            estado_robot = YENDO_RECTO
        elif error_angle < 0:
            up_button.value = False
            left_button.value = False
            estado_robot = GIRANDO_IZQUIERDA
        elif error_angle > 0:
            up_button.value = False
            right_button.value = False
            estado_robot = GIRANDO_DERECHA
        time.sleep(0.5)
        # direction_params = [[[89.66241616750374, 250.0], [54.020775792217606, 291.0]],
        # [[56.03370823986948, 95.0], [44.60471367212629, 287.0]],
        # [[38.626971201911964, 14.0], [82.19106286816542, 280.0]],
        # [[7.351253107892418, 3.0], [98.45078276314737, 271.0]],
        # [[6.869456049574438, 3.0], [107.02843546230764, 262.0]],
        # [[6.961360190409991, 3.0], [111.68587401164575, 233.0]],
        # [[6.114473096960642, 3.0], [83.7278461264097, 171.0]],
        # [[5.92879264552483, 2.0], [78.89705602078718, 85.0]],
        # [[5.321404413393637, 2.0], [76.0338616194957, 39.0]],
        # [[6.9970517273334565, 3.0], [57.210394038431374, 11.0]]]
        # orientation_params = [[[111.37929738925422, 250.0], [59.74314022629991, 291.0]],
        # [[63.222521539122546, 95.0], [67.61792539523809, 287.0]],
        # [[39.97632094597618, 14.0], [118.82413427923768, 280.0]],
        # [[11.646900403912221, 3.0], [141.53812609013048, 271.0]],
        # [[9.88414420514646, 3.0], [139.01494595146517, 262.0]],
        # [[9.994385444717494, 3.0], [138.9363183520781, 233.0]],
        # [[8.324747661367164, 3.0], [125.29770838590994, 171.0]],
        # [[7.366519487517451, 2.0], [120.42692063967195, 85.0]],
        # [[8.271528376884788, 2.0], [108.08023729842654, 39.0]],
        # [[8.716757915930808, 3.0], [84.48999541977294, 11.0]]]
        # distance_params = [[[49.86318547686042, 250.0], [55.20914094992365, 291.0]],
        # [[53.1980657558807, 95.0], [55.91946709729682, 287.0]],
        # [[57.88573142597299, 14.0], [51.96526567485488, 280.0]],
        # [[92.74380011379661, 3.0], [48.605177178596385, 271.0]],
        # [[78.18533481284224, 3.0], [46.91260035157134, 262.0]],
        # [[74.54691890115274, 3.0], [47.39618494710481, 233.0]],
        # [[79.7333947786628, 3.0], [47.80531034305459, 171.0]],
        # [[94.63025473706223, 2.0], [47.8481363502982, 85.0]],
        # [[111.42374300449669, 2.0], [44.76443657536687, 39.0]],
        # [[100.01658739934415, 3.0], [42.14126573112208, 11.0]]]
        # speed = 50
        robot_pathing = True
        # hilo = threading.Thread(target=velocidad)
        # hilo.start()
        # direction_params = [[[10, 1], [10, 1]], [[20, 1], [20, 1]], [[30, 1], [30, 1]], [[40, 1], [40, 1]], [[50, 1], [50, 1]], [[60, 1], [60, 1]], [[70, 1], [70, 1]], [[80, 1], [80, 1]], [[90, 1], [90, 1]], [[100, 1], [100, 1]]]
        # orientation_params = [[[10, 1], [10, 1]], [[20, 1], [20, 1]], [[30, 1], [30, 1]], [[40, 1], [40, 1]], [[50, 1], [50, 1]], [[60, 1], [60, 1]], [[70, 1], [70, 1]], [[80, 1], [80, 1]], [[90, 1], [90, 1]], [[100, 1], [100, 1]]]
        # distance_params = [[[40, 1], [40, 1]], [[40, 1], [40, 1]], [[40, 1], [40, 1]], [[40, 1], [40, 1]], [[40, 1], [40, 1]], [[40, 1], [40, 1]], [[40, 1], [40, 1]], [[40, 1], [40, 1]], [[40, 1], [40, 1]], [[40, 1], [40, 1]]]

        # pathing([[(0, 0), (0.0), 0, 0, YENDO_RECTO], [(0, 0), (0.0), 0, 0, YENDO_RECTO], [(0, 0), (0.0), 0, 0, YENDO_RECTO]], direction_params, orientation_params, distance_params, 0)
        pathing(vision.point, estado_robot, vision.actual_orientation)
        # time.sleep(10)

    def on_target_reached(first_point = None, cont = None, tiempo=None, vez = None):
        nonlocal mov_fuera, seg_fuera, mov_orient, seg_orient
        global angulo_closest_point, closest_point
        print(GREEN + "ha llegado al punto" + END)
        if vez == "primera":
            print(f"Ha tardado {time.time() - tiempo} segundos y {cont} movimientos")
            f.write(f"Ha tardado {time.time() - tiempo} segundos y {cont} movimientos\n")
        else:
            mov_fuera+=cont
            seg_fuera+=(time.time() - tiempo)
        tiempo_inicio = time.time()
        contador = 0
        if first_point == None:
            closest_point = get_closest_path_point(vision.point)
            if closest_point == vision.path[0]: #Mirando si es el primero para que no coja el anterior como el ultimo de la lista
                closest_point = vision.path[1]
            elif closest_point == vision.path[-1]:
                closest_point = vision.path[-2]
            closest_point_index = vision.path.index(closest_point)
            point_before_index = closest_point_index - 1
            point_after_index = closest_point_index + 1
        else:
            closest_point_index = vision.path.index(closest_point)
            point_before_index = closest_point_index - 1
            point_after_index = closest_point_index + 1
        print("el punto más cercano de la recta es: ", closest_point)
        # print(f"el punto más cercano es {closest_point} y la tangente la calculo con {vision.path[point_before_index]}, {vision.path[point_after_index]}")
        while vision.path[point_before_index] == vision.path[point_after_index]:
            if vision.path.index(vision.path[-1]) == point_after_index:
                point_before_index -= -1
            else:
                point_after_index+=1
        angulo_closest_point = vision.orientation_two_points(vision.path[point_before_index], vision.path[point_after_index])
        print("angulo de la recta: ",angulo_closest_point)
        error_angle = angulo_closest_point - vision.actual_orientation
        error_angle = (error_angle + 180) % 360 - 180
        print("angulo de error con el camino:", error_angle)
        up_down="up"
        time.sleep(1)
        while abs(error_angle) > 30:
            contador+=1
            print("angulo de error con el camino:", error_angle)
            print("path orientation", angulo_closest_point)
            move_duration = 0.05 * distance_to_target() / distance_second
            if move_duration < 0.05:
                move_duration = 0.05
            print("orientando a la recta")
            if error_angle > 0:
                # Turn left if the target is to the left
                rotate_duration = abs(error_angle) / (spin_right_second - 20) * 0.1
                print("have to move left")
                move("left", rotate_duration)
                #vision.mostrar_frame()
                #vision.root.update()
                move(up_down, move_duration)
            elif error_angle < 0:
                rotate_duration = abs(error_angle) / spin_right_second * 0.1
                # Turn left if the target is to the left
                print("have to move rigth")
                move("right", rotate_duration)
                #vision.mostrar_frame()
                #vision.root.update()
                move(up_down, move_duration)
            up_down = "up" if up_down == "down" else "down"
            error_angle = angulo_closest_point - vision.actual_orientation
            error_angle = (error_angle + 180) % 360 - 180
            print("\n------------------------------\n")
        print("preparado para seguirla")
        vision.target_point = None
        if vez == "primera":
            f.write(f"tiempo en orientarse {time.time()-tiempo_inicio}\nmovimientos = {contador}\n")
        else:
            seg_orient+=(time.time() - tiempo_inicio)
            mov_orient+=contador
        begin_path()
    # f = open("pathinglog.txt", "w")
    # f.truncate(0)
    move_to_target(lambda cont=None, tiempo=None: on_target_reached(cont=cont, tiempo=tiempo,vez="primera"),threshold=25, tiempo=time.time())
    time.sleep(1)
    vision.mostrar_frame("up")
    print(f"movimientos hasta llegar a la linea: {mov_fuera} y {seg_fuera} segundos\nmovimientos orientandose: {mov_orient} y {seg_orient} segundos\n")
    f.write(f"movimientos hasta llegar a la linea: {mov_fuera} y {seg_fuera} segundos\nmovimientos orientandose: {mov_orient} y {seg_orient} segundos\n")
    f.close()



def distance_to_target():
    delta_x_point = vision.target_point[0] - vision.point[0]
    delta_y_point = vision.target_point[1] - vision.point[1]
    distance = math.sqrt(delta_x_point ** 2 + delta_y_point ** 2)
    return distance

def move_to_target(callback=None, threshold=50, kp=0.1, kd=0.1, file=None, cont =0, tiempo=None):
    global distance_second, avg_distance, avg_rotation, spin_right_second, spin_up_right_secon, robot_in_target
    robot_in_target = False
    # if not vision.listener.running():
    #     return
    # if vision.target_point is None:
    #     print("Error: Target point is None")
    #     return
    # if vision.point is None:
    #     print("Error: point is None")
    #     return
    # Get the initial position
    # print("initial: ", vision.point)
    initial_position = vision.point
    if file!=None:
        print("repathing")
        file.write(f"repathing: point: {initial_position}\n")
    print(f"actual orientation: {vision.actual_orientation}")
    # Calculate the distance to the target
    delta_x_point = vision.target_point[0] - initial_position[0]
    delta_y_point = vision.target_point[1] - initial_position[1]
    distance = distance_to_target()
    print(f"distance to target: {distance}")
    print(f"target: {vision.target_point}")
    # Check if the target is reached
    if distance < threshold:
        print("Target reached")
        print(cont)
        print(tiempo)
        if callback:
            callback(cont = cont, tiempo = tiempo)
        return

    # Calculate the angle to the target
    angle_to_target = math.atan2(delta_y_point, delta_x_point)
    angle_to_target = math.degrees(angle_to_target)
    print(f"angle to target: {angle_to_target}")
    # Calculate the angular error
    error_angle = vision.actual_orientation - angle_to_target

    # Normalize the error_angle to be between -pi and pi
    error_angle = (error_angle + 180) % 360 - 180
    print("error angle:", error_angle)
    # Decide on the robot's movement based on error_angle
    move_duration = (0.1 * distance_to_target()) / distance_second
    if move_duration > 0.4:
        move_duration = 0.4
    elif move_duration < 0.05:
        move_duration = 0.05
    
    if error_angle < 30 and error_angle > -30:
        #Move forward if the error is small
        
        move("up", move_duration)
    elif error_angle < -150:
        move_duration = (0.1 * distance_to_target()) / distance_second
        if move_duration > 0.4:
            move_duration = 0.4
        move("down", move_duration)
    elif error_angle < 0:
        # Turn left if the target is to the left
        if distance < 80:
            rotate_duration = 0.1 * abs(error_angle) / (spin_right_second - 20)
            print("have to move left")
            move("left", rotate_duration)
            #vision.mostrar_frame()
            #vision.root.update()
            move("up", 0.1 * distance_to_target() / distance_second)
        else:
            rotate_duration = 0.1 * abs(error_angle) / (spin_up_right_secon - 10)
            print("have to move up-left")
            move("up-left", rotate_duration)
            #vision.mostrar_frame()
            #vision.root.update()
            move("up", 0.2)
    elif error_angle > 0:
        if distance < 80:
            rotate_duration = 0.1 * abs(error_angle) / spin_right_second
            print("have to move right")
            move("right", rotate_duration)
            #vision.mostrar_frame()
            #vision.root.update()
            move("up", 0.1 * distance_to_target() / distance_second)
        else:
            rotate_duration = 0.1 * abs(error_angle) / spin_up_right_secon
            print("have to move up-right")
            move("up-right", rotate_duration)
            #vision.mostrar_frame()
            #vision.root.update()
            move("up", 0.2)


    print("------------------------------------------------")
    # Call this function again to keep moving
    time.sleep(0.1)
    move_to_target(callback, threshold, file =file,cont=cont+1,tiempo=tiempo)
    

def calibrate_distance(times=10):
    global distance_second, total_distance
    total_distance = []
    distance_second = 0

    # Duración inicial del movimiento
    print("Calibrating distance")
    time.sleep(1)
    for _ in range(times):
        #print("Calibrating distance")
        
        # Mover hacia adelante y hacia atrás alternativamente
        direction = "up" if _ % 2 == 0 else "down"
        
        # Obtener posición inicial
        initial_position = vision.point
        #print(f"Initial position: {initial_position}")
        
        # Mover robot por el tiempo especificado
        tm = random.uniform(0.05, 0.4)
        print(direction)
        move(direction, tm)
        #vision.root.update()
        #time.sleep(1)
        
        # Obtener posición final
        final_position = vision.point
        #print(f"Final position: {final_position}")
        
        # Calcular y mostrar la distancia movida
        delta_x = final_position[0] - initial_position[0]
        delta_y = final_position[1] - initial_position[1]
        distance = math.sqrt(delta_x**2 + delta_y**2)
        print(f"Distance moved in {tm} seconds: {distance} units")
        
        #print("Actual orientation after calibrating distance: ", actual_orientation)
        if distance > 0:
            # Acumular distancia y número de iteraciones
            total_distance.append(distance * 0.1 / tm)
        cv2.waitKey(1)
        
        # Aumentar el tiempo de movimiento para la próxima iteración
        
    # Calcular y mostrar los píxeles movidos en promedio por 0.1 segundos
    avg_distance_per_0_1_second = sum(total_distance) / len(total_distance)
    print(f"Robot moves {avg_distance_per_0_1_second} pixels in 0.1 second on average.")
    #Cambiado a 23 para que no repita tanto
    distance_second = avg_distance_per_0_1_second

def calibrate_rotation():
    print("Calibrate rotation")
    # Move robot forward to establish initial orientation
    initial_position = vision.point
    print(f"Initial position: {initial_position}")    

    giros = []
    time.sleep(1)
    for _ in range(25):
        initial_angle = vision.actual_orientation + 360 if vision.actual_orientation < 0 else vision.actual_orientation
        tm = random.uniform(0.05, 0.2)
        move("up-right", tm)
        #vision.mostrar_frame()
        #vision.root.update()
        move("up", 0.2)
        #vision.mostrar_frame()
        #vision.root.update()
        final_angle = vision.actual_orientation + 360 if vision.actual_orientation < 0 else vision.actual_orientation
        print("actual orientation after calibrating: ", vision.actual_orientation)
        if final_angle > initial_angle:
            rotation = final_angle - initial_angle
            rotation = 360 - rotation
        else:
            rotation = initial_angle - final_angle
        print(f"Rotation in {tm} seconds: {rotation} degrees")
        giros.append((abs(rotation) * 0.1) / tm)
        print(f"rotation in 0.1 = {(abs(rotation) * 0.1) / tm}")
        print(f'average rotation={sum(giros) / len(giros)}\n--------------------------------')
    average_rotation = sum(giros) / 25
    return average_rotation

def calibrate_spin():
    print("Calibrando giro")
    error = float('inf')

    # Lista para almacenar giros por segundo
    giros_por_segundo = []

    # Umbral de error aceptable en grados
    umbral_error = 1.0  # Ajusta según tus necesidades

    while error > umbral_error:
        initial_position = vision.point
        print("actual orientation: ", vision.actual_orientation)
        seconds = random.uniform(0.05, 0.2)

        move("up-right", seconds)
        #vision.mostrar_frame()
        #vision.root.update()
        # thr next line for up-right
        initial_position = vision.point
        move("up", 0.15)
        #vision.mostrar_frame()
        #vision.root.update()

        final_position = vision.point
        delta_x_robot = final_position[0] - initial_position[0]
        delta_y_robot = final_position[1] - initial_position[1]
        final_orientation = math.atan2(delta_x_robot, delta_y_robot)
        final_orientation = math.degrees(final_orientation)
        giro_real = abs(vision.actual_orientation - final_orientation)

        # Calcular giro por segundo
        giro_actual_por_segundo = (giro_real / seconds) * 0.1
        giros_por_segundo.append(giro_actual_por_segundo)

        # Estimar el siguiente giro basado en el promedio de giros anteriores
        print("giros acumulados", giros_por_segundo)
        if len(giros_por_segundo) > 1:
            giro_estimado_segundo = sum(giros_por_segundo) / len(giros_por_segundo)
            print(f'sumatoria de giros: {sum(giros_por_segundo)}, / veces: {len(giros_por_segundo)}')
            print(f"giro estimado por 0.1 segundo: {giro_estimado_segundo}")
            print(f"giro estimado: {(giro_estimado_segundo * seconds) / 0.1}")

        print(f"giro real: {giro_real}")

        # Calcula el error entre la estimación y el giro real
        if len(giros_por_segundo) > 1:
            error = abs(((giro_estimado_segundo * seconds) / 0.1) - giro_real)
            print(f"error: {error}")

        # Actualiza la orientación y posición para la siguiente iteración
        #time.sleep(1)
    print(giro_estimado_segundo)

def pwm_test():
    global up_button, robot_pathing
    print("entra pwm")
    def slow(robot_speed):
        i=1
        global up_button, right_button, left_button, estado_robot, robot_pathing
        try:
            robot_difs=[]
            while robot_pathing == True:
                last_point = vision.point
                initial_orientation = vision.actual_orientation
                # up_button.value = not up_button.value
                # if estado_robot == GIRANDO_DERECHA:
                #     right_button.value = not right_button.value
                # elif estado_robot == GIRANDO_IZQUIERDA:
                #     left_button.value = not left_button.value
                
                
                
                if up_button.value == False:
                    # print(up_button.value, left_button.value)

                    t=0.1
                else:
                    # left_button.value = True
                    t=0.1
                    # time.sleep(0.1 - speed/1000)   
                time.sleep(t)
                # if (i+1)%6 == 0:
                #     left_button.value = False
                #     time.sleep(0.05)
                #     # print(up_button.value, left_button.value)

                #     left_button.value = True
                # else:
                #     time.sleep(0.05)
                up_button.value = not up_button.value
                left_button.value = not left_button.value
                vision.mostrar_frame("up")
                # dif = vision.distance_two_points(last_point, vision.point)
                # robot_speed.append([dis, t, vision.point])
                dif = abs((initial_orientation - vision.actual_orientation + 180) % 360 - 180)
                robot_difs.append(dif/t)
                i+=1
            media = sum(robot_difs) / len(robot_difs)

            print(media * 0.1)

            left_button.value = True
            up_button.value = True
        except KeyboardInterrupt:
            up_button.value = True
            left_button.value = True

            sys.exit(1)

    def desvio():
        global left_button, up_button, robot_pathing
        try:
            cambio = True
            while robot_pathing == True:
                # up_button.value = not up_button.value
                # if estado_robot == GIRANDO_DERECHA:
                #     right_button.value = not right_button.value
                # elif estado_robot == GIRANDO_IZQUIERDA:
                #     left_button.value = not left_button.value
                t = 0
                if up_button.value == False and left_button == True:
                    left_button.value = False
                
                if left_button.value == False:
                    t=0.05
                else:
                    t=0.3
                    # time.sleep(0.1 - speed/1000)   
                time.sleep(t)
                vision.mostrar_frame("up")

            left_button.value = True
            up_button.value = True
        except KeyboardInterrupt:
            up_button.value = True
            left_button.value = True

            sys.exit(1)
        
        

    estado_robot = "yendorecto"
    robot_pathing = True
    # up_button.value = False
    # print(up_button.value)

    robot_speed=[]
    pwm = threading.Thread(target=slow, args=(robot_speed,))
    pwm.start()

    # desv = threading.Thread(target=desvio)
    # desv.start()
    # up_button.value = not up_button.value
    # right_button.value = not right_button.value
    time.sleep(3)
    robot_pathing=False
    pwm.join()
    # desv.join()
    # dist=0
    # tt=0
    # for dis, t, point in robot_speed:
    #     if dis != 0:
    #         dist += dis
    #         tt += t
    # print((dist * 0.1)/tt)
# def perform_calibration(repetitions=1, tm=0.2):
#     global avg_distance, avg_rotation
#     total_distance = 0
#     total_rotation = 0
#     rot = True
#     for _ in range(repetitions):
#         total_distance += calibrate_distance(tm)
#         # if rot:
#         #     total_rotation += calibrate_rotation(tm)
#         # else:
#         #     total_rotation += abs(calibrate_rotation(tm, "up-left"))
#         rot = not rot
#     avg_distance = total_distance / repetitions
#     avg_rotation = total_rotation / repetitions

robot_in_target = False
avg_distance = 0
avg_rotation = 0
#spin in 0.1 seconds to right
spin_right_second=60
#spin in 0.1 seconds to up-right
spin_up_right_secon = 30
#pixels in 0.1 seconds
distance_second = 35
robot_pathing = False
total_distance = []
index_path_spin = 0
spining = False
closest_point = None
angulo_closest_point = None

RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
END = '\033[0m'