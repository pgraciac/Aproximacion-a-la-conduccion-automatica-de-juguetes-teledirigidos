import random
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

def orientation_two_points(initial_point, final_point):
    delta_x_point = final_point[0] - initial_point[0]
    delta_y_point = final_point[1] - initial_point[1]
    angle_to_target = math.atan2(delta_y_point, delta_x_point)
    angle_to_target = math.degrees(angle_to_target)
    #print(f"angle to target: {angle_to_target}")
    # Calculate the angular error
    # Normalize the error_angle to be between -pi and pi
    angle_to_target = (angle_to_target + 180) % 360 - 180
    # print("angulo entre", initial_point, final_point, "= ", angle_to_target)
    return angle_to_target

def distance_two_points(initial_point, final_point):
        delta_x_point = final_point[0] - initial_point[0]
        delta_y_point = final_point[1] - initial_point[1]
        distance = math.sqrt(delta_x_point ** 2 + delta_y_point ** 2)
        return distance


def calcular_angulo_tangente(initial_point, final_point):
    pendiente = (initial_point[0] - final_point[0]) / (initial_point[1] - final_point[1])
    angulo_pendiente = math.degrees(math.atan2(pendiente, 1))
    return (angulo_pendiente + 180) % 360 - 180

def move(direction, distance, mec="auto"):
    global actual_orientation, total_distance, distance_second
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
    vision.mostrar_frame("not_orientation")
    final_position = vision.point
    print("Final position: ", final_position)
    
    if direction == "up" or direction == "up-right" or direction == "up-left":
        delta_x_robot = final_position[0] - initial_position[0]
        delta_y_robot = final_position[1] - initial_position[1]
        robot_orientation = math.degrees(math.atan2(delta_y_robot, delta_x_robot))
        actual_orientation = (robot_orientation % 360) - 360 if robot_orientation % 360 > 180 else robot_orientation % 360
        print("actual orientation after up: ", actual_orientation)
    elif direction == "down" or direction == "down-right" or direction == "down-left":
        delta_x_robot = initial_position[0] - final_position[0]
        delta_y_robot = initial_position[1] - final_position[1] 
        robot_orientation = math.degrees(math.atan2(delta_y_robot, delta_x_robot))
        actual_orientation = (robot_orientation % 360) - 360 if robot_orientation % 360 > 180 else robot_orientation % 360
        print("actual orientation after down: ", actual_orientation)
    vision.add_orientation()
    if direction == "up" or direction == "down":
        pixels = abs(distance_two_points(initial_position, final_position))
        distance_per_second = pixels * 0.1 / distance
        total_distance.append(distance_per_second)
        distance_second = sum(total_distance) / len(total_distance)
    if mec == "auto":
        #vision.mostrar_frame()
        vision.root.update()
        
    if len(vision.limits) > 0:
        if not vision.robot_in_limits():
            go_robot_inside_limits()
    # #vision.mostrar_frame()
    # angles(init_point)

# def robot_orientation():
#     init_position = get_point()
#     move("up")
#     time.sleep(0.5)
#     final_position = get_point()


def follow_path():
    global actual_orientation, spin_right_second, distance_second
    vision.target_point = vision.path[0]
    GIRANDO_DERECHA = 'girando_derecha'
    GIRANDO_IZQUIERDA = 'girando_izquierda'
    YENDO_RECTO = 'yendo_recto'

    def get_closest_path_point():
        # Encuentra el punto más cercano en el camino al robot
        min_distance = float('inf')
        closest_index = -1

        for i, point in enumerate(vision.path):
            d = ((vision.point[0] - point[0])**2 + (vision.point[1] - point[1])**2)**0.5
            if d < min_distance:
                min_distance = d
                closest_index = i

        return vision.path[closest_index]
    
    def pathing(last_point, estado_robot, last_orientation, f, n_errores = 0):
        global angulo_closest_point, closest_point, actual_orientation
        vision.mostrar_frame("not_orientation")
        print("pathing")
        f.write("pathing\n")
        print(f"estado de robot: {estado_robot}")
        f.write(f"estado de robot: {estado_robot}\n")
        print(f"up button = {up_button.value}\nright button = {right_button.value}\nleft button = {left_button.value}")
        f.write(f"up button = {up_button.value}\nright button = {right_button.value}\nleft button = {left_button.value}\n")
        # if estado_robot == None:
        #     orientation_robot_path = orientation_two_points(vision.point, vl)
        #     up_button.value = False
        #     vision.root.after(50, lambda: pathing(vision.point, YENDO_RECTO, actual_orientation, f))
        if vision.point != last_point: ##ver si cambia dos al menos
            print(f"point: {vision.point}")
            f.write(f"point: {vision.point}\n")
            ##SI ES HACIA LA DERECHA VER SI TIENE SENTIDO LA ORIENTACION NUEVA Y TAMBIEN VER SI CAMBIA DEMASIADO LA ORIENTACION QUE NO PUEDE SER
            actual_orientation = orientation_two_points(last_point, vision.point)
            real_orientation = actual_orientation
            have_error = False
            if estado_robot == GIRANDO_DERECHA:
                actual_orientation = actual_orientation +360 if actual_orientation < 0 else actual_orientation
                last_orientation = last_orientation +360 if last_orientation < 0 else last_orientation
                error = (actual_orientation - last_orientation + 180) % 360 - 180
                if error > 20:
                    have_error = True
                    n_errores +=1
                    print(RED + "Error de orientacion" + f" detecta {(actual_orientation +180) % 360 -180}" + END)
                    f.write(RED + "Error de orientacion" + f" detecta {(actual_orientation +180) % 360 -180}" + END + "\n")
                    # print("\n------------------------------\n")
                    # return vision.root.after(1, lambda: pathing(vision.point , estado_robot, actual_orientation))
                    actual_orientation = last_orientation - 35
            elif estado_robot == GIRANDO_IZQUIERDA:
                actual_orientation = actual_orientation +360 if actual_orientation < 0 else actual_orientation
                last_orientation = last_orientation +360 if last_orientation < 0 else last_orientation
                error = (actual_orientation - last_orientation + 180) % 360 - 180
                if error < -20:
                    have_error = True
                    n_errores +=1
                    print(RED + "Error de orientacion" + f" detecta {(actual_orientation +180) % 360 -180}" + END)
                    f.write(RED + "Error de orientacion" + f" detecta {(actual_orientation +180) % 360 -180}" + END + "\n")
                    # print("\n------------------------------\n")
                    # return vision.root.after(1, lambda: pathing(vision.point , estado_robot, actual_orientation))
                    actual_orientation = last_orientation + 35
            # elif estado_robot == YENDO_RECTO:
            #     actual_orientation = actual_orientation +360 if actual_orientation < 0 else actual_orientation
            #     last_orientation = last_orientation +360 if last_orientation < 0 else last_orientation
            #     error = (actual_orientation - last_orientation + 180) % 360 - 180
            #     if abs(error) > 45:
            #         have_error = True
            #         n_errores +=1
            #         print(RED + "Error de orientacion" + f" detecta {(actual_orientation +180) % 360 -180}" + END)
            #         # print("\n------------------------------\n")
            #         # return vision.root.after(1, lambda: pathing(vision.point , estado_robot, last_orientation))
            #         actual_orientation = last_orientation
            actual_orientation = (actual_orientation + 180) % 360 - 180
            if n_errores > 2:
                actual_orientation = real_orientation
            vision.add_orientation()
            print(f"actual orientation: {actual_orientation}")
            f.write(f"actual orientation: {actual_orientation}\n")
            closest_point = get_closest_path_point()
            if closest_point == vision.path[0]: #Mirando si es el primero para que no coja el anterior como el ultimo de la lista
                closest_point = vision.path[1]
            elif closest_point == vision.path[-1]:
                closest_point = vision.path[-2]
            closest_point_index = vision.path.index(closest_point)
            long_path = len(vision.path)
            print(f"len path: {long_path}")
            f.write(f"len path: {long_path}\n")
            print(vision.path)
            f.write("".join([str(i) for i in vision.path]))
            print(f"closest point real: {closest_point}")
            f.write((f"\nclosest point real: {closest_point}\n"))
            if closest_point_index + 40 < long_path:##**CALCULAR VELOCIDAD Y DISTANCIA AL GIRO
                print(f"closest_point_index + 10: {closest_point_index}, {closest_point_index + 10}\n")
                f.write(f"closest_point_index + 10: {closest_point_index}, {closest_point_index + 10}")
                closest_point_index += 40
                closest_point = vision.path[closest_point_index]
            else:
                print("esta casi al final")
                print(f"closest_point_index: {closest_point_index}")
                f.write(f"closest_point_index + 10: {closest_point_index}\n")
                f.write("esta casi al final\n")
            point_before_index = closest_point_index -1
            point_after_index = closest_point_index +1
            print("el punto más cercano de la recta es: ", closest_point)
            f.write(f"el punto más cercano de la recta es: {closest_point}\n")
            # print(f"el punto más cercano es {closest_point} y la tangente la calculo con {vision.path[point_before_index]}, {vision.path[point_after_index]}")
            while vision.path[point_before_index] == vision.path[point_after_index]:
                point_after_index+=1
            angulo_closest_point = orientation_two_points(vision.path[point_before_index], vision.path[point_after_index])
            print("angulo de la recta: ",angulo_closest_point)
            f.write(f"angulo de la recta: {angulo_closest_point}\n")
            # error_angle = angulo_closest_point - actual_orientation
            # error_angle = (error_angle + 180) % 360 - 180
            # print(f"angulo de error entre actual orientation y el angulo de la recta: {error_angle}")
            orientation_robot_path = orientation_two_points(vision.point, closest_point)
            print(f"angulo entre el robot y el punto mas cercano a la recta: {orientation_robot_path}")
            f.write(f"angulo entre el robot y el punto mas cercano a la recta: {orientation_robot_path}\n")
            last_distance = distance_two_points(closest_point, last_point)
            actual_distance = distance_two_points(closest_point, vision.point)
            print("la distancia actual es de: ", actual_distance)
            f.write(f"la distancia actual es de: {actual_distance}\n")
            
            if actual_distance > 80:
                
                up_button.value = True
                right_button.value = True
                left_button.value = True
                print("muy lejos")
                f.write("muy lejos")
                vision.mostrar_frame()
                time.sleep(1)
                vision.mostrar_frame()
                print("mostrar frame")
                f.write("mostrar frame")
                closest_point = get_closest_path_point()
                vision.target_point = closest_point
                actual_orientation = orientation_two_points(last_point, vision.point)
                vision.mostrar_frame()
                vision.root.update()
                print("root update\n-----------------------------------\n")
                f.write("root update\n-----------------------------------\n")
                time.sleep(1)
                move_to_target(lambda: on_target_reached(closest_point), 25)
                return
            elif actual_distance < 80:
                good_orientation = angulo_closest_point
            else:
                factor = 1- (actual_distance / (vision.frame.shape[1] / 3))
                print("the factor is: ",factor)
                f.write(f"the factor is: {factor}\n")
                next_point_path = vision.path[closest_point_index + 1]
                orientation_robot_next_point_path = orientation_two_points(vision.point, next_point_path)
                print(f"orientacion al siguiente punto de la recta: {orientation_robot_next_point_path}")
                f.write(f"orientacion al siguiente punto de la recta: {orientation_robot_next_point_path}\n")
                orientation_robot_path = orientation_robot_path + 360 if orientation_robot_path < 0 else orientation_robot_path
                angulo_closest_point = angulo_closest_point + 360 if angulo_closest_point < 0 else angulo_closest_point
                error_angle = angulo_closest_point - orientation_robot_path
                if abs(error_angle) > 180:
                    orientation_robot_path = (orientation_robot_path + 180) % 360 - 180
                    angulo_closest_point = (angulo_closest_point + 180) % 360 - 180
                good_orientation = (1-factor)*orientation_robot_path+factor*angulo_closest_point
                good_orientation = (good_orientation + 180) % 360 - 180
                # orientation_robot_next_point_path = orientation_robot_next_point_path + 360 if orientation_robot_next_point_path < 0 else orientation_robot_next_point_path
                # if orientation_robot_next_point_path > orientation_robot_path: ##Tener en cuenta distancia
                #     #Izquierda
                #     print("recta va hacia la izquierda")
                #     angulo_closest_point = angulo_closest_point + 360 if angulo_closest_point < 0 else angulo_closest_point
                # else:
                #     print("recta va hacia la derecha")
                #     orientation_robot_path = (orientation_robot_path + 180) % 360 - 180
                # good_orientation = (1-factor)*orientation_robot_path+factor*angulo_closest_point
                # good_orientation = (good_orientation + 180) % 360 - 180
            print("The good orientation is: ",good_orientation)
            f.write(f"The good orientation is: {good_orientation}\n")
            # if estado_robot == GIRANDO_DERECHA:
            #     estimacion_orientacion = actual_orientation - 40
            # elif estado_robot == GIRANDO_IZQUIERDA:
            #     estimacion_orientacion = actual_orientation + 40
            # else:
            #     estimacion_orientacion = actual_orientation
            # estimacion_orientacion = (estimacion_orientacion + 180) % 360 - 180
            estimacion_orientacion = actual_orientation
            print("estimacion orientacion: ", estimacion_orientacion)
            f.write(f"estimacion orientacion: {estimacion_orientacion}\n")
            error_orientation = estimacion_orientacion - good_orientation
            error_orientation = (error_orientation + 180) % 360 - 180
            print(f"error de orientacion entre actual orientation y good orientation: {error_orientation}")
            f.write(f"error de orientacion entre actual orientation y good orientation: {error_orientation}\n")
            if estado_robot == YENDO_RECTO:
                print("yendo recto")
                f.write("yendo recto\n")
                if abs(error_orientation) < 20:
                    print("seguir recto")
                    f.write("seguir recto\n")
                elif error_orientation < 0:
                    print("robot has to go left")
                    f.write("robot has to go left\n")
                    estado_robot = GIRANDO_IZQUIERDA
                    right_button.value = True
                    left_button.value = False
                    up_button.value = False
                elif error_orientation > 0:
                    print("robot has to go right")
                    f.write("robot has to go right\n")
                    estado_robot = GIRANDO_DERECHA
                    right_button.value = False
                    up_button.value = False
                    left_button.value = True
            else:
                if abs(error_orientation) < 20:
                    print("El robot tiene que ir recto")
                    f.write("El robot tiene que ir recto\n")
                    estado_robot = YENDO_RECTO
                    left_button.value = True
                    right_button.value = True
                    up_button.value = False
                elif error_orientation < 0:
                    print("robot has to go left")
                    f.write("robot has to go left\n")
                    estado_robot = GIRANDO_IZQUIERDA
                    left_button.value = False
                    up_button.value = False
                    right_button.value = True
                elif error_orientation > 0:
                    print("robot has to go right")
                    f.write("robot has to go right\n")
                    estado_robot = GIRANDO_DERECHA
                    right_button.value = False
                    left_button.value = True
                    up_button.value = False

            # Verificar si el robot ha alcanzado el final de la trayectoria
            if distance_two_points(vision.point, vision.path[-1]) < 20 or closest_point == vision.path[-2]:  # Suponiendo que 20 es un umbral para estar "suficientemente cerca" del final
                up_button.value = True
                right_button.value = True
                left_button.value = True
                robot_pathing = False
                f.close()
                time.sleep(1)
                vision.mostrar_frame()
            else:
                if have_error:
                    print("Ha habido error")
                    f.write("Ha habido error")
                    actual_orientation = last_orientation
                else:
                    n_errores = 0
                print("esta lejos todavia, rootafter\n------------------------------\n")
                f.write("esta lejos todavia, rootafter\n------------------------------\n")
                vision.root.after(50, lambda: pathing(vision.point, estado_robot, actual_orientation, f, n_errores))
        else:
            print("no se ha actualizado el frame\n------------------------------\n")
            f.write("no se ha actualizado el frame\n------------------------------\n")
            vision.root.after(100, lambda: pathing(last_point, estado_robot, actual_orientation, f))

    def begin_path():
        global up_button, right_button, left_button, actual_orientation, robot_pathing
        print(BLUE + "Empezando ruta" + END)
        # up_button.value = False
        # estado_robot = YENDO_RECTO
        robot_pathing = True
        orientation_robot_path = orientation_two_points(vision.point, get_closest_path_point())
        error_angle = (orientation_robot_path - actual_orientation + 180) % 360 - 180
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
        pathing(vision.point, estado_robot, actual_orientation, f)
        

    def on_target_reached(closest_point = None):
        print(GREEN + "ha llegado al punto" + END)
        if closest_point == None:
            closest_point = get_closest_path_point()
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
            point_after_index+=1
        path_orientation = orientation_two_points(vision.path[point_before_index], vision.path[point_after_index])
        print("angulo de la recta: ",path_orientation)
        error_angle = path_orientation - actual_orientation
        error_angle = (error_angle + 180) % 360 - 180
        print("angulo de error con el camino:", error_angle)
        up_down="up"
        time.sleep(1)
        while abs(error_angle) > 30:
            print("angulo de error con el camino:", error_angle)
            print("path orientation", path_orientation)
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
            error_angle = path_orientation - actual_orientation
            print("\n------------------------------\n")
        print("preparado para seguirla")
        begin_path()
    f = open("pathinglog.txt", "w")
    f.truncate(0)
    move_to_target(on_target_reached,threshold=25)
    time.sleep(1)
    vision.mostrar_frame()



def distance_to_target():
    delta_x_point = vision.target_point[0] - vision.point[0]
    delta_y_point = vision.target_point[1] - vision.point[1]
    distance = math.sqrt(delta_x_point ** 2 + delta_y_point ** 2)
    return distance

def move_to_target(callback=None, threshold=50, kp=0.1, kd=0.1):
    global distance_second, actual_orientation, avg_distance, avg_rotation, spin_right_second, spin_up_right_secon, robot_in_target
    robot_in_target = False
    if not vision.listener.running():
        return
    if vision.target_point is None:
        print("Error: Target point is None")
        return
    if vision.point is None:
        print("Error: point is None")
        return
    # Get the initial position
    # print("initial: ", vision.point)
    initial_position = vision.point

    print(f"actual orientation: {actual_orientation}")
    # Calculate the distance to the target
    delta_x_point = vision.target_point[0] - initial_position[0]
    delta_y_point = vision.target_point[1] - initial_position[1]
    distance = distance_to_target()
    print(f"distance to target: {distance}")
    print(f"target: {vision.target_point}")
    # Check if the target is reached
    if distance < threshold:
        print("Target reached")
        if callback:
            callback()
        return

    # Calculate the angle to the target
    angle_to_target = math.atan2(delta_y_point, delta_x_point)
    angle_to_target = math.degrees(angle_to_target)
    print(f"angle to target: {angle_to_target}")
    # Calculate the angular error
    error_angle = actual_orientation - angle_to_target

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
    elif error_angle < -155:
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
    # elif error_angle > 120:
    #     rotate_duration = abs(error_angle) / spin_right_second * 0.1
    #     # Turn left if the target is to the left
    #     print("have to move left")
    #     rotate_duration = abs(error_angle) / spin_right_second * 0.1
    #     move("left", 0.1)
    #     #vision.mostrar_frame()
    #     #vision.root.update()
    #     move("up", 0.2)
    # elif error_angle < -120:
    #     # Turn left if the target is to the left
    #     print("have to move right")
    #     move("right", 0.1)
    #     #vision.mostrar_frame()
    #     #vision.root.update()
    #     move("up", 0.2)

    #vision.mostrar_frame()
    #vision.root.update()
    # Calculate the robot's orientation
    #time.sleep(4)

    print("------------------------------------------------")
    # Call this function again to keep moving
    vision.root.after(100, lambda: move_to_target(callback, threshold))
    

def go_robot_inside_limits():
    global actual_orientation
    print("into function robot inside limits")

    if vision.robot_in_limits():
        return

    print("Bringing the robot within the limits")

    best_distance = float('inf')
    best_vertex = ()
    for vertex in vision.limits:
        distance = math.dist(vision.point, vertex)  # Usar la función math.dist para simplificar
        if distance < best_distance:
            best_distance = distance
            best_vertex = vertex

    best_point = ()
    for add in [-1, 1]:  # Para iterar dos veces con -1 y 1
        A, B = best_vertex, vision.limits[(vision.limits.index(best_vertex) + add) % len(vision.limits)]
        t = min(max(np.dot(np.subtract(vision.point, A), np.subtract(B, A)) / np.dot(np.subtract(B, A), np.subtract(B, A)), 0), 1)
        point_in_line = np.add(A, np.multiply(t, np.subtract(B, A)))
        distance = math.dist(point_in_line, best_vertex)
        if distance < best_distance:
            best_distance = distance
            best_point = point_in_line

    angle_to_polygon = math.degrees(math.atan2(best_point[1] - vision.point[1], best_point[0] - vision.point[0]))
    error_angle = angle_to_polygon - actual_orientation
    print("error angle: ", error_angle)
    move_duration = min(0.1 * best_distance / distance_second + 0.1, 0.3)

    if abs(error_angle) < 30:
        move("up", move_duration, "manual")
    elif abs(error_angle) > 155:
        move("down", move_duration, "manual")
    else:
        rotate_duration = abs(error_angle) / spin_up_right_secon * 0.1
        move("up-left" if error_angle > 0 else "up-right", rotate_duration, "manual")
        move("up", 0.2, "manual")

    final_position = vision.point
    print("final: ", final_position)
    
    vision.root.after(100, lambda: go_robot_inside_limits())



def calibrate_distance(times=10):
    global distance_second, total_distance
    
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
        tm = random.uniform(0.05, 0.3)
        move(direction, tm)
        #vision.mostrar_frame()
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
        
        # Aumentar el tiempo de movimiento para la próxima iteración
        
    # Calcular y mostrar los píxeles movidos en promedio por 0.1 segundos
    avg_distance_per_0_1_second = sum(total_distance) / len(total_distance)
    print(f"Robot moves {avg_distance_per_0_1_second} pixels in 0.1 second on average.")
    #Cambiado a 23 para que no repita tanto
    distance_second = avg_distance_per_0_1_second

def calibrate_rotation():
    global actual_orientation
    print("Calibrate rotation")
    # Move robot forward to establish initial orientation
    initial_position = vision.point
    print(f"Initial position: {initial_position}")    

    giros = []
    time.sleep(1)
    for _ in range(25):
        initial_angle = actual_orientation + 360 if actual_orientation < 0 else actual_orientation
        tm = random.uniform(0.05, 0.2)
        # Move robot in given direction (e.g., "up-right" or "up-left") for specified time
        move("up-right", tm)
        #vision.mostrar_frame()
        #vision.root.update()
        # Move robot forward to calculate new orientation
        move("up", 0.2)
        #vision.mostrar_frame()
        #vision.root.update()
        # Calculate final orientation
        final_angle = actual_orientation + 360 if actual_orientation < 0 else actual_orientation
        print("actual orientation after calibrating: ", actual_orientation)
        # Calculate and print rotation
        if final_angle > initial_angle:
            rotation = final_angle - initial_angle
            rotation = 360 - rotation
        else:
            rotation = initial_angle - final_angle
        print(f"Rotation in {tm} seconds: {rotation} degrees")
        giros.append((abs(rotation) * 0.1) / tm)
        print(f'average rotation={sum(giros) / len(giros)}\n--------------------------------')
    average_rotation = sum(giros) / 25
    return average_rotation

def calibrate_spin():
    print("Calibrando giro")
    global actual_orientation
    error = float('inf')

    # Lista para almacenar giros por segundo
    giros_por_segundo = []

    # Umbral de error aceptable en grados
    umbral_error = 1.0  # Ajusta según tus necesidades

    while error > umbral_error:
        initial_position = vision.point
        print("actual orientation: ", actual_orientation)
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
        giro_real = abs(actual_orientation - final_orientation)

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
actual_orientation=0
#spin in 0.1 seconds to right
spin_right_second=100
#spin in 0.1 seconds to up-right
spin_up_right_secon = 40
#pixels in 0.1 seconds
distance_second = 25
robot_pathing = False
total_distance = []

RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
END = '\033[0m'