import random
import threading
import time
import keyboard
import vision
import math
import numpy as np

def distance_two_points(initial_point, final_point):
        delta_x_point = final_point[0] - initial_point[0]
        delta_y_point = final_point[1] - initial_point[1]
        distance = math.sqrt(delta_x_point ** 2 + delta_y_point ** 2)
        return distance

def calcular_angulo_tangente(initial_point, final_point):
    pendiente = (initial_point[0] - final_point[0]) / (initial_point[1] - final_point[1])
    angulo_pendiente = math.degrees(math.atan2(pendiente, 1))
    return (angulo_pendiente + 180) % 360 - 180

def move(direction, t=0.1, mec="auto"):
    global spin_right_second, spin_up_right_secon, distance_second
    # print(f"first vision.point = {vision.point}")
    # print(f"time is {time}")
    # print(f"direction is {direction}")
    initial_position = vision.point
    vision.point = list(vision.point)
    if mec != "pathing":
        time.sleep(t)
    if direction in ["left", "right"]:
        print("GIRO")
        # Cambia la orientación según el tiempo y spin_right_second
        angle_change = (((spin_right_second * random.uniform(t, t+0.05)) / 0.1)+180)%360 -180
        vision.actual_orientation += angle_change if direction == "left" else -angle_change
        vision.actual_orientation = (vision.actual_orientation + 180) % 360 - 180
    elif direction in ["up", "down"]:
        # Mueve el punto según el tiempo y distance_per_second
        if mec == "pathing":
            mod = t
        else:
            mod = random.uniform(t-0.1, t+0.1)
        pixel_change = abs(int(distance_second * mod / 0.1))
        orientation = random.uniform(vision.actual_orientation-10, vision.actual_orientation + 10)
        orientation = (orientation + 180) % 360 - 180
        radian = math.radians(orientation)
        dx = int(pixel_change * math.cos(radian))
        dy = int(pixel_change * math.sin(radian))
        vision.point[0] += dx if direction == "up" else -dx
        vision.point[1] += dy if direction == "up" else -dy
    else:
        print("DIAGONAL")
        # Diagonales: mover y cambiar la orientación
        if mec == "pathing":
            mod = t
        else:
            mod = random.uniform(t, t+0.1)
        angle_change = (((spin_up_right_secon * mod) / 0.1) + 180) % 360 - 180
        # print("angle change", angle_change)
        orientation = vision.actual_orientation + angle_change if direction == "up-left" else vision.actual_orientation -angle_change
        orientation = (orientation + 180) % 360 - 180
        if mec == "pathing":
            mod = t
        else:
            mod = random.uniform(t-0.05, t+0.5)
        pixel_change = abs(int(distance_second * mod / 0.1))
        radian = math.radians(orientation)
        vision.point[0] += int(pixel_change * math.cos(radian))
        vision.point[1] += int(pixel_change * math.sin(radian))
    if mec != "pathing":
        time.sleep(1)   


    vision.point = tuple(vision.point)
    final_position = vision.point
    # print("Final position: ", final_position)
    # print("Actual orientation: ", actual_orientation)

    if mec != "pathing":
        vision.mostrar_frame(direction)
        
    if len(vision.limits) > 0:
        if not vision.robot_in_limits():
            go_robot_inside_limits()


def follow_path():
    global spin_right_second, distance_second
    vision.target_point = vision.path[0]
    GIRANDO_DERECHA = 'girando_derecha'
    GIRANDO_IZQUIERDA = 'girando_izquierda'
    YENDO_RECTO = 'yendo_recto'
    PARADO = 'parado'
    f = open("pathinglog.txt", "w")
    f.truncate(0)

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
    
    def moving():
        global estado_robot
        while estado_robot != None:
            if estado_robot == YENDO_RECTO:
                move("up", 0.050, "pathing")
            elif estado_robot == GIRANDO_DERECHA:
                move("up-right", 0.050, "pathing")
            if estado_robot == GIRANDO_IZQUIERDA:
                move("up-left", 0.050, "pathing")
            time.sleep(0.05)


    def pathing(last_point, last_orientation, n_errores = 0,fuera_de_linea=0, cont = 0):
        nonlocal f
        global angulo_closest_point, closest_point, estado_robot#,spining
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
            closest_point = get_closest_path_point()
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
            last_distance = distance_two_points(closest_point, last_point)
            actual_distance = distance_two_points(closest_point, vision.point)
            print("la distancia actual es de: ", actual_distance)
            f.write(f"la distancia actual es de: {actual_distance}\n")
            
            if actual_distance > 80:
                fuera_de_linea+=1
            if actual_distance < 1:
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
                elif error_orientation < 0:
                    print("robot has to go left")
                    f.write("robot has to go left\n")
                    estado_robot = GIRANDO_IZQUIERDA
                elif error_orientation > 0:
                    print("robot has to go right")
                    f.write("robot has to go right\n")
                    estado_robot = GIRANDO_DERECHA
            else:
                if abs(error_orientation) < 20:
                    print("El robot tiene que ir recto")
                    f.write("El robot tiene que ir recto\n")
                    estado_robot = YENDO_RECTO
                elif error_orientation < 0:
                    print("robot has to go left")
                    f.write("robot has to go left\n")
                    estado_robot = GIRANDO_IZQUIERDA
                elif error_orientation > 0:
                    print("robot has to go right")
                    f.write("robot has to go right\n")
                    estado_robot = GIRANDO_DERECHA
            print(f"estado de robot: {estado_robot}")
            f.write(f"estado de robot: {estado_robot}\n")

            # Verificar si el robot ha alcanzado el final de la trayectoria
            if distance_two_points(vision.point, vision.path[-1]) < 20 or closest_point == vision.path[-2]:  # Suponiendo que 20 es un umbral para estar "suficientemente cerca" del final
                estado_robot = PARADO
                robot_pathing = False
                f.close()
                closest_point = None
                angulo_closest_point = None
                print(f"fuera de linea: {fuera_de_linea}, de {cont} veces")
            else:
                cont+=1
                # if have_error:
                #     print("Ha habido error")
                #     #f.write("Ha habido error")
                #     actual_orientation = last_orientation
                # else:
                #     n_errores = 0
                print("esta lejos todavia, rootafter\n------------------------------\n")
                f.write("esta lejos todavia, rootafter\n------------------------------\n")
                # vision.root.after(50, lambda: pathing(vision.point, actual_orientation, f, n_errores))
                time.sleep(0.05)
                pathing(vision.point, vision.actual_orientation, n_errores, fuera_de_linea=fuera_de_linea, cont = cont)
        else:
            print("no se ha actualizado el frame\n------------------------------\n")
            f.write("no se ha actualizado el frame\n------------------------------\n")
            # vision.root.after(100, lambda: pathing(last_point, actual_orientation, f))
            time.sleep(0.1)
            pathing(last_point, vision.actual_orientation, fuera_de_linea=fuera_de_linea, cont = cont)

    def begin_path():
        global robot_pathing, estado_robot
        print(BLUE + "Empezando ruta" + END)
        # up_button.value = False
        # estado_robot = YENDO_RECTO
        robot_pathing = True
        orientation_robot_path = vision.orientation_two_points(vision.point, get_closest_path_point())
        error_angle = (orientation_robot_path - vision.actual_orientation + 180) % 360 - 180
        time.sleep(1)
        if error_angle > -30 or error_angle < 30:
            estado_robot = YENDO_RECTO
        elif error_angle < 0:
            estado_robot = GIRANDO_IZQUIERDA
        elif error_angle > 0:
            estado_robot = GIRANDO_DERECHA
        hilo = threading.Thread(target=moving)
        
        time.sleep(0.5)
        hilo.start()
        pathing(vision.point, estado_robot, vision.actual_orientation)
        

    def on_target_reached(first_point = None,cont=None, tiempo=None):
        global angulo_closest_point, closest_point
        print(GREEN + "ha llegado al punto" + END)
        print(f"Ha tardado {time.time() - tiempo} segundos y {cont} movimientos")
        f.write(f"Ha tardado {time.time() - tiempo} segundos y {cont} movimientos\n")
        if first_point == None:
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
        cont = 0
        tiempo_inicio=time.time()
        while abs(error_angle) > 30:
            cont+=1
            print("angulo de error con el camino:", error_angle)
            print("path orientation", angulo_closest_point)
            move_duration = (0.1 * distance_to_target()) / (distance_second + 10)
            if move_duration < 0.05:
                move_duration = 0.05
            print("orientando a la recta")
            if error_angle > 0:
                rotate_duration = abs(error_angle)  / (spin_right_second) *0.1
                print("have to move left")
                move("left", rotate_duration)
                #vision.mostrar_frame()
                #vision.root.update()
                move(up_down, move_duration)
            elif error_angle < 0:
                rotate_duration = abs(error_angle) / spin_right_second *0.1
                print("have to move rigth")
                move("right", rotate_duration)
                #vision.mostrar_frame()
                #vision.root.update()
                move(up_down, move_duration)
            up_down = "up" if up_down == "down" else "down"
            error_angle = (angulo_closest_point - vision.actual_orientation + 180) % 360 - 180
            print("\n------------------------------\n")
        print("preparado para seguirla")
        f.write(f"tiempo en orientarse {tiempo_inicio-time.time()}\nmovimientos = {cont}\n")

        begin_path()
    # f = open("pathinglog.txt", "w")
    # f.truncate(0)
    move_to_target(on_target_reached,threshold=25, tiempo=time.time())
    time.sleep(1)
    vision.mostrar_frame("up")



def distance_to_target():
    delta_x_point = vision.target_point[0] - vision.point[0]
    delta_y_point = vision.target_point[1] - vision.point[1]
    distance = math.sqrt(delta_x_point ** 2 + delta_y_point ** 2)
    return distance

def move_to_target(callback=None, threshold=50, kp=0.1, kd=0.1, file=None, cont =0, tiempo=None):
    global distance_second, avg_distance, avg_rotation, spin_right_second, spin_up_right_secon, robot_in_target
    robot_in_target = False
    initial_position = vision.point
    if file!=None:
        print("repathing")
        file.write(f"repathing: point: {initial_position}\n")
    print(f"actual orientation: {vision.actual_orientation}")

    delta_x_point = vision.target_point[0] - initial_position[0]
    delta_y_point = vision.target_point[1] - initial_position[1]
    distance = distance_to_target()
    print(f"distance to target: {distance}")
    print(f"target: {vision.target_point}")

    if distance < threshold:
        print("Target reached")
        print(cont)
        print(tiempo)
        if callback:
            callback(cont = cont, tiempo = tiempo)
        return

    angle_to_target = math.atan2(delta_y_point, delta_x_point)
    angle_to_target = math.degrees(angle_to_target)
    print(f"angle to target: {angle_to_target}")

    error_angle = vision.actual_orientation - angle_to_target

    error_angle = (error_angle + 180) % 360 - 180
    print("error angle:", error_angle)

    move_duration = (0.1 * distance_to_target()) / distance_second
    if move_duration > 0.4:
        move_duration = 0.4
    elif move_duration < 0.05:
        move_duration = 0.05
    
    if error_angle < 30 and error_angle > -30:
        
        move("up", move_duration)
    elif error_angle < -150:
        move_duration = (0.1 * distance_to_target()) / distance_second
        if move_duration > 0.4:
            move_duration = 0.4
        move("down", move_duration)
    elif error_angle < 0:

        if distance < 80:
            rotate_duration = 0.1 * abs(error_angle) / (spin_right_second)
            print("have to move left")
            move("left", rotate_duration)
            #vision.mostrar_frame()
            #vision.root.update()
            move("up", 0.1 * distance_to_target() / distance_second)
        else:
            rotate_duration = 0.1 * abs(error_angle) / (spin_up_right_secon)
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

    time.sleep(0.1)
    move_to_target(callback, threshold, file =file,cont=cont+1,tiempo=tiempo)


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
    print("Calibrate rotation")

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



robot_in_target = False
avg_distance = 0
avg_rotation = 0
#spin in 0.1 seconds to right
spin_right_second=100
#spin in 0.1 seconds to up-right
spin_up_right_secon = 30
#pixels in 0.1 seconds
distance_second = 20
robot_pathing = False
total_distance = []
index_path_spin = 0
spining = False
estado_robot = None
closest_point = None
angulo_closest_point = None
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
END = '\033[0m'