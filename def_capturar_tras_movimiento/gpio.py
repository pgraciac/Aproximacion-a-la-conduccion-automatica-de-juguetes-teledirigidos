import random
import time
import board
import keyboard
import digitalio
import vision
import math
import numpy as np

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
    print("angulo entre", initial_point, final_point, "= ", angle_to_target)
    return angle_to_target



def move(direction, distance, mec="auto"):
    global actual_orientation
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
    vision.mostrar_frame()
    final_position = vision.point
    print("Final position: ", final_position)
    
    if direction == "up":
        delta_x_robot = final_position[0] - initial_position[0]
        delta_y_robot = final_position[1] - initial_position[1]
        robot_orientation = math.degrees(math.atan2(delta_y_robot, delta_x_robot))
        actual_orientation = (robot_orientation % 360) - 360 if robot_orientation % 360 > 180 else robot_orientation % 360
        print("actual orientation after up: ", actual_orientation)
    elif direction == "down":
        delta_x_robot = initial_position[0] - final_position[0]
        delta_y_robot = initial_position[1] - final_position[1] 
        robot_orientation = math.degrees(math.atan2(delta_y_robot, delta_x_robot))
        actual_orientation = (robot_orientation % 360) - 360 if robot_orientation % 360 > 180 else robot_orientation % 360
        print("actual orientation after down: ", actual_orientation)
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
    
    def begin_path():
        global up_button, right_button, left_button
        print(BLUE + "Empezando ruta" + END)
        up_button.value = False
        finish_path = False
        while not finish_path:
            #orientacion FALTAA
            vision.mostrar_frame()
            closest_point = get_closest_path_point()
            # Si el robot está a la izquierda del punto, gira a la derecha y viceversa
            if vision.point[0] < closest_point[0]:
                right_button.value = False
                time.sleep(0.1)  # Gira el robot 30 grados a la derecha
                right_button.value = True
            elif vision.point[0] > closest_point[0]:
                left_button.value = False
                time.sleep(0.1)  # Gira el robot 30 grados a la izquierda
                left_button.value = True
            if ((vision.point[0] - vision.path[-1][0])**2 + (vision.point[1] - vision.path[-1][1])**2)**0.5 < 20:
                finish_path = True
                up_button.value = True

    def on_target_reached():
        print(GREEN + "ha llegado al punto" + END)
        path_orientation = orientation_two_points(vision.path[0], vision.path[10])
        error_angle = path_orientation - actual_orientation
        print("angulo de error con el camino:", error_angle)
        up_down="up"
        while abs(error_angle) > 20:
            print("angulo de error con el camino:", error_angle)
            print("path orientation", path_orientation)

            print("orientando a la recta")
            if error_angle > 0:
                # Turn left if the target is to the left
                rotate_duration = abs(error_angle) / (spin_right_second - 20) * 0.1
                print("have to move left")
                move("left", rotate_duration)
                #vision.mostrar_frame()
                #vision.root.update()
                move(up_down, 0.05 * distance_to_target() / distance_second)
            elif error_angle < 0:
                rotate_duration = abs(error_angle) / spin_right_second * 0.1
                # Turn left if the target is to the left
                print("have to move rigth")
                move("right", rotate_duration)
                #vision.mostrar_frame()
                #vision.root.update()
                move(up_down, 0.05 * distance_to_target() / distance_second)
            up_down = "up" if up_down == "down" else "down"
            error_angle = path_orientation - actual_orientation
        print("preparado para seguirla")
        begin_path()

    move_to_target(on_target_reached,threshold=10)
    



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

    if error_angle < 30 and error_angle > -30:
        #Move forward if the error is small
        move_duration = (0.1 * distance_to_target()) / distance_second
        if move_duration > 0.3:
            move_duration = 0.3
        move("up", move_duration)
    elif error_angle < -155:
        move_duration = (0.1 * distance_to_target()) / distance_second
        if move_duration > 0.3:
            move_duration = 0.3
        move("down", move_duration)
    elif error_angle < 0:
        # Turn left if the target is to the left
        if distance < 100:
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
        if distance < 100:
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
    final_position= vision.point
    print("final: ", final_position)
    # Calculate the robot's orientation
    print("actual orientation after move: ", actual_orientation)
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



def calibrate_distance(times=1):
    global distance_second

    total_distance = []
    
    # Duración inicial del movimiento
    print("Calibrating distance")
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
        
        # Acumular distancia y número de iteraciones
        total_distance.append((distance / tm) * 0.1)
        
        # Aumentar el tiempo de movimiento para la próxima iteración
        
    # Calcular y mostrar los píxeles movidos en promedio por 0.1 segundos
    avg_distance_per_0_1_second = sum(total_distance) / len(total_distance)
    print(f"Robot moves {avg_distance_per_0_1_second} pixels in 0.1 second on average.")
    #Cambiado a 23 para que no repita tanto
    distance_second = 25

def calibrate_rotation():
    global actual_orientation
    print("Calibrate rotation")
    # Move robot forward to establish initial orientation
    initial_position = vision.point
    print(f"Initial position: {initial_position}")    

    giros = []
    for _ in range(25):
        initial_angle = actual_orientation + 360 if actual_orientation < 0 else actual_orientation
        tm = random.uniform(0.05, 0.2)
        # Move robot in given direction (e.g., "up-right" or "up-left") for specified time
        move("right", tm)
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
        rotation = 360 - final_angle - initial_angle
        print(f"Rotation in {tm} seconds: {rotation} degrees")
        giros.append((abs(rotation) * 0.1) / tm)
        print(f'average rotation={sum(giros) / len(giros)}')
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
spin_right_second=110
#spin in 0.1 seconds to up-right
spin_up_right_secon = 30
#pixels in 0.1 seconds
distance_second = 25


RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
END = '\033[0m'