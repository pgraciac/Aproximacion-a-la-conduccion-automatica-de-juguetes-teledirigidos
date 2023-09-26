import random
import time
import board
import keyboard
import digitalio
import vision
import math

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
    if mec == "auto":
        vision.mostrar_frame()
        vision.root.update()
        final_position = vision.point
        print("Final position: ", final_position)
        
        if direction == "up":
            delta_x_robot = final_position[0] - initial_position[0]
            delta_y_robot = final_position[1] - initial_position[1]
            robot_orientation = math.atan2(delta_x_robot, delta_y_robot)
            actual_orientation = math.degrees(robot_orientation)
            print("actual orientation after up: ", actual_orientation)
        elif direction == "down":
            delta_x_robot = final_position[0] - initial_position[0]
            delta_y_robot = final_position[1] - initial_position[1]
            robot_orientation = math.atan2(delta_x_robot, delta_y_robot)
            actual_orientation = (math.degrees(robot_orientation) + 180) % 360
            print("actual orientation after down: ", actual_orientation)
            actual_orientation += 10
    # #vision.mostrar_frame()
    # angles(init_point)

# def robot_orientation():
#     init_position = get_point()
#     move("up")
#     time.sleep(0.5)
#     final_position = get_point()

def distance_to_target():
    delta_x_point = vision.target_point[0] - vision.point[0]
    delta_y_point = vision.target_point[1] - vision.point[1]
    distance = math.sqrt(delta_x_point ** 2 + delta_y_point ** 2)
    return distance

def move_to_target(threshold=50, kp=0.1, kd=0.1):
    global distance_second, actual_orientation, avg_distance, avg_rotation, spin_right_second, spin_up_right_secon

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
        return

    # Calculate the angle to the target
    angle_to_target = math.atan2(delta_x_point, delta_y_point)
    angle_to_target = math.degrees(angle_to_target)
    print(f"angle to target: {angle_to_target}")
    # Calculate the angular error
    error_angle = angle_to_target - actual_orientation

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
    elif error_angle > 0:
        # Turn left if the target is to the left
        rotate_duration = abs(error_angle) / spin_up_right_secon * 0.1
        print("have to move up-left")
        move("up-left", rotate_duration)
        #vision.mostrar_frame()
        #vision.root.update()
        move("up", 0.2)
    elif error_angle < 0:
        rotate_duration = abs(error_angle) / spin_up_right_secon * 0.1
        # Turn left if the target is to the left
        print("have to move up-rigth")
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
    vision.root.after(100, lambda: move_to_target())
    
def go_robot_inside_limits():
    while not vision.robot_in_limits:
        best_distance = float('inf')
        best_vertex = ()
        for vertex in vision.limits:
            distance = math.sqrt((abs(vision.point[0] - vertex[0])) ** 2 + (abs(vision.point[1] - vertex[1])) ** 2)
            if distance < best_distance:
                best_distance = distance
                best_vertex = vertex
        distance = float('inf')
        best_point = ()
        for _ in range(2):
            add = -1
            A, B = (best_vertex, vision.limits[vision.limits.index(best_vertex) + add])
            add += 2
            p = best_vertex
            AP = (p[0]-A[0], p[1]-A[1])
            AB = (B[0]-A[0], B[1]-A[1])
            ab2 = AB[0]**2 + AB[1]**2
            ap_ab = AP[0]*AB[0] + AP[1]*AB[1]
            t = ap_ab / ab2

            if t < 0:
                t = 0
            elif t > 1:
                t = 1

            point_in_line = (A[0] + AB[0] * t, A[1] + AB[1] * t)
            distance = math.sqrt((abs(point_in_line[0] - best_vertex[0])) ** 2 + (abs(point_in_line[1] - best_vertex[1])) ** 2)
            if distance < best_distance:
                best_distance = distance
                best_point = point_in_line

        angle_to_polygon = math.atan2((abs(best_point[0] - vision.point[0])) ** 2 + (abs(best_point[1] - vision.point[1])) ** 2)
        angle_to_polygon = math.degrees(angle_to_polygon)

        error_angle = angle_to_polygon - actual_orientation
        initial_position = vision.point
        if error_angle < 30 and error_angle > -30:
            #Move forward if the error is small
            move_duration = (0.1 * distance_to_target()) / distance_second
            move("up", move_duration + 0.1)
        elif error_angle > 0 and error_angle < 120:
            # Turn left if the target is to the left
            rotate_duration = abs(error_angle) / spin_up_right_secon * 0.1
            print("have to move up-left")
            move("up-left", rotate_duration)
        elif error_angle < 0 and error_angle > -120:
            rotate_duration = abs(error_angle) / spin_up_right_secon * 0.1
            # Turn left if the target is to the left
            print("have to move up-rigth")
            move("up-right", rotate_duration)
        elif error_angle > 120:
            rotate_duration = abs(error_angle) / spin_right_second * 0.1
            # Turn left if the target is to the left
            print("have to move left")
            rotate_duration = abs(error_angle) / spin_right_second * 0.1
            move("left", 0.1)
            move("up", 0.1 + 0.1)
        elif error_angle < -120:
            # Turn left if the target is to the left
            print("have to move right")
            move("right", 0.1)
            move("up", 0.1 + 0.1)
        #vision.mostrar_frame()
        #vision.root.update()
        final_position= vision.point
        print("final: ", final_position)
        # Calculate the robot's orientation
        delta_x_robot = final_position[0] - initial_position[0]
        delta_y_robot = final_position[1] - initial_position[1]
        robot_orientation = math.atan2(delta_x_robot, delta_y_robot)
        actual_orientation = math.degrees(robot_orientation)
        vision.root.after(100, lambda: move_to_target())


def calibrate_distance():
    global distance_second
    
    total_distance = []
    
    # Duración inicial del movimiento
    print("Calibrating distance")
    for _ in range(10):
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
        time.sleep(1)
        
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
    distance_second = avg_distance_per_0_1_second

def calibrate_rotation():
    global actual_orientation
    print("Calibrate rotation")
    # Move robot forward to establish initial orientation
    initial_position = vision.point
    print(f"Initial position: {initial_position}")    

    giros = []
    for _ in range(25):
        initial_angle = actual_orientation
        tm = 0.1
        # Move robot in given direction (e.g., "up-right" or "up-left") for specified time
        move("up-right", tm)
        #vision.mostrar_frame()
        #vision.root.update()
        # Move robot forward to calculate new orientation
        move("up", 0.2)
        #vision.mostrar_frame()
        #vision.root.update()
        # Calculate final orientation
        final_angle = actual_orientation
        print("actual orientation after calibrating: ", actual_orientation)
        # Calculate and print rotation
        rotation = initial_angle - final_angle
        print(f"Rotation in {tm} seconds: {rotation} degrees")
        giros.append((abs(rotation) * 0.1) / tm)
        print(f'average rotation={sum(giros) / len(giros)}')
    average_rotation = sum(giros) / 100
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
        time.sleep(1)
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

avg_distance = 0
avg_rotation = 0
actual_orientation=0
#spin in 0.1 seconds to right
spin_right_second=124
#spin in 0.1 seconds to up-right
spin_up_right_secon = 30
#pixels in 0.1 seconds
distance_second = 0