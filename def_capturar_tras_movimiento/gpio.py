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


def move(direction, distance=0.1):
    print(f"Moving {direction} for {distance} seconds")
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
    # vision.mostrar_frame()
    # angles(init_point)

# def robot_orientation():
#     init_position = get_point()
#     move("up")
#     time.sleep(0.5)
#     final_position = get_point()

def move_to_target(threshold=50, kp=0.1, kd=0.1):
    if not vision.listener.running():
        return
    if vision.target_point is None:
        print("Error: Target point is None")
        return
    if vision.point is None:
        print("Error: point is None")
        return
    # Get the initial position
    print("initial: ", vision.point)
    initial_position = vision.point

    # Move the robot slightly forward to get the second point
    move("up", 0.2)
    vision.mostrar_frame()
    vision.root.update()
    # Get the position after moving
    print("next: ", vision.point)
    next_position = vision.point

    # Calculate the robot's orientation
    delta_x_robot = next_position[0] - initial_position[0]
    delta_y_robot = next_position[1] - initial_position[1]
    robot_orientation = math.atan2(delta_x_robot, delta_y_robot)
    robot_orientation = math.degrees(robot_orientation)
    print(f"robot orientation: {robot_orientation}")
    # Calculate the distance to the target
    delta_x_point = vision.target_point[0] - next_position[0]
    delta_y_point = vision.target_point[1] - next_position[1]
    distance = math.sqrt(delta_x_point ** 2 + delta_y_point ** 2)

    # Check if the target is reached
    if distance < threshold:
        print("Target reached")
        return

    # Calculate the angle to the target
    angle_to_target = math.atan2(delta_x_point, delta_y_point)
    angle_to_target = math.degrees(angle_to_target)
    print(f"angle to target: {angle_to_target}")
    # Calculate the angular error
    error_angle = angle_to_target - robot_orientation

    # Normalize the error_angle to be between -pi and pi
    error_angle = (error_angle + 180) % 360 - 180
    print("error angle:", error_angle)
    # Decide on the robot's movement based on error_angle
    #keyboard.wait('space')
    rotate_duration = abs(error_angle / avg_rotation) * 0.1  # Scale with the avg rotation
    # if error_angle < 30:
    #     # Move forward if the error is small
    #     move_duration = abs(distance / avg_distance) * 0.1  # Scale with the avg distance
    #     move("up", move_duration)
    if error_angle > 0 and error_angle < 90:
        # Turn left if the target is to the left
        move("up-left", rotate_duration)
    elif error_angle < 0 and error_angle < -90:
        # Turn left if the target is to the left
        move("up-right", rotate_duration)
    elif error_angle > 90:
        # Turn left if the target is to the left
        move("left", rotate_duration)
    elif error_angle < -90:
        # Turn left if the target is to the left
        move("right", rotate_duration)

    print("------------------------------------------------")
    # Call this function again to keep moving
    vision.root.after(100, lambda: move_to_target())

def calibrate_distance(tm=0.1):
    # Get initial position
    print("Calibrating distance")
    initial_position = vision.point
    # Move robot forward for specified time
    print(f"Initial position: {initial_position}")
    move("up", tm)
    vision.mostrar_frame()
    vision.root.update()
    time.sleep(1)
    # Get new position
    final_position = vision.point
    print(f"Final position: {final_position}")

    # Calculate and print distance moved
    delta_x = final_position[0] - initial_position[0]
    delta_y = final_position[1] - initial_position[1]
    distance = math.sqrt(delta_x**2 + delta_y**2)
    print(f"Distance moved in {tm} seconds: {distance} units")
    return distance

def calibrate_rotation(tm=0.1, direction="up-right"):
    print("Calibrate rotation")
    # Move robot forward to establish initial orientation
    initial_position = vision.point
    print(f"Initial position: {initial_position}")    
    move("up", tm)
    vision.mostrar_frame()
    vision.root.update()
    next_position = vision.point
    print(f"Next position: {next_position}")    
    # Calculate initial orientation
    delta_x_initial = next_position[0] - initial_position[0]
    delta_y_initial = next_position[1] - initial_position[1]
    initial_angle = math.atan2(delta_y_initial, delta_x_initial)
    initial_angle = math.degrees(initial_angle)

    # Move robot in given direction (e.g., "up-right" or "up-left") for specified time
    move(direction, tm)
    vision.mostrar_frame()
    vision.root.update()
    initial_position = vision.point
    print(f"Initial position: {initial_position}")    
    # Move robot forward to calculate new orientation
    move("up", tm)
    vision.mostrar_frame()
    vision.root.update()
    final_position = vision.point
    print(f"Final position: {final_position}")    
    # Calculate final orientation
    delta_x_final = final_position[0] - initial_position[0]
    delta_y_final = final_position[1] - initial_position[1]
    final_angle = math.atan2(delta_y_final, delta_x_final)
    final_angle = math.degrees(final_angle)

    # Calculate and print rotation
    rotation = final_angle - initial_angle
    print(f"Rotation in {tm} seconds: {rotation} degrees")
    return rotation

def perform_calibration(repetitions=1, tm=0.1):
    global avg_distance, avg_rotation
    total_distance = 0
    total_rotation = 0
    rot = True
    for _ in range(repetitions):
        total_distance += calibrate_distance(tm)
        if rot:
            total_rotation += calibrate_rotation(tm)
        else:
            total_rotation += abs(calibrate_rotation(tm, "up-left"))
        rot = not rot
    avg_distance = total_distance / repetitions
    avg_rotation = total_rotation / repetitions

avg_distance = 0
avg_rotation = 0
actual_orientation=0