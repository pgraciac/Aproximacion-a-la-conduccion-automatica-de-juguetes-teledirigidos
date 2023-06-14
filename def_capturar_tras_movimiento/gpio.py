import time
import board
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
        time.sleep(distance)
        left_button.value = True
    elif direction == "up":
        up_button.value = False
        time.sleep(distance)
        up_button.value = True
    elif direction == "right":
        right_button.value = False
        time.sleep(distance)
        right_button.value = True
    elif direction == "down":
        down_button.value = False
        time.sleep(distance)
        down_button.value = True
    elif direction == "up-right":
        up_button.value = False
        right_button.value = False
        time.sleep(distance)
        up_button.value = True
        right_button.value = True
    elif direction == "up-left":
        up_button.value = False
        left_button.value = False
        time.sleep(distance)
        up_button.value = True
        left_button.value = True
    elif direction == "down-right":
        down_button.value = False
        right_button.value = False
        time.sleep(distance)
        down_button.value = True
        right_button.value = True
    elif direction == "down-left":
        down_button.value = False
        left_button.value = False
        time.sleep(distance)
        down_button.value = True
        left_button.value = True
    time.sleep(1)
    vision.mostrar_frame()
    # angles(init_point)

# def robot_orientation():
#     init_position = get_point()
#     move("up")
#     time.sleep(0.5)
#     final_position = get_point()

def move_to_target(threshold=100, kp=0.1, kd=0.1):
    if not vision.listener.running():
        return
    if vision.target_point is None:
        print("Error: Target point is None")
        return
    if vision.point is None:
        print("Error: point is None")
        return
    # Get the initial position
    print(vision.point)
    initial_position = vision.point

    # Move the robot slightly forward to get the second point
    move("up", 0.2)

    # Get the position after moving
    print(vision.point)
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
    error_angle = (error_angle + math.pi) % (2 * math.pi) - math.pi

    # Decide on the robot's movement based on error_angle
    if abs(error_angle) < math.radians(30):
        # Move forward if the error is small
        move("up", kp)
    elif error_angle > 0:
        # Turn left if the target is to the left
        move("right", kd)
    else:
        # Turn right if the target is to the right
        move("left", kd)

    # Give the robot time to move
    time.sleep(1)
    print("------------------------------------------------")
    # Call this function again to keep moving
    vision.root.after(100, lambda: move_to_target())