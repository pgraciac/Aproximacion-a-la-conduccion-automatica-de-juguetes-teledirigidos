import time
import board
import digitalio

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
