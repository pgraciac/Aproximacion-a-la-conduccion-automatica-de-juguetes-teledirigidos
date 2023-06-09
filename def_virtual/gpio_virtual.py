import cv2
import math

def update_point(frame, point, color=(0, 0, 255), radio=5):
    cv2.circle(frame, point, radio, color, -1)

def move_point(direction, point, distance):
    x, y = point
    
    if direction == "left":
        x -= distance
    elif direction == "up":
        y -= distance
    elif direction == "right":
        x += distance
    elif direction == "down":
        y += distance
    elif direction == "up-right":
        x += distance / math.sqrt(2)
        y -= distance / math.sqrt(2)
    elif direction == "up-left":
        x -= distance / math.sqrt(2)
        y -= distance / math.sqrt(2)
    elif direction == "down-right":
        x += distance / math.sqrt(2)
        y += distance / math.sqrt(2)
    elif direction == "down-left":
        x -= distance / math.sqrt(2)
        y += distance / math.sqrt(2)

    return (int(x), int(y))


        