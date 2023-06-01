import cv2


def update_point(frame, point, color=(0, 0, 255), radio=5):
    cv2.circle(frame, point, radio, color, -1)

def move_point(dir,point):
    if dir=="left":
        point = (point[0] - 10, point[1])
    elif dir=="up":
        point = (point[0], point[1] - 10)
    elif dir=="right":
        point = (point[0] + 10, point[1])
    elif dir=="down":
        point = (point[0], point[1] + 10)

    return point
        