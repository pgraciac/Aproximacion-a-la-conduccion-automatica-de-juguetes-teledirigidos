import sys
import time
import tkinter as tk
import cv2
import numpy as np
import math


def set_color_range(event, x, y, flags, param):
    global lower_green, upper_green
    if event == cv2.EVENT_LBUTTONDOWN:
        # Obtén el color del pixel (en BGR)
        color_bgr = param[y, x]
        color_rgb = [color_bgr[2], color_bgr[1], color_bgr[0]]
        
        # Convertir de RGB a HSV
        color_hsv = cv2.cvtColor(np.uint8([[color_rgb]]), cv2.COLOR_RGB2HSV)[0][0]
        
        # Establecer rangos en HSV
        lower_green = np.array([color_hsv[0] - 5, 80, 80])
        upper_green = np.array([color_hsv[0] + 5, 255, 255])
        print("Color seleccionado:", color_rgb)
        print("Lower Green HSV:", lower_green)
        print("Upper Green HSV:", upper_green)


def detectar_verde(frame):
    global point, lower_green, upper_green
    if lower_green is None or upper_green is None:
        return frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Variables para calcular el promedio de los centroides
    sum_cX = 0
    sum_cY = 0
    count = 0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 100:
            continue
        M = cv2.moments(contour)
        sum_cX += int(M["m10"] / M["m00"])
        sum_cY += int(M["m01"] / M["m00"])
        count += 1

    # Si se encontraron contornos, calcular el promedio de los centroides
    if count > 0:
        avg_cX = sum_cX // count
        avg_cY = sum_cY // count
        point = (avg_cX, avg_cY)

def capturar_imagen(cap):
    ret, frame = cap.read()
    if ret:
        return frame    
    else:
        print("Error capturing image")

def regions_callback(event, x, y, flags, param):
    global current_roi
    global current_mark
    global rois
    global marking
    if marking==False:
        regions = tk.Tk()
        regions.title("Regions marked yet?")
        marking = True
        # Crear botones y asociarlos a sus funciones correspondientes
        btn_yes = tk.Button(regions, text="Finish", command=lambda: (setattr(sys.modules[__name__], 'current_mark', None), setattr(sys.modules[__name__], 'marking', False) ,regions.destroy()))

        # Colocar los botones en la ventana
        btn_yes.pack(fill=tk.BOTH, expand=True)
    
    if event == cv2.EVENT_LBUTTONDOWN:
        current_roi=[]
        current_roi.append((x,y))
    elif event == cv2.EVENT_LBUTTONUP:
        (x_initial, y_initial) = current_roi[0]
        x_min, x_max = min(x_initial, x), max(x_initial, x)
        y_min, y_max = min(y_initial, y), max(y_initial, y)
        current_roi = [(xi, yi) for xi in range(x_min, x_max+1) for yi in range(y_min, y_max+1)]
        print("current roi:", current_roi)
        rois.append(current_roi)
        print(rois)

def path_callback(event, x, y, flags, param):
    global current_mark, path, marking
                
    if event == cv2.EVENT_LBUTTONDOWN:
        marking = True
        path.append((x,y))
        print(path)

    elif event == cv2.EVENT_MOUSEMOVE:
        if marking:
            path.append((x,y))
            print(path)
            
    elif event == cv2.EVENT_RBUTTONDOWN:
        marking = False
        current_mark = None

def point_callback(event, x, y, flags, param):
    global current_mark, target_point

    if event == cv2.EVENT_LBUTTONDOWN:
        target_point = (x, y)
        print(target_point)
        current_mark = None

def main_mouse_callback(event, x, y, flags, param):
    if current_mark=="regions":
        regions_callback(event, x, y, flags, param)
    elif current_mark=="path":
        path_callback(event, x, y, flags, param)
    elif current_mark=="target_point":
        point_callback(event, x, y, flags, param)


def set_current_mark(mark):
    global current_mark
    current_mark=mark
    print(current_mark)

def get_point():
    return point

def point_in_rois():
    for roi in rois:
        if point in roi:
            return True
    return False

def angles(initial_position):
    global point, target_point

    # Get the position after moving
    time.sleep(0.5)
    next_position = point

    # Calculate the robot's orientation
    delta_x_orientation = next_position[0] - initial_position[0]
    delta_y_orientation = next_position[1] - initial_position[1]
    robot_orientation = math.atan2(delta_y_orientation, delta_x_orientation)
    robot_orientation = math.degrees(robot_orientation)
    # Calculate the distance to the target
    delta_x = target_point[0] - next_position[0]
    delta_y = target_point[1] - next_position[1]
    distance = math.sqrt(delta_x ** 2 + delta_y ** 2)

    # Calculate the angle to the target
    angle_to_target = math.atan2(delta_y, delta_x)
    angle_to_target = math.degrees(angle_to_target)
    print(f'robot orientation: {robot_orientation}, distance to target: {distance}, angle to target: {angle_to_target}')
    # Calculate the angular error
    error_angle = angle_to_target - robot_orientation
    print(f'error angle not normalized: {error_angle}')
    # Normalize the error_angle to be between -pi and pi
    error_angle = (error_angle + math.pi) % (2 * math.pi) - math.pi
    print(f'error angle normalized: {error_angle}')



marked_regions=False
current_roi=[]
rois=[]
current_point=[]
path=[]
target_point = None
current_mark=None
marking=False
point=None
lower_green = None
upper_green = None
