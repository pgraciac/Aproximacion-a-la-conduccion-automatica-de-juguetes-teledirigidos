import sys
import threading
import time
import tkinter as tk
import cv2
import numpy as np
import math
import gpio_virtual
from KeyListener import KeyListener
from queue import Queue

def transform_point(point):
        return (point[0], frame.shape[0]-point[1])

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
        mostrar_frame()


def set_point_in_frame():
    global point, lower_green, upper_green, frame
    copy_frame = frame#correct_illumination(frame)
    if lower_green is None or upper_green is None:
        return
    hsv = cv2.cvtColor(copy_frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Variables para calcular el promedio de los centroides
    sum_cX = 0
    sum_cY = 0
    count = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 30:
            continue
        M = cv2.moments(contour)
        sum_cX += int(M["m10"] / M["m00"])
        sum_cY += int(M["m01"] / M["m00"])
        count += 1

    # Si se encontraron contornos, calcular el promedio de los centroides
    if count > 0:
        avg_cX = sum_cX // count
        avg_cY = sum_cY // count
        point = (avg_cX, frame.shape[0]-avg_cY)



def correct_illumination(frame):
    # Convert the image to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Apply Gaussian Blur
    hsv = cv2.GaussianBlur(hsv, (15, 15), 0)
    
    # Split the channels
    h, s, v = cv2.split(hsv)
    
    # Apply histogram equalization on the V channel
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    v = clahe.apply(v)
    
    # Merge the channels back
    hsv = cv2.merge([h, s, v])
    
    # Convert the image back to the BGR color space
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

def capturar_imagen():
    global frame, cap
    ret, frame = cap.read()
    if not ret:
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
        current_roi.append(transform_point((x,y)))
    elif event == cv2.EVENT_LBUTTONUP:
        (x_initial, y_initial) = current_roi[0]
        x_min, x_max = min(x_initial, x), max(x_initial, x)
        y_min, y_max = min(y_initial, frame.shape[0]-y), max(y_initial, frame.shape[0]-y)
        current_roi = [(xi, yi) for xi in range(x_min, x_max+1) for yi in range(y_min, y_max+1)]
        print("current roi:", current_roi)
        rois.append(current_roi)
        print(rois)
        update_frame("None")

def average_orientation(path, start_index, num_points, log=False):
    total_sin = 0
    total_cos = 0
    count = 0

    for i in range(start_index, start_index + num_points):
        if i + 1 >= len(path):
            break
        if log:
            print(path[i])
            print(orientation_two_points(path[i], path[i+1]))
        
        angle_deg = orientation_two_points(path[i], path[i+1])
        angle_deg = angle_deg + 360 if angle_deg < 0 else angle_deg
        angle_rad = math.radians(angle_deg)

        total_sin += math.sin(angle_rad)
        total_cos += math.cos(angle_rad)
        count += 1

    if count == 0:
        return None  # O devuelve un valor predeterminado si es más adecuado

    average_sin = total_sin / count
    average_cos = total_cos / count

    mean_angle_rad = math.atan2(average_sin, average_cos)
    mean_angle_deg = math.degrees(mean_angle_rad)  # Omitir si prefieres radianes
    return mean_angle_deg

def path_callback(event, x, y, flags, param):
    global current_mark, path, marking, path_spins
                
    if event == cv2.EVENT_LBUTTONDOWN:
        marking = True
        path = []
        path.append(transform_point((x,y)))
        # print(path)

    elif event == cv2.EVENT_MOUSEMOVE:
        if marking:
            if transform_point((x,y)) != path[-1]:
                path.append(transform_point((x,y)))
            # print(path)
            update_frame("None")
            
    elif event == cv2.EVENT_RBUTTONDOWN:
        print(path)
        marking = False
        current_mark = None
        path_spins = []
        path_orientation = average_orientation(path, 0, 5)
        for i in range(0, len(path) - 5):
            actual_path_orientation = average_orientation(path, i, 5)
            error = (path_orientation - actual_path_orientation + 180) % 360 - 180
            if abs(error) > 60:
                print(average_orientation(path, i + 1, 5, True))
                print(path[i+1], "\n------------------------------------\n")
                path_orientation = average_orientation(path, i + 1, 5)
                path_spins.append((path[i+1], path_orientation))

def point_callback(event, x, y, flags, param):
    global current_mark, target_point

    if event == cv2.EVENT_LBUTTONDOWN:
        target_point = transform_point((x, y))
        print(target_point)
        current_mark = None
        update_frame("None")

def limits_callback(event, x, y, flags, param):
    global limits, current_mark
    if event == cv2.EVENT_LBUTTONDOWN:
        new_point = (x, y)
        limits.append(transform_point(new_point))
        update_frame("None")

    elif event == cv2.EVENT_RBUTTONDOWN:
        current_mark = None
        limits.append(limits[0])
        update_frame("None")


def main_mouse_callback(event, x, y, flags, param):
    if current_mark=="regions":
        regions_callback(event, x, y, flags, param)
    elif current_mark=="path":
        path_callback(event, x, y, flags, param)
    elif current_mark=="target_point":
        point_callback(event, x, y, flags, param)
    elif current_mark=="limits":
        limits_callback(event, x, y, flags, param)

def set_current_mark(mark):
    global current_mark
    current_mark=mark
    print(current_mark)


def get_frame():
    global frame
    return frame

def point_in_rois():
    for roi in rois:
        if point in roi:
            return True
    return False

def mostrar_frame(mov = "None"):
    global point#, root
    capturar_imagen()
    update_frame(mov)
    # cv2.waitKey(0)

def add_orientation(): 
    global combined_frame, window_name
    angle = gpio_virtual.actual_orientation
    orientation_text = f"{gpio_virtual.actual_orientation:.2f}"
    cv2.putText(combined_frame, orientation_text, (transform_point(point)[0] + 10, transform_point(point)[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    # Dibujar el ángulo con una línea
    end_point_angle = (int(transform_point(point)[0] + 50 * math.cos(angle * np.pi / 180.0)), int(transform_point(point)[1] - 50 * math.sin(angle * np.pi / 180.0)))
    cv2.arrowedLine(combined_frame, transform_point(point), end_point_angle, (50, 100, 0), 2, tipLength=0.5)
    cv2.putText(combined_frame, f"Angle: {angle:.2f}", (transform_point(point)[0] + 60, transform_point(point)[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow(window_name, combined_frame)


def update_frame(mov):
    global frame, point, target_point, rois, path, window_name, limits, combined_frame, last_point, actual_orientation

    if target_point is not None:
        cv2.circle(frame, transform_point(target_point), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"{target_point}", (transform_point(target_point)[0] - 50, transform_point(target_point)[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    frame_line = frame.copy()
    for i in range(1, len(path)):
        cv2.line(frame_line, transform_point(path[i - 1]), transform_point(path[i]), (0, 255, 0), 50)
    alpha = 1
    beta = 0.3

    for region in rois:
        cv2.rectangle(frame, transform_point(region[0]), transform_point(region[-1]),(0,0,0),2)

    for i in range(1, len(limits)):
        cv2.line(frame, transform_point(limits[i - 1]), transform_point(limits[i]), (0, 0, 0), 2)

    if point_in_rois():
        cv2.putText(frame, "WARNING: POINT IS IN A INVALID PLACE", (140, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 0, 255), 1)

    if gpio_virtual.angulo_closest_point != None and gpio_virtual.closest_point != None:
        closest_point_trans = transform_point(gpio_virtual.closest_point)
        angle = gpio_virtual.angulo_closest_point

        # Dibujar el punto más cercano
        cv2.circle(frame, closest_point_trans, 5, (50, 100, 0), -1)
        cv2.putText(frame, f"Closest: {gpio_virtual.closest_point}", (closest_point_trans[0] + 10, closest_point_trans[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 100, 0), 2)

        # Dibujar el ángulo con una línea
        end_point_angle = (int(closest_point_trans[0] + 50 * math.cos(angle * np.pi / 180.0)), int(closest_point_trans[1] - 50 * math.sin(angle * np.pi / 180.0)))
        cv2.arrowedLine(frame, closest_point_trans, end_point_angle, (50, 100, 0), 2, tipLength=0.5)
        
    if point is not None:
        cv2.circle(frame, transform_point(point), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"{point}", (transform_point(point)[0] - 50, transform_point(point)[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    if (mov != "right" and mov != "left") and last_point != None and mov != "None":
        print("cambiando orientacion", last_point, point)
        delta_x_robot = 0
        delta_y_robot = 0
        if mov == "up" or mov == "up-right" or mov == "up-left" or mov == "pathing":
            delta_x_robot = point[0] - last_point[0]
            delta_y_robot = point[1] - last_point[1]
        elif mov == "down" or mov == "down-right" or mov == "down-left":
            delta_x_robot = last_point[0] - point[0]
            delta_y_robot = last_point[1] - point[1]
        print(f'delta y: {delta_y_robot} delta x: {delta_x_robot}')
        if abs(delta_y_robot) + abs(delta_x_robot) > 2:
            robot_orientation = math.degrees(math.atan2(delta_y_robot, delta_x_robot))
            actual_orientation = (robot_orientation % 360) - 360 if robot_orientation % 360 > 180 else robot_orientation % 360
            print(f"actual orientation after {mov}: {actual_orientation}")
        
        orientation_text = f"{actual_orientation:.2f}"
        cv2.putText(frame, orientation_text, (transform_point(point)[0] + 10, transform_point(point)[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Dibujar el ángulo con una línea
        end_point_angle = (int(transform_point(point)[0] + 50 * math.cos(actual_orientation * np.pi / 180.0)), int(transform_point(point)[1] - 50 * math.sin(actual_orientation * np.pi / 180.0)))
        cv2.arrowedLine(frame, transform_point(point), end_point_angle, (50, 100, 0), 2, tipLength=0.5)
        cv2.putText(frame, f"Angle: {actual_orientation:.2f}", (transform_point(point)[0] + 60, transform_point(point)[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)


    last_point = point
    combined_frame = cv2.addWeighted(frame, alpha, frame_line, beta, 0.0)
    cv2.imshow(window_name, combined_frame)

def robot_in_limits():
    print("calculating if robot is inside the limits")
    global point, limits
    x, y = point
    odd_nodes = False
    polygon = limits
    j = len(polygon) - 1  # Último punto del polígono
    
    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        
        if yi < y and yj >= y or yj < y and yi >= y:
            if xi + (y - yi) / (yj - yi) * (xj - xi) < x:
                odd_nodes = not odd_nodes
        j = i
    if odd_nodes:
        print("Robot inside the polygon")
    else:
        print("Robot outside the limits ")
    return odd_nodes

def init_vision():
    global cap, window_name, listener, point#,root
    cv2.namedWindow(window_name)
    cap = cv2.VideoCapture(1)

    cap.read()
    mostrar_frame()
    print(f"height 0: {frame.shape[0]}, height 1 {frame.shape[1]}")

    point = (frame.shape[1] // 2, frame.shape[0] // 2)

    mostrar_frame()

    cv2.setMouseCallback(window_name, main_mouse_callback)
        
    # check_thread = threading.Thread(target=listener.check_listener)
    # check_thread.start()


def finish_vision():
    global cap
    cap.release()
    cv2.destroyAllWindows()

def orientation_two_points(initial_point, final_point):
    delta_x_point = final_point[0] - initial_point[0]
    delta_y_point = final_point[1] - initial_point[1]
    angle_to_target = math.atan2(delta_y_point, delta_x_point)
    angle_to_target = math.degrees(angle_to_target)
    #print(f"angle to target: {angle_to_target}")
    angle_to_target = (angle_to_target + 180) % 360 - 180
    # print("angulo entre", initial_point, final_point, "= ", angle_to_target)
    return angle_to_target


marked_regions=False
current_roi=[]
rois=[]
current_point=[]
path=[]
path_spins=[]
limits=[]
target_point = None
current_mark=None
marking=False
point=None
last_point=None
actual_orientation = 0
lower_green = None
upper_green = None
frame = None
#root = tk.Tk()
window_name="Detectar_robot"
events_queue = Queue()

