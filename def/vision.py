import sys
import tkinter as tk
import cv2
import numpy as np

def detectar_verde(frame):
    global point
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
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
        cv2.circle(frame, (avg_cX, avg_cY), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"({avg_cX}, {avg_cY})", (avg_cX - 50, avg_cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return frame

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
        
        # Crear botones y asociarlos a sus funciones correspondientes
        btn_yes = tk.Button(regions, text="Finish", command=lambda: (setattr(sys.modules[__name__], 'current_mark', None), setattr(sys.modules[__name__], 'marking', False) ,regions.destroy()))

        # Colocar los botones en la ventana
        btn_yes.pack(fill=tk.BOTH, expand=True)
    marking = True
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
    global current_mark, current_point, path, marking
                
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

def main_mouse_callback(event, x, y, flags, param):
    if current_mark=="regions":
        regions_callback(event, x, y, flags, param)
    elif current_mark=="path":
        path_callback(event, x, y, flags, param)

def set_current_mark(mark):
    global current_mark
    current_mark=mark
    print(current_mark)

def point_in_rois():
    for roi in rois:
        if point in roi:
            return True
    return False

marked_regions=False
current_roi=[]
rois=[]
current_point=[]
path=[]
current_mark=None
marking=False
point=None