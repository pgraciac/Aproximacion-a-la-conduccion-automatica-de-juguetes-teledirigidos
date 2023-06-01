import sys
import tkinter as tk
import cv2
import numpy as np

def detectar_amarillo(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 150, 100])
    upper_yellow = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 100:
            continue
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"({cX}, {cY})", (cX - 50, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        #print(f"Coordenadas del centroide amarillo: ({cX}, {cY})")

    return frame

def capturar_imagen(cap):
    ret, frame = cap.read()
    if ret:
        return frame    
    else:
        print("Error capturing image")

# def mark_regions(window_name, rois):
#     global marked_regions
#     marked_regions = True
#     regions = tk.Tk()
#     regions.title("Regions marked yet?")
    
#     # Crear botones y asociarlos a sus funciones correspondientes
#     btn_yes = tk.Button(regions, text="Finish", command=lambda: (setattr(sys.modules[__name__], 'marked_regions', False), regions.destroy()))

#     # Colocar los botones en la ventana
#     btn_yes.pack(fill=tk.BOTH, expand=True)

#     def on_mouse_click(event, x, y, flags, params):
#         global current_roi
#         if marked_regions==True:
#             if event == cv2.EVENT_LBUTTONDOWN:
#                 current_roi=[]
#                 current_roi.append((x,y))
#             elif event == cv2.EVENT_LBUTTONUP:
#                 current_roi.append((x,y))
#                 rois.append(current_roi)
#                 print(rois)

#     cv2.setMouseCallback(window_name, on_mouse_click)


# def mark_path():
#     pass

def regions_callback(event, x, y, flags, param):
    global current_roi
    global current_mark
    global rois
    global marking
    if marking==False:
        regions = tk.Tk()
        regions.title("Regions marked yet?")
        
        # Crear botones y asociarlos a sus funciones correspondientes
        btn_yes = tk.Button(regions, text="Finish", command=lambda: (setattr(sys.modules[__name__], 'current_mark', "none"), setattr(sys.modules[__name__], 'marking', False) ,regions.destroy()))

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
        current_mark = "none"

def main_mouse_callback(event, x, y, flags, param):
    if current_mark=="regions":
        regions_callback(event, x, y, flags, param)
    elif current_mark=="path":
        path_callback(event, x, y, flags, param)

def set_current_mark(mark):
    global current_mark
    current_mark=mark
    print(current_mark)

def point_in_rois(point):
    for roi in rois:
        if point in roi:
            return True
        else:
            return False

marked_regions=False
current_roi=[]
rois=[]
current_point=[]
path=[]
current_mark="none"
marking=False