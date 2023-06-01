import cv2
import numpy as np

def marcar_regiones_disponibles(frame, rois):
    height, width = frame.shape[:2]
    available_regions = np.zeros((height, width), dtype=np.uint8)

    for roi in rois:
        x, y, w, h = roi
        available_regions[y:y+h, x:x+w] = 1

    return available_regions



cap = cv2.VideoCapture(1)
cv2.namedWindow("Deteccion de amarillo")

while True:
    key = cv2.waitKey(1) & 0xFF

    if key == ord("r"):
        ret, frame = cap.read()
        if ret:
            # Define las regiones de interés (x, y, ancho, alto)
            rois = [(100, 100, 300, 200), (400, 200, 100, 300)]

            # Marca las regiones disponibles en una matriz binaria
            available_regions = marcar_regiones_disponibles(frame, rois)

            # Muestra la matriz binaria en una ventana
            cv2.imshow("Regiones disponibles", available_regions * 255)

            # Procesa y muestra la imagen capturada
            #frame = detectar_amarillo(frame)
            cv2.imshow("Deteccion de amarillo", frame)

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
