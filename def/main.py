import cv2
import numpy as np

def detectar_amarillo(frame):
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
        cv2.circle(frame, (avg_cX, avg_cY), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"({avg_cX}, {avg_cY})", (avg_cX - 50, avg_cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    return frame

def capturar_imagen():
    ret, frame = cap.read()
    if ret:
        frame = detectar_amarillo(frame)
        cv2.imshow("Deteccion de amarillo", frame)


if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    cv2.namedWindow("Deteccion de amarillo")

    while True:
        key = cv2.waitKey(1) & 0xFF

        if key == ord("r"):
            capturar_imagen()
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
