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

def capturar_imagen():
    ret, frame = cap.read()
    if ret:
        frame = detectar_amarillo(frame)
        cv2.imshow("Deteccion de amarillo", frame)


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    cv2.namedWindow("Deteccion de amarillo")

    while True:
        key = cv2.waitKey(1) & 0xFF

        if key == ord("r"):
            capturar_imagen()
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
