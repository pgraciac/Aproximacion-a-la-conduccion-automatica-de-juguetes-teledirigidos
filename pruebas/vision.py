import cv2
import numpy as np

def captura():
    # Inicializa la captura de video desde la webcam
    cap = cv2.VideoCapture(0)

    while True:
        # Captura frame
        ret, frame = cap.read()

        # Si no devuelve return aborta el while
        if not ret:
            break

        # Convierte frame a HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Limite inferior y superior para el color amarillo en HSV
        yellow_inf = np.array([20, 100, 100])
        yelow_sup = np.array([30, 255, 255])


        # Máscara para el color amarillo
        yellow_mask = cv2.inRange(hsv, yellow_inf, yelow_sup)

        # Aplica la máscara al frame original
        yellow_filt = cv2.bitwise_and(frame, frame, mask=yellow_mask)

        # Camara normal y camara detectando amarillo
        cv2.imshow('Original', frame)
        cv2.imshow('Red Filtered', yellow_filt)

        # Si se presiona la tecla 's', termina el bucle
        if cv2.waitKey(1) & 0xFF == ord('s'):
            break

    # Libera los recursos y cierra las ventanas
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    captura()
