import cv2
import numpy as np

def capturar_imagen(cap):
    ret, frame = cap.read()
    if ret:
        return frame

def mostrar_frame(window_name, frame):
    cv2.imshow(window_name, frame)


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    window_name = "Deteccion de amarillo"
    cv2.namedWindow(window_name)
    
    frame = capturar_imagen(cap)
    
    punto = (frame.shape[1] // 2, frame.shape[0] // 2)
    color_rojo = (0, 0, 255)
    radio = 5

    mostrar_frame(window_name, frame)
    while True:

        key = cv2.waitKey(1) & 0xFF

        if key == ord("a"):  # Tecla de flecha izquierda
            frame = capturar_imagen(cap)
            update_point(frame, punto, color_rojo, radio)
            mostrar_frame(window_name, frame)
            punto = (punto[0] - 10, punto[1])
        elif key == ord("w"):  # Tecla de flecha arriba
            frame = capturar_imagen(cap)
            update_point(frame, punto, color_rojo, radio)
            mostrar_frame(window_name, frame)
            punto = (punto[0], punto[1] - 10)
        elif key == ord("d"):  # Tecla de flecha derecha
            frame = capturar_imagen(cap)
            update_point(frame, punto, color_rojo, radio)
            mostrar_frame(window_name, frame)
            punto = (punto[0] + 10, punto[1])
        elif key == ord("s"):  # Tecla de flecha abajo
            frame = capturar_imagen(cap)
            update_point(frame, punto, color_rojo, radio)
            mostrar_frame(window_name, frame)
            punto = (punto[0], punto[1] + 10)
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
