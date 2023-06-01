import cv2
from vision import capturar_imagen

# Variables globales
dibujar = False
puntos = []
trayectoria = []

def dibujar_trayectoria(event, x, y, flags, param):
    global dibujar, puntos, trayectoria

    if event == cv2.EVENT_LBUTTONDOWN:
        dibujar = True
        puntos = [(x, y)]

    elif event == cv2.EVENT_MOUSEMOVE:
        if dibujar:
            puntos.append((x, y))

    elif event == cv2.EVENT_LBUTTONUP:
        dibujar = False
        trayectoria.append(puntos)

def mostrar_frame(window_name, frame):
    cv2.imshow(window_name, frame)

if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    window_name = "Deteccion de amarillo"
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, dibujar_trayectoria)
    cap.read()

    frame = capturar_imagen(cap)

    while True:
        for puntos in trayectoria:
            for i in range(1, len(puntos)):
                cv2.line(frame, puntos[i - 1], puntos[i], (0, 255, 0), 2)

        if not dibujar:
            mensaje = "Presione y arrastre el boton izquierdo del mouse para dibujar."
            cv2.putText(frame, mensaje, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, cv2.LINE_AA)

        mostrar_frame(window_name, frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("r"):
            frame = capturar_imagen(cap)
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
