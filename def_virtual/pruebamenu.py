import cv2
import numpy as np
import PySimpleGUI as sg
from gpio_virtual import move_point
from vision import capturar_imagen
from gpio_virtual import update_point

def mostrar_frame(window, key, frame):
    imgbytes = cv2.imencode(".png", frame)[1].tobytes()
    window[key].update(data=imgbytes)

def menu():
    layout = [[sg.Button("Marcar regiones"), sg.Button("Marcar trayectoria"), sg.Button("Empezar ejecución"), sg.Button("Salir")]]

    window = sg.Window("Menú", layout)

    while True:
        event, values = window.read()

        if event == "Marcar regiones":
            window.close()
            return "regions"
        elif event == "Marcar trayectoria":
            window.close()
            return "path"
        elif event == "Empezar ejecución":
            window.close()
            return "execute"
        elif event == "Salir" or event == sg.WIN_CLOSED:
            window.close()
            return "quit"

def draw_rectangle(event, values, graph, start_point):
    end_point = (event[0], event[1])
    graph.draw_rectangle(start_point, end_point, line_color="red", fill_color=None)
    return end_point

if __name__ == '__main__':
    cap = cv2.VideoCapture(1)
    red_color = (0, 0, 255)
    cap.read()
    
    frame = capturar_imagen(cap)
    point = (frame.shape[1] // 2, frame.shape[0] // 2)

    action = menu()

    if action == "quit":
        cap.release()
        cv2.destroyAllWindows()
        exit()

    # GUI para mostrar el frame y permitir marcar regiones
    imgbytes = cv2.imencode(".png", frame)[1].tobytes()
    graph = sg.Graph((frame.shape[1], frame.shape[0]), (0, 0), (frame.shape[1], frame.shape[0]), key="graph", enable_events=True)
    layout = [[graph, sg.Image(data=imgbytes, key="frame")],
              [sg.Button("Guardar regiones"), sg.Button("Cancelar")]]
    window = sg.Window("Marcar regiones", layout, finalize=True)
    graph_widget = window["graph"].Widget
    graph_widget.config(cursor="crosshair")

    regions = []
    start_point = None

    while action == "regions":
        event, values = window.read()

        if event == "graph":
            if start_point is None:
                start_point = (values["graph"][0], values["graph"][1])
            else:
                end_point = draw_rectangle(event, values, graph, start_point)
                regions.append((start_point, end_point))
                start_point = None
        elif event == "Guardar regiones":
            break
        elif event == "Cancelar" or event == sg.WIN_CLOSED:
            regions = []
            break

    window.close()

    # Agrega aquí el código para manejar las acciones "path" y "execute"

    while True:
        frame = capturar_imagen(cap)
        
        for region in regions:
            cv2.rectangle(frame, region[0], region[1], red_color, 2)
        
        key = cv2.waitKey(1) & 0xFF

        if key == ord("a"):  # Tecla de flecha izquierda
            point = move_point("left", point)
            frame = capturar_imagen(cap)
            update_point(frame, point)
            cv2.imshow("Deteccion de amarillo", frame)

        elif key == ord("w"):  # Tecla de flecha arriba
            point = move_point("up", point)
            frame = capturar_imagen(cap)
            update_point(frame, point)
            cv2.imshow("Deteccion de amarillo", frame)

        elif key == ord("d"):  # Tecla de flecha derecha
            point = move_point("right", point)
            frame = capturar_imagen(cap)
            update_point(frame, point)
            cv2.imshow("Deteccion de amarillo", frame)

        elif key == ord("s"):  # Tecla de flecha abajo
            point = move_point("down", point)
            frame = capturar_imagen(cap)
            update_point(frame, point)
            cv2.imshow("Deteccion de amarillo", frame)

        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
