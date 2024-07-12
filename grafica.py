import re
import matplotlib.pyplot as plt
import numpy as np

# Patrones de búsqueda para los diferentes elementos
pattern_point = re.compile(r"point: \((-*\d+), (-*\d+)\)")
pattern_repathing = re.compile(r"(?:repathing: )point: \((-*\d+), (-*\d+)\)")
pattern_closest_point = re.compile(r"el punto m.+s cercano de la recta es: *\((\d+), (\d+)\)")
pattern_robot_state = re.compile(r"estado de robot: (\w+)")
pattern_good_orientation = re.compile(r"The good orientation is: *(-*\d+\.\d+)")
patter_actual_orientation = re.compile(r"actual orientation: *(-*\d+\.\d+)")
pattern_punto_estimado = re.compile(r"PUNTO ESTIMADO: \[\((-*\d+\.*\d*), (-*\d+\.*\d*)\)")
pattern_distancia = re.compile(r"la distancia actual es de: *(\d+\.*\d*)")
# Listas para almacenar puntos, puntos más cercanos a la recta y buenas orientaciones
points = []
repath = []
closest_points = []
good_orientations = []
actual_orientations = []
estimates = []
distancias = []
# Diccionario para almacenar los puntos con su estado
robots_states = []

# Leyendo y procesando el archivo
with open('pathinglog.txt', 'r') as file:
    lines = file.readlines()
    for i in range(len(lines)-3):
        # Extracción de puntos y puntos más cercanos a la recta
        if match_point := pattern_repathing.search(lines[i]):
                x, y = map(int, match_point.groups())
                points.append(((x, y),"repath"))
                good_orientations.append("")
                robots_states.append("")
                # closest_points.append(None)
                actual_orientations.append("")

        if match_closest_point := pattern_closest_point.search(lines[i]):
            x, y = map(int, match_closest_point.groups())
            closest_points.append((x, y))

        # Extracción de buenas orientaciones
        if match_orientation := pattern_good_orientation.search(lines[i]):
            good_orientation = float(match_orientation.group(1))
            good_orientations.append(good_orientation)

            if match_point := pattern_point.search(lines[i-7]):#7 para normal 8 para estimar
                x, y = map(int, match_point.groups())
                if match_distancia := pattern_distancia.search(lines[i-1]):
                    distancia = float(match_distancia.group(1))
                    if distancia >= 80:
                        estado = "out"
                    else:
                        estado = "path"
                points.append(((x, y), estado))
                # Extracción del estado del robot
                if i >= 4 and (state_line := lines[i + 3]):
                    if match_state := pattern_robot_state.search(state_line):
                        robots_states.append(match_state.group(1))
            
            if math_actual := patter_actual_orientation.search(lines[i-6]):#6 -> 7
                actual_orientation = float(math_actual.group(1))
                actual_orientations.append(actual_orientation)
            
            
                distancias.append(float(match_distancia.group(1)))

            # if math_estimate := pattern_punto_estimado.search(lines[i-6]):
            #     x_str, y_str = math_estimate.groups()
            #     x = int(float(x_str))
            #     y = int(float(y_str))
            #     estimates.append((x, y))
            
            
# Verificación de la longitud de los arrays
if len(points) != len(good_orientations):
    print(len(points), len(good_orientations))
    print(points, good_orientations)
    raise ValueError("La cantidad de puntos y buenas orientaciones no coincide.")
print(f'Points: {points}\n\nestimate points: {estimates}\n\n good orientations: {good_orientations}\n\n robots_state: {robots_states}\n\n closest points: {closest_points}')
# Creación de la gráfica
plt.figure(figsize=(12, 8))
for i in range(len(points) - 1):  # Restamos 1 para evitar un IndexError en el último punto
    if points[i] is None or points[i+1] is None:
        continue  # Saltamos iteraciones donde el punto actual o el siguiente es None
    (x1, y1), state1 = points[i]
    (x2, y2), state2 = points[i+1]

    # Decidimos el color basado en el estado
    if state1 == 'path' or state2 == 'path':
        color = 'blue'
        state1="robot"
    elif state1 == 'repath' or state2 == 'repath':
        color = 'orange'
    elif state1 == 'out' and state2 == "out":
        color = 'purple'
    
    # Dibujamos el segmento de camino entre los puntos
    plt.plot([x1, x2], [y1, y2], color=color, linestyle='-', marker='o', label=state1)

plt.scatter(*zip(*closest_points), color='red', label='Closest Points to Robot')
for i, ((x,y), state) in enumerate(points):
    if state == "path" or state == "out":
        plt.text(x, y, f"{i}: {robots_states[i]}\ng: {good_orientations[i]:.2f}\nr: {actual_orientations[i]:.2f}\n", fontsize=10)
# x_estimates, y_estimates = zip(*estimates)
# x_estimates = x_estimates[3:]
# y_estimates = y_estimates[3:]
# plt.plot(x_estimates, y_estimates, color='green', linestyle='', marker='o', label='Path os estimate points')
# for i, point in enumerate(estimates):
#     i-=2
#     plt.text(*point, f"{i}", fontsize=15)
plt.title('Points, Closest Points to Line, Robot States, and Good Orientations')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys())
plt.grid(True)

# Ajustando los ticks de los ejes
plt.xlim(0, 1080)  # Obtiene los límites actuales del eje X
plt.ylim(0, 720)
plt.yticks(np.arange(0, 720, 50))
plt.xticks(np.arange(0, 1080, 100))  # Asegúrate de que los límites y el paso sean enteros
plt.show()
