import re
import matplotlib.pyplot as plt

# Patrones de búsqueda para los diferentes elementos
pattern_point = re.compile(r"point: \((\d+), (\d+)\)")
pattern_closest_point = re.compile(r"el punto m.+s cercano de la recta es: \((\d+), (\d+)\)")
pattern_robot_state = re.compile(r"estado de robot: (\w+)")
pattern_good_orientation = re.compile(r"The good orientation is: (-*\d+\.\d+)")

# Listas para almacenar puntos, puntos más cercanos a la recta y buenas orientaciones
points = []
closest_points = []
good_orientations = []

# Diccionario para almacenar los puntos con su estado
points_with_state = {}

# Leyendo y procesando el archivo
with open('pathinglog.txt', 'r') as file:
    lines = file.readlines()
    i = 0
    for line in lines:
        # Extracción de puntos y puntos más cercanos a la recta
        if match_point := pattern_point.search(line):
            x, y = map(int, match_point.groups())
            points.append((x, y))
            # Extracción del estado del robot
            if i >= 4 and (state_line := lines[i - 4]):
                if match_state := pattern_robot_state.search(state_line):
                    points_with_state[(x, y)] = match_state.group(1)

        if match_closest_point := pattern_closest_point.search(line):
            x, y = map(int, match_closest_point.groups())
            closest_points.append((x, y))

        # Extracción de buenas orientaciones
        if match_orientation := pattern_good_orientation.search(line):
            good_orientation = float(match_orientation.group(1))
            good_orientations.append(good_orientation)
        i += 1

# Verificación de la longitud de los arrays
if len(points) != len(good_orientations):
    print(len(points), len(good_orientations))
    print(points, good_orientations)
    raise ValueError("La cantidad de puntos y buenas orientaciones no coincide.")

# Creación de la gráfica
plt.figure(figsize=(12, 8))
x_coords, y_coords = zip(*points)
plt.plot(x_coords, y_coords, color='blue', linestyle='-', marker='o', label='Path of Points')
plt.scatter(*zip(*closest_points), color='red', label='Closest Points to Line')
for point, state in points_with_state.items():
    plt.text(*point, f"{state}, {good_orientations[points.index(point)]:.2f}", fontsize=8)
plt.title('Points, Closest Points to Line, Robot States, and Good Orientations')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.legend()
plt.grid(True)
plt.show()
