import pygame
import sys
import random
import math

def closest_point_on_line(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))  # Asegurar que t esté en el rango [0, 1]
    return x1 + t * dx, y1 + t * dy

# Inicialización de Pygame
pygame.init()

# Dimensiones de la pantalla
width = 800
height = 600

# Crear la pantalla
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Punto siguiendo la línea')

# Lista de puntos que forman la línea
line_points = [(50, 50), (200, 100), (300, 250), (500, 400), (700, 300)]

# Posición inicial del punto
x, y = line_points[0]

# Índice del siguiente punto en la línea
next_point_index = 1

# Umbral de distancia para considerar que el punto está en la línea
distance_threshold = 10

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Limpiar pantalla
    screen.fill((255, 255, 255))

    # Dibujar línea
    pygame.draw.lines(screen, (0, 0, 0), False, line_points, 2)

    # Dibujar punto
    pygame.draw.circle(screen, (255, 0, 0), (x, y), 5)

    # Calcular el punto más cercano en la línea y verificar si el punto está dentro del umbral de distancia
    closest_point = None
    min_distance = float('inf')
    for i in range(len(line_points) - 1):
        point = closest_point_on_line(x, y, line_points[i][0], line_points[i][1], line_points[i + 1][0], line_points[i + 1][1])
        distance = math.sqrt((x - point[0]) ** 2 + (y - point[1]) ** 2)
        if distance < min_distance:
            min_distance = distance
            closest_point = point

    on_line = min_distance <= distance_threshold

    # Generar velocidad aleatoria entre 5 y 20 píxeles
    speed = random.randint(5, 20)

    # Si el punto está en la línea, intentar llegar al siguiente punto
    if on_line:
        next_point = line_points[next_point_index]
        dx, dy = next_point[0] - x, next_point[1] - y
        if abs(dx) > abs(dy):
            x += speed if dx > 0 else -speed
        else:
            y += speed if dy > 0 else -speed
            
        # Verificar si se ha llegado al siguiente punto
        distance_to_next_point = math.sqrt((x - next_point[0]) ** 2 + (y - next_point[1]) ** 2)
        if distance_to_next_point <= distance_threshold:
            next_point_index += 1

        # Si se ha llegado al último punto, reiniciar el índice del siguiente punto
        if next_point_index == len(line_points):
            next_point_index = 0

    # Si el punto no está en la línea, moverse hacia el punto más cercano en la línea
    else:
        dx, dy = closest_point[0] - x, closest_point[1] - y
        if abs(dx) > abs(dy):
            x += speed if dx > 0 else -speed
        else:
            y += speed if dy > 0 else -speed

    pygame.display.flip()
    pygame.time.delay(30)

