import math

def calculate_angle(p1, p2):
    # Calcula el vector entre los dos puntos
    delta_x = p2[0] - p1[0]
    delta_y = p2[1] - p1[1]

    # Recuerda que en la función atan2, el primer argumento es la ordenada y el segundo la abscisa
    angle_rad = math.atan2(delta_y, delta_x)

    # Convierte el ángulo a grados
    angle_deg = math.degrees(angle_rad)

    # Ajusta el ángulo para que esté en el rango [0, 360)
    if angle_deg < 0:
        angle_deg += 360

    return angle_deg

# Prueba de la función
p1 = (50,50)  # Origen
p2 = (55, 50)  # Por ejemplo, un punto en la parte inferior derecha

print(f"The angle between {p1} and {p2} is {calculate_angle(p1, p2)} degrees.")