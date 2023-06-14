import math

def calcular_angulo(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    dx = x2 - x1
    dy = y2 - y1

    radianes = math.atan2(dy, dx)
    grados = math.degrees(radianes)

    return grados

# Ejemplo de uso
punto1 = (637, 345)
punto2 = (700, 345)
angulo = calcular_angulo(punto1, punto2)

print(f"El ángulo entre los puntos es: {angulo} grados")