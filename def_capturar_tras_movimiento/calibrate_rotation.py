import math
import random
import gpio
import vision

def calibrate_rotation():
    actual_orientation = gpio.actual_orientation
    initial_position = vision.point
    error = float('inf')
    
    # Lista para almacenar giros por segundo
    giros_por_segundo = []
    
    # Umbral de error aceptable en grados
    umbral_error = 5.0  # Ajusta según tus necesidades
    
    while error > umbral_error:
        seconds = random.uniform(0.05, 0.2)
        
        gpio.move("right", seconds)
        gpio.move("up", 0.15)
        
        final_position = vision.point
        delta_x_robot = final_position[0] - initial_position[0]
        delta_y_robot = final_position[1] - initial_position[1]
        robot_orientation = math.atan2(delta_x_robot, delta_y_robot)
        giro_real = abs(actual_orientation - math.degrees(robot_orientation))
        
        # Calcular giro por segundo
        giro_actual_por_segundo = giro_real / seconds
        giros_por_segundo.append(giro_actual_por_segundo)
        
        # Estimar el siguiente giro basado en el promedio de giros anteriores
        giro_estimado = sum(giros_por_segundo) / len(giros_por_segundo) * seconds
        
        # Calcula el error entre la estimación y el giro real
        error = abs(giro_estimado - giro_real)
        
        # Actualiza la orientación y posición para la siguiente iteración
        actual_orientation -= giro_real
        initial_position = final_position



        

if __name__ == '__main__':
    vision.init_vision()
    calibrate_rotation()
    while vision.listener.running():
        vision.root.update()

    vision.finish_vision()