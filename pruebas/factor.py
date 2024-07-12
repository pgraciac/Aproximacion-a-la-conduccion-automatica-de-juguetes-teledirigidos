actual_distance = int(input("Actual distance: "))
orientation_robot_path = int(input("Orientation robot path: "))
angulo_closest_point = int(input("Angulo recta: "))
orientation_robot_path = orientation_robot_path + 360 if orientation_robot_path < 0 else orientation_robot_path
angulo_closest_point = angulo_closest_point + 360 if angulo_closest_point < 0 else angulo_closest_point
print(orientation_robot_path, angulo_closest_point)
factor = 1- (actual_distance / (1800/3))
error_angle = angulo_closest_point - orientation_robot_path
if abs(error_angle) > 180:
    orientation_robot_path = (orientation_robot_path + 180) % 360 - 180
    angulo_closest_point = (angulo_closest_point + 180) % 360 - 180
good_orientation = (1-factor)*orientation_robot_path+factor*angulo_closest_point
good_orientation = (good_orientation + 180) % 360 - 180
print(good_orientation)