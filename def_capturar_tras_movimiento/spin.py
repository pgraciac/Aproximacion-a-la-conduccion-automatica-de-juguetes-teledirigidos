import gpio
import vision
import matplotlib.pyplot as plt
import numpy as np
import math

# def average_orientation(path, start_index, num_points, log = False):
#   total_orientation = 0
#   for i in range(start_index, start_index + num_points):
#     if i + 1 >= len(path):
#       break
#     if log == True:
#        print(path[i])
#        print(gpio.orientation_two_points(path[i], path[i+1]))
#     angulo = (gpio.orientation_two_points(path[i], path[i+1]))
#     angulo = angulo + 360 if angulo < 0 else angulo
#     angulo = (angulo - 180) / 360
#     total_orientation += angulo
#   return sum(total_orientation) / len(total_orientation)

def average_orientation(path, start_index, num_points, log=False):
    total_sin = 0
    total_cos = 0
    count = 0

    for i in range(start_index, start_index + num_points):
        if i + 1 >= len(path):
            break
        if log:
            print(path[i])
            print(gpio.orientation_two_points(path[i], path[i+1]))
        
        angle_deg = gpio.orientation_two_points(path[i], path[i+1])
        angle_deg = angle_deg + 360 if angle_deg < 0 else angle_deg
        angle_rad = math.radians(angle_deg)

        total_sin += math.sin(angle_rad)
        total_cos += math.cos(angle_rad)
        count += 1

    if count == 0:
        return None  # O devuelve un valor predeterminado si es más adecuado

    average_sin = total_sin / count
    average_cos = total_cos / count

    mean_angle_rad = math.atan2(average_sin, average_cos)
    mean_angle_deg = math.degrees(mean_angle_rad)  # Omitir si prefieres radianes
    return mean_angle_deg

path = [(291, 469), (291, 468), (291, 467), (291, 466), (290, 466), (290, 464), (289, 463), (289, 461), (289, 460), (289, 459), (288, 457), (288, 456), (287, 455), (287, 453), (287, 452), (286, 451), (285, 449), (285, 447), (285, 446), (285, 444), (284, 443), (283, 442), (283, 440), (283, 439), (282, 438), (282, 436), (281, 435), (281, 434), (281, 433), (281, 432), (281, 430), (280, 429), (280, 428), (280, 427), (280, 425), (280, 424), (280, 423), (280, 421), (280, 420), (280, 419), (280, 418), (280, 415), (280, 412), (280, 408), (280, 
406), (281, 404), (281, 402), (281, 400), (281, 398), (282, 396), (282, 395), (282, 394), (282, 392), (283, 390), (283, 389), (283, 387), (283, 385), (284, 381), (285, 379), (286, 378), (286, 377), (287, 375), (287, 374), (288, 373), (288, 371), (289, 370), (289, 368), (290, 367), (290, 365), (290, 363), (291, 361), (291, 360), (291, 359), (292, 358), (292, 356), (293, 355), (293, 353), (293, 352), (294, 351), (294, 350), (295, 348), (295, 347), (295, 346), (296, 344), (296, 342), (297, 341), (297, 339), (297, 337), (297, 333), (294, 325), (290, 315), (288, 308), (287, 305), (286, 303), (285, 301), (285, 300), (284, 299), (283, 295), 
(282, 294), (282, 293), (282, 291), (282, 288), (281, 286), (281, 284), (281, 282), (281, 280), (280, 278), (280, 276), (280, 273), (280, 271), (280, 269), (280, 268), (280, 266), (280, 264), (280, 262), (280, 259), (280, 257), (280, 255), (280, 253), (280, 251), (280, 248), (280, 247), (281, 245), (281, 244), (281, 242), (281, 239), (281, 238), (281, 237), (281, 236), (281, 233), (281, 232), (281, 230), (281, 227), (281, 225), (281, 223), (281, 221), (281, 218), (281, 216), (281, 215), (281, 213), (281, 211), (281, 209), (281, 208), (281, 207), (281, 206), (281, 204), (281, 203), (281, 202), (280, 200), (280, 199), (280, 198), (280, 
196), (280, 194), (279, 192), (279, 190), (278, 189), (278, 186), (277, 184), (277, 182), (275, 180), (275, 178), (274, 175), (272, 171), (271, 168), (271, 165), (269, 162), (268, 159), (268, 156), (267, 152), (266, 150), (265, 148), (265, 147), (264, 144), (264, 143), (263, 141), (263, 140), (262, 137), (261, 133), (260, 130), (258, 126), (256, 121), (254, 115), (253, 113), (251, 110), (250, 107), (248, 104), (248, 103), (247, 102), (247, 101), (247, 100), (247, 99), (247, 98), (248, 98), (250, 98), (251, 98), (252, 98), (254, 98), (257, 98), (261, 98), (267, 98), (271, 98), (277, 98), (282, 99), (288, 101), (294, 102), (298, 103), (302, 105), (308, 107), (314, 109), (320, 111), (327, 113), (333, 115), (338, 116), (344, 118), (350, 120), 
(356, 122), (361, 124), (367, 126), (373, 128), (377, 130), (382, 132), (386, 133), (392, 135), (398, 137), (404, 139), (410, 141), (415, 143), (421, 145), (427, 147), (431, 148), (437, 149), (442, 149), (447, 150), (452, 150), (459, 151), (464, 151), (470, 151), (475, 151), (481, 151), (488, 152), (496, 152), (504, 153), (512, 155), (520, 156), (528, 156), (535, 157), (543, 158), (551, 160), (558, 161), (567, 162), (574, 164), (581, 166), (586, 168), (593, 169), (599, 169), (604, 170), (610, 171), (616, 173), (621, 174), (627, 175), (633, 176), (638, 177), (641, 178), (644, 178), (648, 179), (651, 179), (653, 179), (656, 180), (657, 
182), (659, 183), (663, 184), (665, 185), (667, 185), (669, 187), (671, 187), (673, 188), (674, 189), (675, 190), (677, 190), (678, 191), (680, 191), (682, 192), (684, 192), (686, 192), (688, 192), (691, 193), (693, 194), (696, 195), (700, 195), (702, 196), (704, 197), (705, 197), (708, 198), (709, 199), (710, 200), (712, 200), (713, 200), (713, 201), (714, 201), (715, 201), (716, 202), (716, 203), (717, 203)]
path_orientation = average_orientation(path, 0, 5)
path_spins = []
for i in range(0, len(path) - 5):
    actual_path_orientation = average_orientation(path, i, 5)
    error = (path_orientation - actual_path_orientation + 180) % 360 - 180
    if abs(error) > 60:
        print(average_orientation(path, i + 1, 5, True))
        print(f"actual orientation: {actual_path_orientation}")
        print(f"path orientation: {path_orientation}")
        print(error)
        print(path[i+1], "\n------------------------------------\n")
        path_spins.append(path[i+1])
        path_orientation = average_orientation(path, i + 1, 5)

x_path, y_path = zip(*path)

# Extrae las coordenadas x e y de path_spins en listas separadas
x_spins, y_spins = zip(*path_spins)

# Crea la gráfica
plt.figure(figsize=(8, 6))
plt.plot(x_path, y_path, 'r-', label='Path')
plt.scatter(x_path, y_path, c='r', label='Path', marker='o')
plt.scatter(x_spins, y_spins, c='g', label='Spins', marker='o')

# Etiqueta los ejes
plt.xlabel('Coordenada X')
plt.ylabel('Coordenada Y')

# Añade una leyenda
plt.legend()

# Muestra la gráfica
plt.show()