import numpy as np
import matplotlib.pyplot as plt
import rospy


# Devuelve la matriz homogénea:
# x, y, z: coordenadas del punto
# giro: radianes de giro
def hom(x,y,z,giro):
    return np.array([[np.cos(giro), -np.sin(giro), 0, x],
                     [np.sin(giro), np.cos(giro), 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

# Pasar del rango de giro [-1,1] a radianes (1 es 180º y -1 es -180º)
def giro_to_rad(giro):
    # Desnormalizar el valor de la escala a grados
    grados = giro * 180
    
    # Convertir grados a radianes
    radianes = grados / (180 / np.pi)
    
    return radianes

# Pasar de radianes a rango de giro [-1,1]
def rad_to_giro(rad):
    # Convertir radianes a grados
    grados = rad * (180 / np.pi)
    
    # Normalizar los grados en la escala de -1 a 1
    valor_normalizado = grados / 180
    return valor_normalizado

# Normaliza el ángulo de giro entre pi y -pi
def norm_pi(angle):
    while abs(angle) > np.pi:
        if angle < 0:
            angle += np.pi * 2
        else:
            angle -= np.pi * 2
            
    return angle


# Devuelve la localización (x, y, z) y el giro (radianes)
def loc(h: np.array):
    return h[0,3], h[1,3], h[2,3], np.arctan2(h[1,0], h[0,0])

# Comprobar si el punto está cerca
def esta_cerca(next_pose, final_pose, epsilon, epsilon_giro):
    return  abs(next_pose.pose.position.x - final_pose[0]) <= epsilon \
            and abs(next_pose.pose.position.y - final_pose[1]) <= epsilon \
            and abs(next_pose.pose.position.z - final_pose[2]) <= epsilon \
            and (abs(norm_pi(giro_to_rad(next_pose.pose.orientation.z)) - final_pose[3]) <= epsilon_giro \
                 or abs(norm_pi(giro_to_rad(next_pose.pose.orientation.z)) - final_pose[3] + 2*np.pi) <= epsilon_giro \
                 or abs(norm_pi(giro_to_rad(next_pose.pose.orientation.z)) - final_pose[3] - 2*np.pi) <= epsilon_giro)
                 

# Ver si se ha pasado de largo
def pasado(next_pose, final_pose, initial_pose):
    dist_ini_fin = np.sqrt((final_pose[0] - initial_pose.pose.position.x)**2 + (final_pose[1] - initial_pose.pose.position.y)**2 + (final_pose[2] - initial_pose.pose.position.z)**2)
    dist_ini_next = np.sqrt((next_pose.pose.position.x - initial_pose.pose.position.x)**2 + (next_pose.pose.position.y - initial_pose.pose.position.y)**2 + (next_pose.pose.position.z - initial_pose.pose.position.z)**2)
    return dist_ini_next > dist_ini_fin

# Calcula los puntos de una circunferencia con centro en (x_c, y_c) y radio r
def calcular_puntos_circunferencia(x_c, y_c, radio, num_puntos):
    puntos = []
    for i in range(num_puntos):
        x = x_c + radio * np.cos(2*np.pi*i/num_puntos)
        y = y_c + radio * np.sin(2*np.pi*i/num_puntos)
        puntos.append((x, y))
    return puntos








