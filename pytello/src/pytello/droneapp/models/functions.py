import numpy as np
import matplotlib.pyplot as plt
import rospy
from math import atan2, asin

''' -------------- Funciones auxiliares para el control del dron -------------- '''

def hom(x,y,z,giro):
    """
    Calcula la matriz de transformación homogénea para una posición y orientación dadas.

    Parámetros:
        x (float): Coordenada x de la posición.
        y (float): Coordenada y de la posición.
        z (float): Coordenada z de la posición.
        giro (float): Ángulo de rotación en radianes.

    Devuelve:
        np.array: Matriz de transformación homogénea.

    """
    return np.array([[np.cos(giro), -np.sin(giro), 0, x],
                     [np.sin(giro), np.cos(giro), 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])


def euler_from_quaternion(x, y, z, w):
    """
    Convierte un cuaternión en ángulos de Euler.

    Parámetros:
        x (float): Componente x del cuaternión.
        y (float): Componente y del cuaternión.
        z (float): Componente z del cuaternión.
        w (float): Componente w del cuaternión.

    Devuelve:
        tuple: Una tupla que contiene los ángulos de Euler en los ejes roll, pitch y yaw.

    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z


def yaw(orientation):
    """
    Calcula el ángulo yaw a partir de una orientación en formato cuaternión.

    Parámetros:
        orientation: Orientación en formato cuaternión (x, y, z, w).

    Devuelve:
        El ángulo yaw en radianes.

    """
    return euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)[2]


def norm_pi(angle):
    """
    Normaliza un ángulo al rango [-pi, pi].

    Parámetros:
        angle (float): El ángulo a normalizar.

    Devuelve:
        float: El ángulo normalizado.

    """
    while abs(angle) > np.pi:
        if angle < 0:
            angle += np.pi * 2
        else:
            angle -= np.pi * 2
            
    return angle


def loc(h: np.array):
    """
    Obtiene las coordenadas x, y, z y el ángulo de orientación de una matriz de transformación 
    homogénea.

    Parámetros:
        h (np.array): Matriz de transformación homogénea de tamaño 4x4.

    Devuelve:
        tuple: Una tupla que contiene las coordenadas x, y, z y el ángulo de orientación.

    """
    return h[0,3], h[1,3], h[2,3], np.arctan2(h[1,0], h[0,0])


def esta_cerca(next_pose, final_pose, epsilon, epsilon_giro):
    """
    Comprueba si la siguiente posición del dron está cerca de la posición final.

    Parámetros:
        next_pose: La posición del dron.
        final_pose: La posición final deseada, representada como una lista [x, y, z, yaw].
        epsilon: El margen de error permitido para las coordenadas x, y, z.
        epsilon_giro: El margen de error permitido para el ángulo de giro.

    Devuelve:
        True si la siguiente posición está cerca de la posición final.
        False en caso contrario.

    """
    return  abs(next_pose.pose.position.x - final_pose[0]) <= epsilon \
            and abs(next_pose.pose.position.y - final_pose[1]) <= epsilon \
            and abs(next_pose.pose.position.z - final_pose[2]) <= epsilon \
            and (abs(norm_pi(yaw(next_pose.pose.orientation)) - final_pose[3]) <= epsilon_giro)


def diff_orientation(radians1, radians2):
    """
    Calcula la diferencia absoluta entre dos ángulos en radianes.

    Parámetros:
        radians1 (float): El primer ángulo en radianes.
        radians2 (float): El segundo ángulo en radianes.

    Devuelve:
        float: La diferencia absoluta entre los dos ángulos.

    """
    return abs(radians2 - radians1)


def pasado(next_pose, final_pose, initial_pose):
    """
    Comprueba si la distancia entre la posición inicial y la siguiente posición es mayor que la 
    distancia entre la posición inicial y la posición final.

    Parámetros:
        next_pose: La siguiente posición del dron.
        final_pose: La posición final del dron.
        initial_pose: La posición inicial del dron.

    Devuelve:
        True si la distancia entre la posición inicial y la siguiente posición es mayor que la 
        distancia entre la posición inicial y la posición final. False en caso contrario.

    """
    dist_ini_fin = np.sqrt((final_pose[0] - initial_pose.pose.position.x)**2 + 
                           (final_pose[1] - initial_pose.pose.position.y)**2 + 
                           (final_pose[2] - initial_pose.pose.position.z)**2)
    dist_ini_next = np.sqrt((next_pose.pose.position.x - initial_pose.pose.position.x)**2 + 
                            (next_pose.pose.position.y - initial_pose.pose.position.y)**2 + 
                            (next_pose.pose.position.z - initial_pose.pose.position.z)**2)
    return dist_ini_next > dist_ini_fin


def distancia_menor(next_pose, final_pose, min_dist): 
    """
    Calcula la distancia entre dos posiciones y verifica si es menor que la distancia mínima 
    encontrada.

    Parámetros:
        next_pose: La pose del dron.
        final_pose: La pose final deseada.
        min_dist: La distancia mínima.

    Devuelve:
        Un booleano indicando si la distancia es menor que la distancia mínima.
        La distancia mínima actualizada.

    """
    dist = np.sqrt((next_pose.pose.position.x - final_pose[0])**2 + 
                   (next_pose.pose.position.y - final_pose[1])**2 + 
                   (next_pose.pose.position.z - final_pose[2])**2) 

    if dist < min_dist:
        min_dist = dist
    
    if min_dist + 0.2 < dist:
        min_dist = dist
        return True, min_dist
    else:
        return False, min_dist
    

def dist_aumenta(dist_ant, next_pose, final_pose):
    """
    Calcula la distancia entre la posición actual y la posición final y compara con la distancia 
    anterior.

    Parámetros:
        dist_ant: La distancia anterior.
        next_pose: La siguiente posición.
        final_pose: La posición final.

    Devuelve:
        True si la distancia actual es mayor que la distancia anterior, False en caso contrario.

    """
    dist_act = np.sqrt((next_pose.pose.position.x - final_pose[0])**2 + 
                       (next_pose.pose.position.y - final_pose[1])**2 + 
                       (next_pose.pose.position.z - final_pose[2])**2)
    return dist_ant < dist_act


def ha_girado(next_pose, final_pose, epsilon):
    """
    Comprueba si el dron ha girado hasta alcanzar la posición final deseada.

    Parámetros:
        next_pose (Pose): La siguiente posición del dron.
        final_pose (list): La posición final deseada, en formato [x, y, z, yaw].
        epsilon (float): El margen de error permitido para el ángulo de giro.

    Devuelve:
        True si el dron ha girado hasta alcanzar la posición final deseada, False en caso 
        contrario.

    """
    return (abs(norm_pi(yaw(next_pose.pose.orientation)) - final_pose[3]) <= epsilon)


def calcularVelocidades(WxR, WxG, kp, ka, kb):
    """
    Calcula las velocidades lineal y angular del dron en función de su posición y orientación
    con respecto a un punto de destino.

    Parámetros:
        WxR (PoseStamped): Posición y orientación actual del dron.
        WxG (list): Coordenadas x, y, z y orientación del punto de destino.
        kp (float): Coeficiente de proporcionalidad para el control de la velocidad lineal.
        ka (float): Coeficiente de proporcionalidad para el control del ángulo de giro.
        kb (float): Coeficiente de proporcionalidad para el control del ángulo de giro.

    Devuelve:
        Una tupla que contiene las velocidades lineal y angular del dron.

    """
    wTr = hom(WxR.pose.position.x, WxR.pose.position.y, WxR.pose.position.z, norm_pi(yaw(WxR.pose.orientation)))
    wTg = hom(WxG[0], WxG[1], WxG[2], WxG[3])
    gTw = np.linalg.inv(wTg)

    # wTr = wTg * gTr --> gTr = gTw * wTr
    gTr = np.dot(gTw, wTr) 

    # Posición y giro del dron respecto al punto de destino
    GxR = loc(gTr)

    # Calcular distancia desde el dron al punto de destino
    p = np.sqrt(GxR[0]**2 + GxR[1]**2)

    # Calcular los grados donde esta el eje de destino
    b = norm_pi(atan2(GxR[1], GxR[0]) + np.pi)

    # Calcular los grados de diferencia entre la orientación del dron y el eje de destino
    a = norm_pi(b - GxR[3])

    rospy.loginfo({'atan2': round(atan2(GxR[1], GxR[0]), 2)})

    rospy.loginfo({'p': round(p, 2), 'a': round(a, 2), 'b': round(b, 2)})

    rospy.loginfo({'x': round(GxR[0], 2), 'y': round(GxR[1], 2), 'theta': round(GxR[3], 2)})

    # Ajustar los valores de los coeficientes
    if distancia(WxR, WxG) > 0.1:
        ka = 1
        kb = -0.4
        kp = 0.6
    else:
        ka = 0.4
        kb = -1
        kp = 0.5

    # Calcular las velocidades
    v = kp * p
    w = ka * a + kb * b


    # Pasar a porcentajes
    w = w * 100 / 1.30
    v = v * 100 / 1.20

    # Limitar velocidades
    if w > 30:
        w = 30
    elif w < -30:
        w = -30

    if v > 8:
        v = 8
    elif v <= -8:
        v = -8

    return v, w


def distancia(p1, p2):
    """
    Calcula la distancia euclídea entre dos puntos en un espacio tridimensional.

    Parámetros:
        p1 (objeto): El primer punto en forma de objeto con atributos de posición x, y, z.
        p2 (lista): El segundo punto en forma de lista con coordenadas x, y, z.

    Devuelve:
        float: La distancia euclídea entre los dos puntos.

    """
    return np.sqrt((p1.pose.position.x - p2[0])**2 + (p1.pose.position.y - p2[1])**2 + 
                   (p1.pose.position.z - p2[2])**2)


def grafica(file, file2):
    """
    Grafica la trayectoria de dos archivos de coordenadas en un gráfico 3D.

    Parámetros:
        file (str): Ruta del primer archivo de coordenadas.
        file2 (str): Ruta del segundo archivo de coordenadas.

    """
    x_coords = []
    y_coords = []
    z_coords = []
    yaw_coords = []
    coords = []

    x_coords2 = []
    y_coords2 = []
    z_coords2 = []
    yaw_coords2 = []
    coords2 = []

    with open(file, 'r') as f:
        for line in f:
            line = line.split(' ')
            x_coords.append(float(line[1]))
            y_coords.append(float(line[2]))
            z_coords.append(float(line[3]))
            yaw_coords.append(float(line[6]))
            coords.append((float(line[1]), float(line[2]), float(line[3]), float(line[6])))

    with open(file2, 'r') as f:
        for line in f:
            line = line.split(' ')
            x_coords2.append(float(line[1]))
            y_coords2.append(float(line[2]))
            z_coords2.append(float(line[3]))
            yaw_coords2.append(float(line[6]))
            coords2.append((float(line[1]), float(line[2]), float(line[3]), float(line[6])))


    # Graficar trayectoria
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_coords, y_coords, z_coords)
    ax.plot(x_coords2, y_coords2, z_coords2)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_zlim(0.4, 1.5)
    ax.legend(['Goal', 'Real'])
    plt.show()





