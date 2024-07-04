import numpy as np
import matplotlib.pyplot as plt
import rospy
from math import atan2, asin


# Devuelve la matriz homogénea:
# x, y, z: coordenadas del punto
# giro: radianes de giro
def hom(x,y,z,giro):
    return np.array([[np.cos(giro), -np.sin(giro), 0, x],
                     [np.sin(giro), np.cos(giro), 0, y],
                     [0, 0, 1, z],
                     [0, 0, 0, 1]])

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
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
     
        return roll_x, pitch_y, yaw_z # in radians


# Devuelve el ángulo de giro en radianes
def yaw(orientation):
    return euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)[2]


# Normaliza el ángulo de giro entre pi y -pi
def norm_pi(angle):
    while abs(angle) > np.pi:
        if angle < 0:
            angle += np.pi * 2
        else:
            angle -= np.pi * 2
            
    return angle


# Devuelve la localización (x, y, z) y orientación (radianes)
def loc(h: np.array):
    return h[0,3], h[1,3], h[2,3], np.arctan2(h[1,0], h[0,0])


# Comprobar si el punto está cerca
def esta_cerca(next_pose, final_pose, epsilon, epsilon_giro):
    return  abs(next_pose.pose.position.x - final_pose[0]) <= epsilon \
            and abs(next_pose.pose.position.y - final_pose[1]) <= epsilon \
            and abs(next_pose.pose.position.z - final_pose[2]) <= epsilon \
            and (abs(norm_pi(yaw(next_pose.pose.orientation)) - final_pose[3]) <= epsilon_giro)


# Diferencia entre dos orientaciones
def diff_orientation(radians1, radians2):
    return abs(radians2 - radians1)


# Ver si se ha pasado de largo
def pasado(next_pose, final_pose, initial_pose):
    dist_ini_fin = np.sqrt((final_pose[0] - initial_pose.pose.position.x)**2 + (final_pose[1] - initial_pose.pose.position.y)**2 + (final_pose[2] - initial_pose.pose.position.z)**2)
    dist_ini_next = np.sqrt((next_pose.pose.position.x - initial_pose.pose.position.x)**2 + (next_pose.pose.position.y - initial_pose.pose.position.y)**2 + (next_pose.pose.position.z - initial_pose.pose.position.z)**2)
    return dist_ini_next > dist_ini_fin


# Calcular si la distancia entre dos puntos es menor que la distancia mínima
def distancia_menor(next_pose, final_pose, min_dist): 
    dist = np.sqrt((next_pose.pose.position.x - final_pose[0])**2 + (next_pose.pose.position.y - final_pose[1])**2 + (next_pose.pose.position.z - final_pose[2])**2) 
    rospy.loginfo({'dist': dist, 'min_dist': min_dist, 'next_pose': next_pose.pose.position, 'final_pose': final_pose})

    if dist < min_dist:
        min_dist = dist
    
    if min_dist + 0.2 < dist:
        min_dist = dist
        return True, min_dist
    else:
        return False, min_dist
    

# Calcular si la distancia anterior es mayor que la distancia actual
def dist_aumenta(dist_ant, next_pose, final_pose):
    dist_act = np.sqrt((next_pose.pose.position.x - final_pose[0])**2 + (next_pose.pose.position.y - final_pose[1])**2 + (next_pose.pose.position.z - final_pose[2])**2)
    rospy.loginfo({'dist_ant': dist_ant, 'dist_act': dist_act})
    return dist_ant < dist_act


# Calcular si ha girado
def ha_girado(next_pose, final_pose, epsilon):
    return (abs(norm_pi(yaw(next_pose.pose.orientation)) - final_pose[3]) <= epsilon)


# Función para calcular las velocidades con el controlador proporcional
def calcularVelocidades(WxR, WxG, kp, ka, kb):
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

    return v, w, GxR, a, -b


# Calcula la distancia entre dos puntos
def distancia(p1, p2):
    return np.sqrt((p1.pose.position.x - p2[0])**2 + (p1.pose.position.y - p2[1])**2 + (p1.pose.position.z - p2[2])**2)


def grafica(file, file2):
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

        # Graficar yaw
        # fig = plt.figure()
        # plt.plot(yaw_coords)
        # plt.plot(yaw_coords2)
        # plt.ylabel('Yaw')
        # plt.legend(['Goal', 'Real'])
        # plt.show()

        # Medir error de posición y graficar
        # error = []
        # for point in coords:
        #     # Encontrar qué punto de coords2 es el más cercano a cada punto de coords
        #     punto = None
        #     dist = 10000
        #     for point2 in coords2:
        #         d = np.sqrt((point2[0] - point[0])**2 + (point2[1] - point[1])**2 + (point2[2] - point[2])**2)
        #         diff_yaw = norm_pi(point2[3] - point[3])
        #         if d < dist and diff_yaw < 1:
        #             dist = d
        #             punto = point2
        #     error.append(dist)
        #     rospy.loginfo({'action': 'grafica', 'dist': dist, 'point': point, 'punto': punto})

        # # Graficar error
        # fig = plt.figure()
        # plt.plot(error)
        # plt.ylabel('Error')
        # plt.show()





