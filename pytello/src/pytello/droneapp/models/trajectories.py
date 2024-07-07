import rospy
import numpy as np
import pytello.droneapp.models.functions as f
import visualization_msgs.msg
import geometry_msgs.msg



''' ------------------- FUNCIONES PARA CALCULAR LAS TRAYECTORIAS ------------------- '''

def calcular_puntos_media_circunferencia(x_c, y_c, z_c, yaw_ini, radio, num_puntos, index):
    """
    Calcula los puntos de una media circunferencia en el plano XY alrededor de un centro dado.

    Parámetros:
        x_c (float): Coordenada X del centro de la circunferencia.
        y_c (float): Coordenada Y del centro de la circunferencia.
        z_c (float): Coordenada Z del centro de la circunferencia.
        yaw_ini (float): Ángulo inicial de orientación en radianes.
        radio (float): Radio de la circunferencia.
        num_puntos (int): Número de puntos a calcular en la circunferencia.
        index (int): Índice de dirección de giro (1 hacia la izquierda).

    Devuelve:
        list: Lista de puntos en formato (x, y, z, yaw).

    """
    puntos = []
    for i in range(num_puntos):
        x = x_c - radio * np.cos(index * np.pi * i /num_puntos + yaw_ini)
        y = y_c - radio * np.sin(index * np.pi * i/num_puntos + yaw_ini)
        z = z_c
        yaw = f.norm_pi(index * np.pi *i/num_puntos + yaw_ini)

        if radio < 0:
            yaw = f.norm_pi(yaw + np.pi)

        if index == 1:
            yaw = f.norm_pi(yaw - np.pi/2)
        else:
            yaw = f.norm_pi(yaw + np.pi/2)

        puntos.append((x, y, z, yaw))
    return puntos


def calcular_puntos_media_circunferencia2(x_c, y_c, z_c, yaw_ini, radio, num_puntos, index, incremento):
    """
    Calcula los puntos en una media circunferencia alrededor de un centro dado e incrementando la 
    coordenada z.

    Parámetros:
        x_c (float): Coordenada x del centro de la circunferencia.
        y_c (float): Coordenada y del centro de la circunferencia.
        z_c (float): Coordenada z del centro de la circunferencia.
        yaw_ini (float): Ángulo inicial de orientación.
        radio (float): Radio de la circunferencia.
        num_puntos (int): Número de puntos a calcular.
        index (int): Índice de giro.
        incremento (float): Incremento en la coordenada z.

    Devuelve:
        list: Lista de tuplas con las coordenadas (x, y, z, yaw) de los puntos calculados.

    """
    puntos = []
    for i in range(num_puntos):
        x = x_c - radio * np.cos(index * np.pi * i /num_puntos + yaw_ini)
        y = y_c - radio * np.sin(index * np.pi * i/num_puntos + yaw_ini)
        z = z_c + incremento * i
        yaw = f.norm_pi(index * np.pi *i/num_puntos + yaw_ini)

        if radio < 0:
            yaw = f.norm_pi(yaw + np.pi)

        if index == 1:
            yaw = f.norm_pi(yaw - np.pi/2)
        else:
            yaw = f.norm_pi(yaw + np.pi/2)

        puntos.append((x, y, z, yaw))
    return puntos


def calcular_centro_circunferencia(position, radio):
    """
    Calcula el centro de una circunferencia dado una posición y un radio.

    Parámetros:
        position: una tupla de tres elementos (x, y, z) que representa la posición actual.
        radio: un número que representa el radio de la circunferencia.

    Devuelve:
        Una tupla de tres elementos (x_c, y_c, z_c) que representa las coordenadas del centro de la 
        circunferencia.

    """
    x_c = position[0] + radio * np.cos(f.norm_pi(position[3] + np.pi/2))
    y_c = position[1] + radio * np.sin(f.norm_pi(position[3] + np.pi/2))
    z_c = position[2]
    return x_c, y_c, z_c


def calcular_puntos_ocho(position, radio, num_puntos):
    """
    Calcula los puntos de una trayectoria en forma de ocho.

    Parámetros:
        position (tuple): La posición inicial del dron en formato (x, y, z, yaw).
        radio (float): El radio de la circunferencia que forma el ocho.
        num_puntos (int): El número de puntos a calcular en cada semicírculo.

    Devuelve:
        list: Una lista de puntos que forman la trayectoria en forma de ocho.

    """
    # Calcular centro de la circunferencia
    x_c = position[0] + radio * np.cos(f.norm_pi(position[3] + np.pi/2))
    y_c = position[1] + radio * np.sin(f.norm_pi(position[3] + np.pi/2))
    z_c = position[2]

    # Calcular el otro centro del 8
    x_c2 = position[0] + 3 * radio * np.cos(f.norm_pi(position[3] + np.pi/2))
    y_c2 = position[1] + 3 * radio * np.sin(f.norm_pi(position[3] + np.pi/2))
    z_c2 = position[2]

    puntos_ocho = []

    # Calcular los puntos de la circunferencia
    puntos = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), radio, num_puntos, 1)
    puntos2 = calcular_puntos_media_circunferencia(x_c2, y_c2, z_c2, f.norm_pi(position[3] + np.pi/2), radio, num_puntos, -1)
    puntos3 = calcular_puntos_media_circunferencia(x_c2, y_c2, z_c2, f.norm_pi(position[3] + np.pi/2), -1 * radio, num_puntos, -1)
    puntos4 = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), -1 * radio, num_puntos, 1)
    punto_fin = (position[0], position[1], position[2], f.norm_pi(position[3]))

    puntos_ocho.extend(puntos)
    puntos_ocho.extend(puntos2)
    puntos_ocho.extend(puntos3)
    puntos_ocho.extend(puntos4)
    puntos_ocho.append(punto_fin)

    return puntos_ocho

def calcular_puntos_cuadrado(position, longitud, num_puntos):
    """
    Calcula los puntos de un cuadrado en función de la posición, longitud y número de puntos.

    Parámetros:
        position (tuple): La posición inicial del cuadrado en formato (x, y, z, yaw).
        longitud (float): La longitud de un lado del cuadrado.
        num_puntos (int): El número de puntos a calcular en cada lado del cuadrado.

    Devuelve:
        list: Una lista de tuplas que representan los puntos del cuadrado en formato (x, y, z, yaw).
    
    """
    puntos_cuadrado = []
    x = position[0]
    y = position[1]
    z = position[2]
    yaw = position[3]

    offset = longitud / num_puntos

    # Calcular los puntos del cuadrado
    # Lado 1
    for i in range(num_puntos):
        x = x + offset * np.cos(f.norm_pi(yaw))
        y = y + offset * np.sin(f.norm_pi(yaw))
        puntos_cuadrado.append((x, y, z, f.norm_pi(yaw)))

    # Lado 2
    yaw = f.norm_pi(yaw + np.pi/2)
    for i in range(num_puntos):
        x = x + offset * np.cos(f.norm_pi(yaw))
        y = y + offset * np.sin(f.norm_pi(yaw))
        puntos_cuadrado.append((x, y, z, f.norm_pi(yaw)))

    # Lado 3
    yaw = f.norm_pi(yaw + np.pi/2)
    for i in range(num_puntos):
        x = x + offset * np.cos(f.norm_pi(yaw))
        y = y + offset * np.sin(f.norm_pi(yaw)) 
        puntos_cuadrado.append((x, y, z, f.norm_pi(yaw)))

    # Lado 4
    yaw = f.norm_pi(yaw + np.pi/2)
    for i in range(num_puntos):
        x = x + offset * np.cos(f.norm_pi(yaw))
        y = y + offset * np.sin(f.norm_pi(yaw)) 
        puntos_cuadrado.append((x, y, z, f.norm_pi(yaw)))

    return puntos_cuadrado

def calcular_puntos_circulo(position, radio, num_puntos):
    """
    Calcula los puntos de un círculo en el espacio tridimensional.

    Parámetros:
        position (tuple): La posición actual del dron en el formato (x, y, z, yaw).
        radio (float): El radio del círculo.
        num_puntos (int): El número de puntos a calcular en la circunferencia.

    Devuelve:
        list: Una lista de puntos en el formato (x, y, z, yaw) que forman el círculo.

    """
    puntos_circulo = []
    x_c = position[0] + radio * np.cos(f.norm_pi(position[3] + np.pi/2))
    y_c = position[1] + radio * np.sin(f.norm_pi(position[3] + np.pi/2))
    z_c = position[2]

    # Calcular los puntos de la circunferencia
    puntos = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), radio, 
                                                  num_puntos, 1)
    puntos2 = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), 
                                                   -1 * radio, num_puntos, 1)
    punto_fin = (position[0], position[1], position[2], f.norm_pi(position[3]))

    puntos_circulo.extend(puntos)
    puntos_circulo.extend(puntos2)
    puntos_circulo.append(punto_fin)

    return puntos_circulo

def calcular_puntos_ovalo(position, longitud, radio, num_puntos):
    """
    Calcula los puntos de un óvalo en el espacio tridimensional.

    Parámetros:
        position (tuple): La posición inicial del óvalo en formato (x, y, z, yaw).
        longitud (float): La longitud del óvalo.
        radio (float): El radio del semicírculo del óvalo.
        num_puntos (int): El número de puntos a calcular en cada lado del óvalo.

    Devuelve:
        list: Una lista de tuplas que representan los puntos del óvalo en formato (x, y, z, yaw).

    """
    puntos_ovalo = []
    x = position[0]
    y = position[1]
    z = position[2]
    yaw = position[3]

    offset = longitud / num_puntos

    # Calcular los puntos del ovalo
    # Lado 1
    for i in range(num_puntos):
        x = x + offset * np.cos(f.norm_pi(yaw))
        y = y + offset * np.sin(f.norm_pi(yaw))
        puntos_ovalo.append((x, y, z, f.norm_pi(yaw)))

    # Semicírculo
    x_c = x + radio * np.cos(f.norm_pi(yaw + np.pi/2))
    y_c = y + radio * np.sin(f.norm_pi(yaw + np.pi/2))
    z_c = z
    puntos = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(yaw + np.pi/2), radio, 
                                                  num_puntos + 1, 1)
    puntos_ovalo.extend(puntos)

    x = x + 2 * radio * np.cos(f.norm_pi(yaw + np.pi/2))
    y = y + 2 * radio * np.sin(f.norm_pi(yaw + np.pi/2))
    z = z
    yaw = f.norm_pi(yaw + np.pi)

    # Lado 2
    for i in range(num_puntos):
        x = x + offset * np.cos(f.norm_pi(yaw))
        y = y + offset * np.sin(f.norm_pi(yaw))
        puntos_ovalo.append((x, y, z, f.norm_pi(yaw)))

    # Semicírculo
    x_c = x + radio * np.cos(f.norm_pi(yaw + np.pi/2))
    y_c = y + radio * np.sin(f.norm_pi(yaw + np.pi/2))
    z_c = z
    puntos = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(yaw + np.pi/2), radio, 
                                                  num_puntos + 1, 1)
    puntos_ovalo.extend(puntos)

    punto_fin = (position[0], position[1], position[2], f.norm_pi(position[3]))
    puntos_ovalo.append(punto_fin)

    return puntos_ovalo


def calcular_puntos_espiral(position, radio, incremento_altura, num_puntos):
    """
    Calcula los puntos de una trayectoria en forma de espiral.

    Args:
        position (tuple): La posición inicial del dron en forma de tupla (x, y, z, yaw).
        radio (float): El radio de la espiral.
        incremento_altura (float): El incremento de altura.
        num_puntos (int): El número de puntos a calcular en cada vuelta de la espiral.

    Returns:
        list: Una lista de tuplas que representan los puntos de la trayectoria en forma de espiral.
    """
    puntos_espiral = []

    x_c = position[0] + radio * np.cos(f.norm_pi(position[3] + np.pi/2))
    y_c = position[1] + radio * np.sin(f.norm_pi(position[3] + np.pi/2))
    z_c = position[2]

    puntos = calcular_puntos_media_circunferencia2(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), radio, 
                                                   num_puntos, 1, incremento_altura)
    puntos_espiral.extend(puntos)

    z_c = z_c + incremento_altura * num_puntos

    puntos2 = calcular_puntos_media_circunferencia2(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), 
                                                    -1 * radio, num_puntos, 1, incremento_altura)
    puntos_espiral.extend(puntos2)

    z = position[2] + incremento_altura * num_puntos * 2
    punto_fin = (position[0], position[1], z, f.norm_pi(position[3]))
    puntos_espiral.append(punto_fin)

    # Repetir el proceso
    z_c = z_c + incremento_altura * num_puntos
    puntos = calcular_puntos_media_circunferencia2(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), radio, 
                                                   num_puntos, 1, incremento_altura)
    puntos_espiral.extend(puntos)

    z_c = z_c + incremento_altura * num_puntos
    puntos2 = calcular_puntos_media_circunferencia2(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), 
                                                    -1 * radio, num_puntos, 1, incremento_altura)
    puntos_espiral.extend(puntos2)

    z = position[2] + incremento_altura * num_puntos * 4
    punto_fin = (position[0], position[1], z, f.norm_pi(position[3]))
    puntos_espiral.append(punto_fin)

    return puntos_espiral



''' ------------------- FUNCIONES PARA DIBUJAR LAS TRAYECTORIAS ------------------- '''

def draw_trajectory_8(position, radio, n_puntos):
    markers = visualization_msgs.msg.Marker()
    markers.header.frame_id = "odom"
    markers.header.stamp = rospy.Time.now()
    markers.ns = "points_and_lines"
    markers.id = 0
    markers.type = visualization_msgs.msg.Marker.POINTS
    markers.action = visualization_msgs.msg.Marker.ADD
    markers.scale.x = 0.03
    markers.scale.y = 0.03
    markers.color.a = 1.0
    markers.color.r = 1.0
    markers.color.g = 1.0
    markers.color.b = 0.0
    
    puntos_ocho = calcular_puntos_ocho([position.pose.position.x, position.pose.position.y, 
                                        position.pose.position.z, f.yaw(position.pose.orientation)], 
                                        radio, n_puntos)
    for punto in puntos_ocho:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        markers.points.append(p)

    return markers


def draw_trajectory_cuadrado(position, longitud, n_puntos):
    markers = visualization_msgs.msg.Marker()
    markers.header.frame_id = "odom"
    markers.header.stamp = rospy.Time.now()
    markers.ns = "points_and_lines"
    markers.id = 0
    markers.type = visualization_msgs.msg.Marker.POINTS
    markers.action = visualization_msgs.msg.Marker.ADD
    markers.scale.x = 0.03
    markers.scale.y = 0.03
    markers.color.a = 1.0
    markers.color.r = 1.0
    markers.color.g = 1.0
    markers.color.b = 0.0
    
    puntos_cuadrado = calcular_puntos_cuadrado([position.pose.position.x, position.pose.position.y, 
                                                position.pose.position.z, f.yaw(position.pose.orientation)], 
                                                longitud, n_puntos)
    for punto in puntos_cuadrado:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        markers.points.append(p)

    return markers

def draw_trajectory_circulo(position, radio, n_puntos):
    markers = visualization_msgs.msg.Marker()
    markers.header.frame_id = "odom"
    markers.header.stamp = rospy.Time.now()
    markers.ns = "points_and_lines"
    markers.id = 0
    markers.type = visualization_msgs.msg.Marker.POINTS
    markers.action = visualization_msgs.msg.Marker.ADD
    markers.scale.x = 0.03
    markers.scale.y = 0.03
    markers.color.a = 1.0
    markers.color.r = 1.0
    markers.color.g = 1.0
    markers.color.b = 0.0
    
    puntos_circulo = calcular_puntos_circulo([position.pose.position.x, position.pose.position.y, 
                                              position.pose.position.z, f.yaw(position.pose.orientation)], 
                                              radio, n_puntos)
    for punto in puntos_circulo:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        markers.points.append(p)

    return markers


def draw_trajectory_ovalo(position, longitud, radio, n_puntos):
    markers = visualization_msgs.msg.Marker()
    markers.header.frame_id = "odom"
    markers.header.stamp = rospy.Time.now()
    markers.ns = "points_and_lines"
    markers.id = 0
    markers.type = visualization_msgs.msg.Marker.POINTS
    markers.action = visualization_msgs.msg.Marker.ADD
    markers.scale.x = 0.03
    markers.scale.y = 0.03
    markers.color.a = 1.0
    markers.color.r = 1.0
    markers.color.g = 1.0
    markers.color.b = 0.0
    
    puntos_ovalo = calcular_puntos_ovalo([position.pose.position.x, position.pose.position.y, 
                                          position.pose.position.z, f.yaw(position.pose.orientation)], 
                                          longitud, radio, n_puntos)
    for punto in puntos_ovalo:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        markers.points.append(p)

    return markers


def draw_trajectory_espiral(position, radio, incremento_altura, n_puntos):
    markers = visualization_msgs.msg.Marker()
    markers.header.frame_id = "odom"
    markers.header.stamp = rospy.Time.now()
    markers.ns = "points_and_lines"
    markers.id = 0
    markers.type = visualization_msgs.msg.Marker.POINTS
    markers.action = visualization_msgs.msg.Marker.ADD
    markers.scale.x = 0.03
    markers.scale.y = 0.03
    markers.color.a = 1.0
    markers.color.r = 1.0
    markers.color.g = 1.0
    markers.color.b = 0.0
    
    puntos_espiral = calcular_puntos_espiral([position.pose.position.x, position.pose.position.y, 
                                              position.pose.position.z, f.yaw(position.pose.orientation)], 
                                              radio, incremento_altura, n_puntos)
    for punto in puntos_espiral:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        markers.points.append(p)

    return markers

