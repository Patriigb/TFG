import rospy
import numpy as np
import pytello.droneapp.models.functions as f
import visualization_msgs.msg
import geometry_msgs.msg

# Calcula los puntos de una circunferencia con centro en (x_c, y_c, z_c) y radio r, 
# si index=1 se dibuja a la derecha, si index=-1 se dibuja a la izquierda
def calcular_puntos_media_circunferencia(x_c, y_c, z_c, yaw_ini, radio, num_puntos, index):
    puntos = []
    for i in range(num_puntos):
        # rospy.loginfo({'i': i, 'index': index})
        # rospy.loginfo({'x_c': x_c, 'y_c': y_c, 'z_c': z_c})
        x = x_c - radio * np.cos(index * np.pi * i /num_puntos + yaw_ini)
        y = y_c - radio * np.sin(index * np.pi * i/num_puntos + yaw_ini)
        z = z_c
        yaw = f.norm_pi(index * np.pi *i/num_puntos + yaw_ini)

        if radio < 0:
            yaw = f.norm_pi(yaw + np.pi)

        if index == 1: # Gira hacia la izquierda
            yaw = f.norm_pi(yaw - np.pi/2)
        else:
            yaw = f.norm_pi(yaw + np.pi/2)

        # rospy.loginfo({'x': x, 'y': y, 'z': z, 'yaw': yaw})

        puntos.append((x, y, z, yaw))
    return puntos

def calcular_puntos_media_circunferencia2(x_c, y_c, z_c, yaw_ini, radio, num_puntos, index, incremento):
    puntos = []
    for i in range(num_puntos):
        # rospy.loginfo({'i': i, 'index': index})
        # rospy.loginfo({'x_c': x_c, 'y_c': y_c, 'z_c': z_c})
        x = x_c - radio * np.cos(index * np.pi * i /num_puntos + yaw_ini)
        y = y_c - radio * np.sin(index * np.pi * i/num_puntos + yaw_ini)
        z = z_c + incremento * i
        yaw = f.norm_pi(index * np.pi *i/num_puntos + yaw_ini)

        if radio < 0:
            yaw = f.norm_pi(yaw + np.pi)

        if index == 1: # Gira hacia la izquierda
            yaw = f.norm_pi(yaw - np.pi/2)
        else:
            yaw = f.norm_pi(yaw + np.pi/2)

        # rospy.loginfo({'x': x, 'y': y, 'z': z, 'yaw': yaw})

        puntos.append((x, y, z, yaw))
    return puntos

def calcular_centro_circunferencia(position, radio):
    x_c = position[0] + radio * np.cos(f.norm_pi(position[3] + np.pi/2))
    y_c = position[1] + radio * np.sin(f.norm_pi(position[3] + np.pi/2))
    z_c = position[2]
    return x_c, y_c, z_c

# Calcular los puntos del ocho entero
def calcular_puntos_ocho(position, radio, num_puntos):
    rospy.loginfo({'position': position, 'radio': radio, 'num_puntos': num_puntos})

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
    # punto_ini = (position[0], position[1], position[2], norm_pi(position[3] - np.pi/2))
    puntos = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), radio, num_puntos, 1)
    # rospy.loginfo({'puntos': puntos})
    puntos2 = calcular_puntos_media_circunferencia(x_c2, y_c2, z_c2, f.norm_pi(position[3] + np.pi/2), radio, num_puntos, -1)
    # rospy.loginfo({'puntos2': puntos2})
    puntos3 = calcular_puntos_media_circunferencia(x_c2, y_c2, z_c2, f.norm_pi(position[3] + np.pi/2), -1 * radio, num_puntos, -1)
    # rospy.loginfo({'puntos3': puntos3})
    puntos4 = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), -1 * radio, num_puntos, 1)
    # rospy.loginfo({'puntos4': puntos4})
    punto_fin = (position[0], position[1], position[2], f.norm_pi(position[3]))

    # puntos_ocho.append(punto_ini)
    puntos_ocho.extend(puntos)
    puntos_ocho.extend(puntos2)
    puntos_ocho.extend(puntos3)
    puntos_ocho.extend(puntos4)
    puntos_ocho.append(punto_fin)

    return puntos_ocho

def calcular_puntos_cuadrado(position, longitud, num_puntos):
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

    # print(puntos_cuadrado)

    return puntos_cuadrado

def calcular_puntos_circulo(position, radio, num_puntos):
    puntos_circulo = []
    x_c = position[0] + radio * np.cos(f.norm_pi(position[3] + np.pi/2))
    y_c = position[1] + radio * np.sin(f.norm_pi(position[3] + np.pi/2))
    z_c = position[2]

    # Calcular los puntos de la circunferencia
    puntos = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), radio, num_puntos, 1)
    puntos2 = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), -1 * radio, num_puntos, 1)
    punto_fin = (position[0], position[1], position[2], f.norm_pi(position[3]))

    puntos_circulo.extend(puntos)
    puntos_circulo.extend(puntos2)
    puntos_circulo.append(punto_fin)

    # print(puntos_circulo)

    return puntos_circulo

def calcular_puntos_ovalo(position, longitud, radio, num_puntos):
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
    puntos = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(yaw + np.pi/2), radio, num_puntos + 1, 1)
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
    puntos = calcular_puntos_media_circunferencia(x_c, y_c, z_c, f.norm_pi(yaw + np.pi/2), radio, num_puntos + 1, 1)
    puntos_ovalo.extend(puntos)

    punto_fin = (position[0], position[1], position[2], f.norm_pi(position[3]))
    puntos_ovalo.append(punto_fin)

    # print(puntos_ovalo)

    return puntos_ovalo


def calcular_puntos_espiral(position, radio, incremento_altura, num_puntos):
    puntos_espiral = []

    x_c = position[0] + radio * np.cos(f.norm_pi(position[3] + np.pi/2))
    y_c = position[1] + radio * np.sin(f.norm_pi(position[3] + np.pi/2))
    z_c = position[2]

    puntos = calcular_puntos_media_circunferencia2(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), radio, num_puntos, 1, incremento_altura)
    puntos_espiral.extend(puntos)

    z_c = z_c + incremento_altura * num_puntos

    puntos2 = calcular_puntos_media_circunferencia2(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), -1 * radio, num_puntos, 1, incremento_altura)
    puntos_espiral.extend(puntos2)

    z = position[2] + incremento_altura * num_puntos * 2
    punto_fin = (position[0], position[1], z, f.norm_pi(position[3]))
    puntos_espiral.append(punto_fin)

    # Repetir el proceso
    z_c = z_c + incremento_altura * num_puntos
    puntos = calcular_puntos_media_circunferencia2(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), radio, num_puntos, 1, incremento_altura)
    puntos_espiral.extend(puntos)

    z_c = z_c + incremento_altura * num_puntos
    puntos2 = calcular_puntos_media_circunferencia2(x_c, y_c, z_c, f.norm_pi(position[3] + np.pi/2), -1 * radio, num_puntos, 1, incremento_altura)
    puntos_espiral.extend(puntos2)

    z = position[2] + incremento_altura * num_puntos * 4
    punto_fin = (position[0], position[1], z, f.norm_pi(position[3]))
    puntos_espiral.append(punto_fin)


    # print(puntos_espiral)

    return puntos_espiral




def draw_trajectory_8(position, radio, n_puntos, idDrone):
    # Dibujar los puntos de la trayectoria en rviz (tipo visualization_msgs::Marker)
    # x, y: posición inicial del dron
    rospy.loginfo({'action': 'draw_trajectory_8', 'position': position, 'radio': radio, 'n_puntos': n_puntos, 'idDrone': idDrone})

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
    
    puntos_ocho = calcular_puntos_ocho([position.pose.position.x, position.pose.position.y, position.pose.position.z, f.yaw(position.pose.orientation)], radio, n_puntos)
    for punto in puntos_ocho:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        # rospy.loginfo({'action': 'draw_trajectory_8', 'punto': punto})
        markers.points.append(p)

    # rospy.loginfo({'action': 'draw_trajectory_8', 'markers': markers})
    return markers

def draw_trajectory_cuadrado(position, longitud, n_puntos, idDrone):
    # Dibujar los puntos de la trayectoria en rviz (tipo visualization_msgs::Marker)
    # x, y: posición inicial del dron
    rospy.loginfo({'action': 'draw_trajectory_cuadrado', 'position': position, 'longitud': longitud, 'n_puntos': n_puntos, 'idDrone': idDrone})

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
    
    puntos_cuadrado = calcular_puntos_cuadrado([position.pose.position.x, position.pose.position.y, position.pose.position.z, f.yaw(position.pose.orientation)], longitud, n_puntos)
    for punto in puntos_cuadrado:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        # rospy.loginfo({'action': 'draw_trajectory_cuadrado', 'punto': punto})
        markers.points.append(p)

    # rospy.loginfo({'action': 'draw_trajectory_cuadrado', 'markers': markers})
    return markers

def draw_trajectory_circulo(position, radio, n_puntos, idDrone):
    # Dibujar los puntos de la trayectoria en rviz (tipo visualization_msgs::Marker)
    # x, y: posición inicial del dron
    rospy.loginfo({'action': 'draw_trajectory_circulo', 'position': position, 'radio': radio, 'n_puntos': n_puntos, 'idDrone': idDrone})

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
    
    puntos_circulo = calcular_puntos_circulo([position.pose.position.x, position.pose.position.y, position.pose.position.z, f.yaw(position.pose.orientation)], radio, n_puntos)
    for punto in puntos_circulo:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        # rospy.loginfo({'action': 'draw_trajectory_circulo', 'punto': punto})
        markers.points.append(p)

    # rospy.loginfo({'action': 'draw_trajectory_circulo', 'markers': markers})
    return markers


def draw_trajectory_ovalo(position, longitud, radio, n_puntos, idDrone):
    # Dibujar los puntos de la trayectoria en rviz (tipo visualization_msgs::Marker)
    # x, y: posición inicial del dron
    rospy.loginfo({'action': 'draw_trajectory_ovalo', 'position': position, 'longitud': longitud, 'radio': radio, 'n_puntos': n_puntos, 'idDrone': idDrone})

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
    
    puntos_ovalo = calcular_puntos_ovalo([position.pose.position.x, position.pose.position.y, position.pose.position.z, f.yaw(position.pose.orientation)], longitud, radio, n_puntos)
    for punto in puntos_ovalo:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        # rospy.loginfo({'action': 'draw_trajectory_ovalo', 'punto': punto})
        markers.points.append(p)

    # rospy.loginfo({'action': 'draw_trajectory_ovalo', 'markers': markers})
    return markers


def draw_trajectory_espiral(position, radio, incremento_altura, n_puntos, idDrone):
    # Dibujar los puntos de la trayectoria en rviz (tipo visualization_msgs::Marker)
    # x, y: posición inicial del dron
    rospy.loginfo({'action': 'draw_trajectory_espiral', 'position': position, 'radio': radio, 'incremento_altura': incremento_altura, 'n_puntos': n_puntos, 'idDrone': idDrone})

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
    
    puntos_espiral = calcular_puntos_espiral([position.pose.position.x, position.pose.position.y, position.pose.position.z, f.yaw(position.pose.orientation)], radio, incremento_altura, n_puntos)
    for punto in puntos_espiral:
        p = geometry_msgs.msg.Point()
        p.x = punto[0]
        p.y = punto[1]
        p.z = punto[2]
        # rospy.loginfo({'action': 'draw_trajectory_espiral', 'punto': punto})
        markers.points.append(p)

    # rospy.loginfo({'action': 'draw_trajectory_espiral', 'markers': markers})
    return markers

