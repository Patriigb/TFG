# UTILIZACIÓN DE UN SISTEMA DE CAPTURA DE MOVIMIENTO EXTERNO 
# PARA EL CONTROL DEL MOVIMIENTO DE EQUIPOS DE DRONES

Aquí se muestran los dos paquetes de ROS necesarios para hacer funcionar el sistema desarrollado,
mediante el que se utiliza OptiTrack para monitorizar y corregir el movimiento de los drones
Tello EDU durante la ejecución de distintas trayectorias.

El paquete optitrack_arena lanza un nodo que recibe la localización enviada por OptiTrack y la
publica en el topic /optitrack/pose_{ID_Dron}.

El paquete pytello lanza un nodo que va leyendo la localización de los drones en el topic anterior
y controla el movimiento de los drones para que sigan una trayectoria predefinida. Se encuentra
aquí todo el código fuente relacionado con la aplicación web, las 3 versiones del generador de
trayectorias y las funciones de control y conexión con los drones.

--------------------------------------------------------------------------------------------------

## Instalación:

Una vez que estos dos paquetes se encuentren en la carpeta src de un workspace de ROS, se deben
compilar con 'catkin build'.

A continuación, hay que asegurarse de tener las dependencias necesarias instaladas. Estas se
encuentran en el archivo "requirements.txt" de la carpeta pytello.

--------------------------------------------------------------------------------------------------

## Ejecución:

Para lanzar el sistema, se deben seguir los siguientes pasos:

1. En la primera terminal, lanzar el máster de ROS:
```
roscore
```

2. En otra terminal, lanzar el nodo que recibe la localización de OptiTrack:
```
roslaunch optitrack_arena optitrack_arena.launch
```

3. En una tercera terminal, lanzar el nodo que controla los drones:
```
rosrun pytello main.py
```

Con esto, en la última terminal se mostrará la dirección IP del servidor web, que se puede abrir
en un navegador al pulsar sobre ella.

Para visualizar la trayectoria que sigue cada dron, se puede iniciar y configurar rviz.


