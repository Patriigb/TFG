
import contextlib
import os
import socket
import threading
import time
import rospy

from multiprocessing import Process, Event
import multiprocessing

import numpy as np
import geometry_msgs.msg
import rospy
import nav_msgs.msg
import visualization_msgs.msg

from pytello.droneapp.models.base import Singleton
import pytello.droneapp.models.functions as fn
import pytello.droneapp.models.trajectories as tr


DEFAULT_DISTANCE = 0.5
DEFAULT_SPEED = 50
DEFAULT_DEGREE = 90
DEFAULT_DIRECTION = 'l'

class DroneManager(metaclass=Singleton):
    def __init__(self, host_ip='', host_port=8889, 
                 drone_port=8889, drones=['192.168.0.116']):
        self.host_ip = host_ip
        self.host_port = host_port
        self.drone_port = drone_port
        self.speed = DEFAULT_SPEED
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host_ip, self.host_port))

        self.response = None
        self.stop_event = threading.Event()
        self._response_thread = threading.Thread(target=self.receive_response,
                                                 args=(self.stop_event,))
        self._response_thread.start()

        self._command_semaphore = threading.Semaphore(1)
        self._command_thread = None

        self.poses = multiprocessing.Manager().list([None, None])
        self.position1_ready_event = Event()
        self.position2_ready_event = Event()
        self.pose_1_lock = threading.Lock()
        self.path_1 = nav_msgs.msg.Path()
        self.pose_2_lock = threading.Lock()
        self.path_2 = nav_msgs.msg.Path()

        self.drones = drones

        self.subs = [None] * len(self.drones)
        self.pubs = [None] * len(self.drones)
        self.pubs_draw = [None] * len(self.drones)

        for i in range(len(self.drones)):
            # Suscribirse al topic /optitrack/pose_{Id dron} para recibir la posición del dron
            # y publicar la posición en el topic /tello_{Id dron}/path
            sub = rospy.Subscriber("/optitrack/pose_" + str(i + 1), geometry_msgs.msg.PoseStamped, 
                                   self.update_position, i, queue_size=1)
            pub = rospy.Publisher("/tello_" + str(i + 1) + "/path", nav_msgs.msg.Path, queue_size=10)
            self.subs[i] = sub
            self.pubs[i] = pub

            # Publicar la trayectoria en el topic /draw_trajectory_{Id dron}
            pub_draw = rospy.Publisher("/draw_trajectory" + str(i + 1), visualization_msgs.msg.Marker, 
                                       queue_size=10)
            self.pubs_draw[i] = pub_draw

        self.send_command(0, 'command')
        self.set_speed(self.speed)
        self.send_command(0, 'battery?')


    
    ''' ------------------------- MÉTODOS PARA RECIBIR Y ACTUALIZAR POSICIÓN ------------------------- '''

    def update_position(self, data, id):
        """
        Actualiza la posición del dron en exclusión mutua.

        Parámetros:
        - data: Los datos de la posición del dron.
        - id: El identificador del dron.

        """
        if id == 0:
            with self.pose_1_lock:
                self.poses[id] = data

                self.path_1.header.frame_id = "odom"
                self.path_1.header.stamp = rospy.Time.now()
                self.path_1.poses.append(data)
                self.pubs[id].publish(self.path_1)

                self.position1_ready_event.set() 
        else:
            with self.pose_2_lock:
                self.poses[id] = data

                self.path_2.header.frame_id = "odom"
                self.path_2.header.stamp = rospy.Time.now()
                self.path_2.poses.append(data)
                self.pubs[id].publish(self.path_2)

                self.position2_ready_event.set()


    def read_pose(self, idDrone):
        """
        Lee la posición del dron.

        Parámetros:
        - idDrone: El ID del dron cuya posición se va a leer.

        Devuelve:
        La posición del dron especificado.

        """
        if idDrone == 1:
            with self.pose_1_lock:
                return self.poses[0]
        else:
            with self.pose_2_lock:
                return self.poses[1]
            

    def wait_for_position(self, idDrone):
        """
        Espera a que se reciba la posición del dron.

        Parámetros:
        - idDrone: El ID del dron que espera recibir la posición.

        """
        if idDrone == 1:
            self.position1_ready_event.wait()
            self.position1_ready_event.clear()
        else:
            self.position2_ready_event.wait()
            self.position2_ready_event.clear()
        

    def stop(self):
        """
        Detiene el dron y cierra la conexión.

        Nota: Si se está ejecutando en Windows, se debe utilizar la señal de terminación CTRL_C_EVENT.

        """
        self.stop_event.set()
        retry = 0
        while self._response_thread.is_alive():
            time.sleep(0.3)
            if retry > 30:
                break
            retry += 1
        self.socket.close()
        os.kill(self.proc.pid, 9)

        ## Para Windows:
        # import signal
        # os.kill(self.proc.pid, signal.CTRL_C_EVENT)
    
    
    def __dell__(self):
        """
        Método especial que se ejecuta cuando se destruye una instancia de la clase. Detiene el dron.

        """
        self.stop()
    


    ''' ------------------------- MÉTODOS PARA ENVIAR Y RECIBIR COMANDOS ------------------------- '''

    def receive_response(self, stop_event):
        """
        Método que recibe la respuesta del socket mientras el stop_event no esté activado.

        Parámetros:
        - stop_event: Evento de parada que indica si se debe detener la recepción de respuestas.

        """
        while not stop_event.is_set():
            try:
                self.response, ip = self.socket.recvfrom(3000)
                rospy.loginfo({'action': 'receive_response', 'response': self.response})

            except socket.error as ex:
                rospy.loginfo({'action': 'receive_response_err', 'ex': ex})
                break




    def send_command(self, idDrone, command, blocking=True):
        """
        Envía un comando a todos los drones en el caso de idDrone = 0 o a un único dron en los demás casos.

        Parámetros:
        - idDrone (int): El ID del dron al que se enviará el comando.
        - command (str): El comando a enviar al dron.
        - blocking (bool): Indica si el método debe bloquear hasta que se complete el comando. Por defecto
          es True.

        """
        if idDrone == 0: # Todos los drones a la vez
            procs = []

            for drone in self.drones:
                procs.append(Process(target=self._send_command, args=((drone, self.drone_port), command, 
                                                                      blocking,)))

            for p in procs:
                p.start()
            
            rospy.loginfo("Comandos enviados")

        else: # Solo un dron
            proc = Process(target=self._send_command, args=((self.drones[idDrone - 1], self.drone_port), 
                                                            command, blocking,))
            proc.start()


    def _send_command(self, drone_address, command, blocking=True):
        """
        Envía un comando al dron especificado.

        Parámetros:
        - drone_address (tuple): La dirección IP y el puerto del dron.
        - command (str): El comando a enviar al dron.
        - blocking (bool, optional): Indica si el envío del comando debe bloquear hasta recibir una
          respuesta, por defecto es True.

        Devuelve:
        - str: La respuesta del dron al comando enviado, o None si no se recibió ninguna respuesta.
        """
        is_acquire = self._command_semaphore.acquire(blocking=blocking)
        if is_acquire:
            with contextlib.ExitStack() as stack:
                stack.callback(self._command_semaphore.release)
                # Si el comando es rc ... no se muestra
                if not command.startswith('rc'):
                    rospy.loginfo({'action2': 'send_command', 'command': command, 'ip': drone_address})
                self.socket.sendto(command.encode('utf-8'), drone_address)

                retry = 0
                while self.response is None:
                    time.sleep(0.3)
                    if retry > 3:
                        break
                    retry += 1

                if self.response is None:
                    response = None
                else:
                    response = self.response.decode('utf-8')
                self.response = None
                return response

        else:
            rospy.logwarn({'action': 'send_command', 'command': command, 'status': 'not_acquire'})





    ''' ------------------------- COMANDOS BÁSICOS DE CONTROL ------------------------- '''

    def takeoff(self, idDrone): # Despegar
        return self.send_command(idDrone, 'takeoff')

    def land(self, idDrone): # Aterrizar
        return self.send_command(idDrone, 'land')

    def move(self, direction, distance, idDrone): # Moverse en una dirección con una distancia dada
        distance = float(distance)
        distance = int(round(distance * 100))
        return self.send_command(idDrone, f'{direction} {distance}')

    def up(self, idDrone, distance=DEFAULT_DISTANCE): # Subir
        return self.move('up', distance, idDrone)

    def down(self, idDrone, distance=DEFAULT_DISTANCE): # Bajar
        return self.move('down', distance, idDrone)

    def left(self, idDrone, distance=DEFAULT_DISTANCE): # Moverse a la izquierda
        return self.move('left', distance, idDrone)

    def right(self, idDrone, distance=DEFAULT_DISTANCE): # Moverse a la derecha
        return self.move('right', distance, idDrone)

    def forward(self, idDrone, distance=DEFAULT_DISTANCE): # Moverse hacia adelante
        return self.move('forward', distance, idDrone)

    def backward(self, idDrone, distance=DEFAULT_DISTANCE): # Moverse hacia atrás
        return self.move('back', distance, idDrone)

    def set_speed(self, speed): # Cambiar la velocidad del dron
        return self.send_command(0, f'speed {speed}')

    def clockwise(self, idDrone, degree=DEFAULT_DEGREE): # Girar en sentido horario
        return self.send_command(idDrone, f'cw {degree}')

    def counter_clockwise(self, idDrone, degree=DEFAULT_DEGREE): # Girar en sentido antihorario
        return self.send_command(idDrone, f'ccw {degree}')

    def flip(self, idDrone, direction=DEFAULT_DIRECTION): # Hacer un flip
        return self.send_command(idDrone, f'flip {direction}')
    
    def set_4_speed(self, lr, fb, ud, yw, idDrone): # Controlar el dron con los 4 valores de velocidad
        self.send_command(idDrone, f'rc {lr} {fb} {ud} {yw}')
    


    ''' ------------------------- FUNCIONES PARA TRAYECTORIAS ------------------------- '''

    def draw(self, idDrone, trajectory=0):
        self.wait_for_position(idDrone)
        position = self.read_pose(idDrone)

        if trajectory == 0:
            markers = tr.draw_trajectory_cuadrado(position, 1, 10)
            self.pubs_draw[idDrone - 1].publish(markers)
        elif trajectory == 1:
            markers = tr.draw_trajectory_ovalo(position, 1, 0.5, 5)
            self.pubs_draw[idDrone - 1].publish(markers)
        elif trajectory == 2:
            markers = tr.draw_trajectory_circulo(position, 0.5, 10)
            self.pubs_draw[idDrone - 1].publish(markers)
        elif trajectory == 3:
            markers = tr.draw_trajectory_espiral(position, 0.5, 0.05, 5)
            self.pubs_draw[idDrone - 1].publish(markers)
        elif trajectory == 4:
            markers = tr.draw_trajectory_8(position, 0.25, 5)
            self.pubs_draw[idDrone - 1].publish(markers)


    def start_trajectory(self, idDrone, idTrajectory=0, version=0):
        """
        Inicia una trayectoria para un dron específico.

        Parámetros:
        - idDrone (int): El número del dron. No puede ser 0.
        - idTrajectory (int, opcional): El identificador de la trayectoria. Por defecto es 0.
        - version (int, opcional): La versión del generador de trayectorias. Por defecto es 0.

        """
        try:
            self.takeoff(idDrone)
            initial_pose = [None, None]
            rospy.loginfo("Despegado")

            # Esperar a que el dron se estabilice y leer la posición inicial
            time.sleep(5) 
            self.wait_for_position(idDrone)
            initial_pose[idDrone - 1] = self.read_pose(idDrone)
            rospy.loginfo({'action': 'start_trajectory', 'initial_pose': initial_pose[idDrone - 1], 
                           'idDrone': idDrone})
            
            # Crear fichero para log
            f = open("log " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
            
        except Exception as e:
            rospy.loginfo({'action': 'start_trajectory', 'error': str(e)})
            return
        
        if version == 0: # Versión "Calcular tiempos"
            if idTrajectory == 0:
                self.cuadrado_open_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 1:
                self.ovalo_open_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 2:
                self.circulo_open_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 3:
                self.espiral_open_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 4:
                self.ocho_open_loop(idDrone, initial_pose[idDrone - 1], f)

        elif version == 1: # Versión "Minimizar distancias"
            if idTrajectory == 0:
                self.cuadrado_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 1:
                self.ovalo_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 2:
                self.circulo_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 3:
                self.espiral_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 4:
                self.ocho_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)

        elif version == 2: # Versión "Controlador de posición"
            if idTrajectory == 0:
                self.cuadrado_closed_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 1:
                self.ovalo_closed_loop(idDrone, initial_pose[idDrone - 1], f)

        f.close()
        rospy.loginfo("Fin")
        self.set_4_speed(0, 0, 0, 0, idDrone)
        self.land(idDrone)
        

    def stop_trajectory(self, idDrone):
        """
        Detiene la trayectoria del dron.

        Parámetros:
        - idDrone: El ID del dron.

        """
        rospy.loginfo({'action': 'stop_trajectory'})
        self.set_4_speed(0, 0, 0, 0, idDrone)
        self.land(idDrone)


    ''' ------------------------- VERSIÓN 0: CALCULAR TIEMPOS ------------------------- '''

    def cuadrado_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CUADRADO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Colocar a los drones en la posición inicial
        if idDrone == 1:
            self.set_4_speed(0, 0, 30, 0, idDrone)
            time.sleep(3)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(1)

        else:
            self.set_4_speed(0, 0, 10, 0, idDrone)
            time.sleep(1)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(3)

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Calcular puntos del cuadrado y dibujar trayectoria
        num_puntos = 10
        pts = tr.calcular_puntos_cuadrado(fn.loc(tIW), 1, num_puntos)
        index = 0
        for p in pts:
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_cuadrado(initial_pose, 1, num_puntos)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Asignar velocidad lineal, angular y medida de un lado
        v = 0.30
        w = 1.152
        m = 1
        
        # Tiempo para hacer un lado
        t = m / v
        rospy.loginfo({'tiempo': t})

        # Tiempo para escribir en el log
        t_log = t / num_puntos

        # Tiempo para girar 90 grados
        t2 = np.pi / 2 / w 
        rospy.loginfo({'tiempo2': t2})
    
        # Cuando pase el tiempo, siguiente paso
        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            self.set_4_speed(0, v * 100 / 1.20, 0, 0, idDrone) # avanza recto

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            j = 1
            while not next:
                timeDrone = rospy.Time.now().to_sec() - t0
                if timeDrone >= t:
                    rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
                    next = True
                elif timeDrone >= t_log * j:
                    index += 1
                    j += 1
                pose = self.read_pose(idDrone)
                position = fn.loc(fn.hom(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                                         fn.norm_pi(fn.yaw(pose.pose.orientation))))
                f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                    + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                
            self.set_4_speed(0, 0, 0, -w * 100 / 1.44, idDrone) # gira a la izquierda

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            next = False
            while not next:
                timeDrone = rospy.Time.now().to_sec() - t0
                if timeDrone >= t2:
                    rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
                    next = True
                pose = self.read_pose(idDrone)
                position = fn.loc(fn.hom(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 
                                         fn.norm_pi(fn.yaw(pose.pose.orientation))))
                f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                    + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")

            i = (i + 1) % 4
            if i == 0:
                fin = True


    def ovalo_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OVALO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Calcular puntos del óvalo y dibujar trayectoria
        pts = tr.calcular_puntos_ovalo(fn.loc(tIW), 1, 0.5, 5)
        index = 0
        for p in pts:
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_ovalo(initial_pose, 1, 0.5, 5)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Asignar velocidad lineal, angular y medidas del ovalo
        v = 0.30
        w = 0.60
        m = 1
        r = 0.5
        
        # Tiempo para hacer un lado
        t1 = m / v
        rospy.loginfo({'tiempo1': t1})

        # Tiempo para girar medio circulo
        t2 = np.pi * r / v
        rospy.loginfo({'tiempo2': t2})

        # Cuando pase el tiempo, siguiente paso
        t = t1
        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            if i == 0 or i == 2:
                t = t1
                self.set_4_speed(0, v * 100, 0, 0, idDrone)
            else:
                t = t2
                self.set_4_speed(0, v * 100 / 1.20, 0, -w * 100 / 1.20, idDrone)

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            t_log = t / 5
            j = 0
            while not next:
                timeDrone = rospy.Time.now().to_sec() - t0

                pose = self.read_pose(idDrone)
                position = fn.loc(fn.hom(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 
                                         fn.norm_pi(fn.yaw(pose.pose.orientation))))
                f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                    + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                
                if timeDrone >= t:
                    rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
                    next = True
                elif timeDrone >= t_log * j:
                    index += 1
                    j += 1
                  
            i = (i + 1) % 4
            if i == 0:
                fin = True

    def circulo_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CIRCULO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Calcular puntos del círculo y dibujar trayectoria
        pts = tr.calcular_puntos_circulo(fn.loc(tIW), 0.5, 10)
        index = 0
        for p in pts:
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_circulo(initial_pose, 0.5, 10)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Asignar velocidad lineal, angular y medida del radio
        v = 0.30
        w = 0.60
        r = v / w

        # Tiempo para hacer una circunferencia
        t = 2 * np.pi * r / v
        rospy.loginfo({'tiempo': t})

        t_2 = t / 20 # Cada t_2 segundos se guarda un punto

        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            self.set_4_speed(0, v * 100 / 1.20 , 0, -w * 100 / 1.30, idDrone) # avanza

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            j = 0
            while not next:
                timeDrone = rospy.Time.now().to_sec() - t0
                pose = self.read_pose(idDrone)
                position = fn.loc(fn.hom(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 
                                         fn.norm_pi(fn.yaw(pose.pose.orientation))))
                f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                    + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                if timeDrone >= t:
                    rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
                    next = True
                elif timeDrone >= t_2 * j:
                    index += 1
                    j += 1
                
            fin = True


    def espiral_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'ESPIRAL'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Calcular puntos de la espiral y dibujar trayectoria
        pts = tr.calcular_puntos_espiral(fn.loc(tIW), 0.5, 0.05, 5)
        index = 0
        for p in pts:
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_espiral(initial_pose, 0.5, 0.05, 5)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Asignar velocidad lineal, angular y medida del radio
        v = 0.30
        w = 0.60
        r = v / w

        # Tiempo para hacer una circunferencia
        t = 2 * np.pi * r / v
        rospy.loginfo({'tiempo': t})

        t_2 = t * 2 / 21 # Cada t_2 segundos se guarda un punto

        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            self.set_4_speed(0, v * 100 / 1.20 , 15, -w * 100 / 1.20, idDrone)

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            j = 0
            while not next:
                timeDrone = rospy.Time.now().to_sec() - t0
                pose = self.read_pose(idDrone)
                position = fn.loc(fn.hom(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 
                                         fn.norm_pi(fn.yaw(pose.pose.orientation))))
                f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                    + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                if timeDrone >= 2 * t:
                    rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
                    next = True
                elif timeDrone >= t_2 * j:
                    index += 1
                    j += 1
                elif timeDrone >= t:
                    self.set_4_speed(0, v * 100 / 1.20 , 16, -w * 100 / 1.20, idDrone)

            fin = True


    def ocho_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OCHO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
                  
        # Colocar a los drones en la posición inicial
        if idDrone == 1:
            self.set_4_speed(0, 0, 40, 0, idDrone)
            time.sleep(3)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(1)
        else:
            self.set_4_speed(0, 0, 10, 0, idDrone)
            time.sleep(3)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(1)

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Calcular puntos del ocho y dibujar trayectoria
        pts = tr.calcular_puntos_ocho(fn.loc(tIW), 0.26, 4)
        index = 0
        for p in pts:
            # rospy.loginfo({'action': 'start_trajectory', 'pose': p})
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_8(initial_pose, 0.26, 4)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Asignar velocidad lineal, angular y medida del radio
        v = 0.30
        w = 1.152
        r = v / w

        # Tiempo para hacer media circunferencia
        t = np.pi * r / v
        rospy.loginfo({'tiempo': t})

        t_2 = t / 4

        # Cuando pase el tiempo, siguiente paso
        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            if i == 0 or i == 3:
                self.set_4_speed(0, v * 100 / 1.20, 0, -w * 100 / 1.44, idDrone) # gira a la izquierda
            else:
                self.set_4_speed(0, v * 100 / 1.20, 0, w * 100 / 1.44, idDrone) # gira a la derecha

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            j = 1
            while not next:
                timeDrone = rospy.Time.now().to_sec() - t0
                
                if timeDrone >= t:
                    rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
                    next = True
                elif timeDrone >= t_2 * j:
                    index += 1
                    j += 1

                pose = self.read_pose(idDrone)
                position = fn.loc(fn.hom(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 
                                         fn.norm_pi(fn.yaw(pose.pose.orientation))))
                f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                    + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")

            index += 1
            i = (i + 1) % 4
            if i == 0:
                fin = True



    ''' ------------------------- VERSIÓN 1: MINIMIZAR DISTANCIAS ------------------------- '''

    def cuadrado_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CUADRADO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Colocar a los drones en la posición inicial
        if idDrone == 1:
            self.set_4_speed(0, 0, 30, 0, idDrone)
            time.sleep(3)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(1)
        else:
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(3)

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        next_pose = initial_pose
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Puntos respecto al dron:
        tFI = [[1, 0, 0, np.pi/2], [1, 1, 0, np.pi], [0, 1, 0, -np.pi/2], [0, 0, 0, 0]]
        
        # Posiciones finales respecto al mundo:
        index = 0
        tFW = [np.dot(tIW, fn.hom(t[0], t[1], t[2], t[3])) for t in tFI]
        final_poses = [fn.loc(t) for t in tFW]
        rospy.loginfo({'action': 'start_trajectory', 'final_poses': final_poses, 'idDrone': idDrone})
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1

        index = 0
        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            initial_pose = next_pose
            self.set_4_speed(0, 20, 0, 0, idDrone) # avanza
            min_dist = 100

            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    dist = fn.distancia(next_pose, final_poses[i])
                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, 
                                             next_pose.pose.position.z, 
                                             fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    
                    # Comprobar si ha llegado al punto
                    if dist < min_dist:
                        min_dist = dist

                    if dist < 0.2 or dist >= min_dist + 0.1 or fn.esta_cerca(next_pose, final_poses[i], 
                                                                             0.2, 0.5):
                        next = True
                        self.set_4_speed(0, 0, 0, 0, idDrone)
                        time.sleep(1)

            self.set_4_speed(0, 0, 0, -50, idDrone) # gira a la izquierda

            next = False
            min_diff_orientation = 100
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:

                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, 
                                             next_pose.pose.position.z, 
                                             fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")

                    # Comprobar si ha girado 90 grados
                    diff_orientation = fn.diff_orientation(abs(fn.norm_pi(fn.yaw(next_pose.pose.orientation))), 
                                                           abs(final_poses[i][3]))
                    rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + 
                                  str(i) + " " + str(round(final_poses[i][3], 2)) + " " + 
                                  str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                    if diff_orientation < min_diff_orientation:
                        min_diff_orientation = diff_orientation

                    if diff_orientation < 0.001 or diff_orientation >= min_diff_orientation + 0.05:
                        next = True
                        
            index += 1
            i = (i + 1) % 4
            if i == 0:
                fin = True

        f2.close()

    def ovalo_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OVALO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        next_pose = initial_pose
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))

        # Calcular puntos del óvalo y dibujar trayectoria
        index = 0
        final_poses = tr.calcular_puntos_ovalo(fn.loc(tIW), 1, 0.5, 5)
        rospy.loginfo({'action': 'start_trajectory', 'final_poses': final_poses, 'idDrone': idDrone})
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1

        markers = tr.draw_trajectory_ovalo(initial_pose, 1, 0.5, 5)
        self.pubs_draw[idDrone - 1].publish(markers)

        index = 0
        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            if 0 <= i <= 3 or 12 <= i <= 14: # Avanza en línea recta
                next = False
                initial_pose = next_pose
                self.set_4_speed(0, 25, 0, 0, idDrone)
                min_dist = 100

                while not next:
                    next_pose = self.read_pose(idDrone)
                    
                    if next_pose is not None:
                        dist = fn.distancia(next_pose, final_poses[i])
                        position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, 
                                                 next_pose.pose.position.z, 
                                                 fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                        f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                                + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                        
                        # Comprobar si ha llegado al punto
                        if dist < min_dist:
                            min_dist = dist

                        if dist < 0.01 or dist >= min_dist + 0.1 or fn.esta_cerca(next_pose, final_poses[i], 
                                                                                  0.05, 0.1):
                            next = True
            
            else: # Gira media circunferencia
                next = False
                initial_pose = next_pose
                self.set_4_speed(0, 25, 0, -47, idDrone)
                min_diff_orientation = 100

                while not next:
                    next_pose = self.read_pose(idDrone)
                    
                    if next_pose is not None:
                        position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, 
                                                 next_pose.pose.position.z, 
                                                 fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                        f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                                + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")

                        # Comprobar si ha llegado al punto
                        diff_orientation = fn.diff_orientation(abs(fn.norm_pi(fn.yaw(next_pose.pose.orientation))), 
                                                               abs(final_poses[i][3]))
                        if diff_orientation < min_diff_orientation:
                            min_diff_orientation = diff_orientation

                        if (diff_orientation < 0.001 or diff_orientation >= min_diff_orientation + 0.1 
                            or fn.esta_cerca(next_pose, final_poses[i], 0.2, 0.1)):
                            next = True

            index += 1
            i = (i + 1) % 22
            if i == 0:
                fin = True

        f2.close()

    def circulo_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CIRCULO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        next_pose = initial_pose
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))

        # Calcular puntos del círculo y dibujar trayectoria
        index = 0
        final_poses = tr.calcular_puntos_circulo(fn.loc(tIW), 0.5, 4)
        rospy.loginfo({'action': 'start_trajectory', 'final_poses': final_poses, 'idDrone': idDrone})
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_circulo(initial_pose, 0.5, 4)
        self.pubs_draw[idDrone - 1].publish(markers)

        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            initial_pose = next_pose
            self.set_4_speed(0, 25, 0, -46, idDrone) 
            min_diff_orientation = 100

            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, 
                                             next_pose.pose.position.z, 
                                             fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    
                    # Comprobar si ha llegado al punto
                    diff_orientation = fn.diff_orientation(abs(fn.norm_pi(fn.yaw(next_pose.pose.orientation))), 
                                                           abs(final_poses[i][3]))
                    rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + 
                                  str(i) + " " + str(round(final_poses[i][3], 2)) + " " + 
                                  str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                    if diff_orientation < min_diff_orientation:
                        min_diff_orientation = diff_orientation

                    if (diff_orientation < 0.001 or diff_orientation >= min_diff_orientation + 0.05 
                        or fn.esta_cerca(next_pose, final_poses[i], 0.2, 0.1)):
                        next = True

            index += 1
            i = (i + 1) % 9
            if i == 0:
                fin = True

        f2.close()

    def espiral_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'ESPIRAL'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Colocar a los drones en la posición inicial
        if idDrone == 1:
            self.set_4_speed(0, 0, 30, 0, idDrone)
            time.sleep(3)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(1)
        else:
            self.set_4_speed(0, 0, 10, 0, idDrone)
            time.sleep(1)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(3)

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        next_pose = initial_pose
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))

        # Calcular puntos de la espiral y dibujar trayectoria
        index = 0
        final_poses = tr.calcular_puntos_espiral(fn.loc(tIW), 0.5, 0.05, 5)
        rospy.loginfo({'action': 'start_trajectory', 'final_poses': final_poses, 'idDrone': idDrone})
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_espiral(initial_pose, 0.5, 0.05, 5)
        self.pubs_draw[idDrone - 1].publish(markers)

        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            initial_pose = next_pose
            self.set_4_speed(0, 24, 15, -47, idDrone) 
            min_diff_orientation = 100

            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, 
                                             next_pose.pose.position.z, 
                                             fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                
                    # Comprobar si ha llegado al punto
                    diff_orientation = fn.diff_orientation(abs(fn.norm_pi(fn.yaw(next_pose.pose.orientation))), 
                                                           abs(final_poses[i][3]))
                    rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + 
                                  str(i) + " " + str(round(final_poses[i][3], 2)) + " " + 
                                  str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                    if diff_orientation < min_diff_orientation:
                        min_diff_orientation = diff_orientation

                    if (diff_orientation < 0.001 or diff_orientation >= min_diff_orientation + 0.05 
                        or fn.esta_cerca(next_pose, final_poses[i], 0.2, 0.5)):
                        next = True

            index += 1
            i = (i + 1) % 22
            if i == 0:
                fin = True

        f2.close()

    def ocho_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OCHO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Colocar a los drones en la posición inicial
        if idDrone == 2:
            self.set_4_speed(0, 0, 30, 0, idDrone)
            time.sleep(3)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(1)
        else:
            time.sleep(4)

        # Posicion inicial respecto al mundo:
        initial_pose = self.read_pose(idDrone)
        next_pose = initial_pose
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))

        # Calcular puntos del ocho y dibujar trayectoria
        final_poses = tr.calcular_puntos_ocho(fn.loc(tIW), 0.25, 3)
        index = 0
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_8(initial_pose, 0.25, 3)   
        self.pubs_draw[idDrone - 1].publish(markers)     

        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            initial_pose = next_pose
            if 0 <= i <= 3 or 9 < i <= 12:
                self.set_4_speed(0, 26, 0, -80, idDrone) # gira a la izquierda
            else:
                self.set_4_speed(0, 20, 0, 75, idDrone) # gira a la derecha
            min_diff_orientation = 100

            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, 
                                             next_pose.pose.position.z, 
                                             fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")

                    # Comprobar si ha llegado al punto
                    diff_orientation = fn.diff_orientation(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), final_poses[i][3])
                    rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + 
                                  str(i) + " " + str(round(final_poses[i][3], 2)) + " " + 
                                  str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                    if diff_orientation < min_diff_orientation:
                        min_diff_orientation = diff_orientation

                    if fn.esta_cerca(next_pose, final_poses[i], 0.1, 0.5) or diff_orientation < 0.1:
                        next = True

            index += 1 
            i = (i + 1) % 13
            if i == 0:
                fin = True

        f2.close()



    ''' ------------------------- VERSIÓN 2: CONTROL DE POSICIÓN ------------------------- '''

    def cuadrado_closed_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CUADRADO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Ajustar parámetros de control (kp > 0;  kb < 0;  ka - kp > 0)
        kp = 0.3
        ka = 0.5
        kb =  -0.5

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Calcular puntos del cuadrado y dibujar trayectoria
        pos = tr.calcular_puntos_cuadrado(fn.loc(tIW), 1, 1)
        markers = tr.draw_trajectory_cuadrado(initial_pose, 1, 1)
        self.pubs_draw[idDrone - 1].publish(markers)

        index = 0
        for p in pos:
            x, y, z, w = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(w) + " 0\n")
            index += 1
        index = 0

        # Frecuencia de envío de velocidades
        t_send_velocities = 2

        for i in range(0, len(pos)):
            t0 = rospy.Time.now().to_sec()
            WxG = pos[i]
            esta = False

            while not esta:
                t1 = rospy.Time.now().to_sec()
                if abs(t1 - t0) > t_send_velocities:
                    WxR = self.read_pose(idDrone)

                    if fn.esta_cerca(WxR, WxG, 0.2, 0.5):
                        rospy.loginfo("CERCA")
                        esta = True
                        index += 1
                    else:
                        distancia = fn.distancia(WxR, WxG)
                        rospy.loginfo("Distancia: " + str(distancia) + " hasta " + str(WxG))
                        if distancia <= 0.1:
                            t_send_velocities = 1
                        else:
                            t_send_velocities = 2
                        
                        # Calcular velocidades
                        v, w = fn.calcularVelocidades(WxR, WxG, kp, ka, kb)
                        w = -1 * w

                        # Escribir log
                        position = fn.loc(fn.hom(WxR.pose.position.x, WxR.pose.position.y, 
                                                 WxR.pose.position.z, fn.norm_pi(fn.yaw(WxR.pose.orientation))))
                        f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                                + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")

                        # Enviar velocidades
                        self.set_4_speed(0, v, 0, w, idDrone) 
                        t0 = rospy.Time.now().to_sec()

        f2.close()


    def ovalo_closed_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OVALO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # Ajustar parámetros de control (kp > 0;  kb < 0;  ka - kp > 0)
        kp = 0.6
        ka = 0.5
        kb =  -0.5

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, 
                     initial_pose.pose.position.z, fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Posiciones finales en el mundo:
        pos = tr.calcular_puntos_ovalo(fn.loc(tIW), 1, 0.5, 2)
        markers = tr.draw_trajectory_ovalo(initial_pose, 1, 0.5, 2)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Escribir log
        index = 0
        for p in pos:
            x, y, z, w = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(w) + " 0\n")
            index += 1
        index = 0

        # Frecuencia de envío de velocidades
        t_send_velocities = 2

        for i in range(0, len(pos)):
            t0 = rospy.Time.now().to_sec()
            WxG = pos[i]
            esta = False

            while not esta:
                t1 = rospy.Time.now().to_sec()
                WxR = self.read_pose(idDrone)

                if fn.esta_cerca(WxR, WxG, 0.1, 5):
                    rospy.loginfo("CERCA")
                    esta = True
                    index += 1
                else:
                    distancia = fn.distancia(WxR, WxG)
                    rospy.loginfo("Distancia: " + str(distancia) + " hasta " + str(WxG))
                    if distancia <= 0.05:
                        t_send_velocities = 1
                    else:
                        t_send_velocities = 2
                    
                    # Calcular velocidades
                    v, w = fn.calcularVelocidades(WxR, WxG, kp, ka, kb)
                    w = -1 * w

                    # Escribir log
                    position = fn.loc(fn.hom(WxR.pose.position.x, WxR.pose.position.y, 
                                             WxR.pose.position.z, fn.norm_pi(fn.yaw(WxR.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    
                    # Enviar velocidades
                    if abs(t1 - t0) > t_send_velocities:
                        self.set_4_speed(0, v, 0, w, idDrone) 
                        t0 = rospy.Time.now().to_sec()

        f2.close()

