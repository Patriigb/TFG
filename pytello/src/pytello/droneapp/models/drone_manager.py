import logging
import contextlib
import os
import socket
import subprocess
import threading
import time
import rospy
from multiprocessing import Process, Value, Event
import multiprocessing

import cv2 as cv
import numpy as np
import geometry_msgs.msg
import rospy
from std_msgs.msg import String
import nav_msgs.msg

from pytello.droneapp.models.base import Singleton
import pytello.droneapp.models.functions as fn

import visualization_msgs.msg


logger = logging.getLogger(__name__)

DEFAULT_DISTANCE = 0.5
DEFAULT_SPEED = 50
DEFAULT_DEGREE = 90
DEFAULT_DIRECTION = 'l'

# FRAME_X = int(960 / 3)
# FRAME_Y = int(720 / 3)
# FRAME_AREA = FRAME_X * FRAME_Y

# FRAME_SIZE = FRAME_AREA * 3
# FRAME_CENTER_X = FRAME_X / 2
# FRAME_CENTER_Y = FRAME_Y / 2

# CMD_FFMPEG = (f'ffmpeg -hwaccel auto -hwaccel_device opencl -i pipe:0 '
#               f'-pix_fmt bgr24 -s {FRAME_X}x{FRAME_Y} -f rawvideo pipe:1')

# FACE_DETECT_XML_FILE = './src/pytello/src/pytello/droneapp/models/haarcascade_frontalface_default.xml'

# SNAPSHOTS_IMAGE_FOLDER = './src/pytello/src/pytello/droneapp/static/img/snapshots/'
# VIDEOS_IMAGE_FOLDER = './src/pytello/src/pytello/droneapp/static/videos/'
# FRAMES_IMAGE_FOLDER = './src/pytello/src/pytello/droneapp/static/img/frames/'


# class ErrorNoFaceDectectXMLFile(Exception):
#     """Error no face detect xml file"""


# class ErrorNoImageDir(Exception):
#     """Error no image dir"""

class DroneManager(metaclass=Singleton):
    def __init__(self, host_ip='', host_port=8889,
                 drone_ip='192.168.0.116', drone_port=8889,
                 is_imperial=False, speed=DEFAULT_SPEED, drones=['192.168.0.116']):
        self.host_ip = host_ip
        self.host_port = host_port
        self.drone_ip = drone_ip
        self.drone_port = drone_port
        self.drone_address = (drone_ip, drone_port)
        self.is_imperial = is_imperial
        self.speed = speed
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host_ip, self.host_port))

        self.response = None
        self.stop_event = threading.Event()
        self._response_thread = threading.Thread(target=self.receive_response,
                                                 args=(self.stop_event,))
        self._response_thread.start()

        # self.patrol_event = None
        # self.is_patrol = False
        # self._patrol_semaphore = threading.Semaphore(1)
        # self._thread_patrol = None

        # self.proc = subprocess.Popen(CMD_FFMPEG.split(' '),
        #                              stdin=subprocess.PIPE,
        #                              stdout=subprocess.PIPE)

        # self.proc_stdin = self.proc.stdin
        # self.proc_stdout = self.proc.stdout

        # self.video_port = 42722

        # self._receive_video_thread = threading.Thread(
        #     target=self.receive_video,
        #     args=(self.stop_event, self.proc_stdin,
        #           self.host_ip, self.video_port,)
        # )
        # self._receive_video_thread.start()

        # if not os.path.exists(FACE_DETECT_XML_FILE):
        #     raise ErrorNoFaceDectectXMLFile(f'No {FACE_DETECT_XML_FILE}')
        # self.face_cascade = cv.CascadeClassifier(FACE_DETECT_XML_FILE)
        # self._is_enable_face_detect = False

        # if not os.path.exists(SNAPSHOTS_IMAGE_FOLDER):
        #     raise ErrorNoImageDir(f'No {SNAPSHOTS_IMAGE_FOLDER} does not exists')
        # self.is_snapshot = False
        # self.is_video = False
        # self.take_frames = False
        # self.stop_frames = False

        # self.numFrame = 0
        # self.dir_frames = None
        # self.frame_time = 0
        # self.frame_total_time = 0
        # self.folder = None
        # self.file_frames = None

        # self.video_out = None
        # self.ini_video = None
        # self.fin_video = None

        self._command_semaphore = threading.Semaphore(1)
        self._command_thread = None

        # self.send_command('streamon')

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
            # Suscribirse al topic /optitrack/pose para recibir la posición del dron
            # y publicar la posición en el topic /tello_n/path
            sub = rospy.Subscriber("/optitrack/pose_" + str(i + 1), geometry_msgs.msg.PoseStamped, self.update_position, i, queue_size=1)
            pub = rospy.Publisher("/tello_" + str(i + 1) + "/path", nav_msgs.msg.Path, queue_size=10)
            self.subs[i] = sub
            self.pubs[i] = pub

            pub_draw = rospy.Publisher("/draw_trajectory" + str(i + 1), visualization_msgs.msg.Marker, queue_size=10)
            self.pubs_draw[i] = pub_draw

        
        # set mode
        # for drone in self.drones:
            # self.drone_ip = drone
            # self.drone_address = (self.drone_ip, self.drone_port)
        self.send_command(0, 'command')
        self.set_speed(self.speed)
        self.send_command(0, 'battery?')
        # rospy.loginfo({'drone': drone, 'battery': battery})
            # thread que lee los comandos

    
    def update_position(self, data, id):
        # Actualizar la posición del dron en exclusión mutua
        # rospy.loginfo({'action': 'update_position', 'id': id, 'data': data})
        if id == 0:
            with self.pose_1_lock:
                # actualizar self.pose intercambiando x e y
                self.poses[id] = data
                # y = self.poses[id].pose.position.y
                # self.poses[id].pose.position.y = self.poses[id].pose.position.x
                # self.poses[id].pose.position.x = y

                # self.poses[id] = data

                self.path_1.header.frame_id = "odom"
                self.path_1.header.stamp = rospy.Time.now()
                self.path_1.poses.append(data)
                self.pubs[id].publish(self.path_1)

                self.position1_ready_event.set() 
        else:
            with self.pose_2_lock:
                # actualizar self.pose intercambiando x e y
                self.poses[id] = data
                # y = self.poses[id].pose.position.y
                # self.poses[id].pose.position.y = self.poses[id].pose.position.x
                # self.poses[id].pose.position.x = y
                # self.poses[id] = data

                self.path_2.header.frame_id = "odom"
                self.path_2.header.stamp = rospy.Time.now()
                self.path_2.poses.append(data)
                self.pubs[id].publish(self.path_2)

                self.position2_ready_event.set()
        
        # rospy.loginfo({'action': 'update_position', 'id': id, 'data': self.poses[id]})
        # pose = self.read_pose(id + 1)
        # rospy.loginfo({'action': 'update_position', 'id': id, 'pose': pose})

    def read_pose(self, idDrone):
        # Leer la posición del dron en exclusión mutua
        if idDrone == 1:
            with self.pose_1_lock:
                return self.poses[0]
        else:
            with self.pose_2_lock:
                return self.poses[1]
            
    def wait_for_position(self, idDrone):
        # Esperar a que se reciba la posición del dron
        if idDrone == 1:
            self.position1_ready_event.wait()
            self.position1_ready_event.clear()
        else:
            self.position2_ready_event.wait()
            self.position2_ready_event.clear()
        
    def set_4_speed(self, lr, fb, ud, yw, idDrone):
        # Cambiar la velocidad del dron en los 4 ejes
        self.send_command(idDrone, f'rc {lr} {fb} {ud} {yw}')
        rospy.loginfo({'action': 'set_4_speed', 'lr': lr, 'fb': fb, 'ud': ud, 'yw': yw})
        

    def receive_response(self, stop_event):
        while not stop_event.is_set():
            try:
                self.response, ip = self.socket.recvfrom(3000)
                rospy.loginfo({'action': 'receive_response', 'response': self.response})

            except socket.error as ex:
                rospy.loginfo({'action': 'receive_response_err', 'ex': ex})
                break

    def __dell__(self):
        self.stop()

    def stop(self):
        self.stop_event.set()
        retry = 0
        while self._response_thread.is_alive():
            time.sleep(0.3)
            if retry > 30:
                break
            retry += 1
        self.socket.close()
        # os.kill(self.proc.pid, 9)
        # import signal
        # os.kill(self.proc.pid, signal.CTRL_C_EVENT)

    def send_command(self, idDrone, command, blocking=True):
        if idDrone == 0: # Todos los drones a la vez
            # proc = [None] * len(self.drones)
            # for drone in self.drones:
            #     j = self.drones.index(drone)
            #     proc[j] = Process(target=self._send_command, args=((drone, self.drone_port), command, blocking,))

            # for p in proc:
            #     p.start()

            # for p in proc:
            #     p.join()

            procs = []

            for drone in self.drones:
                procs.append(Process(target=self._send_command, args=((drone, self.drone_port), command, blocking,)))

            for p in procs:
                p.start()
            
            rospy.loginfo("Comandos enviados")

        else: # Solo un dron
            # self._command_thread = threading.Thread(
            #     target=self._send_command,
            #     args=((self.drones[idDrone - 1], self.drone_port), command, blocking,)
            # )
            # self._command_thread.start()

            proc = Process(target=self._send_command, args=((self.drones[idDrone - 1], self.drone_port), command, blocking,))
            proc.start()

    def _send_command(self, drone_address, command, blocking=True):
        is_acquire = self._command_semaphore.acquire(blocking=blocking)
        if is_acquire:
            with contextlib.ExitStack() as stack:
                stack.callback(self._command_semaphore.release)
                rospy.loginfo({'action': 'send_command', 'command': command, 'ip': drone_address})
                self.socket.sendto(command.encode('utf-8'), drone_address)
                #stack.callback(self._command_semaphore.release)

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

    def takeoff(self, idDrone):
        return self.send_command(idDrone, 'takeoff')

    def land(self, idDrone):
        return self.send_command(idDrone, 'land')

    def move(self, direction, distance, idDrone):
        distance = float(distance)
        if self.is_imperial:
            distance = int(round(distance * 30.48))
        else:
            distance = int(round(distance * 100))
        return self.send_command(idDrone, f'{direction} {distance}')

    def up(self, idDrone, distance=DEFAULT_DISTANCE):
        return self.move('up', distance, idDrone)

    def down(self, idDrone, distance=DEFAULT_DISTANCE):
        return self.move('down', distance, idDrone)

    def left(self, idDrone, distance=DEFAULT_DISTANCE):
        return self.move('left', distance, idDrone)

    def right(self, idDrone, distance=DEFAULT_DISTANCE):
        return self.move('right', distance, idDrone)

    def forward(self, idDrone, distance=DEFAULT_DISTANCE):
        return self.move('forward', distance, idDrone)

    def backward(self, idDrone, distance=DEFAULT_DISTANCE):
        return self.move('back', distance, idDrone)

    def set_speed(self, speed):
        return self.send_command(0, f'speed {speed}')

    def clockwise(self, idDrone, degree=DEFAULT_DEGREE):
        return self.send_command(idDrone, f'cw {degree}')

    def counter_clockwise(self, idDrone, degree=DEFAULT_DEGREE):
        return self.send_command(idDrone, f'ccw {degree}')

    def flip(self, idDrone, direction=DEFAULT_DIRECTION):
        return self.send_command(idDrone, f'flip {direction}')

    # def patrol(self):
    #     if not self.is_patrol:
    #         self.patrol_event = threading.Event()
    #         self._thread_patrol = threading.Thread(
    #             target=self._patrol,
    #             args=(self._patrol_semaphore, self.patrol_event,)
    #         )
    #         self._thread_patrol.start()
    #         self.is_patrol = True

    # def stop_patrol(self):
    #     if self.is_patrol:
    #         self.patrol_event.set()
    #         retry = 0
    #         while self._thread_patrol.is_alive():
    #             time.sleep(0.3)
    #             if retry > 300:
    #                 break
    #             retry += 1
    #         self.is_patrol = False

    # def _patrol(self, semaphore, stop_event):
    #     is_acquire = semaphore.acquire(blocking=False)
    #     if is_acquire:
    #         rospy.loginfo({'action': '_patrol', 'status': 'acquire'})
    #         with contextlib.ExitStack() as stack:
    #             stack.callback(semaphore.release)
    #             status = 0
    #             while not stop_event.is_set():
    #                 status += 1
    #                 if status == 1:
    #                     self.up()
    #                 if status == 2:
    #                     self.clockwise(90)
    #                 if status == 3:
    #                     self.down()
    #                 if status == 4:
    #                     status = 0
    #                 time.sleep(5)
    #     else:
    #         rospy.logwarn({'action': '_patrol', 'status': 'not_acquire'})

    def draw_trajectory_8(self, x, y, radio, n_puntos, idDrone):
        # Dibujar los puntos de la trayectoria en rviz (tipo visualization_msgs::Marker)
        # x, y: posición inicial del dron

        # Calcular centro de la circunferencia
        distX = radio * np.cos(np.pi / 4)
        distY = radio * np.sin(np.pi / 4)
        x_c = x + distX
        y_c = y + distY

        puntos = fn.calcular_puntos_circunferencia(x_c, y_c, radio, n_puntos)
        markers = visualization_msgs.msg.Marker()
        markers.header.frame_id = "odom"
        markers.header.stamp = rospy.Time.now()
        markers.ns = "points_and_lines"
        markers.id = 0
        markers.type = visualization_msgs.msg.Marker.POINTS
        markers.action = visualization_msgs.msg.Marker.ADD
        markers.pose.orientation.w = 1.0
        markers.scale.x = 0.03
        markers.scale.y = 0.03
        markers.color.a = 1.0
        markers.color.r = 1.0
        markers.color.g = 1.0
        markers.color.b = 0.0

        for punto in puntos:
            p = geometry_msgs.msg.Point()
            p.x = punto[0]
            p.y = punto[1]
            p.z = 0
            markers.points.append(p)

        # Calcular el otro centro del 8
        distX = radio * np.cos(np.pi / 4)
        distY = radio * np.sin(np.pi / 4)

        puntos2 = fn.calcular_puntos_circunferencia(x + distX, y + distY, radio, n_puntos)
        for punto in puntos2:
            p = geometry_msgs.msg.Point()
            p.x = punto[0]
            p.y = punto[1]
            p.z = 0
            markers.points.append(p)
        
        self.pubs_draw[idDrone - 1].publish(markers)

    def draw(self, idDrone):
        self.wait_for_position(idDrone)
        position = self.read_pose(idDrone)
        self.draw_trajectory_8(position.pose.position.x, position.pose.position.y, 0.6, 50, 1)


    def start_trajectory(self, idDrone): # idDrone es el número del dron, no puede ser 0
        try:
            self.takeoff(idDrone)
            initial_pose = [None, None]
            time.sleep(2) 
            rospy.loginfo("Despegado")
            self.wait_for_position(idDrone)
            initial_pose[idDrone - 1] = self.read_pose(idDrone)
            rospy.loginfo({'action': 'start_trajectory', 'initial_pose': initial_pose[idDrone - 1], 'idDrone': idDrone})

        except Exception as e:
            rospy.loginfo({'action': 'start_trajectory', 'error': str(e)})
            return
        
        # self.zigzag(idDrone, initial_pose[idDrone - 1])
        self.ocho(idDrone, initial_pose[idDrone - 1])
        # self.espiral(idDrone, initial_pose, tFI)
        # self.medirDistancia(idDrone)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        # self.medirGiro(idDrone)

        rospy.loginfo("Fin")

        self.set_4_speed(0, 0, 0, 0, idDrone)
        self.land(idDrone)
        

    def stop_trajectory(self, idDrone):
        rospy.loginfo({'action': 'stop_trajectory'})
        # Poner trajectory a false para que el dron pare de hacer la trayectoria
        self.set_4_speed(0, 0, 0, 0, idDrone)
        self.land(idDrone)

    def medirDistancia(self, idDrone):
        initial_pose = self.read_pose(idDrone)
        self.set_4_speed(0, 10, 0, 0, idDrone)
        time.sleep(1)
        next_pose = self.read_pose(idDrone)
        distancia = np.sqrt((next_pose.pose.position.x - initial_pose.pose.position.x)**2 + (next_pose.pose.position.y - initial_pose.pose.position.y)**2 + (next_pose.pose.position.z - initial_pose.pose.position.z)**2)
        rospy.loginfo({'action': 'medirDistancia', 'distancia': distancia})

    def medirGiro(self, idDrone):
        initial_pose = self.read_pose(idDrone)
        self.set_4_speed(0, 0, 0, 50, idDrone)
        time.sleep(1)
        next_pose = self.read_pose(idDrone)
        giro = fn.norm_pi(fn.giro_to_rad(next_pose.pose.orientation.z) - fn.giro_to_rad(initial_pose.pose.orientation.z))
        rospy.loginfo({'action': 'medirGiro', 'giro': giro})


    def zigzag(self, idDrone, initial_pose):
        rospy.loginfo({'action': 'ZIGZAG'})

        ## zig zag
        pasos = 3
        next_pose = initial_pose
        for i in range(pasos):
            rospy.loginfo("Paso " + str(i))
            rospy.loginfo("Avanzar")
            # Ir hacia delante 0.5 metros (6 s)
            next = False
            initial_pose = next_pose
            primero = True
            self.set_4_speed(0, 20, 0, 0, idDrone)

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    # Posicion inicial respecto al mundo:
                    tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                                fn.norm_pi(fn.giro_to_rad(initial_pose.pose.orientation.z)))
                    
                    # Posicion final respecto al dron:
                    tFI = fn.hom(0.5, 0, 0, 0)

                    # Posicion final respecto al mundo:
                    tFW = np.dot(tIW, tFI)
                    final_pose = fn.loc(tFW)

                    if primero:
                        posefin = [final_pose[0], final_pose[1], final_pose[2], fn.rad_to_giro(fn.norm_pi(final_pose[3]))]
                        rospy.loginfo({'action': 'start_trajectory', 'final_pose': posefin})
                        primero = False

                    # Comprobar si está cerca del objetivo
                    if fn.esta_cerca(next_pose, final_pose, 0.2, 0.1):
                        rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose})
                        # rospy.loginfo({'action': 'start_trajectory_fin', 'final_pose': final_pose})
                        # rospy.loginfo({'tIW': fn.loc(tIW)})
                        # rospy.loginfo({'tFI': fn.loc(tFI)})

                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0
                        rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
                    elif fn.pasado(next_pose, final_pose, initial_pose):
                        rospy.loginfo({'action': 'start_trajectory_fin2', 'next_pose': next_pose})
                        # rospy.loginfo({'action': 'start_trajectory_fin2', 'final_pose': final_pose})
                        # rospy.loginfo({'tIW': fn.loc(tIW)})
                        # rospy.loginfo({'tFI': fn.loc(tFI)})

                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0
                        rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
            
            # Girar 90 grados a la izquierda o derecha (5 s 180 grados)
            rospy.loginfo("Girar")
            next = False
            initial_pose = next_pose
            primero = True
            if i % 2 == 0:
                self.set_4_speed(0, 0, 0, -50, idDrone)
            else:
                self.set_4_speed(0, 0, 0, 50, idDrone)

            t0 = rospy.Time.now().to_sec()
            while not next:
                next_pose = self.read_pose(idDrone)

                if next_pose is not None:
                    # Posicion inicial respecto al mundo:
                    tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                                 fn.norm_pi(fn.giro_to_rad(initial_pose.pose.orientation.z)))

                    # Posicion final respecto al dron:
                    if i % 2 == 0:
                        tFI = fn.hom(0, 0, 0, np.pi / 2)
                    else:
                        tFI = fn.hom(0, 0, 0, -np.pi / 2)

                    # Posicion final respecto al mundo:
                    tFW = np.dot(tIW, tFI)
                    final_pose = fn.loc(tFW)

                    if primero:
                        posefin = [final_pose[0], final_pose[1], final_pose[2], fn.rad_to_giro(fn.norm_pi(final_pose[3]))]
                        rospy.loginfo({'action': 'start_trajectory', 'posefin': posefin})
                        primero = False

                    # Comprobar si está cerca del objetivo
                    if fn.esta_cerca(next_pose, final_pose, 0.2, 0.05):
                        rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose})
                        # rospy.loginfo({'action': 'start_trajectory', 'final_pose': final_pose})
                        # rospy.loginfo({'tIW': fn.loc(tIW)})
                        # rospy.loginfo({'tFI': fn.loc(tFI)})

                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0
                        rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})

    def ocho(self, idDrone, initial_pose):
        rospy.loginfo({'action': 'OCHO'})

        ## ocho
        pasos = 4
        next_pose = initial_pose
        i = 0
        fin = False
        while not fin:
            # Medio círculo de radio 30 cm
            rospy.loginfo("Paso " + str(i))
            next = False
            initial_pose = next_pose
            primero = True
            if i == 0 or i == 3:
                self.set_4_speed(0, 30, 0, -80, idDrone) # gira a la izquierda
            else:
                self.set_4_speed(0, 30, 0, 80, idDrone) # gira a la derecha # 0.80 rad/s 0.24m/s

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    # Posicion inicial respecto al mundo:
                    tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                                fn.norm_pi(fn.giro_to_rad(initial_pose.pose.orientation.z)))
                    
                    # Posicion final respecto al dron:
                    if i == 0: # paso 0
                        tFI = fn.hom(0, 0.6, 0, np.pi)
                    elif i == 1: # paso 1
                        tFI = fn.hom(0, -0.6, 0, np.pi)
                    elif i == 2: # paso 2
                        tFI = fn.hom(0, -0.6, 0, np.pi)
                    elif i == 3: # paso 3
                        tFI = fn.hom(0, 0.6, 0, np.pi)
                        i = -1

                    # Posicion final respecto al mundo:
                    tFW = np.dot(tIW, tFI)
                    final_pose = fn.loc(tFW)

                    if primero:
                        posefin = [final_pose[0], final_pose[1], final_pose[2], fn.rad_to_giro(fn.norm_pi(final_pose[3]))]
                        rospy.loginfo({'action': 'start_trajectory', 'final_pose': posefin, 'idDrone': idDrone})
                        primero = False

                    # Comprobar si está cerca del objetivo
                    if fn.esta_cerca(next_pose, final_pose, 0.7, 0.05):
                        rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose, 'idDrone': idDrone})
                        # rospy.loginfo({'action': 'start_trajectory', 'final_pose': final_pose})
                        # rospy.loginfo({'tIW': fn.loc(tIW)})
                        # rospy.loginfo({'tFI': fn.loc(tFI)})
                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0
                        rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone, 'idDrone': idDrone})

            i += 1


    def espiral(self, idDrone, initial_pose, tFI):
        rospy.loginfo({'action': 'ESPIRAL'})

        ## ocho
        pasos = 4
        for i in range(pasos):
            # Ir hacia arriba, adelante y girar 180 grados
            next = False
            initial_pose = self.read_pose(idDrone)
            primero = True
            self.set_4_speed(0, 30, 15, 80, idDrone) # gira a la derecha

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    # Posicion inicial respecto al mundo:
                    tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                                fn.norm_pi(fn.giro_to_rad(initial_pose.pose.orientation.z)))
                    
                    # Posicion final respecto al dron:
                    tFI = fn.hom(0.6, 0, 0.3, -np.pi)

                    # Posicion final respecto al mundo:
                    tFW = np.dot(tIW, tFI)
                    final_pose = fn.loc(tFW)

                    if primero:
                        posefin = [final_pose[0], final_pose[1], final_pose[2], fn.rad_to_giro(fn.norm_pi(final_pose[3]))]
                        rospy.loginfo({'action': 'start_trajectory', 'final_pose': posefin})
                        primero = False

                    # Comprobar si está cerca del objetivo
                    if fn.esta_cerca(next_pose, final_pose, 0.5, 0.1):
                        rospy.loginfo({'action': 'start_trajectory', 'next_pose': next_pose})
                        rospy.loginfo({'action': 'start_trajectory', 'final_pose': final_pose})
                        rospy.loginfo({'tIW': fn.loc(tIW)})
                        rospy.loginfo({'tFI': fn.loc(tFI)})
                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0
                        rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone})
    


    # def receive_video(self, stop_event, pipe_in, host_ip, video_port):
    #     with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock_video:
    #         sock_video.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #         sock_video.settimeout(.5)
    #         v = sock_video.bind((host_ip, video_port))
    #         data = bytearray(2048)
    #         while not stop_event.is_set():
    #             try:
    #                 size, addr = sock_video.recvfrom_into(data)
    #                 # logger.info({'action': 'receive_video', 'data': data})
    #                 # <node pkg="tf" type="static_transform_publisher" name="broadcaster_odom" args="$0 0 0 1.57 0 3.14 odom odom_ned 100" />
    #             except socket.timeout as ex:
    #                 logger.warning({'action': 'receive_video', 'ex': ex, 'port': video_port, 'host_ip': host_ip})
    #                 rospy.loginfo({'action': 'receive_video_wr', 'ex': ex})
    #                 time.sleep(0.5)
    #                 continue
    #             except socket.error as ex:
    #                 logger.error({'action': 'receive_video', 'ex': ex})
    #                 rospy.loginfo({'action': 'receive_video_err', 'ex': ex})
    #                 break

    #             try:
    #                 pipe_in.write(data[:size])
    #                 pipe_in.flush()
    #             except Exception as ex:
    #                 logger.error({'action': 'receive_video', 'ex': ex})
    #                 rospy.loginfo({'action': 'receive_video_err2', 'ex': ex})
    #                 break

    # def video_binary_generator(self):
    #     while True:
    #         try:
    #             frame = self.proc_stdout.read(FRAME_SIZE)
    #         except Exception as ex:
    #             logger.error({'action': 'video_binary_generator', 'ex': ex})
    #             continue

    #         if not frame:
    #             continue

    #         frame = np.fromstring(frame, np.uint8).reshape(FRAME_Y, FRAME_X, 3)
    #         yield frame

    # def enable_face_detect(self):
    #     self._is_enable_face_detect = True

    # def disable_face_detect(self):
    #     self._is_enable_face_detect = False

    # def video_jpeg_generator(self):
    #     for frame in self.video_binary_generator():
    #         if self._is_enable_face_detect:
    #             if self.is_patrol:
    #                 self.stop_patrol()

    #             gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    #             faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
    #             for (x, y, w, h) in faces:
    #                 cv.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    #                 face_center_x = x + (w / 2)
    #                 face_center_y = y + (h / 2)
    #                 diff_x = FRAME_CENTER_X - face_center_x
    #                 diff_y = FRAME_CENTER_Y - face_center_y
    #                 face_area = w * h
    #                 percent_face = face_area / FRAME_AREA

    #                 drone_x, drone_y, drone_z, speed = 0, 0, 0, self.speed
    #                 if diff_x < -30:
    #                     drone_y = -30
    #                 if diff_x > 30:
    #                     drone_y = 30
    #                 if diff_y < -15:
    #                     drone_z = -30
    #                 if diff_y > 15:
    #                     drone_z = 30
    #                 if percent_face > 0.30:
    #                     drone_x = -30
    #                 if percent_face < 0.02:
    #                     drone_x = 30
    #                 self.send_command(f'go {drone_x} {drone_y} {drone_z} {speed}',
    #                                   blocking=False)
    #                 break

    #         _, jpeg = cv.imencode('.jpg', frame)
    #         jpeg_binary = jpeg.tobytes()

    #         if self.is_video:
    #             # logger.info({'action': 'write_frame', 'frame': or_frame})
    #             self.video_out.write(frame)

    #         if self.take_frames:
    #             backup_file = "frame" + str(self.numFrame) + '.jpg'
    #             file_path = os.path.join(
    #                 self.folder, backup_file
    #             )
    #             current_time = time.time()
    #             dif_time = current_time - self.frame_time
    #             self.frame_time = current_time
    #             with open(file_path, 'wb') as f:
    #                 f.write(jpeg_binary)
    #             self.frame_total_time += dif_time
    #             fichero = open(self.file_frames, 'a')
    #             fichero.write("Tiempo frame  " + str(self.numFrame) + ": " + str(dif_time) + "\n")
    #             fichero.close()
    #             self.numFrame += 1

    #         if self.is_snapshot:
    #             backup_file = time.strftime("%Y%m%d-%H%M%S") + '.jpg'
    #             snapshot_file = 'snapshot.jpg'
    #             for filename in (backup_file, snapshot_file):
    #                 file_path = os.path.join(
    #                     SNAPSHOTS_IMAGE_FOLDER, filename
    #                 )
    #                 with open(file_path, 'wb') as f:
    #                     f.write(jpeg_binary)
    #             self.is_snapshot = False

    #         yield jpeg_binary

    # def snapshot(self):
    #     self.is_snapshot = True
    #     retry = 0
    #     while retry < 3:
    #         if not self.is_snapshot:
    #             return True
    #         time.sleep(0.1)
    #         retry += 1
    #     return False

    # def takeVideo(self):
    #     if not self.is_video:
    #         backup_file = time.strftime("%Y%m%d-%H%M%S") + '.avi'
    #         file_path = os.path.join(
    #             VIDEOS_IMAGE_FOLDER, backup_file
    #         )
    #         self.video_out = cv.VideoWriter(file_path,
    #                                         cv.VideoWriter_fourcc('M', 'J', 'P', 'G'),
    #                                         30.0, (FRAME_X, FRAME_Y))
    #         self.ini_video = time.time()
    #         self.is_video = True
    #         logger.info({'action': 'take_video'})

    # def stopVideo(self):
    #     logger.info({'action': 'stop_video'})
    #     self.is_video = False
    #     self.video_out.release()
    #     self.fin_video = time.time()
    #     tiempo = self.fin_video - self.ini_video
    #     logger.info({'Tiempo tomar video': tiempo})

    # def takeFrames(self):
    #     self.dir_frames = 'frames' + time.strftime("%Y%m%d-%H%M%S")
    #     self.numFrame = 0
    #     self.folder = FRAMES_IMAGE_FOLDER + self.dir_frames
    #     os.mkdir(self.folder)
    #     self.file_frames = FRAMES_IMAGE_FOLDER + self.dir_frames + "times.txt"
    #     fichero = open(self.file_frames, 'w')
    #     fichero.close()
    #     self.frame_time = time.time()
    #     self.frame_total_time = 0
    #     self.take_frames = True

    # def stopFrames(self):
    #     self.stop_frames = False
    #     med_time = self.frame_total_time / float(self.numFrame)
    #     fichero = open(self.file_frames, 'a')
    #     fichero.write("Tiempo medio por frame: " + str(med_time) + '\n')
    #     fichero.close()


