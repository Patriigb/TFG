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
import std_msgs.msg
import nav_msgs.msg

from pytello.droneapp.models.base import Singleton
import pytello.droneapp.models.functions as fn
import pytello.droneapp.models.trajectories as tr

import visualization_msgs.msg
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from geometry_msgs.msg import Twist


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
                 drone_port=8889, drones=['192.168.0.116']):
        self.host_ip = host_ip
        self.host_port = host_port
        # self.drone_ip = drone_ip
        self.drone_port = drone_port
        # self.drone_address = (drone_ip, drone_port)
        # self.is_imperial = is_imperial
        self.speed = DEFAULT_SPEED
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
        self.pubs_velocity = [None] * len(self.drones)
        self.pubs_a = [None] * len(self.drones)
        self.pubs_b = [None] * len(self.drones)

        for i in range(len(self.drones)):
            # Suscribirse al topic /optitrack/pose para recibir la posición del dron
            # y publicar la posición en el topic /tello_n/path
            sub = rospy.Subscriber("/optitrack/pose_" + str(i + 1), geometry_msgs.msg.PoseStamped, self.update_position, i, queue_size=1)
            pub = rospy.Publisher("/tello_" + str(i + 1) + "/path", nav_msgs.msg.Path, queue_size=10)
            self.subs[i] = sub
            self.pubs[i] = pub

            pub_draw = rospy.Publisher("/draw_trajectory" + str(i + 1), visualization_msgs.msg.Marker, queue_size=10)
            self.pubs_draw[i] = pub_draw

            pub_velocity = rospy.Publisher("/tello_" + str(i + 1) + "/velocity", Twist, queue_size=10)
            self.pubs_velocity[i] = pub_velocity

            pub_a = rospy.Publisher("/tello_" + str(i + 1) + "/a", std_msgs.msg.Float64, queue_size=10)
            self.pubs_a[i] = pub_a

            pub_b = rospy.Publisher("/tello_" + str(i + 1) + "/b", std_msgs.msg.Float64, queue_size=10)
            self.pubs_b[i] = pub_b

        
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
        # rospy.loginfo({'action': 'set_4_speed', 'lr': lr, 'fb': fb, 'ud': ud, 'yw': yw})
        

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
                # Si el comando es rc ... no se muestra
                if not command.startswith('rc'):
                    rospy.loginfo({'action2': 'send_command', 'command': command, 'ip': drone_address})
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
        # if self.is_imperial:
        #     distance = int(round(distance * 30.48))
        # else:
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
    

    def draw(self, idDrone, position=None):
        if position is None:
            position = self.read_pose(idDrone)

        markers = tr.draw_trajectory_8(position, 0.26, 4, idDrone)  
        self.pubs_draw[idDrone - 1].publish(markers)

        # markers = tr.draw_trajectory_cuadrado(position, 1, 10, idDrone)
        # self.pubs_draw[idDrone - 1].publish(markers)

        # markers = tr.draw_trajectory_circulo(position, 0.5, 5, idDrone)
        # self.pubs_draw[idDrone - 1].publish(markers)

        # markers = tr.draw_trajectory_ovalo(position, 0.5, 0.3, 5, idDrone)
        # self.pubs_draw[idDrone - 1].publish(markers)

        # markers = tr.draw_trajectory_espiral(position, 0.5, 0.1, 10, idDrone)
        # self.pubs_draw[idDrone - 1].publish(markers)


    def start_trajectory(self, idDrone, idTrajectory=0, version=0): # idDrone es el número del dron, no puede ser 0
        try:
            self.takeoff(idDrone)
            initial_pose = [None, None]
            time.sleep(5) 
            rospy.loginfo("Despegado")
            self.wait_for_position(idDrone)
            initial_pose[idDrone - 1] = self.read_pose(idDrone)

            # Cambiar signo a la orientación
            # initial_pose[idDrone - 1].pose.orientation.z = -1 * initial_pose[idDrone - 1].pose.orientation.z
            rospy.loginfo({'action': 'start_trajectory', 'initial_pose': initial_pose[idDrone - 1], 'idDrone': idDrone})
            
            # Crear fichero para log
            f = open("log " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
            
        except Exception as e:
            rospy.loginfo({'action': 'start_trajectory', 'error': str(e)})
            return
        
        if version == 0:
            if idTrajectory == 0:
                self.cuadrado_open_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 1:
                self.circulo_open_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 2:
                self.ovalo_open_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 3:
                self.espiral_open_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 4:
                self.ocho_open_loop(idDrone, initial_pose[idDrone - 1], f)
        elif version == 1:
            if idTrajectory == 0:
                self.cuadrado_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 1:
                self.circulo_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 2:
                self.ovalo_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 3:
                self.espiral_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 4:
                self.ocho_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
        elif version == 2:
            if idTrajectory == 0:
                self.cuadrado_closed_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 1:
                self.circulo_closed_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 2:
                self.ovalo_closed_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 3:
                self.espiral_closed_loop(idDrone, initial_pose[idDrone - 1], f)
            elif idTrajectory == 4:
                self.ocho_closed_loop(idDrone, initial_pose[idDrone - 1], f)
    

        # self.ocho_sin_recalcular(idDrone, initial_pose[idDrone - 1],f)
        # self.ocho_open_loop(idDrone, initial_pose[idDrone - 1], f)
        # self.ocho_closed_loop(idDrone, initial_pose[idDrone - 1], f)

        # self.cuadrado_open_loop(idDrone, initial_pose[idDrone - 1], f)
        # self.cuadrado_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
        # self.cuadrado_closed_loop(idDrone, initial_pose[idDrone - 1], f)

        # self.circulo_open_loop(idDrone, initial_pose[idDrone - 1], f)
        # self.circulo_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
        # self.circulo_closed_loop(idDrone, initial_pose[idDrone - 1], f)

        # self.ovalo_open_loop(idDrone, initial_pose[idDrone - 1], f)
        # self.ovalo_sin_recalcular(idDrone, initial_pose[idDrone - 1], f)
        # self.ovalo_closed_loop(idDrone, initial_pose[idDrone - 1], f)

        # self.espiral_open_loop(idDrone, initial_pose[idDrone - 1], f)

        f.close()

        rospy.loginfo("Fin")

        self.set_4_speed(0, 0, 0, 0, idDrone)
        self.land(idDrone)
        

    def stop_trajectory(self, idDrone):
        rospy.loginfo({'action': 'stop_trajectory'})
        # Poner trajectory a false para que el dron pare de hacer la trayectoria
        self.set_4_speed(0, 0, 0, 0, idDrone)
        self.land(idDrone)


    def ocho_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OCHO'}) # 0.30 m/s 1.152 rad/s radio 0.26 m
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        if idDrone == 2:
            self.set_4_speed(0, 0, 30, 0, idDrone)
            time.sleep(3)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(1)
        else:
            time.sleep(4)

        initial_pose = self.read_pose(idDrone)

        ## ocho sin recalcular (con los puntos ya calculados)
        next_pose = initial_pose

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        # f.write("Pose: " + str(fn.loc(tIW)) + "\n")
        
        # Puntos respecto al dron:
        # tFI = [[0, 0.52, 0, np.pi], [0, 1.04, 0, 0], [0, 0.52, 0, np.pi], [0, 0, 0, 0]]
        
        # # Posiciones finales respecto al mundo:
        # tFW = [np.dot(tIW, fn.hom(t[0], t[1], t[2], t[3])) for t in tFI]
        # final_poses = [fn.loc(t) for t in tFW]
        # rospy.loginfo({'action': 'start_trajectory', 'final_poses': final_poses, 'idDrone': idDrone})

        final_poses = tr.calcular_puntos_ocho(fn.loc(tIW), 0.25, 3)
        index = 0
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1

        markers = tr.draw_trajectory_8(initial_pose, 0.25, 3, idDrone)   
        self.pubs_draw[idDrone - 1].publish(markers)     

        index = 0

        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            initial_pose = next_pose
            # if 0 <= i < 4 or 12 <= i <= 16:
            if 0 <= i <= 3 or 9 < i <= 12:
                self.set_4_speed(0, 26, 0, -80, idDrone) # gira a la izquierda
            else:
                self.set_4_speed(0, 20, 0, 75, idDrone) # gira a la derecha

            # min_dist = fn.distancia(self.read_pose(idDrone), final_poses[i])
            # disminuye = True
            min_dist = 100
            min_diff_orientation = 100

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:

                    # Comprobar si está cerca del objetivo
                    # if fn.esta_cerca(next_pose, final_poses[i], 0.5, 0.2):
                    #     rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose, 'idDrone': idDrone})
                    #     next = True
                    #     timeDrone = rospy.Time.now().to_sec() - t0
                    #     rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone, 'idDrone': idDrone})

                    # aumenta, dist = fn.distancia_menor(next_pose, final_poses[i], min_dist)
                    # if aumenta:
                    #     # rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose, 'idDrone': idDrone, 'min_dist': min_dist})
                    #     next = True
                    #     timeDrone = rospy.Time.now().to_sec() - t0
                    #     # rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone, 'idDrone': idDrone})
                    # else:
                    #     min_dist = dist
                    #     rospy.loginfo("Distancia: " + str(min_dist) + " hasta " + str(i))

                    dist = fn.distancia(next_pose, final_poses[i])
                    rospy.loginfo("Distancia: " + str(dist) + " hasta " + str(i))

                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    # if dist < min_dist:
                    #     min_dist = dist

                    # if dist < 0.2 or dist >= min_dist + 0.3 or fn.esta_cerca(next_pose, final_poses[i], 0.3, 0.1):
                    #     # rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose, 'idDrone': idDrone})
                    #     next = True
                    #     timeDrone = rospy.Time.now().to_sec() - t0
                    #     # rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone, 'idDrone': idDrone})

                    diff_orientation = fn.diff_orientation(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), final_poses[i][3])
                    rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + str(i) + " " + str(round(final_poses[i][3], 2)) + " " + str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                    if diff_orientation < min_diff_orientation:
                        min_diff_orientation = diff_orientation

                    if fn.esta_cerca(next_pose, final_poses[i], 0.1, 0.5) or diff_orientation < 0.1:
                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0

            index += 1 
            i = (i + 1) % 13
            if i == 0:
                fin = True

        f2.close()

    def cuadrado_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CUADRADO'}) # 0.30 m/s 1.152 rad/s radio 0.26 m
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        if idDrone == 1:
            self.set_4_speed(0, 0, 30, 0, idDrone)
            time.sleep(3)
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(1)
        else:
            self.set_4_speed(0, 0, 0, 0, idDrone)
            time.sleep(3)

        initial_pose = self.read_pose(idDrone)

        ## ocho sin recalcular (con los puntos ya calculados)
        next_pose = initial_pose

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                     fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
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

            # min_dist = fn.distancia(self.read_pose(idDrone), final_poses[i])
            # disminuye = True
            min_dist = 100

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    dist = fn.distancia(next_pose, final_poses[i])
                    # rospy.loginfo("Distancia: " + str(dist) + " hasta " + str(final_poses[i]))
                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    if dist < min_dist:
                        min_dist = dist

                    if dist < 0.2 or dist >= min_dist + 0.1 or fn.esta_cerca(next_pose, final_poses[i], 0.2, 0.5):
                        # rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose, 'idDrone': idDrone})
                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0
                        self.set_4_speed(0, 0, 0, 0, idDrone)
                        time.sleep(1)
                        # rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone, 'idDrone': idDrone})

            self.set_4_speed(0, 0, 0, -50, idDrone)

            t0 = rospy.Time.now().to_sec()
            next = False
            min_diff_orientation = 100
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:

                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")

                    diff_orientation = fn.diff_orientation(abs(fn.norm_pi(fn.yaw(next_pose.pose.orientation))), abs(final_poses[i][3]))
                    rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + str(i) + " " + str(round(final_poses[i][3], 2)) + " " + str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                    if diff_orientation < min_diff_orientation:
                        min_diff_orientation = diff_orientation

                    if diff_orientation < 0.001 or diff_orientation >= min_diff_orientation + 0.05:
                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0
                        # self.set_4_speed(0, 0, 0, 0, idDrone)
                        # time.sleep(1)

                    # if fn.esta_cerca(next_pose, final_poses[i], 0.5, 0.01):
                    #     # rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose, 'idDrone': idDrone})
                    #     # rospy.loginfo({'final': final_poses[i]})
                    #     next = True
                    #     timeDrone = rospy.Time.now().to_sec() - t0
                    #     # rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone, 'idDrone': idDrone})
            
            index += 1
            i = (i + 1) % 4
            if i == 0:
                fin = True
                print(next_pose)

        f2.close()

    def circulo_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CIRCULO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        # if idDrone == 1:
        #     self.set_4_speed(0, 0, 30, 0, idDrone)
        #     time.sleep(3)
        #     self.set_4_speed(0, 0, 0, 0, idDrone)
        #     time.sleep(1)
        # else:
        #     self.set_4_speed(0, 0, 0, 0, idDrone)
        #     time.sleep(3)

        # initial_pose = self.read_pose(idDrone)

        ## circulo sin recalcular (con los puntos ya calculados)
        next_pose = initial_pose

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z,
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # # Puntos respecto al dron:
        # tFI = [[0.25, 0.25, 0, np.pi/4],[0.5, 0.5, 0, np.pi/2], [0.75, 0.25, 0, np.pi/2 + np.pi/4],
        #        [0, 1, 0, np.pi], [-0.25, 0.75, 0, -np.pi + np.pi/4],[-0.5, 0.5, 0, -np.pi/2], 
        #        [-0.25, 0.25, 0, -np.pi/4],[0, 0, 0, 0]]

        # # Posiciones finales respecto al mundo:
        # index = 0
        # tFW = [np.dot(tIW, fn.hom(t[0], t[1], t[2], t[3])) for t in tFI]
        # final_poses = [fn.loc(t) for t in tFW]

        index = 0
        final_poses = tr.calcular_puntos_circulo(fn.loc(tIW), 0.5, 4)
        rospy.loginfo({'action': 'start_trajectory', 'final_poses': final_poses, 'idDrone': idDrone})
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1

        markers = tr.draw_trajectory_circulo(initial_pose, 0.5, 4, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        index = 0
        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            initial_pose = next_pose
            self.set_4_speed(0, 25, 0, -46, idDrone) 

            # min_dist = fn.distancia(self.read_pose(idDrone), final_poses[i])
            # disminuye = True
            min_diff_orientation = 100

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    
                    diff_orientation = fn.diff_orientation(abs(fn.norm_pi(fn.yaw(next_pose.pose.orientation))), abs(final_poses[i][3]))
                    rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + str(i) + " " + str(round(final_poses[i][3], 2)) + " " + str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                    if diff_orientation < min_diff_orientation:
                        min_diff_orientation = diff_orientation

                    if diff_orientation < 0.001 or diff_orientation >= min_diff_orientation + 0.05 or fn.esta_cerca(next_pose, final_poses[i], 0.2, 0.1):
                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0

            index += 1
            i = (i + 1) % 9
            if i == 0:
                fin = True

        f2.close()

    
    def ovalo_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OVALO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        ## ovalo sin recalcular (con los puntos ya calculados)
        next_pose = initial_pose

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z,
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # # Puntos respecto al dron:
        # tFI = [[1, 0, 0, 0], [1.5, 0.5, 0, np.pi/2], [1, 1, 0, np.pi], [0, 1, 0, np.pi], [-0.5, 0.5, 0, -np.pi/2], [0, 0, 0, 0]]

        # # Posiciones finales respecto al mundo:
        # index = 0
        # tFW = [np.dot(tIW, fn.hom(t[0], t[1], t[2], t[3])) for t in tFI]
        # final_poses = [fn.loc(t) for t in tFW]

        index = 0
        final_poses = tr.calcular_puntos_ovalo(fn.loc(tIW), 1, 0.5, 5)
        rospy.loginfo({'action': 'start_trajectory', 'final_poses': final_poses, 'idDrone': idDrone})
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1

        markers = tr.draw_trajectory_ovalo(initial_pose, 1, 0.5, 5, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        index = 0
        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            # if i == 0 or i == 1 or i == 2 or i == 7 or i == 8 or i == 9: # Avanza en línea recta
            if 0 <= i <= 3 or 12 <= i <= 14: # Avanza en línea recta
                next = False
                initial_pose = next_pose
                self.set_4_speed(0, 25, 0, 0, idDrone)

                min_dist = 100

                # Medir tiempo
                t0 = rospy.Time.now().to_sec()
                while not next:
                    next_pose = self.read_pose(idDrone)
                    
                    if next_pose is not None:
                        dist = fn.distancia(next_pose, final_poses[i])
                        # rospy.loginfo("Distancia: " + str(dist) + " hasta " + str(final_poses[i]))
                        position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z, 
                        fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                        f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                                + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                        if dist < min_dist:
                            min_dist = dist

                        if dist < 0.01 or dist >= min_dist + 0.1 or fn.esta_cerca(next_pose, final_poses[i], 0.05, 0.1):
                            # rospy.loginfo({'action': 'start_trajectory_fin', 'next_pose': next_pose, 'idDrone': idDrone})
                            next = True
                            timeDrone = rospy.Time.now().to_sec() - t0
                            # rospy.loginfo({'action': 'start_trajectory', 'time': timeDrone, 'idDrone': idDrone})
            
            else: # Gira
                next = False
                initial_pose = next_pose
                self.set_4_speed(0, 25, 0, -47, idDrone)

                min_diff_orientation = 100

                # Medir tiempo
                t0 = rospy.Time.now().to_sec()
                while not next:
                    next_pose = self.read_pose(idDrone)
                    
                    if next_pose is not None:
                        position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z, 
                        fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                        f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                                + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")

                        diff_orientation = fn.diff_orientation(abs(fn.norm_pi(fn.yaw(next_pose.pose.orientation))), abs(final_poses[i][3]))
                        # rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + str(i) + " " + str(round(final_poses[i][3], 2)) + " " + str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                        if diff_orientation < min_diff_orientation:
                            min_diff_orientation = diff_orientation

                        if diff_orientation < 0.001 or diff_orientation >= min_diff_orientation + 0.1 or fn.esta_cerca(next_pose, final_poses[i], 0.2, 0.1):
                            next = True
                            timeDrone = rospy.Time.now().to_sec() - t0

            index += 1
            i = (i + 1) % 22
            if i == 0:
                fin = True

        f2.close()

    def espiral_sin_recalcular(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'ESPIRAL'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

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


        initial_pose = self.read_pose(idDrone)

        ## circulo sin recalcular (con los puntos ya calculados)
        next_pose = initial_pose

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z,
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))

        index = 0
        final_poses = tr.calcular_puntos_espiral(fn.loc(tIW), 0.5, 0.05, 5)
        rospy.loginfo({'action': 'start_trajectory', 'final_poses': final_poses, 'idDrone': idDrone})
        for p in final_poses:
            x, y, z, yaw = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(yaw) + " 0\n")
            index += 1

        markers = tr.draw_trajectory_espiral(initial_pose, 0.5, 0.05, 5, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        index = 0
        i = 0
        fin = False
        while not fin:
            rospy.loginfo("Paso " + str(i))
            next = False
            initial_pose = next_pose
            self.set_4_speed(0, 24, 15, -47, idDrone) 

            # min_dist = fn.distancia(self.read_pose(idDrone), final_poses[i])
            # disminuye = True
            min_diff_orientation = 100

            # Medir tiempo
            t0 = rospy.Time.now().to_sec()
            while not next:
                next_pose = self.read_pose(idDrone)
                
                if next_pose is not None:
                    position = fn.loc(fn.hom(next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(next_pose.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    
                    diff_orientation = fn.diff_orientation(abs(fn.norm_pi(fn.yaw(next_pose.pose.orientation))), abs(final_poses[i][3]))
                    rospy.loginfo("Diferencia de orientación: " + str(diff_orientation) + " paso " + str(i) + " " + str(round(final_poses[i][3], 2)) + " " + str(round(fn.norm_pi(fn.yaw(next_pose.pose.orientation)), 2)))
                    if diff_orientation < min_diff_orientation:
                        min_diff_orientation = diff_orientation

                    if diff_orientation < 0.001 or diff_orientation >= min_diff_orientation + 0.05 or fn.esta_cerca(next_pose, final_poses[i], 0.2, 0.5):
                        next = True
                        timeDrone = rospy.Time.now().to_sec() - t0

            index += 1
            i = (i + 1) % 22
            if i == 0:
                fin = True

        f2.close()


    def ocho_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OCHO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
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

        initial_pose = self.read_pose(idDrone)

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        pts = tr.calcular_puntos_ocho(fn.loc(tIW), 0.26, 4)
        index = 0
        for p in pts:
            # rospy.loginfo({'action': 'start_trajectory', 'pose': p})
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1
        # t_delay = 1.2
        index = 0

        markers = tr.draw_trajectory_8(initial_pose, 0.26, 4, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        # ocho calculando el tiempo según la velocidad lineal y angular
        v = 0.30 # 0.30 m/s
        w = 1.152 # 1.152 rad/s
        r = v / w

        # Tiempo para hacer media circunferencia
        t = np.pi * r / v # - t_delay
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
                # rospy.loginfo("Gira a la izquierda")
            else:
                self.set_4_speed(0, v * 100 / 1.20, 0, w * 100 / 1.44, idDrone) # gira a la derecha
                # rospy.loginfo("Gira a la derecha")

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

    def cuadrado_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CUADRADO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
        # if idDrone == 1:
        #     self.set_4_speed(0, 0, 30, 0, idDrone)
        #     time.sleep(3)
        #     self.set_4_speed(0, 0, 0, 0, idDrone)
        #     time.sleep(1)
        # else:
        #     time.sleep(4)

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

        num_puntos = 10
        initial_pose = self.read_pose(idDrone)

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        pts = tr.calcular_puntos_cuadrado(fn.loc(tIW), 1, 10)
        index = 0
        for p in pts:
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1

        markers = tr.draw_trajectory_cuadrado(initial_pose, 1, 10, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Cuadrado calculando el tiempo según la velocidad lineal y angular
        v = 0.30 # 0.30 m/s     0.25 = 0.30 m/s  1 = 1.20 m/s
        w = 1.152 # 1.152 rad/s
        m = 1
        
        # Tiempo para hacer un lado
        t = m / v # - t_delay
        rospy.loginfo({'tiempo': t})

        t_log = t / num_puntos

        # Tiempo para girar 90 grados
        t2 = np.pi / 2 / w 
        rospy.loginfo({'tiempo2': t2})
    
        index = 0

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

            # index += 1
            i = (i + 1) % 4
            if i == 0:
                fin = True

    def circulo_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CIRCULO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
        # if idDrone == 1:
        #     self.set_4_speed(0, 0, 30, 0, idDrone)
        #     time.sleep(3)
        #     self.set_4_speed(0, 0, 0, 0, idDrone)
        #     time.sleep(1)
        # else:
        #     time.sleep(4)

        initial_pose = self.read_pose(idDrone)

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z,
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        pts = tr.calcular_puntos_circulo(fn.loc(tIW), 0.5, 10)
        index = 0
        for p in pts:
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_circulo(initial_pose, 0.5, 10, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Circulo calculando el tiempo según la velocidad lineal y angular
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


    def ovalo_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OVALO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
        # if idDrone == 1:
        #     self.set_4_speed(0, 0, 30, 0, idDrone)
        #     time.sleep(3)
        #     self.set_4_speed(0, 0, 0, 0, idDrone)
        #     time.sleep(1)
        # else:
        #     time.sleep(4)

        initial_pose = self.read_pose(idDrone)

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z,
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        pts = tr.calcular_puntos_ovalo(fn.loc(tIW), 1, 0.5, 5)
        index = 0
        for p in pts:
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1

        markers = tr.draw_trajectory_ovalo(initial_pose, 1, 0.5, 5, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Ovalo calculando el tiempo según la velocidad lineal y angular
        v = 0.30
        w = 0.60 # 1.152 rad/s
        m = 1
        
        # Tiempo para hacer un lado
        t1 = m / v # - t_delay
        rospy.loginfo({'tiempo1': t1})

        # Tiempo para girar medio circulo
        t2 = np.pi / w
        rospy.loginfo({'tiempo2': t2})

        index = 0

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

    def espiral_open_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'ESPIRAL'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
        # if idDrone == 1:
        #     self.set_4_speed(0, 0, 30, 0, idDrone)
        #     time.sleep(3)
        #     self.set_4_speed(0, 0, 0, 0, idDrone)
        #     time.sleep(1)
        # else:
        #     time.sleep(4)

        initial_pose = self.read_pose(idDrone)

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z,
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        pts = tr.calcular_puntos_espiral(fn.loc(tIW), 0.5, 0.05, 5)
        index = 0
        for p in pts:
            f.write(str(index) + " " + str(p[0]) + " " + str(p[1]) 
                    + " " + str(p[2]) + " 0 0 " + str(p[3]) + " 0\n")
            index += 1
        index = 0

        markers = tr.draw_trajectory_espiral(initial_pose, 0.5, 0.05, 5, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Espiral calculando el tiempo según la velocidad lineal y angular
        v = 0.30
        w = 0.60 # 1.152 rad/s
        r = v / w

        # Tiempo para hacer una circunferencia
        t = 2 * np.pi * r / v
        rospy.loginfo({'tiempo': t})

        # Velocidad para subir 1 m en t segundos
        v_z = 1 / t
        rospy.loginfo({'velocidad_z': v_z})

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




    def ocho_closed_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OCHO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")

        vmax = 0.30
        wmax = 0.80
        pmax = 1
        amax = np.pi
        bmax = np.pi

        # kp > 0;  kb > 0;  ka - kp > 0
        kp = vmax/pmax
        ka = wmax/amax
        kb = wmax/bmax

        kp = 0.3
        ka = 0.2
        kb = 0.2

        index = 0
        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        # f.write(str(index) + " " + str(fn.loc(tIW)) + "\n")
        
        # Posiciones finales en el mundo:
        pos = tr.calcular_puntos_ocho(fn.loc(tIW), 0.26, 4)

        pos = [(1,1,0.8,np.pi/2), (1,2,0.8,np.pi/2), (1,3,0.8,np.pi/2)]
        index = 0
        for p in pos:
            x, y, z, w = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(w) + " 0\n")
            index += 1
        primero = True

        index = 0

        pos_ant = None
        w_ant = 0
        v_ant = 0
        for i in range(0, len(pos)):
            WxG = pos[i]
            
            esta = False
            primero = True
            while not esta:
                WxR = self.read_pose(idDrone)

                if fn.esta_cerca(WxR, WxG, 0.2, 0.5):
                    rospy.loginfo("CERCA")
                    esta = True
                    index += 1
                else:
                    v, w, GxR, primero = fn.calcularVelocidades(WxR, WxG, kp, ka, kb, primero)
                    # if abs(v - v_ant) > 1:
                    position = fn.loc(fn.hom(WxR.pose.position.x, WxR.pose.position.y, WxR.pose.position.z, 
                                             fn.norm_pi(fn.yaw(WxR.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                             + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    
                    primero = True
                    pos_ant = WxR
                    w_ant = w
                    v_ant = v

                    self.set_4_speed(0, round(v,2), 0, round(w, 2), idDrone) 

                    time.sleep(0.4)

        f2.close()

    def cuadrado_closed_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'CUADRADO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
        index = 0

        # kp > 0;  kb < 0;  ka - kp > 0
        kp = 0.3
        ka = 0.5
        kb =  -0.5

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Posiciones finales en el mundo:
        pos = tr.calcular_puntos_cuadrado(fn.loc(tIW), 1, 1)
        # pos = [(1, -1, 0.8, 0)]
        markers = tr.draw_trajectory_cuadrado(initial_pose, 1, 1, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Escribir log
        index = 0
        for p in pos:
            x, y, z, w = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(w) + " 0\n")
            index += 1
        index = 0

        t_send_velocities = 2

        for i in range(0, len(pos)):
            t0 = rospy.Time.now().to_sec()
            WxG = pos[i]
            esta = False

            while not esta:
                t1 = rospy.Time.now().to_sec()
                if abs(t1 - t0) > t_send_velocities: # Cada 2 o 1 s
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
                        v, w, GxR, a, b = fn.calcularVelocidades(WxR, WxG, kp, ka, kb)
                        w = -1 * w

                        # Escribir log
                        position = fn.loc(fn.hom(WxR.pose.position.x, WxR.pose.position.y, WxR.pose.position.z, 
                                                fn.norm_pi(fn.yaw(WxR.pose.orientation))))
                        f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                                + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                        
                        rospy.loginfo({'v': v, 'w': w})
                        
                        # Publicar velocidad, alpha y beta
                        msg = Twist()
                        msg.linear.x = v
                        msg.angular.z = w
                        self.pubs_velocity[idDrone - 1].publish(msg)
                        self.pubs_a[idDrone - 1].publish(a)
                        self.pubs_b[idDrone - 1].publish(b)

                        self.set_4_speed(0, v, 0, w, idDrone) 
                        t0 = rospy.Time.now().to_sec()

                    rospy.loginfo("\n")

        f2.close()


    def ovalo_closed_loop(self, idDrone, initial_pose, f):
        rospy.loginfo({'action': 'OVALO'})
        f2 = open("log2 " + str(idDrone) + "-" + time.strftime("%Y%m%d-%H%M%S") + ".txt", "w")
        index = 0

        # kp > 0;  kb < 0;  ka - kp > 0
        kp = 0.6
        ka = 0.5
        kb =  -0.5

        # Posicion inicial respecto al mundo:
        tIW = fn.hom(initial_pose.pose.position.x, initial_pose.pose.position.y, initial_pose.pose.position.z, 
                    fn.norm_pi(fn.yaw(initial_pose.pose.orientation)))
        
        # Posiciones finales en el mundo:
        pos = tr.calcular_puntos_ovalo(fn.loc(tIW), 1, 0.5, 2)
        markers = tr.draw_trajectory_ovalo(initial_pose, 1, 0.5, 2, idDrone)
        self.pubs_draw[idDrone - 1].publish(markers)

        # Escribir log
        index = 0
        for p in pos:
            x, y, z, w = p
            f.write(str(index) + " " + str(x) + " " + str(y) + " " + str(z) + " 0 0 " + str(w) + " 0\n")
            index += 1
        index = 0

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
                    v, w, GxR, a, b = fn.calcularVelocidades(WxR, WxG, kp, ka, kb)
                    w = -1 * w

                    # Escribir log
                    position = fn.loc(fn.hom(WxR.pose.position.x, WxR.pose.position.y, WxR.pose.position.z, 
                                            fn.norm_pi(fn.yaw(WxR.pose.orientation))))
                    f2.write(str(index) + " " + str(position[0]) + " " + str(position[1]) 
                            + " " + str(position[2]) + " 0 0 " + str(position[3]) + " 0\n")
                    
                    rospy.loginfo({'v': v, 'w': w})
                    
                    # Publicar velocidad, alpha y beta
                    msg = Twist()
                    msg.linear.x = v
                    msg.angular.z = w
                    self.pubs_velocity[idDrone - 1].publish(msg)
                    self.pubs_a[idDrone - 1].publish(a)
                    self.pubs_b[idDrone - 1].publish(b)

                    if abs(t1 - t0) > t_send_velocities: # Cada 2 o 1 s

                        self.set_4_speed(0, v, 0, w, idDrone) 
                        t0 = rospy.Time.now().to_sec()

                    rospy.loginfo("\n")

        f2.close()

