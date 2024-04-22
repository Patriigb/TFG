import logging

from flask import jsonify
from flask import render_template
from flask import request
from flask import Response

from pytello.droneapp.models.drone_manager import DroneManager

import pytello.config
import rospy
import sys
from multiprocessing import Process

logging.basicConfig(level=logging.DEBUG)
                    #filename=pytello.config.LOG_FILE)
logger = logging.getLogger(__name__)
app = pytello.config.app

# Lista con las ip de los drones
drones = []

def get_drone():
    return DroneManager(drones=drones)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/controller/')
def controller():
    return render_template('controller.html')


@app.route('/api/command/', methods=['POST'])
def command():
    try:
        cmd = request.form.get('command')
        idDrone = request.form.get('idDrone') # 0 si son todos, 1 si es el primero, 2 si es el segundo, etc.

        # Comprobar que el id es menor que el número de drones
        if idDrone is not None:
            idDrone = int(idDrone)
            if idDrone > len(drones):
                rospy.loginfo({'error': 'El id del dron no es válido'})
                idDrone = 0
        else:
            idDrone = 0 # Por defecto, todos los drones

        rospy.loginfo({'action': 'command', 'cmd': cmd, 'idDrone': idDrone})
        drone = get_drone()

        if cmd == "takeOff":
            drone.takeoff(idDrone)
        if cmd == "land":
            drone.land(idDrone)
        if cmd == 'speed':
            speed = request.form.get('speed')
            if speed:
                drone.set_speed(int(speed))
        if cmd == "up":
            drone.up(idDrone)
        if cmd == "down":
            drone.down(idDrone)
        if cmd == "forward":
            drone.forward(idDrone)
        if cmd == "back":
            drone.backward(idDrone)
        if cmd == "left":
            drone.left(idDrone)
        if cmd == "right":
            drone.right(idDrone)
        if cmd == "clockwise":
            drone.clockwise(idDrone)
        if cmd == "counterClockwise":
            drone.counter_clockwise(idDrone)
        if cmd == "flipF":
            drone.flip(idDrone, 'f')
        if cmd == "flipB":
            drone.flip(idDrone, 'b')
        if cmd == "flipL":
            drone.flip(idDrone, 'l')
        if cmd == "flipR":
            drone.flip(idDrone, 'r')
        if cmd == "patrol":
            drone.patrol()
        if cmd == "stopPatrol":
            drone.stop_patrol()
        # if cmd == 'faceDetectAndTrack':
        #     drone.enable_face_detect()
        # if cmd == 'stopFaceDetectAndTrack':
        #     drone.disable_face_detect()
        # if cmd == 'snapshot':
        #     if drone.snapshot():
        #         return jsonify(status='success'), 200
        #     else:
        #         return jsonify(status='fail'), 400
        # if cmd == 'takeVideo':
        #     drone.takeVideo()
        # if cmd == 'stopVideo':
        #     drone.stopVideo()
        # if cmd == 'takeFrames':
        #     drone.takeFrames()
        # if cmd == 'stopFrames':
        #     drone.stopFrames()
        if cmd == 'startTrajectory':
            if idDrone == 0:
                for id_drone in range(len(drones)):
                    process = Process(target=drone.start_trajectory, args=(id_drone + 1,))
                    process.start()
            else:
                drone.start_trajectory(idDrone)
        if cmd == 'stopTrajectory':
            drone.stop_trajectory(idDrone)
        if cmd == 'battery':
            drone.battery(idDrone)
        if cmd == 'draw':
            drone.draw(idDrone)

        rospy.loginfo(jsonify(status='success'))
        return jsonify(status='success'), 200

    except Exception as e:
            rospy.loginfo({'error': str(e)})
            return jsonify(status='fail'), 400


# def video_generator():
#     drone = get_drone()
#     for jpeg in drone.video_jpeg_generator():
#         yield (b'--frame\r\n'
#                b'Content-Type: image/jpeg\r\n\r\n' +
#                jpeg +
#                b'\r\n\r\n')


# @app.route('/video/streaming')
# def video_feed():
#     return Response(video_generator(), mimetype="multipart/x-mixed-replace; boundary=frame")


def run(_drones):

    # Inicializar la lista de drones
    global drones
    drones = _drones

    rospy.loginfo('Servidor web iniciado en http://127.0.0.1:5000/')
    app.run(host=pytello.config.WEB_ADDRESS, port=pytello.config.WEB_PORT, threaded=True)
