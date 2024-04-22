#!/usr/bin/env python
import threading
import rospy
from visualization_msgs.msg import MarkerArray 
import subprocess
import pytello.droneapp.controllers.server
import pytello.droneapp.models.drone_manager
import geometry_msgs.msg
from std_msgs.msg import String

# sub = None
# pub = None

# def publish_position(data):
#     points_str = str(data.pose.position.x) + ' ' + str(data.pose.position.y) + ' ' + str(data.pose.position.z)
#     pub.publish(points_str)
    
#nmap -sn 192.168.0.0/24   
def run():
    # Lanza la app del dron cuando haya un subscriber
    drones = ['192.168.0.242','192.168.0.116'] #, '192.168.0.229'], '192.168.0.243'
    pytello.droneapp.controllers.server.run(drones)


def initDrone():
    # Inicializa el nodo ROS 
    rospy.init_node('drone', anonymous=True)

    # Subscribirse al topic /mavros/vision_pose/pose para recibir la posición del dron
    # global sub
    # sub = rospy.Subscriber("/mavros/vision_pose/pose", geometry_msgs.msg.PoseStamped, publish_position)

    # global pub
    # pub = rospy.Publisher("/tello/position", String, queue_size=10)

    run()

    rospy.spin()

def main():
    try:
        initDrone()
    except rospy.ROSInterruptException:
        print('Error en el nodo ROS')
        pass
    except Exception as e:
        print(str(e))

main()


    