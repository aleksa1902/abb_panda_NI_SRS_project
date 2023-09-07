#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

# Definisanje globalnih promenljivih za Pandu
panda_x = 0.0
panda_y = 0.0
panda_z = 0.0
panda_W = 0.0
panda_X = 0.0
panda_Y = 0.0
panda_Z = 0.0

def panda_position_callback(data):
    global panda_x, panda_y, panda_z, panda_W, panda_X, panda_Y, panda_Z
    # Ova funkcija će se pozivati svaki put kada stigne nova poruka na temu /Panda_pose
    panda_x = data.position.x
    panda_y = data.position.y
    panda_z = data.position.z
    panda_W = data.orientation.w
    panda_X = data.orientation.x
    panda_Y = data.orientation.y
    panda_Z = data.orientation.z

def publish_panda_data():
    global panda_x, panda_y, panda_z, panda_W, panda_X, panda_Y, panda_Z

    # Stvorite niz sa podacima
    data_array = Float64MultiArray()
    data_array.data = [panda_x, panda_y, panda_z, panda_W, panda_X, panda_Y, panda_Z]

    # Stvorite ROS izdavača za niz sa podacima i objavite ga
    pub = rospy.Publisher("/panda_data", Float64MultiArray, queue_size=10)
    
    # Dodatno: Postavite brzinu objavljivanja (opcionalno)
    rate = rospy.Rate(20)  # Postavite brzinu na 10 Hz

    while not rospy.is_shutdown():
        data_array.data = [panda_x, panda_y, panda_z, panda_W, panda_X, panda_Y, panda_Z]
        pub.publish(data_array)
        rate.sleep()

def call_services_and_topics():
    global panda_x, panda_y, panda_z, panda_W, panda_X, panda_Y, panda_Z
    # Inicijalizacija ROS čvora
    rospy.init_node("panda_master")
    rospy.Subscriber("/Panda_pose", Pose, panda_position_callback)

if __name__ == '__main__':
    try:
        call_services_and_topics()
        publish_panda_data()
    except rospy.ROSInterruptException:
        pass