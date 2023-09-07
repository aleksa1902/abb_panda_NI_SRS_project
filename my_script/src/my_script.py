#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64MultiArray
from abb_robot_msgs.srv import TriggerWithResultCode
from sensor_msgs.msg import JointState
import numpy as np
from sympy import symbols, cos, sin, diff, sqrt, atan2, asin, acos
from sympy.physics.vector import ReferenceFrame
from sympy import Matrix, solve, Eq
import math
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from abb_egm_msgs.msg import EGMState

# Definisanje globalnih promenljivih za q1 do q6
q1 = 0.0
q2 = 0.0
q3 = 0.0
q4 = 0.0
q5 = 0.0
q6 = 0.0

# Definisanje globalnih promenljivih za Pandu
panda_x = 0.0
panda_y = 0.0
panda_z = 0.0
panda_W = 0.0
panda_X = 0.0
panda_Y = 0.0
panda_Z = 0.0

q_symbols = symbols('q1:7')  # Simboli za zglobove q1 do q6

active = False
    
dh_params = [
    [0.0, -np.pi/2, 0.290, q_symbols[0]],
    [0.27, 0.0, 0.0, q_symbols[1] - np.pi/2],
    [0.07, -np.pi/2, 0.0, q_symbols[2]],
    [0.0, np.pi/2, 0.302, q_symbols[3]],
    [0.0, -np.pi/2, 0.0, q_symbols[4]],
    [0.0, 0.0, 0.072, q_symbols[5]]
]

# panda_master = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

example_pose_vector = None
jacobian_matrix = None

def compute_symbolic_transform_matrix(alpha, a, d, theta):
    transform = Matrix([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])
    return transform

def symbolic_forward_kinematics(dh_params, joint_symbols):
    transform = Matrix.eye(4)  # Koristimo 4x4 matricu
    for param, symbol in zip(dh_params, joint_symbols):
        a, alpha, d, theta = param
        transform_i = compute_symbolic_transform_matrix(alpha, a, d, theta)
        transform = transform * transform_i

    return transform

def jacobian_matrix_compute(q_symbols, example_pose_vector):
    # Izračunavanje parcijalnih izvoda
    jacobian_elements = []
    for i in range(6):
        jacobian_row = []
        for joint_symbol in q_symbols:
            jacobian_entry = diff(example_pose_vector[i], joint_symbol)
            jacobian_row.append(jacobian_entry)
        jacobian_elements.append(jacobian_row)

    # Kreiranje Jakobijan matrice
    jacobian_matrix = Matrix(jacobian_elements)

    return jacobian_matrix

def init_symbol():
    global example_pose_vector, jacobian_matrix, q_symbols, dh_params

    example_pose = symbolic_forward_kinematics(dh_params, q_symbols)

    rotation_matrix = example_pose[:3, :3]

    x = example_pose[0, 3]
    y = example_pose[1, 3]
    z = example_pose[2, 3]

    RX = atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
    RY = atan2(-rotation_matrix[2, 0], sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
    RZ = atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    example_pose_vector = Matrix([x, y, z, RX, RY, RZ]) 

    jacobian_matrix = jacobian_matrix_compute(q_symbols, example_pose_vector)


def joint_states_callback(data):
    global q1, q2, q3, q4, q5, q6
    # Ova funkcija će se pozivati svaki put kada stigne nova poruka na temu /egm/joint_states
    q1 = data.position[0]
    q2 = data.position[1]
    q3 = data.position[2]
    q4 = data.position[3]
    q5 = data.position[4]
    q6 = data.position[5]

# def egm_states_callback(data):
#     global active
#     # Ova funkcija će se pozivati svaki put kada stigne nova poruka na temu /egm/joint_states
#     active = data.egm_channels[0].active

def panda_position_callback(data):
    global panda_x, panda_y, panda_z, panda_W, panda_X, panda_Y, panda_Z
    # Ova funkcija će se pozivati svaki put kada stigne nova poruka na temu /Panda_pose
    panda_x = data.position.x
    panda_y = data.position.y
    panda_z = data.position.z
    panda_z = data.position.z
    panda_W = data.orientation.w
    panda_X = data.orientation.x
    panda_Y = data.orientation.y
    panda_Z = data.orientation.z

def panda_master_callback(data):
    global panda_master
    # Ova funkcija će se pozivati svaki put kada stigne nova poruka na temu /Panda_pose
    panda_master = data.data

def call_service(service_name):
    try:
        rospy.loginfo("Sada pokrecemo: " + service_name)
        service = rospy.ServiceProxy(service_name, TriggerWithResultCode)
        response = service()

        rospy.loginfo(response)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def call_services_and_topics():
    global q1, q2, q3, q4, q5, q6, example_pose_vector, jacobian_matrix, q_symbols, panda_x, panda_y, panda_z, panda_W, panda_X, panda_Y, panda_Z, active, panda_master
    # Inicijalizacija ROS čvora
    rospy.init_node("abb_robot_py_node")
    init_symbol()
    rospy.Subscriber("/egm/joint_states", JointState, joint_states_callback)
    rospy.Subscriber("/Panda_pose", Pose, panda_position_callback)
    # rospy.Subscriber("/panda_data", Float64MultiArray, panda_master_callback)
    rospy.Subscriber("/egm/egm_states", EGMState, egm_states_callback)

    # Pozivanje servisa
    services = [
        '/rws/stop_rapid',
        '/rws/pp_to_main',
        '/rws/start_rapid',
        '/rws/sm_addin/start_egm_joint',
    ]

    for service_name in services:
        rospy.wait_for_service(service_name)
        call_service(service_name)
        rospy.sleep(0.4)

    # Pozivanje servisa za prebacivanje kontrolera
    switch_controller = rospy.ServiceProxy('/egm/controller_manager/switch_controller', SwitchController)
    response = switch_controller(
        start_controllers=['joint_group_velocity_controller'],
        stop_controllers=[''],
        strictness=1,
        start_asap=False,
        timeout=0.0
    )

    if response.ok:
        rospy.loginfo("Prebacivanje kontrolera uspešno.")
    else:
        rospy.logerr("Neuspešno prebacivanje kontrolera. Poruka o grešci: %s" % response.ok)

    pub = rospy.Publisher('/egm/joint_group_velocity_controller/command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(20)  # 10 Hz

    # rospy.loginfo("Trenutna pozicija zglobova (iz teme /egm/joint_states):")
    # rospy.loginfo("q1: %f" % q1)
    # rospy.loginfo("q2: %f" % q2)
    # rospy.loginfo("q3: %f" % q3)
    # rospy.loginfo("q4: %f" % q4)
    # rospy.loginfo("q5: %f" % q5)
    # rospy.loginfo("q6: %f" % q6)

    # update_time = rospy.Time.now() + rospy.Duration(10)
    # check = True
    # desired_pose = [0.364353826675634, 0.1, 0.593999995848543, 1.83697005749691e-16 - math.pi, -1.04719748461757, -1.06057507565334e-16]

    while not rospy.is_shutdown():
        # rospy.loginfo(pub)
        #example_joint_angles = [q1, q2, q3, q4, q5, q6]

        #symbolic_pose_vector = example_pose_vector.subs(dict(zip(q_symbols, example_joint_angles)))

        #rospy.loginfo("Final position:")
        #rospy.loginfo(symbolic_pose_vector)
        example_joint_angles = [q1, q2, q3, q4, q5, q6]

        #rospy.loginfo("Current position q1-q6:")
        #rospy.loginfo(example_joint_angles)

        symbolic_pose_vector = example_pose_vector.subs(dict(zip(q_symbols, example_joint_angles)))

        #rospy.loginfo("Symbolic Pose Vector (6x1):")
        #rospy.loginfo(symbolic_pose_vector)

        jacobian_matrix_numeric = jacobian_matrix.subs(zip(q_symbols, example_joint_angles))

        jacobian_matrix_numeric_f = np.array(jacobian_matrix_numeric).astype(float)

        # rospy.loginfo("Numeric Jacobian Matrix:")
        # rospy.loginfo(jacobian_matrix_numeric)
        
        # current_time = rospy.Time.now()
        # if current_time >= update_time and check:
        #     desired_pose = [0.364353826675634, -0.1, 0.593999995848543, 1.83697005749691e-16 - math.pi, -1.04719748461757, -1.06057507565334e-16]
        #     check = False
        desired_pose_panda = [panda_x, panda_y, panda_z, panda_X, panda_Y, panda_Z]

        # rospy.loginfo("Panda position")
        # rospy.loginfo(desired_pose_panda)
        
        # Pravimo rotacionu matricu iz uglova RPY
        r_current = R.from_euler('xyz', [symbolic_pose_vector[3], symbolic_pose_vector[4], symbolic_pose_vector[5]], degrees=False)

        # Pretvorimo rotacionu matricu u kvaternion
        quat = r_current.as_quat()

        # Pristupimo elementima rotacione matrice i kreiramo finalni vektor
        final_current = [
            symbolic_pose_vector[0],
            symbolic_pose_vector[1],
            symbolic_pose_vector[2],
            quat[0],
            quat[1],
            quat[2]
        ]

        # rospy.loginfo("Finall current:")
        # rospy.loginfo(final_current)

        # Pravimo rotacionu matricu iz uglova RPY
        # r_desired = R.from_euler('xyz', [desired_pose[3], desired_pose[4], desired_pose[5]], degrees=False)

        # Pretvorimo rotacionu matricu u kvaternion
        # quat_d = r_desired.as_quat()

        # Pristupimo elementima rotacione matrice i kreiramo finalni vektor
        # final_desired = [
        #     desired_pose[0],
        #     desired_pose[1],
        #     desired_pose[2],
        #     quat_d[0],
        #     quat_d[1],
        #     quat_d[2]
        # ]

        # rospy.loginfo("Finall desired:")
        # rospy.loginfo(final_desired)

        # Izračunajte grešku pozicije
        # error_pose = [desired - current for desired, current in zip(final_desired, final_current)]
        error_pose = [desired - current for desired, current in zip(desired_pose_panda, final_current)]

        # rospy.loginfo("Error pose(desired - current):")
        # rospy.loginfo(error_pose)

        # Definirajte brzinu kretanja kao Jacobijan matricu pomnoženu s greškom pozicije
        q_dot_test = np.dot(np.dot(jacobian_matrix_numeric_f.T,np.linalg.inv(np.dot(jacobian_matrix_numeric_f, jacobian_matrix_numeric_f.T))), np.array(error_pose) * 3)

        q_dot_test_num = np.array([q_dot_value.evalf() for q_dot_value in q_dot_test])

        # rospy.loginfo("Final result:")
        # rospy.loginfo(q_dot_test_num)
        if not active:
            call_service('/rws/start_rapid')
            rospy.sleep(0.4)
            call_service('/rws/sm_addin/start_egm_joint')
            rospy.sleep(0.4)
            # Pozivanje servisa za prebacivanje kontrolera
            switch_controller = rospy.ServiceProxy('/egm/controller_manager/switch_controller', SwitchController)
            response = switch_controller(
                start_controllers=['joint_group_velocity_controller'],
                stop_controllers=[''],
                strictness=1,
                start_asap=False,
                timeout=0.0
            )
        command_msg = Float64MultiArray(data=q_dot_test_num)
        rospy.loginfo(command_msg)
        pub.publish(command_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        call_services_and_topics()
    except rospy.ROSInterruptException:
        pass
