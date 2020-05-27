#! /usr/bin/env python
# ------------------------------------------------
# importacion de las librerias necesarias para utilizar python y los mensajes y servicios de ROS
import random
from std_srvs.srv import Empty
from threading import Thread
import rospy
from geometry_msgs.msg import Twist  # Mensajes de geometria para la velocidad
from turtlesim.msg import Pose  # Mensajes de pose para la posicion
import math
import time
from std_srvs.srv import Empty
import turtlesim.srv


# Variables para la tortuga 1
x = 0
y = 0
z = 0
theta = 0
PI = 3.1415926535897

# Variables para la tortuga 2
x2 = 0
y2 = 0
z2 = 0
theta2 = 0

# Variables para la tortuga 3
x3 = 0
y3 = 0
z3 = 0
theta3 = 0

# Variables para la tortuga 4
x4 = 0
y4 = 0
z4 = 0
theta4 = 0
# ------------------------------------------------

#Funcion que recibe los datos de la tortuga1 del subscriber
def poseCallback(pose_message):
    global x
    global y
    global z
    global theta

    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

#Funcion que recibe los datos de la tortuga2 del subscriber
def poseCallback2(pose_message):
    global x2
    global y2
    global z2
    global theta2

    x2 = pose_message.x
    y2 = pose_message.y
    theta2 = pose_message.theta

#Funcion que recibe los datos de la tortuga3 del subscriber
def poseCallback3(pose_message):
    global x3
    global y3
    global z3
    global theta3

    x3 = pose_message.x
    y3 = pose_message.y
    theta3 = pose_message.theta

# Funcion que recibe los datos de la tortuga2 del subscriber
def poseCallback4(pose_message):
    global x4
    global y4
    global z4
    global theta4

    x4 = pose_message.x
    y4 = pose_message.y
    theta4 = pose_message.theta

# funcion go to goal para la tortuga2 recibe como parametros las coordenadas del punto deseado
def go_to_goal(xgoal, ygoal):
    #tortuga 1 variables
    global x
    global y
    global theta

    #tortuga 2 variables
    global x2
    global y2
    global theta2

    # tortuga 3 variables
    global x3
    global y3
    global theta3

    # tortuga 4 variables
    global x4
    global y4
    global theta4


    #Antes de que la tortuga comience a moverse, hago que gire a la meta para que cuanso se mueva lo haga en linea recta.
    velocity_message2 = Twist()
    desired_angle_goal = math.atan2(ygoal - y2, xgoal - x2)
    velocity_message2.linear.x = 0
    velocity_message2.angular.z = desired_angle_goal - theta2
    velocity_publisher2.publish(velocity_message2)
    time.sleep(1.5)

    #Contador que sirve para que la tortuga recobre su rumbo en caso de que lo pierda
    contador = 0
    # ------------------------------------------------

    #Ciclo para calcular la velocidad lineal y angular de la tortuga2
    while (True):

        #ESTRATEGIA PARA NO CHOCAR
        # ciclo para hacer que la tortuga 2 se detenga y cambie de sentido si la 1 esta cerca
        while (True):

            # Calculo si la tortuga 2 esta cerca de la tortuga 1
            xgo = (x2 + x) / 2
            ygo = (y2 + y) / 2
            distance = abs(math.sqrt(((xgo - x) ** 2) + ((ygo - y) ** 2)))
            if (distance > 1):  # Si la tortuga2 esta a mas de 1 de distancia de la otra, se sale del ciclo.
                break

            #En caso de que la tortuga1 este cerca de la tortuga2:
            #La tortuga 2 disminuye su velocidad lineal a 1
            #La tortuga 2 cambia su direccion, igualando su velocidad angular al angulo en sentido contrario que tenga la tortuga1
            velocity_message2.linear.x = 1
            velocity_message2.angular.z = -(theta) * 0.2
            velocity_publisher2.publish(velocity_message2)


        # ciclo para hacer que la tortuga 2 se detenga y cambie de sentido si la 3 esta cerca
        while (True):

            # Calculo si la tortuga 2 esta cerca de la tortuga 3
            xgo = (x2 + x3) / 2
            ygo = (y2 + y3) / 2
            distance = abs(math.sqrt(((xgo - x3) ** 2) + ((ygo - y3) ** 2)))
            if (distance > 1):  # Si la tortuga2 esta a mas de 1 de distancia de la otra, se sale del ciclo.
                break

            # En caso de que la tortuga3 este cerca de la tortuga2:
            # La tortuga 2 disminuye su velocidad lineal a 1
            # La tortuga 2 cambia su direccion, igualando su velocidad angular al angulo en sentido contrario que tenga la tortuga3
            velocity_message2.linear.x = 1
            velocity_message2.angular.z = -(theta3) * 0.2
            velocity_publisher2.publish(velocity_message2)

        # ciclo para hacer que la tortuga 2 se detenga y cambie de sentido si la 4 esta cerca
        while (True):

            # Calculo si la tortuga 2 esta cerca de la tortuga 4
            xgo = (x2 + x4) / 2
            ygo = (y2 + y4) / 2
            distance = abs(math.sqrt(((xgo - x4) ** 2) + ((ygo - y4) ** 2)))
            if (distance > 1):  # Si la tortuga2 esta a mas de 1 de distancia de la otra, se sale del ciclo.
                break

            # En caso de que la tortuga4 este cerca de la tortuga2:
            # La tortuga 2 disminuye su velocidad lineal a 1
            # La tortuga 2 cambia su direccion, igualando su velocidad angular al angulo en sentido contrario que tenga la tortuga4
            velocity_message2.linear.x = 1
            velocity_message2.angular.z = -(theta4) * 0.2
            velocity_publisher2.publish(velocity_message2)


        #En caso de que ninguna tortuga este cerca de la tortuga2, calculo las velocidades lineal y angular a las que se movera la tortuga 2

        #Calculo la velocidad lineal de la tortuga2
        kv =1
        distance = abs(math.sqrt(((xgoal - x2) ** 2) + ((ygoal - y2) ** 2)))
        linear_speed = kv * distance

        #Calculo la velocidad angular de la tortuga 2
        ka = 3
        auxtheta = theta2
        if auxtheta > PI:
            auxtheta = ((PI * 2) - theta2) * -1
        desired_angle_goal = math.atan2(ygoal - y2, xgoal - x2)
        dtheta = desired_angle_goal - auxtheta
        if abs(dtheta) > PI:
            dtheta = desired_angle_goal + auxtheta
            linear_speed = 0 * distance
        angular_speed = ka * (dtheta)

        # ------------------------------------------------
        #Publico la velocidad lineal y angular que calcule sobre la tortuga 2

        # Escritura de las velocidades calculadas en el topico que se va a publicar
        velocity_message2.linear.x = linear_speed
        velocity_message2.angular.z = angular_speed
        velocity_publisher2.publish(velocity_message2)

        # Si la tortuga2 esta a 0.1 de distancia de la meta, que se detenga.
        if (distance < 0.1):
            break

        #Normalmente la tortuga llega a la meta antes de 5000 ciclos, si ya pasaron y no ha llegado entonces cambio su direccion y hago que se dirija directo a la meta.
        contador = contador + 1
        if contador > 5000:
            desired_angle_goal = math.atan2(ygoal - y2, xgoal - x2)
            velocity_message2.linear.x = 0
            velocity_message2.angular.z = desired_angle_goal - theta2
            velocity_publisher2.publish(velocity_message2)
            time.sleep(1.5)
            contador = 0



if __name__ == '__main__':
    try:

        rospy.init_node('turtlebot_controller', anonymous=True)

        # ------------------------------------------------
        # publicacion del topico de velocidad (cmd_vel) de la tortuga 2
        cmd_vel_topic2 = '/turtle2/cmd_vel'
        velocity_publisher2 = rospy.Publisher(cmd_vel_topic2, Twist, queue_size=10)

        # ------------------------------------------------
        # suscripcion al topico de posicion (Pose) de las 4 tortugas
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)

        position_topic2 = "/turtle2/pose"
        pose_subscriber = rospy.Subscriber(position_topic2, Pose, poseCallback2)

        position_topic3 = "/turtle3/pose"
        pose_subscriber = rospy.Subscriber(position_topic3, Pose, poseCallback3)

        position_topic4 = "/turtle4/pose"
        pose_subscriber = rospy.Subscriber(position_topic4, Pose, poseCallback4)

        time.sleep(2)

        # Puntos a los que ira la tortuga 2
        go_to_goal(10, 10)
        go_to_goal(7, 1)
        go_to_goal(1, 10)


    # ------------------------------------------------
    except rospy.ROSInterruptException:
        pass
