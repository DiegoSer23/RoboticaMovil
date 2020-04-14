#! /usr/bin/env python
# ------------------------------------------------
# importacion de las librerias necesarias para utilizar python y los mensajes y servicios de ROS

import rospy
from geometry_msgs.msg import Twist  # Mensajes de geometria para la velocidad
from turtlesim.msg import Pose  # Mensajes de pose para la posicion
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
z = 0
theta = 0
PI = 3.1415926535897

# ------------------------------------------------
# se hace el callback a raiz de haber obtenido la pose, es decir, se asignan las variables x, y y theta con los datos de posicion actual recibidos desde ROS
def poseCallback(pose_message):
    global x
    global y
    global z
    global theta

    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta


# funcion go to goal con las coordenadas de la meta argumentos y el topico de comando de velocidad (del tipo twist) como salida
def go_to_goal(xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    rotate(xgoal, ygoal)
    # ------------------------------------------------
    # calculo de las velocidades lineal y angular (escaladas por factores kv y ka) a partir de la trigonometria entre las posiciones actual y deseada
    while (True):
        kv = 0.5
        distance = abs(math.sqrt(((xgoal - x) ** 2) + ((ygoal - y) ** 2)))
        linear_speed = kv * distance

        ka = 6
	auxtheta = theta
	if auxtheta > PI:
	   auxtheta = ((PI * 2) - theta) * -1
        desired_angle_goal = math.atan2(ygoal - y, xgoal - x)  
        dtheta = desired_angle_goal - auxtheta
	if abs(dtheta) > PI:
	    dtheta = desired_angle_goal + auxtheta 
        angular_speed = ka * (dtheta)

        # ------------------------------------------------
        # escritura de las velocidades calculadas en el topico que se va a publicar

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)

        if (distance < 0.01):  # Si la tortuga esta a 0.01 que se detenga.
            break


def rotate(xgoal, ygoal):
    global x
    global y
    global theta
    global PI

    vel_msg = Twist()
    angular_speed = 5*2*PI/360
    auxtheta = theta
    if auxtheta > PI:
	auxtheta = ((PI * 2) - theta) * -1
    d_angle = math.atan2(ygoal - y, xgoal - x)
    relative_angle = d_angle - auxtheta
    if abs(relative_angle) > PI:
	relative_angle = d_angle + auxtheta
    print("ra-", relative_angle, "da-", d_angle, "theta-", theta)

    # no nos movemos linearmente solo se mueve el angulo en z para rotar
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # velocidad de rotacion
    vel_msg.angular.z = abs(angular_speed)
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while current_angle < relative_angle:
	velocity_publisher.publish(vel_msg)
	t1 = rospy.Time.now().to_sec()
	current_angle = angular_speed*(t1-t0)
    
    # detener rotacion
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)

# ------------------------------------------------

if __name__ == '__main__':
    try:
	
        rospy.init_node('turtlebot_controller', anonymous=True)

        # ------------------------------------------------
        # publicacion del topico de velocidad (cmd_vel)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        # ------------------------------------------------
        # suscripcion al topico de posicion (Pose)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        # ------------------------------------------------
        # llamado de la funcion principal

        for i in range(360):
            print(i)
            x1 = math.cos(math.radians(i)) * 4+5.5
            y1 = math.sin(math.radians(i)) * 4+5.5
	    print(x1, y1)
            go_to_goal(x1, y1)


        # setDesiredOrientation(math.radians(90))
    # ------------------------------------------------
    except rospy.ROSInterruptException:
        pass
