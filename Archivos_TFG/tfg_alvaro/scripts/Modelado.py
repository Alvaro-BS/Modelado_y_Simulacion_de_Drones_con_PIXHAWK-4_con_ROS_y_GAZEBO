#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TwistStamped
from rospy.impl.tcpros_service import ServiceProxy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from tfg_alvaro.srv import*
import time


def main():
    rospy.init_node('Modelado', anonymous=True)
    pub_vel = rospy.Publisher('/copter_node/SetVelocity',TwistStamped,queue_size=10) #Topic envio de velocidades
    takeoff_service = rospy.ServiceProxy('/copter_node/Takeoff',takeoff) #servicio despegue
    Guidedmode_service = rospy.ServiceProxy('/copter_node/Guided',Trigger) #servicio modo guiado
    VelLinear = 1 #m/s
    Velangular = 1 #rad/s
    takeoff_service(20)  #despegue 20 metros
    time.sleep(45)
    Guidedmode_service() #activamos el modo de control de velocidad
    #EJE X
    vel = TwistStamped()
    pub_vel.publish(vel)
    time.sleep(5)
    vel.twist.linear.x = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Primer pulso en X")
    time.sleep(30)
    vel.twist.linear.x = 0
    pub_vel.publish(vel)
    time.sleep(30)
    vel.twist.linear.x = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Segundo pulso en X")
    time.sleep(30)
    vel.twist.linear.x = 0
    pub_vel.publish(vel)
    time.sleep(30)
    vel.twist.linear.x = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Tercer pulso en X")
    time.sleep(30)
    vel.twist.linear.x = 0
    pub_vel.publish(vel)
    time.sleep(30)
    #Eje Y
    vel.twist.linear.y = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Primer pulso en Y")
    time.sleep(30)
    vel.twist.linear.y = 0
    pub_vel.publish(vel)
    time.sleep(30)
    vel.twist.linear.y = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Segundo pulso en Y")
    time.sleep(30)
    vel.twist.linear.y = 0
    pub_vel.publish(vel)
    time.sleep(30)
    vel.twist.linear.y = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Tercer pulso en Y")
    time.sleep(30)
    vel.twist.linear.y = 0
    pub_vel.publish(vel)
    time.sleep(30)
    #Eje Z
    vel.twist.linear.z = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Primer pulso en Z")
    time.sleep(30)
    vel.twist.linear.z = 0
    pub_vel.publish(vel)
    time.sleep(30)
    vel.twist.linear.z = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Segundo pulso en Z")
    time.sleep(30)
    vel.twist.linear.z = 0
    pub_vel.publish(vel)
    time.sleep(30)
    vel.twist.linear.z = VelLinear
    pub_vel.publish(vel)
    rospy.loginfo("Tercer pulso en Z")
    time.sleep(30)
    vel.twist.linear.z = 0
    pub_vel.publish(vel)
    time.sleep(30)
    #Giro eje Z
    vel.twist.angular.z = Velangular
    pub_vel.publish(vel)
    rospy.loginfo("Primer pulso giro")
    time.sleep(30)
    vel.twist.angular.z = 0
    pub_vel.publish(vel)
    time.sleep(30)
    vel.twist.angular.z = Velangular
    pub_vel.publish(vel)
    rospy.loginfo("Segundo pulso giro")
    time.sleep(30)
    vel.twist.angular.z = 0
    pub_vel.publish(vel)
    time.sleep(30)
    vel.twist.angular.z = Velangular
    pub_vel.publish(vel)
    rospy.loginfo("Tercer pulso giro")
    time.sleep(30)
    vel.twist.angular.z = 0
    pub_vel.publish(vel)
    time.sleep(30)
    
if __name__ == '__main__':
    main()