#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import math

def Prueba_wp():
	rospy.init_node('Prueba_wp', anonymous=True)	
	push_service = rospy.ServiceProxy('mavros/mission/push', WaypointPush)
	clear_service = rospy.ServiceProxy('mavros/mission/clear', WaypointClear)
	
	try:
		clear_service()
	except rospy.ServiceException as e:
		rospy.loginfo("Error: " + str(e))  
	
	wl = []

	wp = Waypoint()
	wp.frame = 3
	wp.command = 22 # takeoff
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  
	wp.param2 = 0
	wp.param3 = 0
	wp.param4 = 0
	wp.x_lat = -35.362395 
	wp.y_long = 149.165232
	wp.z_alt = 10
	wl.append(wp)
	

	wp = Waypoint()
	wp.frame = 2
	wp.command = 178  # cambiar velocidad
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 1   #tipo de velocidad(1 con respecto a tierra)
	wp.param2 = 4   #velocidad
	wp.param3 = -1  #giro motores (-1 no cambio)
	wp.param4 = 0
	wp.x_lat = 0
	wp.y_long = 0
	wp.z_alt = 0
	wl.append(wp)

	
	wp = Waypoint() 
	wp.frame = 3
	wp.command = 16  #Navigate to waypoint.
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  # delay 
	wp.param2 = 0  # radio de aceptacion
	wp.param3 = 0  # radio de paso
	wp.param4 = 0  # Yaw
	wp.x_lat = -35.362395 
	wp.y_long = 149.165232
	wp.z_alt = 10
	wl.append(wp)

	wp = Waypoint() 
	wp.frame = 2
	wp.command = 20  #RTL
	wp.is_current = False
	wp.autocontinue = True
	wp.param1 = 0  #todos los parametros a cero
	wp.param2 = 0
	wp.param3 = 0
	wp.param4 = 0
	wp.x_lat = 0
	wp.y_long = 0
	wp.z_alt = 0
	wl.append(wp)	
	
	print(wl)
	push_service(start_index=0, waypoints=wl)      

if __name__ == '__main__':
	Prueba_wp()
