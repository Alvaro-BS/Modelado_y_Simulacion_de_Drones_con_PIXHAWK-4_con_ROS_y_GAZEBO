#!/usr/bin/env python

import rospy
import time
from tfg_alvaro.srv import*
from std_srvs.srv import Trigger, TriggerResponse
from mavros_msgs.srv import WaypointPush, WaypointPushResponse, WaypointPushRequest
from mavros_msgs.srv import WaypointClear, WaypointClearResponse, WaypointClearRequest
from mavros_msgs.srv import SetMode, SetModeResponse, SetModeRequest
from mavros_msgs.msg import Waypoint

class Mission_node():
    def __init__(self):
        rospy.init_node("mission_node")
        #service
        rospy.Service("/mission_node/upload",fileMission,self.upload_function)
        rospy.Service("/mission_node/startMission", Trigger,self.start_function)
        #client
        self.push_service = rospy.ServiceProxy("mavros/mission/push", WaypointPush)
        self.clear_service = rospy.ServiceProxy("mavros/mission/clear", WaypointClear)
        self.takeoff_service = rospy.ServiceProxy("copter_node/Takeoff", takeoff)
        self.SetMode_service = rospy.ServiceProxy("/mavros/set_mode",SetMode)
        rospy.spin()

    def upload_function(self,req):
        mission,error = self.read_file(req.filePath)
        if mission == None:
            rospy.loginfo(error)
            return fileMissionResponse(False,error)
        try:
            if self.clear_service().success: #borramos mission actual
                if self.push_service(0,mission).success: #cargar nueva mision
                    rospy.loginfo("Mission uploaded")
                    return fileMissionResponse(True,"Mission uploaded")
                rospy.loginfo("Error: no se pudo cargar la mision correctamente")
                return fileMissionResponse(False,"Error: no se pudo cargar la mision correctamente")
        except rospy.ServiceException as error:
            rospy.loginfo("Error: " + str(error)) 

    def start_function(self,req):
        self.takeoff_service(10)
        time.sleep(15)
        try:
            if self.SetMode_service(0,'auto').mode_sent:
                rospy.loginfo("Mission started")
                return TriggerResponse(True,"Mission started")
            return TriggerResponse(False,"Error: no se puede iniciar la mision")
        except rospy.ServiceException as error:
            rospy.loginfo("Error: " + str(error)) 
             
    def read_file(self,file): 
        try:
            with open(file) as f:
                Lines = f.readlines()
                data = []
                for line in Lines:
                    if line[0] == '#':
                        continue
                    num = ''
                    waypoint = []
                    for i in range(0,len(line)):
                        if line[i] != '\t' and line[i] != '\n':
                            num += line[i]  
                        else:
                            waypoint.append(float(num))
                            num = ''
                    data.append(waypoint)
            #conversion de los datos leidos al tipo Waypoint usado por el nodo mavros 
            mission = []
            #primer comando lo ignora mavros
            wp = Waypoint()
            wp.frame = 3
            wp.command = 16
            wp.is_current = False
            wp.autocontinue = True
            mission.append(wp)
            #obtencion de cada waypoint
            for waypoint in data:
                if waypoint[3] < 0 or not(waypoint[4] > 0):
                    return None, "Error: Algun campo esta incorrecto, revise el archivo"
                wp = Waypoint()
                wp.is_current = False
                wp.autocontinue = True
                #cambio de velocidad
                wp.frame = 2
                wp.command = 178  
                wp.param1 = 1   # tipo de velocidad (velocidad en tierra) 
                wp.param2 = waypoint[4]   # velocidad
                wp.param3 = -1  # cambio velocidad de los motores en porcentaje(-1 no cambia)
                mission.append(wp)
                #dijigirse a un waypoint
                wp = Waypoint()
                wp.is_current = False
                wp.autocontinue = True
                wp.frame = 3
                wp.command = 16
                wp.x_lat = waypoint[0]   # latitud
                wp.y_long = waypoint[1]  # longitud
                wp.z_alt = waypoint[2]   # altura relativa al punto base
                wp.param1 = waypoint[3]  # delay
                mission.append(wp)
            #vuelta a tierra (RTL)
            wp = Waypoint()
            wp.frame = 3
            wp.command = 20
            wp.is_current = False
            wp.autocontinue = True
            mission.append(wp)
            rospy.loginfo(mission)
            return mission, ""
        except:
            return None, "Error: la ruta introducida es incorrecta o algun campo esta imcompleto"             

if __name__ == "__main__":
    Mission_node()
