#!/usr/bin/env python

import rospy
from tf.transformations import quaternion_from_euler
from math import sqrt, atan2
from std_srvs.srv import Trigger, TriggerResponse
from tfg_alvaro.srv import*
from mavros_msgs.srv import SetMode, SetModeResponse, SetModeRequest  
from mavros_msgs.srv import CommandBool, CommandBoolResponse, CommandBoolRequest
from mavros_msgs.srv import CommandTOL, CommandTOLResponse, CommandTOLRequest
from mavros_msgs.srv import SetMavFrame, SetMavFrameResponse, SetMavFrameRequest
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from sensor_msgs.msg import NavSatFix, BatteryState
from mavros_msgs.msg import HomePosition, State

class Copter_node():
    def __init__(self):
        rospy.init_node("copter_node") #inicializar nodo
        #variables
        self.rate = rospy.Rate(30)      #30 Hz
        self.Send_Velocity = False      #variable que representa si se envia o no mensajes de velocidad
        self.reach_Point = False        #variable que representa si el dron debe alcanzar un punto especificado
        self.localPosition = None       #posicion local del dron
        self.Goal_Point = None          #posicion especificada que se pretende alcanzar
        self.not_inicialPoint = True    #variable representa si ya se ha registrado la posicion inicial como casa 
        self.velocity = TwistStamped()  #velocidad local del UAV
        #Publicaciones
        self.pub_localposition = rospy.Publisher('/copter_node/Local_position',PoseStamped,queue_size=10)
        self.pub_globalposition = rospy.Publisher('/copter_node/Global_position',NavSatFix,queue_size=10)
        self.pub_battery = rospy.Publisher('/copter_node/Battery',BatteryState,queue_size=10)
        self.pub_velocity = rospy.Publisher('/copter_node/Velocity',TwistStamped,queue_size=10)
        self.pub_state = rospy.Publisher('/copter_node/State',State,queue_size=10)
        self.pub_setVelocity = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
        self.pub_setPosition = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
        self.pub_setHome = rospy.Publisher('/mavros/home_position/set',HomePosition,queue_size=10)
        #subscripciones
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.localPosition_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.globalPosition_callback)
        rospy.Subscriber('/mavros/battery', BatteryState, self.battery_callback)
        rospy.Subscriber('/mavros/local_position/velocity_body',TwistStamped , self.velocity_callback)
        rospy.Subscriber('/mavros/state',State,self.state_callback)
        rospy.Subscriber('/copter_node/SetVelocity', TwistStamped, self.setVelocity_callback)
        rospy.Subscriber('/copter_node/SetPosition', Point, self.setPosition_callback)
        #servicios
        rospy.Service("/copter_node/Takeoff",takeoff,self.takeoff_function)
        rospy.Service("/copter_node/Land",Trigger,self.land_function)
        rospy.Service("/copter_node/RTL",Trigger,self.rtl_function)
        rospy.Service("/copter_node/Stop",Trigger,self.Stop_function)
        rospy.Service("/copter_node/Guided",Trigger,self.Guided_function)
        #clientes
        self.SetMode_service = rospy.ServiceProxy("/mavros/set_mode",SetMode)
        self.Arming_service = rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
        self.takeoff_service = rospy.ServiceProxy("/mavros/cmd/takeoff",CommandTOL)
        self.mav_frame_velocity_service = rospy.ServiceProxy("/mavros/setpoint_velocity/mav_frame",SetMavFrame)

        self.mav_frame_velocity_service(8)
        rospy.loginfo("copter_node iniciado")
        self.bucle()
    
    def takeoff_function(self,req):
        self.Send_Velocity = False 
        if self.change_Mode("guided"): #cambio a modo guiado
            Armado = self.Arming_service(True) #Armado de los motores
            if Armado.success:
                Despegue = self.takeoff_service(0,0,0,0,req.altitude) #despegue 
                if Despegue.success:
                    self.reach_Point = True
                    self.Goal_Point = Point(self.localPosition.x,self.localPosition.y,req.altitude)                  
                    return takeoffResponse(True,"Despegue a "+ str(req.altitude) +" m iniciado con exito")
                return takeoffResponse(False,"Error al iniciar el despegue")
            else:
                return takeoffResponse(False,"Error en el armado de los motores")
        else:
            return takeoffResponse(False,"Error, no se ha podido cambiar el modo de vuelo a guiado")
    
    def rtl_function(self,req):
        if not self.not_inicialPoint:
            self.pub_setHome.publish(self.initialPoint)  
        if self.change_Mode("rtl"):
            self.Send_Velocity = False 
            self.reach_Point = False
            return TriggerResponse(True,"Iniciada secuencia de vuelta al punto de despegue")
        return TriggerResponse(False,"Error")

    def land_function(self,req):
        if self.change_Mode("land"):
            self.Send_Velocity = False 
            self.reach_Point = False
            return TriggerResponse(True,"Iniciada secuencia de aterrizaje") 
        return TriggerResponse(False,"Error")
    
    def Stop_function(self,req):
        if self.change_Mode("brake"):
            self.Send_Velocity = False
            self.reach_Point = False
            return TriggerResponse(True,"Deteniendo UAV") 
        return TriggerResponse(False,"Error")
    
    def Guided_function(self,req):
        if self.change_Mode("guided"):
            return TriggerResponse(True,"Modo guiado activado") 
        return TriggerResponse(False,"Error")
    
    #filtro 
    def localPosition_callback(self,msg):
        self.localPosition = msg.pose.position
        self.pub_localposition.publish(msg)
    
    def globalPosition_callback(self,msg):
        if self.not_inicialPoint:
            #registro del punto inicial
            self.initialPoint = HomePosition()
            self.initialPoint.geo.latitude = msg.latitude
            self.initialPoint.geo.longitude = msg.longitude
            self.initialPoint.geo.altitude = msg.altitude - 19.4
            self.not_inicialPoint = False
        self.pub_globalposition.publish(msg)
     
    def battery_callback(self,msg):
        self.pub_battery.publish(msg)
    
    def velocity_callback(self,msg):
        self.velocity = msg

    def state_callback(self,msg):
        self.pub_state.publish(msg)

    def setVelocity_callback(self,msg):
        self.Send_Velocity = True
        self.reach_Point = False
        self.Vel_msg = msg
        rospy.loginfo("consigna de velocidad: " + str(msg.twist))

    def setPosition_callback(self,msg):
        if not self.Send_Velocity:
            if not self.not_inicialPoint:
                self.pub_setHome.publish(self.initialPoint) 
            if self.change_Mode("guided"):
                self.reach_Point = True
                self.Goal_Point = msg
                position_msg = PoseStamped()
                position_msg.pose.position = msg
                yaw = atan2(msg.y - self.localPosition.y,msg.x - self.localPosition.x)
                Q = quaternion_from_euler(0,0,yaw)
                position_msg.pose.orientation.x = Q[0]
                position_msg.pose.orientation.y = Q[1]
                position_msg.pose.orientation.z = Q[2]
                position_msg.pose.orientation.w = Q[3]
                self.pub_setPosition.publish(position_msg)
                rospy.loginfo("Dirigiendose al punto: " + str(msg)) 
            else:
                rospy.loginfo("Error, no se ha podido cambiar el modo de vuelo a guiado")
   
    def bucle(self):
        while not rospy.is_shutdown():
            if self.Send_Velocity:
                self.pub_setVelocity.publish(self.Vel_msg)
            elif self.reach_Point and (self.distance(self.localPosition,self.Goal_Point) < 0.4):
                self.reach_Point = False
                self.change_Mode("brake")
            self.pub_velocity.publish(self.velocity)
            self.rate.sleep()

    def change_Mode(self,Mode):
        try:
            if self.SetMode_service(0,Mode).mode_sent:
                return True  
            return False
        except rospy.ServiceException as error:
            rospy.loginfo("Ha habido un error: " + str(error))
    
    def distance(self,a,b):
        D = sqrt((b.x - a.x)**2 + (b.y - a.y)**2 + (b.z - a.z)**2)
        return D
        
if __name__ == "__main__":
    Copter_node()