from .submodules.motors import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

import threading
import math
import time

class ROS2Dynamixel(Node):
    """
    Create a ros node for the motors
    Parameters :
        - rosName : name for the node
        - rate : rate message in Hz
    """

    def __init__(self,rosName : str,rate : float = 1/0.01) :

        # Init ROS
        Node.__init__(self,rosName)
        # Spin in a separate thread
        thread = threading.Thread(target=rclpy.spin, args=(self, ), daemon=True)
        thread.start()
        self.rosRate = self.create_rate(rate)

        # set parameters
        self.declare_parameter('ids',[1,2])
        self.declare_parameter('gearRatio',[30,20])

        self.ids = self.get_parameter('ids').get_parameter_value().integer_array_value
        self.gearRatio = self.get_parameter('gearRatio').get_parameter_value().double_array_value
        
        # # deduce parameters
        self.numMotors = len(self.ids)
        self.goalPositions = [0 for _ in self.ids]
        self.displacements = [0 for _ in self.ids]

        # init motors
        self.motors =  MotorsSync(self.ids)
        self.initPosition = self.motors.read_position()
        
        # init topics
        self.initgraph()

    ############################## 
    # ROS Function
    ##############################

    def initgraph(self):
        self.pub_current = [self.create_publisher(Float64,'Motor'+str(i)+'/state/current',10)  for i in range(self.numMotors)]
        self.pub_pp = [self.create_publisher(Float64,'Motor'+str(i)+'/state/displacement',10)  for i in range(self.numMotors)]
        self.sub0 = self.create_subscription(Float64,'Motor0/ref/displacement', self.on_receive_callback0,10)
        self.sub1 = self.create_subscription(Float64,'Motor1/ref/displacement', self.on_receive_callback1,10)
        self.sub2 = self.create_subscription(Float64,'Motor2/ref/displacement', self.on_receive_callback2,10)
        self.sub3 = self.create_subscription(Float64,'Motor3/ref/displacement', self.on_receive_callback3,10)


    def publish_current(self,data,index):
        msg = Float64()
        msg.data = float(data)
        self.pub_current[index].publish(msg)

    # for present position
    def publish_present_position(self,data,index):
        msg = Float64()
        msg.data = float(data)
        self.pub_pp[index].publish(msg)

    def on_receive_callback0(self,data):
        self.displacements[0] = data.data
    def on_receive_callback1(self,data):
        self.displacements[1] = data.data
    def on_receive_callback2(self,data):
        self.displacements[2] = data.data
    def on_receive_callback3(self,data):
        self.displacements[3] = data.data


    ############################## 
    # Actuate functions
    ##############################

    def getGoalPositions(self):
        """ transform displacements of the cable into a rotation of the motor"""
        for i in range(self.numMotors):
            rotation = self.displacements[i]/self.gearRatio[0]
            rotation = rotation*4095/(2*math.pi)
            self.goalPositions[i] = int(self.initPosition[i]+rotation)
            
    def actuate(self):
        
        while rclpy.ok():
            t1 = time.time()
            # write position
            self.getGoalPositions()
            self.motors.write_position(self.goalPositions)
            
            # read and publish
            self.present_position = self.motors.read_position()
            self.present_current = self.motors.read_current()
            for i in range(self.numMotors):
                self.publish_present_position(self.present_position[i],i)
                self.publish_current(self.present_current[i],i)
            
            self.rosRate.sleep()
            print(time.time()-t1)

        self.motors.turnOFF()


def main():
    # parameters 
    rate = 1/0.01
    rosName = "MotorsRosNode"

    # init ros
    rclpy.init()
    motors = ROS2Dynamixel(rosName,rate)

    motors.actuate()
    
if __name__=='__main__':
    main()