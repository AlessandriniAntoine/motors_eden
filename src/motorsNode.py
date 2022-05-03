#!/usr/bin/env python3 

import math
import time
import threading

from motor import *

import rospy
from std_msgs.msg import Float64


########### class multiple motor ######################
class MotorsSubscriber():
    """"
    connect to the ros component 
    control all the motors
    Parameters :
        rate : rate message in Hz
    """
    
    def __init__(self,rate : float = 1/0.01):

        self.rate = rospy.Rate(rate)

        # get parameters
        self.ids = rospy.get_param('ids',[1,2,3,4])
        self.radius = rospy.get_param('radius',[40.,4.4,105.25,2.])

        self.num_motors = len(self.ids)
        print(self.ids)
        # Initialize variables
        self.motors = [XM430(self.ids[i]) for i in range(self.num_motors)]        # list of motors
        self.rotations = [0]*self.num_motors                                                            # rotation for the motors
        self.initPositions = [0]*self.num_motors                                                        # Init position of each motor
        
        for i in range(self.num_motors):
           
            # motors
            self.motors[i].read_present_position()
            self.initPositions[i] = self.motors[i].dxl_present_position
            self.motors[i].write_pid_coefficient(800,0,4700)

    def rotation(self,i):
        # radius of the winch 
        rotation = self.motors[i].length/self.radius[i] # in radian
        # for motor 2pi = 4095
        rotation = rotation*4095/(2*math.pi)
        self.rotations[i]=self.initPositions[i]-int(rotation)

    
    def actuate(self):
        t1 = time.time()

        for i in range(self.num_motors) :
            # write position
            self.rotation(i)
            angleRotation = self.rotations[i] 
            self.motors[i].write_goal_position(angleRotation)
        
            # read and publish data
            self.motors[i].read_present_position()
            self.motors[i].read_present_current()
            self.motors[i].publish(self.motors[i].dxl_present_current)
            self.motors[i].publish_pp(self.motors[i].dxl_present_position)
        
def main(args=None):

    # parameters
    rate = 1/0.01   # Hz

    # ros init
    rospy.init_node("RosMotorsNode")

    # motors init
    motors = MotorsSubscriber(rate)

    while True :
        
        motors.actuate()
        motors.rate.sleep()

        if rospy.is_shutdown() :
            break

    # disable torque and close port
    for i in range(motors.num_motors):
        motors.motors[i].stop()


if __name__ == '__main__':
    main()
    