#!/usr/bin/env python3 

import rospy
from std_msgs.msg import Float64


########### class multiple motor ######################
class RateLimiter():
    """"
    Limit angles speed
    Parameters :
        rate : rate message in Hz
    """
    
    def __init__(self,rate : float = 1/0.01):

        self.rate = rate
        self.rosRate = rospy.Rate(rate)

        # get parameters
        self.rate_limit = rospy.set_param('rate_limit',3.14/2)
        self.id = rospy.set_param("id",1)

        # get parameters
        self.rate_limit = rospy.get_param('~rate_limit')
        self.id = rospy.get_param("~id")
        print(self.id)
        self.output = 0
        self.input = 0

        # Initialize ROS variables
        self.sub = rospy.Subscriber("Angle"+str(self.id)+"/ref",Float64,self.callback,queue_size=10)
        self.pub = rospy.Publisher("AngleSat"+str(self.id)+"/state",Float64,queue_size=10)

    ############# ROS functions ###############
    def callback(self,data):
        self.input = data.data

    # for current
    def publish(self,data):
        msg = Float64()
        msg.data = float(data)
        self.pub.publish(msg)

    ###########################

    def rateLimit(self,r,u,y,dt):
        
        rate = (u-y)/dt
        if rate > r : 
            output = dt*r + y
        elif rate < -r:
            output = -r*dt + y
        else : 
            output = u
        return output
    
    def actuate(self):

        self.output = self.rateLimit(self.rate_limit,self.output,self.input,1/self.rate)
        self.publish(self.output)
            
        
def main(args=None):

    # parameters
    rate = 1/0.01   # Hz

    # ros init
    rospy.init_node("RosRateNode")

    # motors init
    rateLimiter = RateLimiter(rate)

    while True :
        
        rateLimiter.actuate()
        rateLimiter.rosRate.sleep()

        if rospy.is_shutdown() :
            break

if __name__ == '__main__':
    main()