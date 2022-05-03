from dynamixel_sdk import *

import rospy
from std_msgs.msg import Float64

#################### Class one motor ######################

class XM430():
    """
    class for the motor dynamixel XM430 
    Parameters:
        id : motor id
        rosNode : node ros2 to attach publisher and subscriber
        base : define if the motor is for the translation or not  
    """

    # Control table address
    ADDR_BAUD_RATE              = 8
    ADDR_OPERATING_MODE         = 11
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    ADDR_P_GAIN_POSITION        = 84
    ADDR_I_GAIN_POSITION        = 82
    ADDR_D_GAIN_POSITION        = 80
    ADDR_GOAL_CURRENT           = 102
    ADDR_PRESENT_CURRENT        = 126

    # Protocol version
    PROTOCOL_VERSION            = 1.0         

    # Use the actual port assigned to the U2D2.
    # Linux: "/dev/ttyUSB*"
    DEVICENAME                  = '/dev/ttyUSB0'

    BAUDRATE                    = 57600
    CONTROL_MODE                = 4                 # The value of  Control Mode (extended position = 4, current/position = 5)
    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    MAX_POSITION_VALUE          = 1048575           # Of MX with firmware 2.0 and X-Series the revolution on Extended Position Control Mode is 256 rev
    DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel will rotate between this value


    def __init__(self,id : int):

        self.dxl_id = id
        self.length = 0

        #Initialize ROS Subscriber
        self.initgrah()

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        # Set operating mode to extended position control mode
        self.dxl_comm_result, self.dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, self.ADDR_OPERATING_MODE, self.CONTROL_MODE)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
        else:
            print("Operating mode changed to extended position control mode.")

        # Enable Dynamixel Torque
        self.dxl_comm_result, self.dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
        else:
            print("Dynamixel has been successfully connected")

    ############################
    # PID Coefficient 
    ############################   

    def write_pid_coefficient(self,proportionnal,integration,derivation):
        self.dxl_comm_result, self.dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.dxl_id, self.ADDR_P_GAIN_POSITION, proportionnal)
        self.dxl_comm_result, self.dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.dxl_id, self.ADDR_I_GAIN_POSITION, integration)
        self.dxl_comm_result, self.dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.dxl_id, self.ADDR_D_GAIN_POSITION, derivation)

    ########################### Data  ######################

    ############################
    # POSITION 
    ############################

    # Write goal position
    def write_goal_position(self,goal_position):

        self.dxl_comm_result, self.dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.dxl_id, self.ADDR_GOAL_POSITION, goal_position)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))

    # Read present position
    def read_present_position(self):

        self.dxl_present_position, self.dxl_comm_result, self.dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler,self.dxl_id, self.ADDR_PRESENT_POSITION)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))

   
    ############################
    # Current 
    ############################

    # # Write goal current
    def write_goal_current(self,goal_current):

        self.dxl_comm_result, self.dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.dxl_id, self.ADDR_GOAL_CURRENT, goal_current)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))

    # Read present current
    def read_present_current(self):

        self.dxl_present_current, self.dxl_comm_result, self.dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler,self.dxl_id, self.ADDR_PRESENT_CURRENT)
        if self.dxl_present_current > 0x7fff:
            self.dxl_present_current = self.dxl_present_current - 65536
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))


    ############################### ROS functions #########################

    def initgrah(self):
        self.sub = rospy.Subscriber('Motor'+str(self.dxl_id)+'/ref', Float64, self.on_receive_callback)
        self.pub_pp = rospy.Publisher('Motor'+str(self.dxl_id)+'/state', Float64, queue_size = 10)
        self.pub = rospy.Publisher('Motor'+str(self.dxl_id)+'/current', Float64, queue_size =10)


    def on_receive_callback(self,data):
            self.length = data.data

    # for current
    def publish(self,data):
        msg = Float64()
        msg.data = float(data)
        self.pub.publish(msg)

    # for present position
    def publish_pp(self,data):
        msg = Float64()
        msg.data = float(data)
        self.pub_pp.publish(msg)

    ############################
    # Turn off 
    ############################

    # Clear Multi-Turn Information
    def clear_informations(self):

        self.dxl_comm_result, self.dxl_error = self.packetHandler.clearMultiTurn(self.portHandler, self.dxl_id)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
    

    # Disable Dynamixel Torque
    def stop(self):

        self.dxl_comm_result, self.dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.dxl_id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if self.dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
        elif self.dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
        
        self.portHandler.closePort()
        