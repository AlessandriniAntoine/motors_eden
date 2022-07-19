#!/usr/bin/env python
# -*- coding: utf-8 -*-

from dynamixel_sdk import *

class MotorsSync():

    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_POSITION       = 132
    ADDR_PRESENT_CURRENT        = 126
    ADDR_OPERATING_MODE         = 11

    LEN_GOAL_POSITION           = 4         # Data Byte Length
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    LEN_GOAL_CURRENT            = 4         # Data Byte Length
    LEN_PRESENT_CURRENT         = 4         # Data Byte Length

    BAUDRATE                    = 1000000
    
    # DYNAMIXEL Protocol Version (1.0 / 2.0)
    # https://emanual.robotis.com/docs/en/dxl/protocol2/
    PROTOCOL_VERSION            = 2.0

    # Use the actual port assigned to the U2D2.
    # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
    DEVICENAME                  = '/dev/ttyUSB0'

    CONTROL_MODE                = 4
    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque

    def __init__(self,ids):
        self.ids = ids
        self.numMotors = len(self.ids)

        self.initMotors()
        
    def initMotors(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite_position = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)

        # Initialize GroupSyncRead instace for Present Position
        self.groupSyncRead_position = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)

        # Initialize GroupSyncRead instace for Present current
        self.groupSyncRead_current = GroupSyncRead(self.portHandler, self.packetHandler, self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)

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
        for id in self.ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_OPERATING_MODE, self.CONTROL_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Operating mode changed to extended position control mode.")

        # Enable Dynamixels Torque
        for id in self.ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % id)


        # Add parameter storage for Dynamixel present position value
        for id in self.ids:
            dxl_addparam_result = self.groupSyncRead_position.addParam(id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % id)
                quit()

        # Add parameter storage for Dynamixels current value
        for id in self.ids:
            dxl_addparam_result = self.groupSyncRead_current.addParam(id)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncRead current addparam failed" % id)
                quit()

    ###############################
    # Positions Functions
    ##############################

    def write_position(self,positions):
        # Allocate goal position value into byte array
        param_goal_position = [[DXL_LOBYTE(DXL_LOWORD(position)), DXL_HIBYTE(DXL_LOWORD(position)), DXL_LOBYTE(DXL_HIWORD(position)), DXL_HIBYTE(DXL_HIWORD(position))] for position in positions]

        # Add Dynamixel goal position value to the Syncwrite parameter storage
        for i in range(self.numMotors):
            dxl_addparam_result = self.groupSyncWrite_position.addParam(self.ids[i], param_goal_position[i])
            if dxl_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % self.ids[i])
                quit()

        # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite_position.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite_position.clearParam()

    def read_position(self):
        # Syncread present position
        dxl_comm_result = self.groupSyncRead_position.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixels is available
        for id in self.ids:
            dxl_getdata_result = self.groupSyncRead_position.isAvailable(id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
                quit()

        # Get Dynamixel#1 present position value
        self.dxl_present_positions = [self.groupSyncRead_position.getData(id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION) for id in self.ids]


        # print("[ID:%03d]  PresPos:%03d\t[ID:%03d]  PresPos:%03d" % (self.ids[0],  self.dxl_present_positions[0], self.ids[1], self.dxl_present_positions[1]))
        return self.dxl_present_positions

    ###############################
    # current Functions
    ##############################            

    def read_current(self):
        # Syncread present position
        dxl_comm_result = self.groupSyncRead_current.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixels is available
        for id in self.ids:
            dxl_getdata_result = self.groupSyncRead_current.isAvailable(id, self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % id)
                quit()

        # Get Dynamixel#1 present position value
        dxl_present_currents = [self.groupSyncRead_current.getData(id, self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT) for id in self.ids]


        # print("[ID:%03d]  PresCur:%03d\t[ID:%03d]  PreCur:%03d" % (self.ids[0],  dxl_present_currents[0], self.ids[1], dxl_present_currents[1]))
        return dxl_present_currents

    ###############################
    # other Functions
    ############################## 

    def turnOFF(self):
        # Clear syncread parameter storage
        self.groupSyncRead_current.clearParam()
        self.groupSyncRead_position.clearParam()

        # Disable Dynamixels Torque
        for id in self.ids :
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()