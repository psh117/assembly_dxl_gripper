#!/usr/bin/env python

import dynamixel_sdk as dxl

dxl_id_map = {'panda_right_finger_left':1, 'panda_right_finger_right':2,
              'panda_left_finger_left':3, 'panda_left_finger_right':4, 
              'panda_top_finger_left':5, 'panda_top_finger_right':6}

hand_name_map = {'panda_right' : ['panda_right_finger_right','panda_right_finger_left']
                # ,'panda_left' : ['panda_left_finger_right','panda_left_finger_left']
                ,'panda_top' : ['panda_top_finger_right','panda_top_finger_left']
                }

desired_length = {'panda_right' : 0.0
                ,'panda_left' : 0.0
                ,'panda_top' : 0.0
                }

desired_current = {'panda_right' : 0
                ,'panda_left' : 0
                ,'panda_top' : 0
                }

# Control table address
ADDR_OPERATING_MODE     = 11
ADDR_CURRENT_LIMIT      = 36
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model


ADDR_GOAL_CURRENT       = 102
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_VELOCITY   = 128
ADDR_PRESENT_POSITION   = 132

# Data Byte Length
LEN_GOAL_CURRENT        = 2
LEN_CURRENT_LIMIT       = 2
LEN_GOAL_POSITION       = 4
LEN_PRESENT_VELOCITY    = 4
LEN_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

INIT_CURRENT                = -30


CURRENT_CONTROL_MODE = 0
EXT_POSITION_CONTROL_MODE = 4
CURRENT_POSITION_CONTROL_MODE = 5

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600

#MAX_GRIPPER_POS = 15300
# MAX_GRIPPER_POS = 14785
MAX_GRIPPER_POS = 13180
# M_TO_POS = 188679.245283019
# M_TO_POS = 171527.272727273
M_TO_POS = 164750.0

CURRENT_LIMIT = 110

def error_handle(dxl_comm_result, dxl_error, packet_handler):
    if dxl_comm_result != dxl.COMM_SUCCESS:
        print("%s" % packet_handler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packet_handler.getRxPacketError(dxl_error))
