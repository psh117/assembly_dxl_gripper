#!/usr/bin/env python

from __future__ import print_function
import rospy
import numpy as np
import os
import ctypes
import dynamixel_sdk as dxl
from assembly_dxl_gripper import *
from assembly_dxl_gripper.srv import *

if __name__ == '__main__':    
    rospy.init_node('gripper_server')

    portHandler = dxl.PortHandler(DEVICENAME)
    packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

    init_pos = {}
    for key in dxl_id_map:
        # rospy.get_param('/assembly_dxl_gripper/gripper_init', 1)
        init_pos[key] = rospy.get_param('/assembly_dxl_gripper/' + key + '_init_pos')    
    
    # Initialize GroupSyncWrite instance
    groupSyncWrite = dxl.GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    groupSyncRead = dxl.GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY+LEN_PRESENT_POSITION)
    
    for key in dxl_id_map:
        groupSyncRead.addParam(dxl_id_map[key])

    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")

    # Set mode to current mode
    for key in dxl_id_map:
        e = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        error_handle(e[0], e[1], packetHandler)

    # Enable Dynamixel Torque
    for key in dxl_id_map:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        error_handle(e[0], e[1], packetHandler)

    def move_gripper(req):
        print ("move gripper")
        desired_length = req.length[0]
        
        if req.length[0] > 0.08:
            print (req.length[0], 'over the limit')
            return MoveResponse()
        if req.length[0] < 0.0:
            print (req.length[0], 'over the limit')
            return MoveResponse()
        # desired_position = int(desired_length)

        groupSyncWrite.clearParam()

        for key in dxl_id_map:
            desired_position = MAX_GRIPPER_POS - int(desired_length * M_TO_POS) + init_pos[key]
            print('desired_position', desired_position)
            param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(desired_position)), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(desired_position)), 
            dxl.DXL_LOBYTE(dxl.DXL_HIWORD(desired_position)), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(desired_position))]
            groupSyncWrite.addParam(dxl_id_map[key], param_goal_position)
        
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("%s" % packet_handler.getTxRxResult(dxl_comm_result))

        return MoveResponse()

    s = rospy.Service('/assembly_dxl_gripper/move', Move, move_gripper)
    rospy.spin()

    # set desried current
    for key in dxl_id_map:
        e = packetHandler.write2ByteTxRx(portHandler, dxl_id_map[key], ADDR_GOAL_CURRENT, 0)
        error_handle(e[0], e[1], packetHandler)

    # Disable Dynamixel Torque
    for key in dxl_id_map:
        e = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        error_handle(e[0], e[1], packetHandler)
        
    print('done')


