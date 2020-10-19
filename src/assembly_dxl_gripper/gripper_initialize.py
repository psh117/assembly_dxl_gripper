#!/usr/bin/env python

from __future__ import print_function
import rospy
import numpy as np
import os
import ctypes
import dynamixel_sdk as dxl
from assembly_dxl_gripper import *

if __name__ == '__main__':    
    rospy.init_node('gripper_init')

    portHandler = dxl.PortHandler(DEVICENAME)
    packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncWrite instance
    groupSyncWrite = dxl.GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    groupSyncRead = dxl.GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY+LEN_PRESENT_POSITION)
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
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

    # Enable Dynamixel Torque & current control mode
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            e = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
            error_handle(e[0], e[1], packetHandler)
            rospy.sleep(0.05)
            e = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            error_handle(e[0], e[1], packetHandler)
            rospy.sleep(0.05)

    # set desried current
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            e = packetHandler.write2ByteTxRx(portHandler, dxl_id_map[key], ADDR_GOAL_CURRENT, INIT_CURRENT)
            error_handle(e[0], e[1], packetHandler)

    rospy.sleep(1.0)

    while True:
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        is_stopped = True
        pos = {}
        for arm in hand_name_map:
            for key in hand_name_map[arm]:
                vel = groupSyncRead.getData(dxl_id_map[key], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
                pos[key] = int(np.int32(groupSyncRead.getData(dxl_id_map[key], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)))
                print (key, '- vel: ', vel)
                print (key, '- pos: ', pos[key])
                if vel is not 0:
                    is_stopped = False
                    break

        if is_stopped:
            for arm in hand_name_map:
                for key in hand_name_map[arm]:
                    rospy.set_param('/assembly_dxl_gripper/gripper_init', 1)
                    rospy.set_param('/assembly_dxl_gripper/' + key + '_init_pos', pos[key])
            break

    # set current to ZERO
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            e = packetHandler.write2ByteTxRx(portHandler, dxl_id_map[key], ADDR_GOAL_CURRENT, 0)
            error_handle(e[0], e[1], packetHandler)

    # Disable Dynamixel Torque
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            e = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            error_handle(e[0], e[1], packetHandler)

    print('done')
