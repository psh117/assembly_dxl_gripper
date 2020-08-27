#!/usr/bin/env python

from __future__ import print_function
import rospy
import numpy as np
import os
import ctypes
import dynamixel_sdk as dxl
from sensor_msgs.msg import JointState
from assembly_dxl_gripper import *
from assembly_dxl_gripper.srv import *
from threading import Lock
import math

if __name__ == '__main__':  

    rospy.init_node('gripper_server')
    lock = Lock()
    init_pos = {}
    pos = {}
    vel = {}
    
    portHandler = dxl.PortHandler(DEVICENAME)
    packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
    groupSyncWrite = dxl.GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    groupSyncRead = dxl.GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY+LEN_PRESENT_POSITION)

    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            init_pos[key] = rospy.get_param('/assembly_dxl_gripper/' + key + '_init_pos')
            groupSyncRead.addParam(dxl_id_map[key])

    # Open port
    if portHandler.openPort(): print("Succeeded to open the port")
    else: print("Failed to open the port")

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE): print("Succeeded to change the baudrate")
    else: print("Failed to change the baudrate")
    rospy.sleep(0.1)

    # Enable Dynamixel Torque & ext position control mode
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            e = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
            error_handle(e[0], e[1], packetHandler)
            rospy.sleep(0.05)
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            error_handle(e[0], e[1], packetHandler)
            rospy.sleep(0.05)

    def move_gripper(req):
        print ("move gripper")

        print ('position control started')
        groupSyncWrite.clearParam()
        for i in range(len(req.length)):
            try: arm = req.hand[i]
            except: arm = 'panda_right'
            desired_length[arm] = req.length[i]
            try: desired_current[arm] = req.max_current[i]
            except: desired_current[arm] = 0.0

            print('controlling',arm)
            if desired_length[arm] > 0.08:
                print (arm, desired_length[arm], 'over the limit')
                return MoveResponse()
            if desired_length[arm] < -0.008:
                print (arm, desired_length[arm], 'over the limit')
                return MoveResponse()

            for key in hand_name_map[arm]:
                desired_position = MAX_GRIPPER_POS - int(desired_length[arm] * M_TO_POS) + init_pos[key]
                print('desired_position', desired_position)
                param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(desired_position)), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(desired_position)), 
                dxl.DXL_LOBYTE(dxl.DXL_HIWORD(desired_position)), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(desired_position))]
                groupSyncWrite.addParam(dxl_id_map[key], param_goal_position)
                rospy.sleep(0.05)

        with lock:
            dxl_comm_result = groupSyncWrite.txPacket()
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        rospy.sleep(0.5)
        
        while True:
            is_stopped = True
            for arm in req.hand:
                for key in hand_name_map[arm]:
                    if abs(vel[key]) > 1:
                        print('abs(vel[key])',abs(vel[key]))
                        is_stopped = False
            if is_stopped is True:
                break
            rospy.sleep(0.1)

        return MoveResponse()

    s = rospy.Service('/assembly_dxl_gripper/move', Move, move_gripper)
    joint_pub = rospy.Publisher('/panda_right_gripper/joint_states',JointState, queue_size=3)
    rate = rospy.Rate(30)
    msg = JointState()
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            msg.name.append(key + '_joint')
    while not rospy.is_shutdown():
        with lock:
            dxl_comm_result = groupSyncRead.txRxPacket()
            if dxl_comm_result != dxl.COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        msg.position = []
        msg.velocity = []
        for arm in hand_name_map:
            for key in hand_name_map[arm]:
                vel[key] = groupSyncRead.getData(dxl_id_map[key], ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
                pos[key] = (MAX_GRIPPER_POS - int(np.int32(groupSyncRead.getData(dxl_id_map[key], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))) + init_pos[key]) / M_TO_POS / 2

                msg.position.append(pos[key])
                msg.velocity.append(vel[key])
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        joint_pub.publish(msg)
        rate.sleep()

    # Disable Dynamixel Torque
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            e = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            error_handle(e[0], e[1], packetHandler)

    print('done')


