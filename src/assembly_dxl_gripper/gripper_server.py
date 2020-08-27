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

    def open_port(portHandler):
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")

        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")

    rospy.init_node('gripper_server')

    lock = Lock()

    portHandler = dxl.PortHandler(DEVICENAME)
    # portHandler_test = dxl.PortHandler(DEVICENAME)
    packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
    # packetHandler_test = dxl.PacketHandler(PROTOCOL_VERSION)
    pos = {}
    vel = {}
    groupSyncWrite = dxl.GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    groupSyncRead = dxl.GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY+LEN_PRESENT_POSITION)
    print('test1')
    # groupSyncWrite_test = dxl.GroupSyncWrite(portHandler_test, packetHandler_test, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    # groupSyncRead_test = dxl.GroupSyncRead(portHandler_test, packetHandler_test, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY+LEN_PRESENT_POSITION)
    print('test2')
    init_pos = {}
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            # rospy.get_param('/assembly_dxl_gripper/gripper_init', 1)
            init_pos[key] = rospy.get_param('/assembly_dxl_gripper/' + key + '_init_pos')
    
    # Initialize GroupSyncWrite instance

    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            groupSyncRead.addParam(dxl_id_map[key])

    # Enable Dynamixel Torque & position control
    open_port(portHandler)
    for arm in hand_name_map:
        for key in hand_name_map[arm]:
            e = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
            error_handle(e[0], e[1], packetHandler)
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id_map[key], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            error_handle(e[0], e[1], packetHandler)

    # # Enable Dynamixel Torque & current control
    # open_port(portHandler_test)
    # for arm in hand_name_map:
    #     for key in hand_name_map[arm]:
    #         e = packetHandler_test.write1ByteTxRx(portHandler_test, dxl_id_map[key], ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
    #         error_handle(e[0], e[1], packetHandler_test)
    #         dxl_comm_result, dxl_error = packetHandler_test.write1ByteTxRx(portHandler_test, dxl_id_map[key], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    #         error_handle(e[0], e[1], packetHandler_test)


    def move_gripper(req):
        print ("move gripper")
        # set current to ZERO
        for arm in hand_name_map:
            for key in hand_name_map[arm]:
                e = packetHandler.write2ByteTxRx(portHandler, dxl_id_map[key], ADDR_GOAL_CURRENT, 0)
                error_handle(e[0], e[1], packetHandler)


        print ('position control started')
        groupSyncWrite.clearParam()
        if len(req.hand) == 0 and len(req.length) > 0: req.hand.append('panda_right')
        elif len(req.hand) == 1 :
            if req.hand[0] == '':
                req.hand[0] = 'panda_right'
        for i in range(len(req.hand)):
            arm = req.hand[i]
            desired_length[arm] = req.length[i]
            desired_current[arm] = req.max_current[i]
            print('controlling',arm)
            # # Set mode to position control mode
            # for key in hand_name_map[arm]:

            
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



        # print('current control started')
        # # set current to desried current
        # for arm in req.hand:
        #     # Set mode to current mode
        #     for key in hand_name_map[arm]:
        #         e = packetHandler.write1ByteTxRx(portHandler_test, dxl_id_map[key], ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
        #         error_handle(e[0], e[1], packetHandler)
        #     for key in hand_name_map[arm]:
        #         if abs(desired_current[arm]) < 0.5: e = packetHandler.write2ByteTxRx(portHandler_test, dxl_id_map[key], ADDR_GOAL_CURRENT, 0)
        #         else: e = packetHandler.write2ByteTxRx(portHandler_test, dxl_id_map[key], ADDR_GOAL_CURRENT, desired_current[arm])
        #         error_handle(e[0], e[1], packetHandler)


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
        
    # set desried current
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


