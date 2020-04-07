'''
Obtain datasets of UWB measurements in TWR mode with crazyflies
Requirements: optiTrack system and crazyflie python library
Author: Sven Pfeiffer, MAVLab, TUDelft
'''
import logging
import time
#from threading import Timer

import cflib.crtp
from cflib.crazyflie import Crazyflie
#from cflib.crazyflie.log import LogConfig
from cflib.drivers.crazyradio import Crazyradio
from cflib.crazyflie.extpos import Extpos

#import ast
#import sys
from NatNetClient import NatNetClient
from FileLogger import FileLogger

import numpy as np
#import math
import trajectories
import customUtils as util

import preparedTrajectories

# system configurations
cf_uri = 'radio://0/100/2M/E7E7E7E7E7'
optiTrackID = 1 # drone's ID in optiTrack
fileName = '../Logs/apr7/twr_hover_3.csv'

# Trajectory
#setpoints = preparedTrajectories.traj6() # traj1 to traj6
setpoints = [(0, 0, 2.0, 0)]            # single setpoint for hover
doHover = True # hover at first setpoint


"""
initial_pos = [0,0]
altitude = 1.5
yaw = 0
setpoints = []
setpoints += trajectories.takeoff(initial_pos, altitude, yaw)
setpoints += trajectories.xySquare(2.0, altitude, yaw)
setpoints += trajectories.land(initial_pos, altitude, yaw)
"""
# optitrack data for control
ot_position = np.zeros(3)
ot_attitude = np.zeros(3)

cflib.crtp.init_drivers(enable_debug_driver=False)
cf = Crazyflie(rw_cache='./cache')

# logger setup
flogger = FileLogger(cf, cf_uri, fileName)
flogger.enableConfig('otpos')
flogger.enableConfig('otatt')
flogger.enableConfig('attitude')
flogger.enableConfig('gyros')
flogger.enableConfig('acc')
flogger.enableConfig('twr')
flogger.start()

def OT2NED(vector3D_ot):
    # convert vector from OT coordinates to NED
    vector3D_ned = [0,0,0]
    vector3D_ned[0] = vector3D_ot[0]  # NED.x = OT.x
    vector3D_ned[1] = vector3D_ot[2]  # NED.y = OT.z
    vector3D_ned[2] = -vector3D_ot[1] # NED.z = -OT.y
    return vector3D_ned

def OT2CONTROL(vector3D_ot):
    # convert vector from OT coordinates to NED
    vector3D_ned = [0,0,0]
    vector3D_ned[0] = vector3D_ot[2]  # CONTROL.x = OT.z
    vector3D_ned[1] = vector3D_ot[0]  # CONTROL.y = OT.x
    vector3D_ned[2] = vector3D_ot[1] # CONTROL.z = OT.y
    return vector3D_ned

# This is a callback function that gets connected to the NatNet client and called once per mocap frame.
def receiveNewFrame(frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
    pass

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receiveRigidBodyFrame(id, position, rotation):
    global flogger, ot_position, ot_attitude
    if id == optiTrackID:
        # register position
        ot_position = OT2CONTROL(position)
        ot_pos_dict = {'otX': ot_position[0],
                       'otY': ot_position[1],
                       'otZ': ot_position[2]}
        flogger.registerData('otpos', ot_pos_dict)
        #ot_position = OT2CONTROL(position)
        # register attitude
        rotation_euler = util.quat2euler(rotation)
        ot_attitude = OT2CONTROL(rotation_euler)
        ot_att_dict = {'otRoll': ot_attitude[0],
                       'otPitch': ot_attitude[1],
                       'otYaw': ot_attitude[2]}
        flogger.registerData('otatt', ot_att_dict)

streamingClient = NatNetClient() # Create a new NatNet client
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
streamingClient.run() # Run perpetually on a separate thread.
print("Optitrack streaming client started")
logging.basicConfig(level=logging.ERROR) # Only output errors from the logging framework


def followSetpoints(crazyflie, setpoints):
    global ot_position

    i = 0
    current_setpoint = setpoints[i]
    crazyflie.commander.send_position_setpoint(current_setpoint[0], current_setpoint[1], current_setpoint[2], current_setpoint[3])    
    while True:
        #check_battery()
        dist_from_setpoint = np.sqrt((ot_position[0]-current_setpoint[0])**2 + (ot_position[1]-current_setpoint[1])**2 + (ot_position[2]-current_setpoint[2])**2)
        if dist_from_setpoint < 0.1:
            i = i+1
            current_setpoint = setpoints[i]
            print("New setpoint: ({},{},{})".format(current_setpoint[0],current_setpoint[1],current_setpoint[2]))
        
        crazyflie.extpos.send_extpos(ot_position[0],ot_position[1],ot_position[2])
        #print('sending external position: ({},{},{})'.format(ot_position[0],ot_position[1],ot_position[2]))
        crazyflie.commander.send_position_setpoint(current_setpoint[0], current_setpoint[1], current_setpoint[2], current_setpoint[3])
        time.sleep(0.05)

        if i == len(setpoints)-1:
            print("Reached end of sequence \n")
            time.sleep(3)
            break


def hover(crazyflie, setpoint, duration):
    global ot_position

    crazyflie.commander.send_position_setpoint(setpoint[0],setpoint[1],setpoint[2],setpoint[3])

    print("start hovering")
    hovertime = 0
    while hovertime < duration:
        #check_battery()
        crazyflie.commander.send_position_setpoint(setpoint[0],setpoint[1],setpoint[2],setpoint[3])
        crazyflie.extpos.send_extpos(ot_position[0],ot_position[1],ot_position[2])
        hovertime = hovertime + 0.05
        time.sleep(0.05)
    print('Finished hovering')

'''
def check_battery():
    global battery_voltage
    if battery_voltage < 3.5:
        msg = "Low battery: %2d" % battery_voltage
        raise customExceptions.BatteryLow(msg)
'''

if __name__ == '__main__':
    while ot_position[0] == 0 and ot_position[1] == 0 and ot_position[2] == 0:
        print("waiting for optitrack fix...")
        time.sleep(1)
    print("reseting estimator...")
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)
    i = 0
    current_setpoint = setpoints[i]
    cf.commander.send_position_setpoint(current_setpoint[0], current_setpoint[1], current_setpoint[2], current_setpoint[3])
    print("New setpoint: ({},{},{})".format(current_setpoint[0],current_setpoint[1],current_setpoint[2]))
        
    try:
        if doHover:
            hover(cf, current_setpoint, 60.0)
            for i in range(10):
                cf.commander.send_position_setpoint(ot_position[0], ot_position[1], 0, 0)
                cf.extpos.send_extpos(ot_position[0],ot_position[1],ot_position[2])
                time.sleep(0.2)
        else:
            followSetpoints(cf, setpoints)

    except KeyboardInterrupt:
        print("stop")
        print("Landing")
        for i in range(10):
            cf.commander.send_position_setpoint(ot_position[0], ot_position[1], 0, 0)
            cf.extpos.send_extpos(ot_position[0],ot_position[1],ot_position[2])
            time.sleep(0.2)
    
    time.sleep(2)
    print("exiting... \n")
    cf.close_link()


