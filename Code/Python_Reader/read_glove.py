# -*- coding: utf-8 -*-
"""
Created on Thu May 28 04:15:18 2020

@authors: Jochen, Kristof
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import serial
import keyboard
import time
import numpy as np
import struct
import csv
import datetime
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

serport = '/dev/ttyACM0' #'COM5'

mode = 1
showData = 1
dataCount = 1000000
maxNumModes = 2

singleNode_Id = 0
glove_v1_Id = 1
glove_v2_Id = 2

sensors = ['wrist', 'palm', 'thumb', 'index', 'mid', 'ring', 'pinky']

path = './'


# Write CSV file
def write_CSV(filename, sensorNames, data, mode=0):
    try:
        with open(filename, 'w') as csvfile:
            i = 0
            for sName in sensorNames:
                if mode == 0:
                    csvfile.write("Sensor," + str(i) + ",4," + sName +",Orientation\n")
                elif mode == 1:
                    csvfile.write("Sensor," + str(i) + ",7," + sName +",OrientationAcceleration\n")
                i += 1
                
            csv_writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
            csv_writer.writerows(data)
    except Exception as e:
        print('Error in write_CSV:')
        print(e)
        return 0
    return 1

# helper functions for quaternion viz:
def rotate_vector(vector, rotation_matrix):
    return np.dot(rotation_matrix, vector)
def calculate_endpoint(start, a, b, c, d):
    rotation_matrix = R.from_quat([a, b, c, d]).as_matrix()
    unit_vector = np.array([0, 0, 1])
    endpoint = rotate_vector(unit_vector, rotation_matrix)
    return start + endpoint
start_point = np.array([0,0,0])

# returns packet data length in bytes (without header bytes)
def getPacketLength(mode, deviceId, packetId):
    if mode == 0:
        if deviceId == singleNode_Id:
            return 8, 1
        
        elif deviceId == glove_v1_Id:
            return 18, 3
        
        elif deviceId == glove_v2_Id:
            if packetId == 1:
                return 24, 4
            elif packetId == 2:
                return 18, 3
            
    elif mode == 1:
        if deviceId == singleNode_Id:
            return 14, 1
        
        elif deviceId == glove_v1_Id:
            return 24, 2
        
        elif deviceId == glove_v2_Id:
            if packetId == 1:
                return 24, 2
            elif packetId == 2:
                return 30, 3
            elif packetId == 3:
                return 30, 3
    
    # if nothing above, input was invalid -> return -1
    return -1


def isPacketValid(mode, deviceId, packetId):
    # for now, only mode = 0 or 1 is allowed. Change this accordingly
    if mode > 1:
        return False
    if packetId == 0 or packetId > 3:
        return False
    if deviceId == 0 and packetId > 1:
        return False
    if mode == 1 and packetId > 2:
        return False
    return True


######################################################################################
def cubeDraw(rotM):  # render and rotate a 3D Box according to a 4x4 rotation matrix
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glPushMatrix()
    glMultMatrixf(rotM)
    glBegin(GL_QUADS)
    glColor3fv((1.0,1.0,0.0))
    for i in range(len(surf)):
        glColor3fv(colors[i])
        for j in surf[i]:
            glVertex3fv(vertices[j])
    glEnd()
    glPopMatrix()
    pygame.display.flip()
    pygame.time.wait(1)
######################################################################################
    
    
    

##################################
# Graphics viz
##################################
pygame.init()
pygame.display.set_caption('Palm')
display = (400, 400)
pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
glEnable(GL_DEPTH_TEST) 
glMatrixMode(GL_PROJECTION)
gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
glLoadIdentity()
aspect_ratio = display[0]/display[1]
near_clip = 0.1
far_clip = 50.0
fov = 50.0 # 45.0
##################################
# perspective projection matrix:
f = 1.0 / np.tan(np.radians(fov)/2.0)
projM = np.array( [ [f/aspect_ratio, 0, 0, 0], [0, f, 0, 0], 
		    [0, 0, (far_clip+near_clip)/(near_clip-far_clip), -1], 
		    [0, 0, 2*far_clip*near_clip/(near_clip-far_clip), 0] ], dtype=np.float32)
glMultMatrixf(projM)
##################################
# modelview matrix:
glMatrixMode(GL_MODELVIEW)
glLoadIdentity()
glTranslatef(0.0, 0.0, -10.0)
# create 3D box:
vertices = ( ( 1, -1, -1), ( 1,  1, -1), (-1,  1, -1), (-1, -1, -1),
             ( 1, -1,  1), ( 1,  1,  1), (-1, -1,  1), (-1,  1,  1) )
surf = ((0, 1, 2, 3), (3, 2, 7, 6), (6, 7, 5, 4), (4, 5, 1, 0), (1, 5, 7, 2), (4, 0, 3, 6))
colors = ((1, 0, 0), (0, 1, 0), (1, 0.5, 0), (1, 1, 0), (0, 1, 1), (0, 0, 1))
# init the cube:
rotM = np.eye(4)  # identity matrix
cubeDraw(rotM)



#################################
# open serial port and log file
#################################
ser = serial.Serial()
ser.baudrate = 500000
ser.port = serport
ser.timeout = 0 # = None means wait forever, = 0 means do not wait
ser.open()
# iteration variables for storing to a log file:
count = 0
nextTs = 1000
# helper variables:
syncBytes = b'\xAB\xCD'
#syncBytes = b'\x80\x80'
doSync = False
# object to store all data
data = np.zeros((dataCount,9))
filename = 'log.csv'



try:
    ser.reset_input_buffer()
    
    inData = b""
    mode = -1
    
    glove_v2_sample_5_pos = -1
    
    print("start synchronizing")
    
    # initial synchronization step. Done before starting the timer to not mess up time stamps
    while ser.is_open:
        inData += ser.read(100)
        syncPos = inData.find(syncBytes)
        if syncPos >= 0:
            inData = inData[syncPos+2:]
            break
    
    # start timer
    t = time.perf_counter()
    
    print("start receive loop")
    # receive loop
    while ser.is_open:
        
        # if requested: do synchronization
        if doSync:
            while ser.is_open:
                inData += ser.read(100)
                syncPos = inData.find(syncBytes)
                if syncPos >= 0:
                    inData = inData[syncPos+2:]
                    doSync = False
                    break
        
        # time stamp
        ts = int((time.perf_counter() - t)*1000)
        
        inData += ser.read(100)
        if len(inData) < 4:
            continue
        
        # read header (might be sync bytes)
        header = inData[:2]
        if header == syncBytes:
            header = inData[2:4]
            inData = inData[4:]
            # header always has packetId = 1 after sync
            if header[1] & 0x03 != 1:
                print("got invalid sync sequence. Skip to next sync point")
                doSync = True
                continue
        else:
            # strip header data from inData
            inData = inData[2:]
            
        if header[0] & 0x08 or header[1] & 0x10:
            print("Received invalid packet header: Control bits invalid. Header is:")
            print(header)
            print("Skip to next sync point...")
            doSync = True
            continue
        
        # get header data
        nodeId = header[0] >> 4
        deviceId = header[0] & 0x07
        mode = header[1] >> 5
        sampleId = (header[1] >> 2) & 0x03
        packetId = header[1] & 0x03
        
        # do some basic header check
        if not isPacketValid(mode, deviceId, packetId):
            print("Received invalid packet header: Mode, deviceId, or packetId invalid.")
            print(f"Mode:     {mode}")
            print(f"DeviceID: {deviceId}")
            print(f"PacketId: {packetId}")
            print("Skip to next sync point...")
            doSync = True
            continue
            
        # Todo: do some more sanity check with header data
        # i.e. check sampleID and packetID not equal to the one before etc.
        
        packetLength, numSamples = getPacketLength(mode, deviceId, packetId)
        
        
        # ensure we already received the whole packet, otherwise wait until it arrives
        while len(inData) < packetLength:
            inData += ser.read(100)
        
        packet = inData[:packetLength]
        inData = inData[packetLength:]
        
        
        if mode == 0:
            if deviceId == singleNode_Id:
                #packetLength = 8, 1
                quat = np.array(struct.unpack('<4h', packet)) / 16384
                data[count] = [nodeId << 4, ts, quat[1], -quat[3], quat[2], quat[0], 0, 0, 0]
                count += 1
            else:
                #packetLength = 18, 3 or 24, 4
                for i in range(numSamples):
                    quatV = np.array(struct.unpack('<3h', packet[6*i:6*(i+1)])) / 16384
                    # TODO: Below can result in NaN (Runtimewarning sqrt), dirty fix:
                    sumsq = np.sum(quatV*quatV)
                    if sumsq <= 1:
                         quatW = np.array([np.sqrt(1.0 - sumsq)])
                    else:
                         quatW = np.array([0])
                    
                    if deviceId == glove_v2_Id:
                        ID = (nodeId << 4) + 4*(packetId-1) + i
                    else:
                        ID = (nodeId << 4) + numSamples*(packetId-1) + i
                    data[count] = [ID, ts, quatV[0], -quatV[2], quatV[1], quatW[0], 0, 0, 0]
                    
                    #######################################################################################
                    if ID == 0:  # here we catch the first (thumb) sensor:
                        print( str( data[count][1].astype(int) ).zfill(7)+' {:+.3f}'.format(data[count][2])
                               +' {:+.3f}'.format(data[count][3])+' {:+.3f}'.format(data[count][4])
                               +' {:.3f}'.format(data[count][5]))
                        imuQ = [data[count][5], data[count][2], data[count][3], data[count][4]] # get quaternion
                        rotM[:3,:3] = R.from_quat(imuQ).as_matrix()  # convert to rotation matrix
                        cubeDraw(rotM)
                        count += 1
                    #######################################################################################

                                   
                
        elif mode == 1:
            if deviceId == singleNode_Id:
                # packetLength = 14, 1
                rcvd = np.array(struct.unpack('<7h', packet))
                quat = rcvd[:4] / 16384
                acc = rcvd[4:] / 100
                
                data[count] = [nodeId << 4, ts, quat[1], -quat[3], quat[2], quat[0], -acc[0], acc[2], -acc[1]]
            
            elif deviceId == glove_v1_Id:
                # packetLength = 24, 2
                for i in range(numSamples):
                    rcvd = np.array(struct.unpack('<6h', packet[12*i:12*(i+1)]))
                    quatV = rcvd[:3] / 16384
                    quatW = np.array([np.sqrt(1.0 - np.sum(quatV*quatV))])
                    acc = rcvd[3:] / 100
                    
                    ID = (nodeId << 4) + numSamples*(packetId-1) + i
                    data[count] = [ID, ts, quatV[0], -quatV[2], quatV[1], quatW, -acc[0], acc[2], -acc[1]]
                    count += 1
            
            elif deviceId == glove_v2_Id:
                if packetId == 1:
                    # packetLength = 24, 2
                    for i in range(numSamples):
                        rcvd = np.array(struct.unpack('<6h', packet[12*i:12*(i+1)]))
                        quatV = rcvd[:3] / 16384
                        quatW = np.array([np.sqrt(1.0 - np.sum(quatV*quatV))])
                        acc = rcvd[3:] / 100
                        
                        ID = (nodeId << 4) + numSamples*(packetId-1) + i
                        data[count] = [ID, ts, quatV[0], -quatV[2], quatV[1], quatW, -acc[0], acc[2], -acc[1]]
                        count += 1
                        
                        # here we need to pay attention for packets 2 and 3 because sample #5
                        # is spread across both packets
                        # When we see packet 1, we invalidate it in case packet 2 or 3 went missing
                        glove_v2_sample_5_pos = -1
                        
                elif packetId == 2:
                    # packetLength = 30, 3
                    for i in range(numSamples):
                        rcvd = np.array(struct.unpack('<6h', packet[12*i:12*(i+1)]))
                        quatV = rcvd[:3] / 16384
                        quatW = np.array([np.sqrt(1.0 - np.sum(quatV*quatV))])
                        acc = rcvd[3:] / 100
                        
                        ID = (nodeId << 4) + numSamples*(packetId-1) + i
                        data[count] = [ID, ts, quatV[0], -quatV[2], quatV[1], quatW, -acc[0], acc[2], -acc[1]]
                        count += 1
                        
                    quatV = np.array(struct.unpack('<3h', packet[24:30])) / 16384
                    quatW = np.array([np.sqrt(1.0 - np.sum(quatV*quatV))])
                    data[count] = [ID, ts, quatV[0], -quatV[2], quatV[1], quatW, 0, 0, 0]
                    
                    glove_v2_sample_5_pos = count
                    count += 1
                    
                elif packetId == 3:
                    # packetLength = 30, 3
                    for i in range(numSamples):
                        rcvd = np.array(struct.unpack('<6h', packet[12*i:12*(i+1)]))
                        quatV = rcvd[:3] / 16384
                        quatW = np.array([np.sqrt(1.0 - np.sum(quatV*quatV))])
                        acc = rcvd[3:] / 100
                        
                        ID = (nodeId << 4) + numSamples*(packetId-1) + i
                        data[count] = [ID, ts, quatV[0], -quatV[2], quatV[1], quatW, -acc[0], acc[2], -acc[1]]
                        count += 1
                        
                    acc = np.array(struct.unpack('<3h', packet[24:30])) / 100
                    
                    if glove_v2_sample_5_pos > 0:
                        data[glove_v2_sample_5_pos][6:9] = [-acc[0], acc[2], -acc[1]]
                        count += 1
                    #else:
                        # packet 2 is missing
        
        
        if ts >= nextTs:
            #print('time: ' + str(ts/1000) + '  count: ' + str(count))
            nextTs += 1000
        
        if 0: #keyboard.is_pressed('escape'):
            if count > 0:
                # adjust data for displaying purposes
                data = data[:count]
                data[:,1] -= np.min(data[:,1])
            
            # set count to zero not write to a file
            count = 0
            break
except Exception as e: 
    print('An error occured during recording')
    print(e)


finally:
    ser.close()


if count > 0:
    # adjust data to write
    data = data[:count]
    data[:,1] -= np.min(data[:,1])
    
    
    print('\nwriting to file...')
    
    if mode == 0:
        dataToWrite = [[int(d[0]), int(round(d[1])), d[2], d[3], d[4], d[5]] for d in data]
    elif mode == 1:
        dataToWrite = [[int(d[0]), int(round(d[1])), d[2], d[3], d[4], d[5], d[6], d[7], d[8]] for d in data]
    
    # write data to file (otherwise recording was aborted)
    if write_CSV(path+filename, sensors, dataToWrite, mode):
        print('written to ' + path + filename)
    
else:
    print('recording aborted or no data available')
	
