#!/usr/bin/env python

#!/usr/bin/env python3

# Data Collection for Both IMU and Vicon
# Data Collection for Both IMU and Vicon
# Data Collection for Both IMU and Vicon
# Data Collection for Both IMU and Vicon
# Data Collection for Both IMU and Vicon
# Data Collection for Both IMU and Vicon

#Importing python libraries
from multiprocessing import Process,Pipe
import time
from math import sin, cos, sqrt, atan2
import numpy as np
import time

#Importing Custom Functions
from gaitDetectClass import * #class gaitDetect, func testVal(shank gyZ, heel gyZ)
from packageHandlerFunc import * #package_handler_raw(tup)
from sensorClass import * #class sensorObject, newValues(valueArray), angularAccCalc(), angleCalc(gaitDetectObject)
from serialSend import * #ardno(msg as string)
from slipAlgorithmFunc import * #slipAlgorithm(pelvis_forward_acc, heel_forward_acc, L_hh)
from userinput import *
from variableDeclarations import *
from IMU_read import *
from IMU_initialization import *
from IMU_receiver import *
import numpy as np
from vicon_initialization import *
from vicon_receiver import *


#Importing Custom Functions
from gaitDetectClass import * #class gaitDetect, func testVal(shank gyZ, heel gyZ)
from packageHandlerFunc import * #package_handler_raw(tup)
from sensorClass import * #class sensorObject, newValues(valueArray), angularAccCalc(), angleCalc(gaitDetectObject)
from serialSend import * #ardno(msg as string)
from slipAlgorithmFunc import * #slipAlgorithm(pelvis_forward_acc, heel_forward_acc, L_hh)
from userinput import *
from variableDeclarations import *
from IMU_read import *
from IMU_initialization import *

#Turns data collection for particular sensors on/off if necessary.
toggleFlagDict = {
    "rThigh":  toggle_rThigh,
    "rShank":  toggle_rShank,
    "rHeel":   toggle_rHeel,
    "lThigh":  toggle_lThigh,
    "lShank":  toggle_lShank,
    "lHeel":   toggle_lHeel,
    "lowBack": toggle_lowBack,
    "topBack": toggle_topBack
}

#Variable initializations that can't be offloaded to another file (time)
timeCurrent = time.time()
timeStart = timeCurrent
timeLastRun = timeCurrent
    
###########################################################################################
#Setup
if __name__ == "__main__":
#Variable initializations

    #Create sensor objects to store and manipulate data for each sensor
    objRThigh = sensorObject("RT")
    objRShank = sensorObject("RS")
    objRHeel = sensorObject("RH")

    objLThigh = sensorObject("LT")
    objLShank = sensorObject("LS")
    objLHeel = sensorObject("LH")

    objLowBack = sensorObject("LB")
    
    #create gait detect objects for each leg
    gaitDetectRight = gaitDetect()
    gaitDetectLeft = gaitDetect()
    
    #Create lists that can be cycled through to iterate over every object for exporting (and creating the dump file data header).
    objects = []
    stringObjects = []
    
    if toggleFlagDict['lowBack'] == True:
        objects.append(objLowBack)
        stringObjects.append("LowBack")  
    if toggleFlagDict['rThigh'] == True:
        objects.append(objRThigh)
        stringObjects.append("RThigh")  
    if toggleFlagDict['lThigh'] == True:
        objects.append(objLThigh)
        stringObjects.append("LThigh")   
    if toggleFlagDict['rShank'] == True:
        objects.append(objRShank)
        stringObjects.append("RShank")  
    if toggleFlagDict['lShank'] == True:
        objects.append(objLShank)
        stringObjects.append("LShank")      
    if toggleFlagDict['rHeel'] == True:
        objects.append(objRHeel)
        stringObjects.append("RHeel")    
    if toggleFlagDict['lHeel'] == True:
        objects.append(objLHeel)
        stringObjects.append("LHeel")    
  
    stringAxes = ["x","y","z"]
    #stringSensors = ["gy","ac","mg"] # Use in place of below line if adding magnetometer readings to file output
    stringSensors = ["av","ac"]
    
    #########################################################################################
    # IMU_stream
    serial_connection = IMU_init()
    (parent_conn_IMU, child_conn_IMU) = Pipe()
    sender_IMU = Process(target = async_IMU, args = (parent_conn_IMU, serial_connection))
    sender_IMU.start()
    #########################################################################################

    #########################################################################################
    # vicon_stream, subject_name, marker_count = vicon_init("192.168.10.203")
    IP_ADDRESS = "192.168.10.200"
    vicon_enable = True
    #########################################################################################


    if vicon_enable:
        (parent_conn_vicon, child_conn_vicon) = Pipe()
        sender_vicon = Process(target = async_vicon, args = (parent_conn_vicon, IP_ADDRESS))
        sender_vicon.start()

    timestr = time.strftime("%Y%m%d-%H%M%S")
    file_name = "Rutgers/Data/" + "data-" + timestr + ".txt"
    file1 = open(file_name, "w")


    period_list = []
    current_frame = 0
    IMU_data_list = []
    current_f = 0
    totalResult = []
    error_sum = 0
    ty_sum = 0
    angle_error_sum = 0
    angle_pred_list = []
    outputAll = []
    DATA_LEN = 20
    propagation_list = []
    output_idx_list = []
    data_record_list = []


    header = "time\ttimeToRun\tgaitStageR\tgaitStageL\tslipR\tslipL\t"
    for x in stringObjects:
        for y in stringSensors:
            for z in stringAxes:
                header += f"{x}/{y}/{z}\t"
        header += f"zAngleZeroed/{x}\t"
        header += f"\t"
    header += f"\n"
    file1.write(header)

    #  Zero z_angles
    IMU_data = child_conn_IMU.recv()
    print('IMU Length',len(IMU_data))
    i = 0
    for x in objects:
        x.zAngleZeroed = float(IMU_data[i+6])
        i+=9

    t_calibrate_count =0
    while t_calibrate_count < 100:
        i = 0
        #  read data
        IMU_data = child_conn_IMU.recv()
        # Define z-zeroed angles:
        for x in objects:
            x.zAngleZeroed = (x.zAngleZeroed + float(IMU_data[i+6]))/2
            i+=9
        t_calibrate_count +=1

    print("Right Heel Z-angle zeroed:",objRHeel.zAngleZeroed)
    print("Left Heel Z-angle zeroed:",objLHeel.zAngleZeroed)

    t0 = time.time()

    try:
        while True:
            time_start = time.time()
            # IMUs: Trunk, left thigh, left shank, right thigh, right shank
            # Data: Periods, angles, angle velocities, quaternion
            # Range of measurement: x, y, z as follows; x+ facing left, y+ facing up, z+ facing forward
            # https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1100611599/LPMS+User+Manual
            
            IMU_data = child_conn_IMU.recv()
            # print(IMU_data)
            # print('IMU received')

            vicon_data_flat = []

            if vicon_enable:
                vicon_data = child_conn_vicon.recv()
                vicon_data_list = [None] * 117 # number of markers * 3
                # if vicon_data[0] is not None:
                    # print("VICON data:", vicon_data[0])
                for i, data in enumerate(vicon_data):
                    if data is not None:
                        data_list = data.tolist()
                    else:
                        data_list = [None] * 3
                # vicon_data_list = [x.tolist() for x in vicon_data]
                    vicon_data_list[i*3:i*3+3] = data_list
                assert len(vicon_data_list) == 117
                vicon_data_flat = vicon_data_list
                # vicon_data_flat = [item for x in vicon_data_list for item in x]
                    # print("VICON data:", vicon_data_flat[0:3])
            # print('Vicon received')

            current_freq = 1/(time.time() - time_start)
            # Run detection
            #Timers
            timeLastRun = timeCurrent
            timeCurrent = time.time()
            timeToRun = timeCurrent - timeLastRun

            #Right and Left Gait Detection
            gaitDetectRight.testVal(objRThigh.gyZ, objRShank.gyZ, objRHeel.gyZ)
            gaitDetectLeft.testVal(objLThigh.gyZ, objLShank.gyZ, objLHeel.gyZ)

            forwardFootAccRight = (objRHeel.acX * np.cos( (objRHeel.zAngle - objRHeel.zAngleZeroed) * .01745)) - (objRHeel.acY * np.sin( (objRHeel.zAngle - objRHeel.zAngleZeroed) * .01745))
            forwardFootAccLeft = (objLHeel.acX * np.cos( (objLHeel.zAngle - objLHeel.zAngleZeroed) * .01745)) - (objLHeel.acY * np.sin( (objLHeel.zAngle - objLHeel.zAngleZeroed) * .01745))
            #Slip Algorithm - Calculates Slip Indicator from Trkov IFAC 2017 paper
            slipRight = gaitDetectRight.slipTrkov(objLowBack.acX, forwardFootAccRight, hip_heel_length)
            slipLeft = gaitDetectLeft.slipTrkov(objLowBack.acX, forwardFootAccLeft , hip_heel_length)

            # Data format of output is plain txt
            data_record = [current_frame, current_freq]
            data_record.append(gaitDetectRight.gaitStage)
            data_record.append(gaitDetectLeft.gaitStage)
            data_record.append(slipRight)
            data_record.append(slipRight)

            if not IMU_data:
                IMU_data = [None] * 63
            if not vicon_data_flat:
                vicon_data_flat = [None] * 117 # change this number to number of markers * 3

            if vicon_enable:
                data_record.extend(vicon_data_flat)
                data_record.extend(IMU_data)
                if vicon_data_flat[0] and IMU_data[0]:
                    print('IMU and Vicon')
            else:
                data_record.extend(IMU_data)
                if IMU_data[0]:
                    print("IMU only" )
            data_record_list.append(data_record)
            # print(vicon_data_flat)
            file1.writelines(','.join(str(j) for j in data_record) + '\n')

            current_frame += 1

    except KeyboardInterrupt:
    
        serial_connection.close()
        file1.close()
        print('Program Interrupted! Closing...')








