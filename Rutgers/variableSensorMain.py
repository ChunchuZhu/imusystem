#!/usr/bin/env python3

#Importing python libraries
from multiprocessing import Process,Queue,Pipe
import serial
import time
from math import sin, cos, sqrt, atan2
import numpy as np
import sys
import time

#Importing Custom Functions
from gaitDetectClass import * #class gaitDetect, func testVal(shank gyZ, heel gyZ)
from packageHandlerFunc import * #package_handler_raw(tup)
from sensorClass import * #class sensorObject, newValues(valueArray), angularAccCalc(), angleCalc(gaitDetectObject)
from serialSend import * #ardno(msg as string)
from slipAlgorithmFunc import * #slipAlgorithm(pelvis_forward_acc, heel_forward_acc, L_hh)
from CUNYreceiver import *
from NUCreceiver import *
from ARDUINOreceiver import *
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
    
    serial_conn = serial.Serial('/dev/cu.usbmodem102558701', 115200, timeout=None)
    # serial_conn = serial.Serial('/dev/ttyACM0', 115200, timeout=None)
    timestr = time.strftime("%Y%m%d-%H%M%S")
    file_name = "Data/" + "data-" + timestr + ".txt"
    file1 = open(file_name, "w")

    header = "time\ttimeToRun\tgaitStageR\tgaitStageL\tslipR\tslipL\t\t"
    for x in stringObjects:
        for y in stringSensors:
            for z in stringAxes:
                header += f"{x}/{y}/{z}\t"
        header += f"zAngleZeroed/{x}\t"
        header += f"\t"
    header += f"\n"
    file1.write(header)


    # Calibrate Z zero angles 
    dataPacket=serial_conn.readline()
    while not '\\n'in str(dataPacket):         # check if full data is received. 
        # This loop is entered only if serial read value doesn't contain \n
        # which indicates end of a sentence. 
        # str(val) - val is byte where string operation to check `\\n` 
        # can't be performed
        time.sleep(.001)                # delay of 1ms 
        temp = serial_conn.readline()           # check for serial output.
        if len(dataPacket) < 18:
            dataPacket = [dataPacket, temp]
    # Define z-zeroed angles:
    dataPacket=str(dataPacket,'utf-8')
    splitPacket=dataPacket.split(',')
    
    i = 0
    for x in stringObjects:
            x.zAngleZeroed = (float(splitPacket[i+8]))
            i+=9

    t_calibrate_count =0
    while t_calibrate_count < 100:
        i = 0
        #  read data
        dataPacket=serial_conn.readline()
        while not '\\n'in str(dataPacket):         # check if full data is received. 
            # This loop is entered only if serial read value doesn't contain \n
            # which indicates end of a sentence. 
            # str(val) - val is byte where string operation to check `\\n` 
            # can't be performed
            time.sleep(.001)                # delay of 1ms 
            temp = serial_conn.readline()           # check for serial output.
            if len(dataPacket) < 18:
                dataPacket = [dataPacket, temp]
        # Define z-zeroed angles:
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(',')
        
        for x in stringObjects:
            x.zAngleZeroed = (x.zAngleZeroed + float(splitPacket[i+8]))/2
            i+=9
    t_calibrate_count +=1

             
    try:
        #  Read Data From Teensy
        time_start = time.time()

        dataPacket=serial_conn.readline()
        while not '\\n'in str(dataPacket):         # check if full data is received. 
            # This loop is entered only if serial read value doesn't contain \n
            # which indicates end of a sentence. 
            # str(val) - val is byte where string operation to check `\\n` 
            # can't be performed
            time.sleep(.001)                # delay of 1ms 
            temp = serial_conn.readline()           # check for serial output.
            if len(dataPacket) < 18:
                dataPacket = [dataPacket, temp]

        freq = 1/(time.time()-time_start)
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(',')
        print(freq)

        i = 0
        for x in stringObjects:
            x.AssignIMUData(splitPacket[i:i+6])
            i+=9
        
        # file1.writelines(','.join(str(j) for j in one_reading) + '\n')

        # Runs calculations every [processing_frequency] with whatever values are present -----------------------------------------------------------------
        if (time.time() - timeCurrent) >= (1/processing_frequency):
            
            #DETECTION ALGORITHMS AND OTHER SECONDARY CALCS -------------------------------------------------------------------------------------------------
            #Code is broken into reader above and algorithms below for increased customization and ease of changing algorithm. Everything below this line is almost entirely customizable.

            #Timers
            timeLastRun = timeCurrent
            timeCurrent = time.time()
            timeToRun = timeCurrent - timeLastRun

            #Right and Left Gait Detection
            gaitDetectRight.testVal(objRThigh.gyZ, objRShank.gyZ, objRHeel.gyZ)
            gaitDetectLeft.testVal(objLThigh.gyZ, objLShank.gyZ, objLHeel.gyZ)

            #Slip Algorithm - Calculates Slip Indicator from Trkov IFAC 2017 paper
            slipRight = gaitDetectRight.slipTrkov(objLowBack.acX, ((objRHeel.acX * np.cos( (objRHeel.zAngle - objRHeel.zAngleZeroed) * .01745)) - (objRHeel.acY * np.sin( (objRHeel.zAngle - objRHeel.zAngleZeroed) * .01745))), hip_heel_length)
            slipLeft = gaitDetectLeft.slipTrkov(objLowBack.acX, ((objLHeel.acX * np.cos( (objLHeel.zAngle - objLHeel.zAngleZeroed) * .01745)) - (objLHeel.acY * np.sin( (objLHeel.zAngle - objLHeel.zAngleZeroed) * .01745))), hip_heel_length)


        ###########################################################################################
            #DATA OUTPUT (FILE) -------------------------------------------------------------------------------------------------------------

            #Create beginning of output string - time, time between measurements, right gait stage, left gait stage, left slip detector, right slip detector
            outputString = f"{time.time() - timeStart}\t{timeToRun}\t{gaitDetectRight.gaitStage}\t{gaitDetectLeft.gaitStage}\t{slipRight}\t{slipLeft}\t\t"

            #Cycle through all sensor objects to append formatted version of every sensor's raw data to output string
            for x in objects:
                outputString += f"{x.gyX}\t"
                outputString += f"{x.gyY}\t"
                outputString += f"{x.gyZ}\t"

                outputString += f"{x.acX}\t"
                outputString += f"{x.acY}\t"
                outputString += f"{x.acZ}\t"

                outputString += f"{x.zAngleZeroed}\t"
                
                outputString += f"\t"

            # Add output string to file
            file1.write(f"{outputString}")


    except KeyboardInterrupt:
        serial_conn.close()
            # 5: Encode and send torques to teensy
        file1.close()
        print('Program Interrupted! Closing...')
