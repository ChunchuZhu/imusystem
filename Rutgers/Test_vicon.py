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

if __name__ == "__main__":

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


    try:
        while True:
            time_start = time.time()
            # IMUs: Trunk, left thigh, left shank, right thigh, right shank
            # Data: Periods, angles, angle velocities, quaternion
            # Range of measurement: x, y, z as follows; x+ facing left, y+ facing up, z+ facing forward
            # https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1100611599/LPMS+User+Manual
            
            

            vicon_data_flat = []

            if vicon_enable:
                vicon_data = child_conn_vicon.recv()
                vicon_data_list = [None] * 30 # number of markers * 3
                # if vicon_data[0] is not None:
                    # print("VICON data:", vicon_data[0])
                for i, data in enumerate(vicon_data):
                    if data is not None:
                        data_list = data.tolist()
                    else:
                        data_list = [None] * 3
                # vicon_data_list = [x.tolist() for x in vicon_data]
                    vicon_data_list[i*3:i*3+3] = data_list
                assert len(vicon_data_list) == 30
                vicon_data_flat = vicon_data_list
                # vicon_data_flat = [item for x in vicon_data_list for item in x]
                    # print("VICON data:", vicon_data_flat[0:3])
            # print('Vicon received')

            current_freq = 1/(time.time() - time_start)
            # Run detection
     


            # Data format of output is plain txt
            data_record = [current_frame, current_freq]

            if not vicon_data_flat:
                vicon_data_flat = [None] * 30 # change this number to number of markers * 3

            if vicon_enable:
                data_record.extend(vicon_data_flat)
            data_record_list.append(data_record)
            # print(vicon_data_flat)
            file1.writelines(','.join(str(j) for j in data_record) + '\n')

            current_frame += 1

    except KeyboardInterrupt:
    
        file1.close()
        print('Program Interrupted! Closing...')








