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
# import numpy as np
import time
from EEGreceiver import *

#Importing Custom Functions
from packageHandlerFunc import * #package_handler_raw(tup)
# from sensorClass import * #class sensorObject, newValues(valueArray), angularAccCalc(), angleCalc(gaitDetectObject)
from serialSend import * #ardno(msg as string)
from userinput import *
from IMU_read import *
from IMU_initialization import *
from IMU_receiver import *

from vicon_initialization import *
from vicon_receiver import *

#Importing Custom Functions
from packageHandlerFunc import * #package_handler_raw(tup)
from variableDeclarations import *
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


    # IMU_stream
    serial_connection = IMU_init()
    (parent_conn_IMU, child_conn_IMU) = Pipe()
    sender_IMU = Process(target = async_IMU, args = (parent_conn_IMU, serial_connection))
    sender_IMU.start()
    #########################################################################################

    #########################################################################################
    # vicon_stream, subject_name, marker_count = vicon_init("192.168.10.203")
    IP_ADDRESS = "10.0.0.10"
    # IP_ADDRESS = "192.168.1.6"
    vicon_enable = True
    #########################################################################################


    if vicon_enable:
        
        (parent_conn_vicon, child_conn_vicon) = Pipe()
        sender_vicon = Process(target = async_vicon, args = (parent_conn_vicon, IP_ADDRESS))
        sender_vicon.start()

    timestr = time.strftime("%Y%m%d-%H%M%S")
    file_name = "data-" + timestr + ".txt"
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

    header = "frame\tfrequency\tVicon\tIMU"
    
    file1.write(header)

    IMU_data = child_conn_IMU.recv()
    print('IMU received')


    try:
        while True:
            time_start = time.time()
            # https://lp-research.atlassian.net/wiki/spaces/LKB/pages/1100611599/LPMS+User+Manual
	        # print(time_start)
            IMU_data = child_conn_IMU.recv()
            #print(IMU_data)
            # print('IMU received')
            
          # Data format of output is plain txt
            current_freq = 1/(time.time() - time_start)

            data_record = [current_frame, current_freq]

            vicon_data_flat = []
            if vicon_enable:
                #print("here")
                vicon_data = child_conn_vicon.recv()
                vicon_data_list = [None] * 39 # number of markers * 3
                # if vicon_data[0] is not None:
                    # print("VICON data:", vicon_data[0])
                for i, data in enumerate(vicon_data):
                    if data is not None:
                        data_list = data.tolist()
                    else:
                        data_list = [None] * 3
                # vicon_data_list = [x.tolist() for x in vicon_data]
                    vicon_data_list[i*3:i*3+3] = data_list
                assert len(vicon_data_list) == 39
                vicon_data_flat = vicon_data_list
                # vicon_data_flat = [item for x in vicon_data_list for item in x]
                    # print("VICON data:", vicon_data_flat[0:3])
            # print('Vicon received')
            
            if not IMU_data:
                IMU_data = [None] * 63
            if not vicon_data_flat:
                vicon_data_flat = [None] * 39 # change this number to number of markers * 3

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
            
            
            
            # print(EEG_data)
            # print(vicon_data_flat)
            file1.writelines(','.join(str(j) for j in data_record) + '\n')
            # print(data_record)
            current_frame += 1

    except KeyboardInterrupt:
    
        serial_connection.close()
        file1.close()
        print('Program Interrupted! Closing...')








