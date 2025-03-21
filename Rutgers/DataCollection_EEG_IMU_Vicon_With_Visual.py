# !/usr/bin/env python

# !/usr/bin/env python3

# Data Collection for EEG, IMU, and Vicon

from matplotlib import pyplot as plt
from matplotlib import image as mpimg
import time
import simpleaudio as sa
import random

# Importing python libraries
from EEGreceiver import *

# Importing Custom Functions
from userinput import *
from IMU_receiver import *
from vicon_receiver import *

# Importing Custom Functions
from IMU_initialization import *

# Turns data collection for particular sensors on/off if necessary.
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

# Variable initializations that can't be offloaded to another file (time)
timeCurrent = time.time()
timeStart = timeCurrent
timeLastRun = timeCurrent
    
###########################################################################################
# Setup
if __name__ == "__main__":

    EEG_enable = True
        
    if EEG_enable is True:
        
        (parent_conn_EEG, child_conn_EEG) = Pipe()
        sender_EEG = Process(target = async_EEG, args = (parent_conn_EEG,))
        sender_EEG.start()
    #########################################################################################
    # IMU_stream
    IMU_enable = False
    if IMU_enable:
        serial_connection = IMU_init()
        (parent_conn_IMU, child_conn_IMU) = Pipe()
        sender_IMU = Process(target = async_IMU, args = (parent_conn_IMU, serial_connection))
        sender_IMU.start()
    #########################################################################################

    #########################################################################################
    # vicon_stream, subject_name, marker_count = vicon_init("192.168.10.203")
    IP_ADDRESS = "10.0.0.10"
    # IP_ADDRESS = "192.168.1.6"
    vicon_enable = False
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

    header = "frame\tfrequency\tEEG\tVicon\tIMU"
    
    file1.write(header)

    if IMU_enable:#  Zero y_angles
        IMU_data = child_conn_IMU.recv()
        print('IMU received')
    #print('total Length',len(IMU_data))

    ID = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2])
    current_ID = 0
    Sound_Played = 0
    Ready_Played = 0
    Relax_Played = 0
    Figure_Played = 0
    count = 0

    image = mpimg.imread("/Users/chunchu/GitHub/imusystem/Rutgers/images/Start.jpg")
    plt.figure(figsize=(20, 12))
    plt.imshow(image, interpolation='nearest', aspect='auto')
    plt.show()
    previous_time = time.time()
    print(previous_time)

    try:
        while True:
            time_start = time.time()
            current_time = time.time()
            # print(current_time - previous_time)
            if current_time - previous_time > 3:
                if Ready_Played == 0:
                    image = mpimg.imread("/Users/chunchu/GitHub/imusystem/Rutgers/images/BeReady.jpg")
                    plt.figure(figsize=(20, 12))
                    plt.imshow(image, interpolation='nearest', aspect='auto')
                    plt.show()
                    Ready_Played = 1

                if Sound_Played == 0:
                    wave_obj = sa.WaveObject.from_wave_file("/Users/chunchu/GitHub/imusystem/Rutgers/images/beep.wav")
                    wave_obj.play()
                    Sound_Played = 1

            if 6 < current_time - previous_time < 16:
                if Figure_Played == 0:
                    random_index = random.randrange(len(ID))
                    current_ID = ID[random_index]
                    print(current_ID)
                    np.delete(ID, random_index)
                    Figure_Played = 1
                    if current_ID == 1:
                        image = mpimg.imread("/Users/chunchu/GitHub/imusystem/Rutgers/images/LeftItem.jpg")
                        plt.figure(figsize=(20, 12))
                        plt.imshow(image, interpolation='nearest', aspect='auto')
                        plt.show()

                    if current_ID == 2:
                        image = mpimg.imread("/Users/chunchu/GitHub/imusystem/Rutgers/images/RightItem.jpg")
                        plt.figure(figsize=(20, 12))
                        plt.imshow(image, interpolation='nearest', aspect='auto')
                        plt.show()

            if current_time - previous_time > 16 and current_time - previous_time < 21:
                if Relax_Played == 0:
                    image = mpimg.imread("/Users/chunchu/GitHub/imusystem/Rutgers/images/Relax.jpg")
                    plt.figure(figsize=(20, 12))
                    plt.imshow(image, interpolation='nearest', aspect='auto')
                    plt.show()
                    Relax_Played = 1
                    current_ID = 0

            if current_time - previous_time > 21:
                Sound_Played = 0
                Ready_Played = 0
                Relax_Played = 0
                Figure_Played = 0
                previous_time = current_time
                count = count + 1

            if count == 20:
                break

            # IMU_data = child_conn_IMU.recv()

            current_freq = 1/(time.time() - time_start)

            data_record = [current_frame, current_freq]
            
            ###EEG###   
            if EEG_enable is True:
                EEG_data = child_conn_EEG.recv()
            
                if not EEG_data:
                    EEG_data = [0] * 38
            
                data_record.extend(EEG_data)
            else:
                EEG_data = [0] * 38
                data_record.extend(EEG_data)

            vicon_data_flat = []
            if vicon_enable:
                #print("here")
                vicon_data = child_conn_vicon.recv()
                vicon_data_list = [None] * 48 # number of markers * 3
                # if vicon_data[0] is not None:
                    # print("VICON data:", vicon_data[0])
                for i, data in enumerate(vicon_data):
                    if data is not None:
                        data_list = data.tolist()
                    else:
                        data_list = [None] * 3
                # vicon_data_list = [x.tolist() for x in vicon_data]
                    vicon_data_list[i*3:i*3+3] = data_list
                assert len(vicon_data_list) == 48
                vicon_data_flat = vicon_data_list
                # vicon_data_flat = [item for x in vicon_data_list for item in x]
                    # print("VICON data:", vicon_data_flat[0:3])
            # print('Vicon received')

            if not IMU_data:
                IMU_data = [None] * 18
            if not vicon_data_flat:
                vicon_data_flat = [None] * 48 # change this number to number of markers * 3

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








