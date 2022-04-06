import serial
import time
import struct
import numpy as np

g = 9.81
convL = 16.0 / 32768.0 * g  # m/s**2
convAv = 2000.0 / 32768.0  # deg/s
convA = 180.0 / 32768  # deg
convM = 1.0  #
rad2deg = 180 / np.pi
time_list = []
# num = 0


def receive_from_IMU(serial_conn):
    try:
        time_start = time.time()
        dataPacket=serial_conn.readline()
        while not '\\n'in str(dataPacket):         # check if full data is received. 
            # This loop is entered only if serial read value doesn't contain \n
            # which indicates end of a sentence. 
            # str(val) - val is byte where string operation to check `\\n` 
            # can't be performed
            time.sleep(.001)                # delay of 1ms 
            temp = serial_conn.readline()   # check for serial output.
            if len(dataPacket) < 18:
                dataPacket = [dataPacket, temp]
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(',')
        print(splitPacket)
        freq = 1/(time.time()-time_start)

    except KeyboardInterrupt:

        serial_conn.close()
        print('Program Interrupted! Closing...')

    return splitPacket
