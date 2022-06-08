import serial
import time
import struct
import numpy as np

# serialPort = '/dev/cu.usbmodem102558701'
serialPort = '/dev/cu.usbmodem84041401'
serialBaud = 115200
# Connect to serial port
print('Trying to connect to ' + str(serialPort) +
        ' at ' + str(serialBaud) + ' BAUD.')
try:
    serial_conn = serial.Serial(serialPort, 115200, timeout=None)
    print('Teensy Connected!')
except:
    print("Failed to connect with " + str(serialPort) +
            ' at ' + str(serialBaud) + ' BAUD.')

timestr = time.strftime("%Y%m%d-%H%M%S")
file_name = "/Users/chunchu/My Drive/[3] Rutgers/[1] Research/[2] On-going Project/[3] Slip Recovery/TeensyControlCodes/ImpedanceTestNewerVersion/data-" + timestr + ".txt"
file1 = open(file_name, "w")

try:
    while True:
        data = []
        time_start = time.time()
        dataPacket=serial_conn.readline()
        dataPacket=str(dataPacket,'utf-8')
        splitPacket=dataPacket.split(',')
        freq = str(1/(time.time()-time_start))
        # splitPacket.append(str(freq))
        data.append(freq)
        data.append(splitPacket)
        print(data)
        file1.writelines(','.join(str(j) for j in data) + '\n')
        

except KeyboardInterrupt:
    serial_conn.close()
    file1.close()
    print('Program Interrupted! Closing...')
