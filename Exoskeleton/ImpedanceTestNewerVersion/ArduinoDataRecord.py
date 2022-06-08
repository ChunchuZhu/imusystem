import time
import serial
ser = serial.Serial(
    port='/dev/cu.usbmodem84041401',                   
    baudrate=115200,
    timeout=None
)

# timestr = time.strftime("%Y%m%d-%H%M%S")
# file_name = "test.txt"
# file1 = open(file_name, "a")

try:
    while True:
        data_write = [ ]
        file1 = open('Stiffness.txt', 'a')
        t = time.time()
        data_read = ser.readline()
        # print(data_read)
        print('DataReceived: ')
        data_write.append(data_read.decode('utf-8'))
        file1.writelines(data_write)
        file1.close()
except KeyboardInterrupt:

    ser.close()
    file1.close()
    print('Program Interrupted! Closing...')