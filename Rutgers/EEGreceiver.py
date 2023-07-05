# -*- coding: utf-8 -*-
"""
Created on Mon Feb 20 13:09:20 2023

@author: hesam
"""

from multiprocessing import Process, Pipe
import socket
import numpy as np
import time
import csv


def async_EEG(pipe):
    num = 0
    #####EEG#####
    print('Waiting for EEG Connection')
    s_EEG_bytes = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_EEG_bytes.bind(('192.168.2.2', 10346))
    s_EEG_bytes.listen()
        
    s_EEG = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s_EEG.bind(('192.168.2.2', 10345))
    s_EEG.listen()

    print('EEG Connected')

    #####EEG#####
    EEG_data, client_address = s_EEG.accept()
    EEG_data_byte, addrEEG = s_EEG_bytes.accept()

    try:

        while True:
            def EEG_ready(EEG_data_byte):
                siz_EEG = EEG_data_byte.recv(3).decode('utf-8')
                return int(siz_EEG);
            
            sizEEG = EEG_ready(EEG_data_byte)

            EEGdata = EEG_data.recv(sizEEG)
            EEGdata_record = EEGdata.decode('utf-8')
            a = ""
            if EEGdata_record.__contains__("end"):
                
                break

            else:

                resEEG = EEGdata_record[2:len(EEGdata_record)-2];

                nums_EEG = [float(a) for a in resEEG.split("', '")]

                pipe.send(nums_EEG)
                
            
    except:
        print('AN ERROR in SERVER EEG LOOP')
        pipe.close()
