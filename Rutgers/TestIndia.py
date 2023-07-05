import numpy as np
import random
import time
# Following are the libraries for Qt application and multiprocessing
import sys
import os
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QLabel, QDesktopWidget
# from PyQt5.QtMultimedia import QSound
from multiprocessing import Process, Value, Array

# for producing beep
# import winsound

beep_frequency = 2500  # Set Frequency To 2500 Hertz
beep_duration = 500  # Set Duration in ms

sampRate = 250      # sampling rate of sensors (EEG or EMG or IMU)
imStatus = np.zeros((5,1), dtype=int)
beepStatus = 0
beepstarttiming = 2 # seconds
beepStartcounter = 0 # initializing a counter to start counting the samples once any particular trial starts
acquisitionDuration = 467   # acquisition duration for 40 trials with an initial interval of 30 sec and inter trial interval of 3sec

# randomly generating class labels considering only two classes
# Class1: Rest or Left Hand
# Class2: Extension or Right Hand

classLabels = np.zeros([40, 1], dtype = int)

def BCI_ExperimentalEventsTiming():
    # global label_signal,trig_signal,planning_signal,classLabels
    initDelay = 30  # delay in seconds before recording the data to get stable EEG recording
    nofTrials = 40

    # print("\nMatrix classLabels : \n", classLabels)
    Class1_Label_counter = 0
    Class2_Label_counter = 0
    for i in range(nofTrials):
        rand_class = random.choice([1, 2])
        if rand_class == 1 and Class1_Label_counter <= nofTrials / 2:
            classLabels[i, 0] = 1
            Class1_Label_counter = Class1_Label_counter + 1
        elif rand_class == 1 and Class2_Label_counter <= nofTrials / 2:
            classLabels[i, 0] = 2
            Class2_Label_counter = Class2_Label_counter + 1

        if rand_class == 2 and Class2_Label_counter <= nofTrials / 2:
            classLabels[i, 0] = 2
            Class2_Label_counter = Class2_Label_counter + 1
        elif rand_class == 2 and Class1_Label_counter <= nofTrials / 2:
            classLabels[i, 0] = 1
            Class1_Label_counter = Class1_Label_counter + 1
    # np.count_nonzero(classLabels == 1)
    # np.count_nonzero(classLabels == 2)

    # parameters of experimental paradigm
    global sampRate, beepstarttiming
    trialLength = 8  # trial length in seconds
    cuestarttiming = 3  # seconds
    cueduration = 2  # seconds
    beepduration = 0.5  # seconds
    feedbackstarttiming = 4.5  # seconds
    interTrialInterval = 3  # inter trial interval in seconds

    # generating timestamps
    total_runtime = initDelay + trialLength * nofTrials + interTrialInterval * (nofTrials - 1)
    no_of_samples = sampRate * total_runtime;  # plus one for the 0th second
    tmStamps = np.transpose(np.linspace(1 / sampRate, total_runtime, num=no_of_samples))

    # generating trigger signals and label signals(this signal goes to 1 when trial is running and 0 during inter-trial interval and initial delay)
    # Generating Planning signals
    # Planning=0 during first 3 seconds, Message shown: Get Ready
    # Planning=1 during next 5 seconds, Message shown: Left/Right Hand Cue
    planning_signal = np.zeros_like(tmStamps, dtype=int)
    # print(planning_signal.dtype)
    trig_signal = np.zeros_like(tmStamps, dtype=int)
    # print(trig_signal.dtype)
    label_signal = np.zeros_like(tmStamps, dtype=int)
    # print(label_signal.dtype)

    for currentTrial in range(1, nofTrials + 1):
        # print(currentTrial)
        startTime_currentTrial = initDelay + interTrialInterval * (currentTrial - 1) + (
                    currentTrial - 1) * trialLength + (1 / sampRate)
        # print(startTime_currentTrial)
        endTime_currentTrial = initDelay + interTrialInterval * (currentTrial - 1) + currentTrial * trialLength
        # print(endTime_currentTrial)
        start_timeIndex_currentTrial = (int)(startTime_currentTrial * sampRate - 1);  # python indexing starts from 0
        # print(start_timeIndex_currentTrial)
        end_timeIndex_currentTrial = (int)(endTime_currentTrial * sampRate - 1);  # python indexing starts from 0
        # print(end_timeIndex_currentTrial)
        print()
        for i in range(start_timeIndex_currentTrial,
                       end_timeIndex_currentTrial + 1):  # to include the last index also in for loop
            if tmStamps[i] <= initDelay:
                trig_signal[i] = 0
                label_signal[i] = 0
                planning_signal[i] = 0
            elif tmStamps[i] <= (initDelay + currentTrial * trialLength + interTrialInterval * (currentTrial - 1)):
                trig_signal[i] = 1
                # print("Trigger signal assigned value 1")
                label_signal[i] = classLabels[currentTrial - 1]
                if tmStamps[i] <= (initDelay + (currentTrial - 1) * trialLength + interTrialInterval * (
                        currentTrial - 1) + cuestarttiming):
                    planning_signal[i] = 0
                elif tmStamps[i] <= (initDelay + (currentTrial - 1) * trialLength + interTrialInterval * (
                        currentTrial - 1) + feedbackstarttiming):
                    planning_signal[i] = 1
                    # print("Planning signal assigned value 1")
                else:
                    planning_signal[i] = 2
                    # print("Planning signal assigned value 2")
            else:
                trig_signal[i] = 0
                label_signal[i] = 0
                planning_signal[i] = 0

    # clearing the temporary variables from workspace
    del i, Class1_Label_counter, Class2_Label_counter, currentTrial, end_timeIndex_currentTrial, endTime_currentTrial, rand_class, start_timeIndex_currentTrial, startTime_currentTrial

    return trig_signal, label_signal, planning_signal

if __name__ == '__main__':
    trig, label, planning = BCI_ExperimentalEventsTiming()
    print(trig)
    print(label)
    print(planning)
