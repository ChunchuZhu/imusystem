#!/usr/bin/env python3
import numpy as np
class gaitDetect:
    def __init__(self):
        self.firstVar = 0
        self.movingArrShank = [0]
        self.movingArrThigh = [0]
        self.movingArrHeel = [0]
        self.movingAvgShank = 0
        self.movingAvgThigh = 0
        self.movingAvgHeel = 0
        self.lastAvgShank = 0
        self.lastAvgThigh = 0
        self.movingAvgAccuracy = 2
        
        self.significance = 0
        self.gaitStage = 0 #0-Stance / 1-Heel Off / 2-Toe Off
        self.eventTimer = .1
        
        self.timeLastHeelStrike = 0
        self.timeLastHeelOff = 0
        self.timeLastToeOff = 0
        self.timeLastStanding = 0
        
        self.slipToeOffWaitThreshold = .2
        self.slipHeelStrikeWaitThreshold = .1
        self.standing = False
        self.standingLimit = 14 #deg/s
        self.concurrentZeroes = 0
        self.concurrentZeroesLimit = 5
        
        self.lastDiffHeel = 0
        self.gamma = 40 #-562 or -377 #deg/s	
        self.indicatorThreshold = 10 ** 30
        self.isSlipping = False
        self.timeSlipStart = 0
        self.ankH = 100
        self.g    = 9.81
        self.Lt = 0.4467133
        self.Ls = 0.4821936
        self.L1   = 0.4821936 
        self.L1   = 0.4821936+100/1000
        self.L2   = 0.4467133
        self.Mb  = 30*2
        self.Mt    = 7.3
        self.Ms    = 3.3
        self.Mf    = 1.0
        self.M1   = 3.3+1;   
        self.M2   = 60+7.3*2+3.3+1
        self.ct = 0.19186
        self.cs = 0.19367
        self.a1   = 0.4821936-0.19367+100/1000
        self.a2   = 0.4467133

                
    def testVal(self, thigh, shank, heel):
        import time
        self.movingArrThigh.append(thigh)
        self.movingArrShank.append(shank)
        self.movingArrHeel.append(heel)
        
#Limits number of previous values in array
        if len(self.movingArrShank) > self.movingAvgAccuracy:
            self.movingArrShank.pop(0)
        self.movingAvgShank = np.mean(self.movingArrShank)
        
        if len(self.movingArrHeel) > self.movingAvgAccuracy:
            self.movingArrHeel.pop(0)
        self.movingAvgHeel = np.mean(self.movingArrHeel)
        
        if len(self.movingArrThigh) > self.movingAvgAccuracy:
            self.movingArrThigh.pop(0)
        self.movingAvgThigh = np.mean(self.movingArrThigh)
        

#Significance changes for timing reasons. When not zero, it waits for the given timeframe before resetting to zero so that gait change can be detected again
        if self.significance == 0:

#detects negative to positive, aka toe off or start of swing phase
            if self.movingAvgThigh > 0 and self.lastAvgThigh < 0 and self.gaitStage == 1:
                self.significance = 1
                self.timeLastToeOff = time.time()
                self.gaitStage = 2
                
#detects positive to negative, aka heel strike or start of stance phase
            elif self.movingAvgShank < 0 and self.lastAvgShank > 0 and self.gaitStage == 2: 
                self.significance = -1
                self.timeLastHeelStrike = time.time()
                self.gaitStage = 0

#detects heel off occurrence
            elif np.mean(np.diff(self.movingArrHeel)) < -20 and self.gaitStage == 0:
                if self.lastDiffHeel <= 25 and self.lastDiffHeel >= -25:
                    self.timeLastHeelOff = time.time()
                    self.significance = 2
                    self.gaitStage = 1
            #Keep records of previous values for checking at the beginning of function
            self.lastDiffHeel = np.mean(np.diff(self.movingArrHeel))
         
#When significance has been changed, wait the appropriate amount of time before resetting to allow for next gait update
        elif self.significance != 0:
            if time.time() - self.timeLastHeelStrike > self.eventTimer and time.time() - self.timeLastToeOff > self.eventTimer:
                self.significance = 0
            if self.significance == 2:
                self.significance = 0

        self.lastAvgShank = self.movingAvgShank
        self.lastAvgThigh = self.movingAvgThigh
        if (self.gaitStage == 2):
            self.gaitOutput = 1
        else:
            self.gaitOutput = 0
        
#Trkov IFAC 2017 slip detection algorithm
    def slipTrkov(self, pelvisAcc, forwardFootAcc, L_hh):
        import time
        
#Limits the window of slip detection to [slipToeOffWaitThreshold seconds after toe off -- heel strike]. Note that gait does not reset until after slip is done.
        if (self.gaitStage == 0 and time.time() - self.timeLastHeelStrike < self.slipHeelStrikeWaitThreshold) or (self.gaitStage == 2 and time.time() - self.timeLastToeOff > self.slipToeOffWaitThreshold):
            dd_q_hh = (pelvisAcc - forwardFootAcc) / L_hh
            slip_indicator = forwardFootAcc / (2.718 ** (dd_q_hh - self.gamma))
            return slip_indicator
        else:
            return 0

    def ekf(self, obs_vector_z_k ,state_estimate_k_minus_1, P_k_minus_1, q1,dq1,ddq1,q2,dq2,ddq2):

        ddxs = (np.sin(q1)*(self.M1*self.a1 + self.L1*self.M2)*dq1^2 + self.M2*self.a2*np.sin(q2)*dq2^2 - ddq1*np.cos(q1)*(self.M1*self.a1 + self.L1*self.M2) - self.M2*self.a2*ddq2*np.cos(q2))/(self.M1 + self.M2);

        F_theta = np.array([ [0,1,0,0],
                    [ 0 ,0, ( (self.M1*self.a1 + self.M2*self.L2)*(np.cos(q1)*dq1*dq1 + ddq1*np.sin(q1))/(self.M1+self.M2)), ((self.M2*self.L2)(np.cos(q2)*dq2*dq2 + ddq2*np.sin(q2))/(self.M1+self.M2))],
                    [0,0,1,0],
                    [0,0,0,1]])

        F = np.eye(4) + 0.01 * F_theta

        state_estimate_k = state_estimate_k_minus_1+0.01* np.array([state_estimate_k_minus_1[1],ddxs, q1,q2])

        Q_k = np.array([[0.0001,   0,   0,    0],
                        [  0,     10,   0,   0],
                        [  0,      0, 0.1, 0],
                        [  0,   0,      0, 0.01]])
                         
# Sensor measurement noise covariance matrix R_k
# Has the same number of rows and columns as sensor measurements.
# If we are sure about the measurements, R will be near zero.
        R_k = np.array([[250000,   0],
                        [  0,     50]])  
                                        
                          
            # Predict the state covariance estimate based on the previous
            # covariance and some noise
        P_k = F @ P_k_minus_1 @ F.T + (
                Q_k)

        H_k =   np.array( [[0,0,(self.M1*self.a1+self.M2*self.L2)/(self.M1+self.M2)*(np.cos(q1* .01745)*dq1^2+ddq1*np.sin(q1* .01745)),(self.M2*self.L2)/(self.M1+self.M2)*(np.cos(q2* .01745)*dq2^2+ddq2*np.sin(q2* .01745))],[ 0 ,-1, self.L1*dq1*np.sin(q1* .01745),  self.L2*dq2*np.sin(q2* .01745)] ])
        ################### Update (Correct) ##########################
        # Calculate the difference between the actual sensor measurements
        # at time k minus what the measurement model predicted 
        # the sensor measurements would be for the current timestep k.
        
        ddxS_IMUHeel = (np.sin(q1* .01745)*(self.M1*self.a1 + self.L1*self.M2)*dq1^2 + self.M2*self.a2*np.sin(q2* .01745)*dq2^2 - ddq1*np.cos(q1* .01745)*(self.M1*self.a1 + self.L1*self.M2) - self.M2*self.a2*ddq2*np.cos(q2* .01745))/(self.M1 + self.M2)

        dxS_Kin = state_estimate_k[1] - self.L1*dq1*np.cos(q1* .01745) - self.L2*dq2*np.cos(q2* .01745) - state_estimate_k_minus_1[1]


        measurement_residual_y_k = obs_vector_z_k - np.array([ddxS_IMUHeel], [dxS_Kin])


        S=H_k @ P_k @ H_k.T + R_k
        K=P_k @ H_k.T /S
        
        state_estimate_k=state_estimate_k+K @ measurement_residual_y_k
        P_k = (np.eye(4)-K@H_k)*P_k
        # Calculate the measurement residual covariance
        S_k = H_k @ P_k @ H_k.T + R_k
                
        # # Calculate the near-optimal Kalman gain
        # # We use pseudoinverse since some of the matrices might be
        # # non-square or singular.
        # K_k = P_k @ H_k.T @ np.linalg.pinv(S_k)
            
        # # Calculate an updated state estimate for time k
        # state_estimate_k = state_estimate_k + (K_k @ measurement_residual_y_k)
        
        # # Update the state covariance estimate for time k
        # P_k = P_k - (K_k @ H_k @ P_k)
        
        # Print the best (near-optimal) estimate of the current state of the robot
    
        # Return the updated state and covariance estimates
        return state_estimate_k, P_k
            
        