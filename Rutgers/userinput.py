#!/usr/bin/env python3	

#Frequency that the system outputs at (Hz)(recommended max 100)
processing_frequency = 50

#Activate/deactivate individual sensor recording
#Note that disabling some (e.g. shank) will cause some detectors to throw errors
toggle_rThigh  = True
toggle_rShank  = True
toggle_rHeel   = True
toggle_lThigh  = True
toggle_lShank  = True
toggle_lHeel   = True
toggle_lowBack = True
toggle_topBack = False


#Hip to heel length for Trkov slip detection (m)
hip_heel_length = 1


#Torque Controller Options: 
# "pid" - DS developed, uses knee angle and thigh ang. vel.
# "yusu" - Based on paper by Yu&Su. Uses thigh kinematics plus lower back.
# "ramp" - Constant ramping torque.
# "trkov" - Developed by MT, adapted by DS, uses full lower body kinematics plus lower back.
controller_type = "trkov"	

#General Torque Controller Parameters	
mass = 80 #Subject mass (kg)
height = 1.78 #Subject height (m)
torqueCutoff = 30 #Maximum allowable torque (Nm	) - pass to kneeling object
NMKG = 0.25 #Approximate torque per subject unit weight (Nm/kg)

#YU&SU Controller Proportionality Constants	
alpha = .1	

#Ramping Specific Constants	
ramping_delay_time = 5 #seconds	
ramping_hold_time = 5 #seconds	
ramping_slope = 20 #Nm/s	
