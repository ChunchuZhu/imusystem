#  read csv file from folder
import time
import pandas as pd
import numpy as np

starting_time = time.time()
file=r"DataForLearning.csv"
names = ['loax','loay','loaz','lavx','lavy','lacz','laccx','laccy','laccz','roax','roay','roaz','ravx','ravy','racz','raccx','raccy','raccz','ID']
# 2-4 orientation angles; 5-7: raw angular velocities;8-10 linear accelerations; 11-14 quaternions
# names = ['gx','gy','gz','acx','acy','acz','q1','q2','q3','q4','ang', 'v1', 'v2', 'av', 'vel', 'ID', 's_ang']
DATA_LEN = 30
df = pd.read_csv(file, names=names)
x = df.iloc[:,0:18]
y = df.iloc[:,18]