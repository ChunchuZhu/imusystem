from matplotlib import pyplot as plt
from matplotlib import image as mpimg
import time
import numpy as np
import simpleaudio as sa
import random


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
while True:
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

    if 6 < current_time - previous_time < 9:
        if Figure_Played == 0:
            random_index = random.randrange(len(ID))
            current_ID = ID[random_index]
            print(current_ID)
            np.delete(ID, random_index)
            Figure_Played = 1
            if current_ID == 1:
                image = mpimg.imread("/Users/chunchu/GitHub/imusystem/Rutgers/images/left.jpg")
                plt.figure(figsize=(20, 12))
                plt.imshow(image, interpolation='nearest', aspect='auto')
                plt.show()

            if current_ID == 2:
                image = mpimg.imread("/Users/chunchu/GitHub/imusystem/Rutgers/images/right.jpg")
                plt.figure(figsize=(20, 12))
                plt.imshow(image, interpolation='nearest', aspect='auto')
                plt.show()

    if current_time - previous_time > 9 and current_time - previous_time > 11:
        if Relax_Played == 0:
            image = mpimg.imread("/Users/chunchu/GitHub/imusystem/Rutgers/images/Relax.jpg")
            plt.figure(figsize=(20, 12))
            plt.imshow(image, interpolation='nearest', aspect='auto')
            plt.show()
            Relax_Played = 1
            current_ID = 0

    if current_time - previous_time > 11:
        Sound_Played = 0
        Ready_Played = 0
        Relax_Played = 0
        Figure_Played = 0
        previous_time = current_time
        count = count + 1

    if count == 20:
        break
    print(current_ID)
