#!/usr/bin/env python3

class gaitDetect:
    import numpy as np
    import time
    
    def __init__(self, firstVar):
        self.firstVar = firstVar
        self.movingArr = [firstVar]
        self.significance = 0
        self.movingAvgAccuracy = 2
        self.movingAvg = 0
        self.lastAvg = 0
        self.timeLastHeelStrike = 0
        self.timeLastToeOff = 0
        self.gaitStage = 0 #1 for swing, 0 for stance
        self.eventTimer = .1
        self.standing = False
        self.standingLimit = 200
        self.concurrentZeroes = 0
        self.concurrentZeroesLimit = 5
        
    def testVal(self, nextVal):
        self.movingArr.append(nextVal)
        if len(self.movingArr) > self.movingAvgAccuracy:
            self.movingArr.pop(0)
        self.movingAvg = np.mean(self.movingArr)
        
        if self.standing == True:
            self.gaitStage = 0
            if self.movingAvg < - self.standingLimit or self.movingAvg > self.standingLimit:
                self.standing = False
                self.lastAvg = - self.movingAvg
                
        if self.standing == False:
            if self.movingAvg < self.standingLimit and self.movingAvg > - self.standingLimit:
                self.concurrentZeroes += 1
            else:
                self.concurrentZeroes = 0
                
            if self.concurrentZeroes > self.concurrentZeroesLimit:
                self.standing = True
        
        if self.significance == 0 and not self.standing:
            if self.movingAvg > 0 and self.lastAvg < 0: #detects negative to positive, aka heel strike or start of stance phase
                self.significance = 1
                self.timeLastHeelStrike = time.time()
                self.gaitStage = 0
            elif self.movingAvg < 0 and self.lastAvg > 0: #detects positive to negative, aka toe off or start of swing phase
                self.significance = -1
                self.timeLastToeOff = time.time()
                self.gaitStage = 1
                
        elif self.significance != 0:
            if time.time() - self.timeLastHeelStrike > self.eventTimer and time.time() - self.timeLastToeOff > self.eventTimer:
                self.significance = 0
        
        #Implement other leg IMU - other leg heel strike must occur before measured leg toe off. (and vice versa)
        
        print(f"{self.gaitStage}")

        self.lastAvg = self.movingAvg
        

if __name__ == "__main__":
    inputShank = [1,-1,-1,1,0,-2,1,-1,1,-2,1,1,2,2,1,7,5,4,1,-1,-3,1,1,3,1,-1,2,3,2,1,2,3,3,3,5,1,3,-2,2,-1,1,3,1,1,0,1,-1,-2,-4,-4,-6,-2,-10,-19,-20,-12,6,2,18,-13,-34,-67,-174,-318,-723,-1091,-1331,-1228,-650,836,1540,1890,2030,2039,1909,1675,1054,-196,-965,-1407,-695,-1220,-960,-402,-666,-493,-416,-426,-337,-391,-484,-636,-867,-1068,-1252,-1320,-1377,-1619,-1945,-2307,-2065,-1295,803,1689,2851,3414,4076,4481,4221,3097,1249,-1106,-1964,-1261,-1440,-1051,-525,-735,-608,-505,-589,-573,-766,-910,-1060,-1284,-1498,-1756,-1998,-2232,-2133,-1499,382,1556,2402,3340,3874,4373,4523,3847,1565,-695,-1904,-2012,-2072,-1041,-573,-707,-428,-485,-525,-691,-931,-1067,-1178,-1402,-1503,-1889,-2035,-2018,-2025,-1292,1114,1941,2898,3656,4156,4408,4157,3428,432,-1269,-1557,-2384,-1739,-914,-828,-643,-526,-589,-532,-578,-660,-852,-1132,-1475,-1601,-1941,-1816,-1792,-1973,-926,691,1549,2302,3420,4138,4341,4243,3750,1026,-1467,-2369,-1237,-1523,-1329,-540,-736,-504,-481,-537,-634,-777,-990,-1077,-1314,-1442,-1656,-2017,-1989,-2155,-1132,-440,1797,2558,3773,4410,4685,4760,4155,491,-1396,-2092,-2102,-1543,-901,-767,-806,-428,-540,-468,-490,-590,-767,-909,-1173,-1374,-1517,-1793,-1981,-2300,-2012,-1247,555,1632,2807,3317,3922,4444,4519,3591,2049,-703,-2021,-558,-1890,-1156,-711,-763,-544,-466,-417,-343,-362,-391,-606,-766,-855,-851,-885,-1029,-1190,-1431,-1473,-1508,-785,849,1889,2759,2523,1795,1729,1087,133,616,249,71,16,-102,-125,-84,-22,-27,-55,-56,-54,-43,-15,18,19,10,5,6,5,14,17,20,19,16,6,5,13,11,8,-3,-7,3,8,15,11,6,-2,-8,3,8,13,15,18,17,19,17,23,23,20,13,9,6,7,10,12,14,8,5,-4,-3,3,4,0,-5,-9,-8,-5,2,1,-2,0,1,4,2,7,3,4,4,2,0,1,0,-1,2,2,7,1,3,1,2,2,-1,-8]
    inputHeel = [-46,-45,-47,-45,-45,-45,-45,-44,-45,-45,-43,-43,-45,-44,-45,-45,-43,-44,-45,-44,-44,-44,-44,-45,-43,-45,-44,-44,-42,-44,-44,-44,-45,-45,-45,-45,-44,-43,-45,-45,-45,-42,-44,-43,-44,-43,-44,-41,-41,-40,-38,-33,-33,-31,-38,-37,-37,-35,-33,-38,-39,-48,-71,-169,-431,-971,-738,-520,473,931,1186,1020,1380,1962,1419,940,263,-912,-1760,-2171,-1544,-487,-203,-130,-120,-109,-88,-81,-119,-155,-199,-272,-524,-860,-1214,-2374,-3305,-4568,-5116,-3930,-1431,1380,3017,4132,4202,4212,3882,3762,3678,2357,-633,-2034,-2918,-1642,-571,-187,-120,-168,-138,-143,-196,-254,-405,-615,-984,-1607,-2389,-3497,-6595,-5868,-3724,1629,2652,4193,4497,4306,3790,3650,3905,1754,-211,-1682,-2717,-1256,-306,-85,-103,-122,-84,-120,-201,-336,-496,-753,-1433,-1849,-2507,-5243,-6503,-5549,772,2486,3166,4512,4433,4366,3687,3656,3743,1068,-619,-4510,-2095,-464,-110,-114,-140,-104,-112,-153,-190,-281,-423,-703,-1346,-1818,-2576,-4704,-5842,-5232,-1671,2736,3009,4913,4452,4337,3814,3098,3970,1950,-187,-1489,-3120,-2642,-254,-111,-92,-114,-103,-180,-268,-360,-444,-773,-1198,-1702,-2701,-4214,-6089,-5521,-3403,2326,3511,5071,5434,4719,4232,3674,3429,1555,-750,-3977,-1998,-1352,-229,-154,-165,-155,-142,-178,-189,-218,-283,-494,-710,-1055,-1898,-2722,-4255,-6414,-5203,-1911,2116,3011,4524,4321,4174,3733,2781,3852,1972,-230,-1481,-2387,-1810,-679,-311,-206,-196,-90,-66,-65,-134,-190,-263,-377,-418,-444,-751,-1310,-1885,-2821,-3425,-3042,-416,1219,2088,2985,3280,2789,1473,-77,-1313,-320,-114,-41,-133,-120,-125,-61,-38,-46,-37,-41,-48,-46,-44,-40,-36,-43,-44,-47,-49,-49,-53,-50,-49,-49,-47,-49,-46,-40,-39,-39,-44,-42,-45,-47,-43,-48,-48,-50,-50,-51,-52,-52,-56,-51,-52,-54,-47,-48,-48,-49,-49,-51,-48,-48,-49,-47,-48,-47,-50,-50,-49,-46,-47,-47,-48,-50,-51,-50,-48,-50,-50,-51,-50,-49,-47,-48,-48,-44,-44,-47,-46,-49,-49,-47,-49,-47,-48,-48,-50,-45,-47,-49,-51,-49]

    gaitDetectRight = gaitDetect(0)
    outputArr = []
    for enum, x in enumerate(inputShank):
        avgValRight = (inputHeel[enum] + x) / 2
            
        gaitDetectRight.testVal(avgValRight)

        outputArr.append(gaitDetectRight.gaitStage)
        
        time.sleep(.04)
    
    print(outputArr)