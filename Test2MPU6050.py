import time
from mpu6050 import mpu6050                      #new library for IMU
import math


def getIMU1(period):
    global init1, pitch_out1, roll_out1
    
    Dt=period
    acc_data1 = sensor1.get_accel_data()
    gyro_data1=sensor1.get_gyro_data()

    accX1=acc_data1["x"]
    accY1=acc_data1["y"]
    accZ1=acc_data1["z"]
    gyroX1=gyro_data1["x"] - GyroXcal1
    gyroY1=gyro_data1["y"] - GyroYcal1
    gyroZ1=gyro_data1["z"] - GyroZcal1


    #Angle calculation by axelerometer
    roll_acc1 = math.degrees(math.atan(accY1/math.sqrt(math.pow(accX1,2)+math.pow(accZ1,2))))
    pitch_acc1 = math.degrees(math.atan(-accX1/math.sqrt(math.pow(accY1,2)+math.pow(accZ1,2))))
    #yawA=m.degrees((m.atan(m.sqrt(m.pow(accX,2)+m.pow(accY,2)),rawZ)))
    
    if (init1):
        roll_gyro1 = roll_acc1
        pitch_gyro1 = pitch_acc1
        init1 = 0
    else:
        roll_gyro1 = Dt*gyroX1 + roll_out1
        pitch_gyro1 = Dt*gyroY1 + pitch_out1
    

    roll_out1 = kr*roll_gyro1 + (1-kr)*roll_acc1
    pitch_out1 = kp*pitch_gyro1 + (1-kp)*pitch_acc1

    return roll_out1, pitch_out1

def getIMU2(period):
    global init2, pitch_out2, roll_out2

    Dt=period
    acc_data2 = sensor2.get_accel_data()
    gyro_data2=sensor2.get_gyro_data()

    accX2=acc_data2["x"]
    accY2=acc_data2["y"]
    accZ2=acc_data2["z"]
    gyroX2=gyro_data2["x"] - GyroXcal2
    gyroY2=gyro_data2["y"] - GyroYcal2
    gyroZ2=gyro_data2["z"] - GyroZcal2


    #Angle calculation by axelerometer
    roll_acc2 = math.degrees(math.atan(accY2/math.sqrt(math.pow(accX2,2)+math.pow(accZ2,2))))
    pitch_acc2 = math.degrees(math.atan(-accX2/math.sqrt(math.pow(accY2,2)+math.pow(accZ2,2))))
    #yawA=m.degrees((m.atan(m.sqrt(m.pow(accX,2)+m.pow(accY,2)),rawZ)))
    
    if (init2):
        roll_gyro2 = roll_acc2
        pitch_gyro2 = pitch_acc2
        init2 = 0
    else:
        roll_gyro2 = Dt*gyroX2 + roll_out2
        pitch_gyro2 = Dt*gyroY2 + pitch_out2
    

    roll_out2 = kr*roll_gyro2 + (1-kr)*roll_acc2
    pitch_out2 = kp*pitch_gyro2 + (1-kp)*pitch_acc2

    return roll_out2, pitch_out2

def getRawGyro1():
    giroscope_data1=sensor1.get_gyro_data()
    rawXp1=giroscope_data1["x"]
    rawYp1=giroscope_data1["y"]
    rawZp1=giroscope_data1["z"]
    return rawXp1, rawYp1, rawZp1

def getRawGyro2():
    giroscope_data2=sensor2.get_gyro_data()
    rawXp2=giroscope_data2["x"]
    rawYp2=giroscope_data2["y"]
    rawZp2=giroscope_data2["z"]
    return rawXp2, rawYp2, rawZp2

def RawGyroCalibration():
    print("Doing gyro calibration...(Keep stable!)")
    gyroX1 = []
    gyroY1 = []
    gyroZ1 = []
    gyroX2 = []
    gyroY2 = []
    gyroZ2 = []  
    for i in range(0,500):
        rawGyro1 = getRawGyro1()
        rawGyro2 = getRawGyro2()
        gyroX1.append(rawGyro1[0])
        gyroY1.append(rawGyro1[1])
        gyroZ1.append(rawGyro1[2])
        gyroX2.append(rawGyro2[0])
        gyroY2.append(rawGyro2[1])
        gyroZ2.append(rawGyro2[2])
        time.sleep(0.01)

    ave_gyroX1 = sum(gyroX1)/len(gyroX1)
    ave_gyroY1 = sum(gyroY1)/len(gyroY1)
    ave_gyroZ1 = sum(gyroZ1)/len(gyroZ1)
    ave_gyroX2 = sum(gyroX2)/len(gyroX2)
    ave_gyroY2 = sum(gyroY2)/len(gyroY2)
    ave_gyroZ2 = sum(gyroZ2)/len(gyroZ2)

    return ave_gyroX1, ave_gyroY1, ave_gyroZ1, ave_gyroX2, ave_gyroY2, ave_gyroZ2

global GyroXcal1, GyroYcal1, GyroZcal1, GyroXcal2, GyroYcal2, GyroZcal2
global kr, kp

sensor1 = mpu6050(0x68)
sensor2 = mpu6050(0x69)

gyroCal = RawGyroCalibration()

GyroXcal1 = gyroCal[0]
GyroYcal1 = gyroCal[1]
GyroZcal1 = gyroCal[2]
GyroXcal2 = gyroCal[3]
GyroYcal2 = gyroCal[4]
GyroZcal2 = gyroCal[5]


### Rasheed's method


init1 = 1
init2 = 1
kr = 0.99
kp = 0.99
roll3= []
pitch3=[]
period = 0.02

while True:
	startTime = time.time()

	rollDegree1, pitchDegree1 = getIMU1(period)
	rollDegree2, pitchDegree2 = getIMU2(period)


	print("Pitch1: %f" %pitchDegree1)
	print("Roll1: %f" %rollDegree1)
	print("----------------------------")
	print("Pitch2: %f" %pitchDegree2)
	print("Roll2: %f" %rollDegree2)
	period = time.time() - startTime
	print period
	print("============================")






	'''
	acc_data1 = sensor1.get_accel_data()
	gyro_data1=sensor1.get_gyro_data()

	ax1=acc_data1["x"]
	ay1=acc_data1["y"]
	az1=acc_data1["z"]
	gx1=gyro_data1["x"]
	gy1=gyro_data1["y"]
	gz1=gyro_data1["z"]

	acc_data2 = sensor2.get_accel_data()
	gyro_data2 = sensor2.get_gyro_data()

	ax2=acc_data2["x"]
	ay2=acc_data2["y"]
	az2=acc_data2["z"]
	gx2=gyro_data2["x"]
	gy2=gyro_data2["y"]
	gz2=gyro_data2["z"]

	print ax1
	print ay1
	print az1
	print gx1
	print gy1
	print gz1
	print "-----------------"
	print ax2
	print ay2
	print az2
	print gx2
	print gy2
	print gz2
	print "================="
	'''