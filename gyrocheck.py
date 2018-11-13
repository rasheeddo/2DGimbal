from mpu6050 import mpu6050
import math as m
import numpy as np
import time

sensor = mpu6050(0x68)

def getRawGyro():
	giroscope_data=sensor.get_gyro_data()
	rawXp=giroscope_data["x"]
	rawYp=giroscope_data["y"]
	rawZp=giroscope_data["z"]
	return rawXp, rawYp, rawZp

def getRawAccelero():
	accelerometer_data = sensor.get_accel_data()
	rawX=accelerometer_data["x"]
	rawY=accelerometer_data["y"]
	rawZ=accelerometer_data["z"]
	return rawX, rawY, rawZ

def RawGyroCalibration():
	print("Doing gyro calibration...(Keep stable!)")
	gyroX = []
	gyroY = []
	gyroZ = [] 
	for i in range(0,500):
		rawGyro = getRawGyro()
		gyroX.append(rawGyro[0])
		gyroY.append(rawGyro[1])
		gyroZ.append(rawGyro[2])
		time.sleep(0.01)

	ave_gyroX = sum(gyroX)/len(gyroX)
	ave_gyroY = sum(gyroY)/len(gyroY)
	ave_gyroZ = sum(gyroZ)/len(gyroZ)

	return ave_gyroX, ave_gyroY, ave_gyroZ


def getIMUAngle():
        global init,Kpitch,Kroll,Kyaw
        accelerometer_data = sensor.get_accel_data()
        giroscope_data=sensor.get_gyro_data()
        
        rawX=accelerometer_data["x"]
        rawY=accelerometer_data["y"]
        rawZ=accelerometer_data["z"]
        rawXp=giroscope_data["x"]
        rawYp=giroscope_data["y"]
        rawZp=giroscope_data["z"]
        
        #Angle calculation by axelerometer
        pitchA=m.degrees((m.atan2(rawX,m.sqrt(m.pow(rawY,2)+m.pow(rawZ,2)))))
        rollA=m.degrees((m.atan2(rawY,m.sqrt(m.pow(rawX,2)+m.pow(rawZ,2)))))
        yawA=m.degrees((m.atan2(m.sqrt(m.pow(rawX,2)+m.pow(rawY,2)),rawZ)))

        #Get angle variation from gyroscope:
        if(init):
            pitchG=pitchA
            rollG=rollA
            yawG=0
            init=0
            print("init")
        else:
            #Update the position
            pitchG=Dt*rawYp+Kpitch
            rollG=Dt*rawXp+Kroll
            yawG=Dt*rawZp+Kyaw

        #Fusion with Kalman filter (not tunned)
        Kpitch=k_p*pitchG+(1-k_p)*pitchA
        Kroll=k_r*rollG+(1-k_r)*rollA
        Kyaw=k*yawG+(1-k)*yawA

        return Kpitch,Kroll,Kyaw

def getIMU3():
	global init3, pitch_out, roll_out
	acc_data = sensor.get_accel_data()
	gyro_data=sensor.get_gyro_data()

	accX=acc_data["x"]
	accY=acc_data["y"]
	accZ=acc_data["z"]
	gyroX=gyro_data["x"] - GyroXcal
	gyroY=gyro_data["y"] - GyroYcal
	gyroZ=gyro_data["z"] - GyroZcal


	#Angle calculation by axelerometer
	roll_acc = m.degrees(m.atan(accY/m.sqrt(m.pow(accX,2)+m.pow(accZ,2))))
	pitch_acc = m.degrees(m.atan(-accX/m.sqrt(m.pow(accY,2)+m.pow(accZ,2))))
	#yawA=m.degrees((m.atan(m.sqrt(m.pow(accX,2)+m.pow(accY,2)),rawZ)))
	
	if (init3):
		roll_gyro = roll_acc
		pitch_gyro = pitch_acc
		init3 = 0
	else:
		roll_gyro = Dt*gyroX + roll_out
		pitch_gyro = Dt*gyroY + pitch_out
	

	roll_out = kr*roll_gyro + (1-kr)*roll_acc
	pitch_out = kp*pitch_gyro + (1-kp)*pitch_acc

	return roll_out, pitch_out


def getIMUAngleWithCalibration():
    global init,Kpitch,Kroll,Kyaw
    accelerometer_data = sensor.get_accel_data()
    giroscope_data=sensor.get_gyro_data()
    
    rawX=accelerometer_data["x"]
    rawY=accelerometer_data["y"]
    rawZ=accelerometer_data["z"]
    rawXp=giroscope_data["x"]
    rawYp=giroscope_data["y"]
    rawZp=giroscope_data["z"]
    
    #Angle calculation by axelerometer
    pitchA=m.degrees((m.atan2(rawX,m.sqrt(m.pow(rawY,2)+m.pow(rawZ,2)))))
    rollA=m.degrees((m.atan2(rawY,m.sqrt(m.pow(rawX,2)+m.pow(rawZ,2)))))
    yawA=m.degrees((m.atan2(m.sqrt(m.pow(rawX,2)+m.pow(rawY,2)),rawZ)))

    #Get angle variation from gyroscope:
    if(init):
        pitchG=pitchA
        rollG=rollA
        yawG=0
        init=0
        print("init")
    else:
        #Update the position
        pitchG=Dt*(rawYp-GyroYcal)+Kpitch
        rollG=Dt*(rawXp-GyroXcal)+Kroll
        yawG=Dt*(rawZp-GyroZcal)+Kyaw

    #Fusion with Kalman filter (not tunned)
    Kpitch=k_p*pitchG+(1-k_p)*pitchA
    Kroll=k_r*rollG+(1-k_r)*rollA
    Kyaw=k*yawG+(1-k)*yawA

    return Kpitch,Kroll,Kyaw

def calibrationOffset():
    print("Start calibration....")
    n=100
    X=0
    Y=0
    Z=0
    for a in range(0,n):
        V=getIMUAngle()
        X=V[0]+X
        Y=V[1]+Y
        Z=V[2]+Z
    X_offset=-X/n
    Y_offset=-Y/n
    Z_offset=-Z/n
    time.sleep(0.04)
    print("....Finished")
    print("offsets:",X_offset,Y_offset,Z_offset)
    return X_offset,Y_offset,Z_offset

def smoother(n,rawPitch,rawRoll,rawYaw):
        #Do the average on the 5 last measured value to smooth the sensor output
        global initAcq
        if(initAcq):
                initAcq=0
                for i in range(0,n):
                        avPitch.append(rawPitch)
                        avRoll.append(rawRoll)
                        avYaw.append(rawYaw)
                        
        else:
                avPitch.pop(0)
                avPitch.append(rawPitch)
                avRoll.pop(0)
                avRoll.append(rawRoll)
                avYaw.pop(0)
                avYaw.append(rawYaw)
                
        return int(sum(avPitch)/len(avPitch)),int(sum(avRoll)/len(avRoll)),int(sum(avYaw)/len(avYaw))


def FindOffsetAngle():
    print("Find off set angle (Keep it steady)...")
    P = []
    R = []
    for i in range(0,1000):
        PRY = getIMUAngle()
        P.append(PRY[0])
        R.append(PRY[1])

    ave_pitch = sum(P)/len(P)
    ave_roll = sum(R)/len(R)

    return ave_pitch, ave_roll


#X_offset,Y_offset,Z_offset=calibrationOffset()
#pitch_offset, roll_offset = FindOffsetAngle()
rawXoffset=0
rawYoffset=0
rawZoffset=0
rawXpoffset=0
rawYpoffset=0
rawZpoffset=0

### Ben's method
#Dt=0.02 #intervalle of time between update
init=1
k_r=0.980 #arbitrary value (see sources)
k_p=0.995
k = k_r
initAcq=1
avPitch=[]
avRoll=[]
avYaw=[]
Kpitch=0
Kroll=0
Kyaw=0

### Rasheed's method
init3 = 1
global Dt
Dt = 0.02
kr = 0.99
kp = 0.99
roll3= []
pitch3=[]




gyroCal = RawGyroCalibration()
global GyroXcal, GyroYcal, GyroZcal
GyroXcal = gyroCal[0]
GyroYcal = gyroCal[1]
GyroZcal = gyroCal[2]
'''
offset = FindOffsetAngle()
pitch_offset = offset[0]
roll_offset = offset[1]
''' 

pitch = []
roll = []
pitch2 = []
roll2 = []
yaw = []
smPitch = []
smRoll = []
smYaw = []

rawXp = []
rawYaw = []
rawYp = []
print("Get signal...")

gyro_x = []
gyro_y = []
gyro_z = []

acc_x = []
acc_y = []
acc_z = []
print("start shaking!")
acqui = 1000

for i in range(0,acqui):
	raw_gyro = getRawGyro()
	raw_acc = getRawAccelero()
	gyro_x.append(raw_gyro[0] - gyroCal[0])
	gyro_y.append(raw_gyro[1] - gyroCal[1])
	gyro_z.append(raw_gyro[2] - gyroCal[2])
	acc_x.append(raw_acc[0])
	acc_y.append(raw_acc[1])
	acc_z.append(raw_acc[2])

	#full=getIMUAngleWithCalibration()
	#pitch.append(full[0])
	#roll.append(full[1])

	angle = getIMU3()
	roll3.append(angle[0])
	pitch3.append(angle[1])

	#print("gyro_x: %f" %raw_gyro[0], "gyro_y: %f" %raw_gyro[1], "gyro_z: %f" %raw_gyro[2])
	#print("acc_x: %f" %raw_acc[0], "acc_y: %f" %raw_acc[1], "acc_z: %f" %raw_acc[2])
	time.sleep(0.02)
	
print("...Finished")

print("DATA visualisation:")
import matplotlib.pyplot as plt

t=np.arange(0,acqui,1)

plt.figure()

plt.subplot(2, 1, 1)
plt.plot(t,gyro_x,'r',t,gyro_y,'g',t,gyro_z,'b')
plt.grid()
plt.title("Raw Gyro")

plt.subplot(2, 1, 2)
plt.plot(t,acc_x,'r',t,acc_y,'g',t,acc_z,'b')
plt.grid()
plt.title("Raw Acc")

plt.figure(2)

plt.subplot(2,1,1)
plt.plot(t,roll3,'r')
plt.grid()
plt.title("Roll Angle")

plt.subplot(2,1,2)
plt.plot(t,pitch3,'g')
plt.grid()
plt.title("Pitch Angle")

plt.legend()
plt.show()
