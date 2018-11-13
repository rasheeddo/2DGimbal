#Script for testing filtering on the MPU data:

from mpu6050 import mpu6050
import math as m
import numpy as np
import time
from dronekit import connect, VehicleMode

sensor = mpu6050(0x68)

rawXoffset=0
rawYoffset=0
rawZoffset=0
rawXpoffset=0
rawYpoffset=0
rawZpoffset=0

Dt=0.02 #intervalle of time between update
init=1
k_r=0.984 #arbitrary value (see sources)
k_p=0.96
k = k_r
initAcq=1

avPitch=[]
avRoll=[]
avYaw=[]
Kpitch=0
Kroll=0
Kyaw=0

def getIMUAngle():
        global init,Kpitch,Kroll,Kyaw
        accelerometer_data = sensor.get_accel_data()
        giroscope_data=sensor.get_gyro_data()
        
        rawX=accelerometer_data["x"]
        rawY=accelerometer_data["y"]
        rawZ=accelerometer_data["z"]
        rawXp=giroscope_data["x"]
        rawYp=-giroscope_data["y"]
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
    print("Keep it steady...")
    P = []
    R = []
    for i in range(0,1000):
        PRY = getIMUAngle()
        P.append(PRY[0])
        R.append(PRY[1])

    ave_pitch = sum(P)/len(P)
    ave_roll = sum(R)/len(R)

    return ave_pitch, ave_roll


X_offset,Y_offset,Z_offset=calibrationOffset()
pitch_offset, roll_offset = FindOffsetAngle()
                   
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

#vehicle = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', wait_ready=True, baud=9600 ) 
print("Start shaking!")
acqui=1000
for n in range(0,acqui):

        full=getIMUAngle()
        pitch2.append(full[0]-pitch_offset)
        roll2.append(full[1]-roll_offset)
        #Roll = (vehicle.attitude.roll*180.0/m.pi-0.0)
        #Pitch = (vehicle.attitude.pitch*180.0/m.pi-0.0)
        #pitch.append(Pitch)
        #roll.append(Roll)
        yaw.append(full[2]+Z_offset)
        
        smfull=smoother(3,pitch2[n],roll2[n],yaw[n])
        
        smPitch.append(smfull[0])
        smRoll.append(smfull[1])
        #smYaw.append(smfull[2])


        giroscope_data=sensor.get_gyro_data()
        rawXp.append(giroscope_data["x"])
        rawYp.append(giroscope_data["y"])

        
        time.sleep(Dt)
                   
print("...Finished")

print("DATA visualisation:")
import matplotlib.pyplot as plt

t=np.arange(0,acqui,1)

plt.figure(1)

plt.subplot(3, 1, 1)
#plt.plot(t,pitch,'r',t,smPitch,'b')
plt.plot(t,pitch2,'b',t,smPitch)
plt.grid()
plt.title("Pitch")

plt.subplot(3, 1, 2)
#plt.plot(t,roll,'r',t,smRoll,'b')
plt.plot(t,roll2,'b',t,smRoll)
plt.grid()
plt.title("Roll")

plt.subplot(3,1,3)
plt.plot(t,rawYp,'r',t,rawXp,'b')
plt.grid()


plt.legend()
plt.show()





