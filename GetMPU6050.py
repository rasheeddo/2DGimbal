import time
from mpu6050 import mpu6050                      #new library for IMU
import math
from simple_pid import PID
from dynamixel_sdk import *                     # Uses Dynamixel SDK library

def RunServo(DEG1,DEG2):
    servo_ang1 = map(DEG1, 0.0, 360.0, 0, 4095)
    servo_ang2 = map(DEG2, 0.0, 360.0, 0, 4095)

    dxl1_goal_position = int(servo_ang1)
    dxl2_goal_position = int(servo_ang2)
    

    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)

def RunServo1(inputDeg1):
    pos1 = inputDeg1
    servo_com1 = map(pos1,0.0,360.0,0.0,4095.0)
    dxl1_goal_position = int(servo_com1)
    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, dxl1_goal_position)
    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error1))

def RunServo2(inputDeg2):
    pos2 = inputDeg2
    servo_com2 = map(pos2,0.0,360.0,0.0,4095.0)
    dxl2_goal_position = int(servo_com2)
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, dxl2_goal_position)
    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
    elif dxl_error2 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error2))

def ReadAngle12():
    dxl_present_position1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
    dxl_present_position2, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PRESENT_POSITION)   
    pre_pos1 = map(dxl_present_position1, 0.0, 4095.0, 0.0, 360.0)
    pre_pos2 = map(dxl_present_position2, 0.0, 4095.0, 0.0, 360.0)

    return pre_pos1,pre_pos2

def GoHome12():
    Home = 135
    servo_ang1 = map(Home, 0.0, 360.0, 0, 4095)
    servo_ang2 = map(Home, 0.0, 360.0, 0, 4095)
    servo_ang3 = map(Home, 0.0, 360.0, 0, 4095)

    dxl1_goal_position = servo_ang1  
    dxl2_goal_position = servo_ang2

    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))

def SetOperationMode(MODE):
    TorqueOff()
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, MODE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, MODE)
    
def CheckOperationMode():
    present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE)
    present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE)
    if present_mode == 0:
        # Current (Torque) Control Mode
        print("Now Operating Mode is Torque Control")
    elif present_mode == 3:
        # Position Control Mode
        print("Now Operating Mode is Position Control")
    elif present_mode == 5:
        # Current-based Position Control Mode
        print("Now Operating Mode is Current-based Position Control")
    else:
        print("In other Mode that didn't set!")

def TorqueOn():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    print("Enable Toruqe...")

def TorqueOff():
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    print("Disable Toruqe...")
    

def SetProfile1(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 1023       # 350 Default                  [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 1: %d" %set_V_PRFL)
    print("A PRFL 1: %d" %set_A_PRFL)
    print("--------------------------------")

def SetProfile2(set_V_PRFL,set_A_PRFL):
    ######################### Set Velocity / Acceleration Profile  ##############################
    set_A_Limit = 32767     # 32767 default                [214.577 rev/min^2]
    set_V_Limit = 1023      # 350 Default                  [0.229RPM]

    #set_A_PRFL = 30      # between 0 ~ set_A_limit      [214.577 rev/min^2]
    #set_V_PRFL = 200      # between 0 ~ set_V_Limit      [0.229RPM]

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT, set_A_Limit)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

    acceleration_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT)
    velocity_limit, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT)

    print("V PRFL 2: %d" %set_V_PRFL)
    print("A PRFL 2: %d" %set_A_PRFL)
    print("--------------------------------") 

def SetPID1(set_P_Gain,set_I_Gain,set_D_Gain):
    
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)
    
    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 1: %d" %position_P_gain)
    print("Position I Gain 1: %d" %position_I_gain)
    print("Position D Gain 1: %d" %position_D_gain)
    print("------------------------------")

def SetPID2(set_P_Gain,set_I_Gain,set_D_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

    position_D_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_D_GAIN)
    position_I_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_I_GAIN)
    position_P_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_POSITION_P_GAIN)

    print("Position P Gain 2: %d" %position_P_gain)
    print("Position I Gain 2: %d" %position_I_gain)
    print("Position D Gain 2: %d" %position_D_gain)
    print("------------------------------")


def SetFFGain1(set_FF1_Gain,set_FF2_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

    FF1_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN)
    FF2_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN)

    print("Feedforward 1st Gain 1: %d" %FF1_gain)
    print("Feedforward 2nd Gain 1: %d" %FF2_gain)
    print("------------------------------") 

def SetFFGain2(set_FF1_Gain,set_FF2_Gain):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN, set_FF1_Gain)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN, set_FF2_Gain)

    FF1_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_1st_GAIN)
    FF2_gain, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_FEEDFORWARD_2nd_GAIN)

    print("Feedforward 1st Gain 2: %d" %FF1_gain)
    print("Feedforward 2nd Gain 2: %d" %FF2_gain)
    print("------------------------------")

def AverageData(scale,pitch,roll):

    Buff_P = list()
    Buff_R = list()

    for i in range(0,scale):
        Buff_P.append(pitch)
        Buff_R.append(roll)

        if i == (scale-1):
            Ave_Pitch = sum(Buff_P)/len(Buff_P)
            Ave_Roll = sum(Buff_R)/len(Buff_R)
            Buff_P = list()
            Buff_R = list()

    return Ave_Pitch, Ave_Roll

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def getIMUAngle(period):
        global init,Kpitch,Kroll,Kyaw
        Dt=period
        accelerometer_data = sensor.get_accel_data()
        giroscope_data=sensor.get_gyro_data()
        
        rawX=accelerometer_data["x"]+rawXoffset
        rawY=accelerometer_data["y"]+rawYoffset
        rawZ=accelerometer_data["z"]+rawZoffset
        rawXp=giroscope_data["x"]+rawXpoffset
        rawYp=giroscope_data["y"]+rawYpoffset
        rawZp=giroscope_data["z"]+rawZpoffset
        
        #Angle calculation by axelerometer
        pitchA=math.degrees((math.atan2(rawX,math.sqrt(math.pow(rawY,2)+math.pow(rawZ,2)))))
        rollA=math.degrees((math.atan2(rawY,math.sqrt(math.pow(rawX,2)+math.pow(rawZ,2)))))
        yawA=math.degrees((math.atan2(math.sqrt(math.pow(rawX,2)+math.pow(rawY,2)),rawZ)))

        #Get angle variation from gyroscope:
        if(init):
            pitchG=pitchA
            rollG=rollA
            yawG=0
            init=0
        else:
            #Update the position
            pitchG=Dt*rawYp+Kpitch
            rollG=Dt*rawXp+Kroll
            yawG=Dt*rawZp+Kyaw

        #Fusion with Kalman filter (not tunned)
        Kpitch=k_p*pitchG+(1-k_p)*pitchA
        Kroll=k_r*rollG+(1-k_r)*rollA
        Kyaw=k*yawG+(1-k)*yawA
        prev_rawYp=rawYp
        return Kpitch,Kroll,Kyaw


def getIMUAngleWithCalibration(period):
    global init,Kpitch,Kroll,Kyaw,prev_rawYp
    Dt=period
    accelerometer_data = sensor.get_accel_data()
    giroscope_data=sensor.get_gyro_data()
    
    rawX=accelerometer_data["x"]
    rawY=accelerometer_data["y"]
    rawZ=accelerometer_data["z"]
    rawXp=giroscope_data["x"]
    rawYp=giroscope_data["y"]
    rawZp=giroscope_data["z"]
    
    #Angle calculation by axelerometer
    pitchA=math.degrees((math.atan2(rawX,math.sqrt(math.pow(rawY,2)+math.pow(rawZ,2)))))
    rollA=math.degrees((math.atan2(rawY,math.sqrt(math.pow(rawX,2)+math.pow(rawZ,2)))))
    yawA=math.degrees((math.atan2(math.sqrt(math.pow(rawX,2)+math.pow(rawY,2)),rawZ)))

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
    T=0.05
    X=0
    Y=0
    Z=0
    for a in range(0,n):
        V=getIMUAngle(T)
        X=V[0]+X
        Y=V[1]+Y
        Z=V[2]+Z
    X_offset=-X/n
    Y_offset=-Y/n
    Z_offset=-Z/n
    time.sleep(T)
    print("....Finished")
    print("offsets:",X_offset,Y_offset,Z_offset)
    return X_offset,Y_offset,Z_offset

def smoother(n,rawPitch,rawRoll,rawYaw):
        #Do the average on the N last measured value to smooth the sensor output
        global initSmooth
        if(initSmooth):
                initSmooth=0
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
                
        return sum(avPitch)/len(avPitch),sum(avRoll)/len(avRoll),sum(avYaw)/len(avYaw)

def FindOffsetAngle():
    print("Keep it steady...")
    P = []
    R = []
    for i in range(0,20):
        PRY = getIMUAngle(0.02)
        P.append(PRY[0])
        R.append(PRY[1])

    ave_pitch = sum(P)/len(P)
    ave_roll = sum(R)/len(R)

    return ave_pitch, ave_roll

def getRawGyro():
    giroscope_data=sensor.get_gyro_data()
    rawXp=giroscope_data["x"]
    rawYp=giroscope_data["y"]
    rawZp=giroscope_data["z"]
    return rawXp, rawYp, rawZp

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

####################################################### Set Servo Configuration #############################################################

                        ############################## Control table address ##############################

ADDR_PRO_BAUDRATE           = 8

ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

ADDR_PRO_CURRENT_LIMIT      = 38
ADDR_PRO_GOAL_CURRENT       = 102
ADDR_PRO_PRESENT_CURRENT    = 126 

ADDR_PRO_OPERATING_MODE     = 11

ADDR_PRO_GOAL_VELOCITY      = 104

ADDR_PRO_ACCELERATION_LIMIT = 40
ADDR_PRO_VELOCITY_LIMIT     = 44
ADDR_PRO_PROFILE_ACCELERATION  = 108
ADDR_PRO_PROFILE_VELOCITY   = 112

ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84
ADDR_PRO_FEEDFORWARD_2nd_GAIN = 88
ADDR_PRO_FEEDFORWARD_1st_GAIN = 90

ADDR_PRO_MOVING             = 122
ADDR_PRO_MOVING_STATUS       = 123

CURRENT_CONTROL                     = 0
POSITION_CONTROL                    = 3 # Default
CURRENT_BASED_POSITION_CONTROL      = 5
# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                      = 1                             # Dynamixel ID: 1
DXL2_ID                      = 2                             # Dynamixel ID: 2
DXL3_ID                      = 3                             # Dynamixel ID: 3
DXL4_ID                      = 4
DXL5_ID                      = 5
DXL6_ID                      = 6



BAUDRATE                    = 57600            # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()


TorqueOn()

######################### Set Velocity / Acceleration Profile  ##############################
SetProfile1(0,0)
SetProfile2(0,0)

######################### Set PID Gain Position Loop  ##############################
#SetPID1(750,9000,6000)
#SetPID2(750,9000,6000)
#SetFFGain1(0,0)
#SetFFGain2(0,0)

GoHome12()
time.sleep(2)


period = 0 # initial value of period

sensor = mpu6050(0x68)

rawXoffset=0
rawYoffset=0
rawZoffset=0
rawXpoffset=0
rawYpoffset=0
rawZpoffset=0
#Variable for Kalman filter:
init=1
k_r=0.980 
k_p=0.995
k = k_r
prev_rawYp=0
modeK=1 #trigger for mode

Kpitch=0
Kroll=0
Kyaw=0
#Variable for the smoother function:
initSmooth=1
avPitch=[]
avRoll=[]
avYaw=[]

#Quick Calibration
#X_offset,Y_offset,Z_offset=calibrationOffset()
gyroCal = RawGyroCalibration()
global GyroXcal, GyroYcal, GyroZcal
GyroXcal = gyroCal[0]
GyroYcal = gyroCal[1]
GyroZcal = gyroCal[2]


#P_offset,R_offset = FindOffsetAngle()

while True:

    startTime = time.time()

    #Read Accelerometer raw value
    pitchDegree,rollDegree,yawDegree=getIMUAngleWithCalibration(period)
    #Offset correction:
    rollDegree=rollDegree #+Y_offset
    pitchDegree=pitchDegree #+X_offset
    rollDegree,pitchDegree,yawDegree=smoother(3,rollDegree,pitchDegree,yawDegree)

    ServoAng1 = 135 - rollDegree
    ServoAng2 = 135 - pitchDegree 

    RunServo(ServoAng1,ServoAng2)


    print("Pitch: %f" %pitchDegree)
    print("Roll: %f" %rollDegree)
    endTime = time.time() 
    period = endTime - startTime
    print("Period: %f" %period)
    print("----------------------------")