import time
from mpu6050 import mpu6050                      #new library for IMU
import math
from simple_pid import PID
from dynamixel_sdk import *                     # Uses Dynamixel SDK library

def map(val, in_min, in_max, out_min, out_max):

	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def SetOperatingMode(MODE):

    TorqueOff()

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, MODE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, MODE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, MODE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_OPERATING_MODE, MODE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_OPERATING_MODE, MODE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_OPERATING_MODE, MODE)


    present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE)
    if present_mode == 0:
        # Current (Torque) Control Mode
        print("Now Operating Mode is Torque Control")
    elif present_mode == 1:
        # Position Control Mode
        print("Now Operating Mode is Velocity Control")
    elif present_mode == 3:
        # Position Control Mode
        print("Now Operating Mode is Position Control")
    elif present_mode == 5:
        # Current-based Position Control Mode
        print("Now Operating Mode is Current-based Position Control")
    else:
        print("In other Mode that didn't set!")

def RunServo(DEG1,DEG2):
	servo_ang1 = map(DEG1, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(DEG2, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1
	dxl2_goal_position = servo_ang2
	

	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))

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
	Home = 135.0
	servo_ang1 = map(Home, 0.0, 360.0, 0, 4095)
	servo_ang2 = map(Home, 0.0, 360.0, 0, 4095)

	dxl1_goal_position = servo_ang1  
	dxl2_goal_position = servo_ang2

	dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_POSITION, int(dxl1_goal_position))
	dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_POSITION, int(dxl2_goal_position))

def TorqueOn():
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	print("Enable Toruqe...")

def TorqueOff():
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
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

def SetVelocityLimit1(set_V_Limit):
    # Min 0 ~ Max 1023
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
    print("Velocity Limit 1 is set: %f" %set_V_Limit)

def SetVelocityLimit2(set_V_Limit):
    # Min 0 ~ Max 1023
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, set_V_Limit)
    print("Velocity Limit 2 is set: %f" %set_V_Limit)

def DriveServoSpeed1(input1):
    dxl1_goal_velocity = int(input1)
    dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_GOAL_VELOCITY, dxl1_goal_velocity)
    if dxl_comm_result1 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result1))
    elif dxl_error1 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error1))

def DriveServoSpeed2(input2):
    dxl2_goal_velocity = int(input2)
    dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_GOAL_VELOCITY, dxl2_goal_velocity)
    if dxl_comm_result2 != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result2))
    elif dxl_error2 != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error2))

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

####################################################### Set Servo Configuration #############################################################

						############################## Control table address ##############################
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
VELOCITY_CONTROL                    = 1
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

BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
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

######################### Set Velocity / Acceleration Profile  ##############################
#SetProfile1(1000,500)
#SetProfile2(1000,500)

######################### Set PID Gain Position Loop  ##############################
#SetPID1(1000,10,1000)
#SetPID2(1000,10,1000)
SetOperatingMode(POSITION_CONTROL)

TorqueOn()

GoHome12()
time.sleep(2)

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

### PID
P = 5.5         #5.8
I = 2.0         #2.5
D = 0.01       #0.015

pid_pitch = PID(P, I, D, setpoint=0.0)
pid_roll = PID(P, I, D, setpoint=0.0)

pid_pitch.sample_time = 0.001
pid_roll.sample_time = pid_pitch.sample_time

pid_pitch.auto_mode = True
pid_roll.auto_mode = True

pid_pitch.output_limits = (-50.0,50.0)
pid_roll.output_limits = (-50.0,50.0)

#v = controlled_system.update(0)

K = 0.7

while True:

    startTime = time.time()


    rollDegree1, pitchDegree1 = getIMU1(period)
    rollDegree2, pitchDegree2 = getIMU2(period)

    outputPIDroll = pid_roll(rollDegree2)
    outputPIDpitch = pid_pitch(pitchDegree2)

    CompensateRoll = outputPIDroll*K + rollDegree1*(1-K)
    CompensatePitch = outputPIDpitch*K + pitchDegree1*(1-K)

    ServoAng1 = 140 + CompensateRoll
    ServoAng2 = 135 + CompensatePitch

    RunServo(ServoAng1,ServoAng2) 

    print("Roll1: %f" %rollDegree1)
    print("Pitch1: %f" %pitchDegree1)
    print("Roll2: %f" %rollDegree2)
    print("Pitch2: %f" %pitchDegree2)
    print("OutputPIDroll: %f" %outputPIDroll)
    print("OutputPIDpitch: %f" %outputPIDpitch)
    print("CompensateRoll: %f" %CompensateRoll)
    print("CompensatePitch: %f" %CompensatePitch)
    print("ServoAng1: %f" %ServoAng1)
    print("ServoAng2: %f" %ServoAng2)
    endTime = time.time()
    period = endTime - startTime


    print("Period: %f" %period)
    print("----------------------------")