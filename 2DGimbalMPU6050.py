import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from dronekit import connect, VehicleMode
import pygame
import math
from simple_pid import PID
from mpu6050 import mpu6050                      #new library for IMU

pygame.init()
j = pygame.joystick.Joystick(0)
j.init()

def rc_map(x,in_min,in_max,out_min,out_max):
    result = (x - in_min)*(out_max - out_min) / (in_max - in_min) + out_min
    return result

def getButton():
    #Read input from the two joysticks
    pygame.event.pump()
    #unknown1 = j.get_axis(0)
    #unknown2 = j.get_axis(1)
    #throttle = j.get_axis(2)
    #roll = j.get_axis(4)
    #pitch = j.get_axis(3)
    #yaw = j.get_axis(5)
    button0 = j.get_button(0)
    button1 = j.get_button(1)
    button2 = j.get_button(2)
    button3 = j.get_button(3)
    button4 = j.get_button(4)
    button5 = j.get_button(5)
    button6 = j.get_button(6)
    button7 = j.get_button(7)
    button8 = j.get_button(8)
    button9 = j.get_button(9)
    button10 = j.get_button(10)
    joy_button = [button0, button1, button2, button3, button4, button5, button6, button7,button8, button9, button10]
    
    return joy_button

def getAxis():
    #Read input from the two joysticks
    pygame.event.pump()
    axis0 = j.get_axis(0)
    axis1 = j.get_axis(1)
    axis2 = j.get_axis(2)
    axis3 = j.get_axis(4)
    axis4 = j.get_axis(3)
    axis5 = j.get_axis(5)
    joy_axis = [axis0, axis1, axis2, axis3, axis4, axis5]
    return joy_axis

def getHat():
    pygame.event.pump()
    hat0 = j.get_hat(0)
    
    joy_hat = hat0
    return joy_hat

def map(val, in_min, in_max, out_min, out_max):

	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def ChangeBaudRate(BaudRateNo):
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_BAUDRATE, BaudRateNo)
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_BAUDRATE, BaudRateNo)

	present_baud, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_BAUDRATE)
	present_baud, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_BAUDRATE)

	if present_baud == 0:
		print("Now Baud Rate is 9600")
	elif present_baud == 1:
		print("Now Baud Rate is 57,600")
	elif present_baud == 2:
		print("Now Baud Rate is 115,200")
	elif present_baud == 3:
		print("Now Baud Rate is 1,000,000")
	elif present_baud == 4:
		print("Now Baud Rate is 2,000,000")
	elif present_baud == 5:
		print("Now Baud Rate is 3,000,000")
	elif present_baud == 6:
		print("Now Baud Rate is 4,000,000")
	elif present_baud == 7:
		print("Now Baud Rate is 4,500,000")


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
            pitchG=Dt*rawXp+Kpitch
            rollG=Dt*rawYp+Kroll
            yawG=Dt*rawZp+Kyaw

        #Fusion with Kalman filter (not tunned)
        Kpitch=k*pitchG+(1-k)*pitchA
        Kroll=k*rollG+(1-k)*rollA
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

####################################################### Set Servo Configuration #############################################################

						############################## Control table address ##############################

ADDR_PRO_BAUDRATE			= 8

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



BAUDRATE                    = 3000000            # Dynamixel default baudrate : 57600
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

######################### Setting ##############################

#ChangeBaudRate(2)
#time.sleep(2)

SetOperationMode(POSITION_CONTROL)

#vehicle = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', wait_ready=True, baud=9600 ) 

TorqueOn()

######################### Set Velocity / Acceleration Profile  ##############################
SetProfile1(1000,400)
SetProfile2(1000,400)

######################### Set PID Gain Position Loop  ##############################
SetPID1(600,0,0)
SetPID2(600,0,0)
SetFFGain1(0,0)
SetFFGain2(0,0)


##############################################################################################

GoHome12()
time.sleep(2)

ServoAng1 = 140
ServoAng2 = 135

LastRoll = 0
LastPitch = 0

GainRoll = 3.5
GainPitch = 3.5

#a(i+1) = tiny*data(i+1) + (1.0-tiny)*a(i)
tiny = 0.3
#################################################################
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
k=0.9 #arbitrary value (see sources)
Kpitch=0
Kroll=0
Kyaw=0

#Quick Calibration
X_offset,Y_offset,Z_offset=calibrationOffset()

############################# PID #######################################

P = 3.0
I = 1.0
D = 0.0

pid_pitch = PID(1.0, 0.0, 0.0, setpoint=0.0)
pid_roll = PID(P, I, D, setpoint=0.0)

pid_pitch.sample_time = 0.001
pid_roll.sample_time = pid_pitch.sample_time

pid_pitch.auto_mode = True
pid_roll.auto_mode = True

pid_pitch.output_limits = (-60.0,60.0)
pid_roll.output_limits = (-60.0,60.0)

LastServoAng1 = 140 

while True:

    startTime = time.time()

    #Read Accelerometer raw value
    pitchDegree,rollDegree,yawDegree=getIMUAngle(period)
    #Offset correction:
    rollDegree=rollDegree+Y_offset
    pitchDegree=pitchDegree+X_offset

    #Roll = (vehicle.attitude.roll*180/math.pi-0.0)
    #Pitch = (vehicle.attitude.pitch*180/math.pi-0.0)

    output_pid_roll = pid_roll(rollDegree)
    output_pid_pitch = pid_pitch(pitchDegree)

    #ServoAng1 = map(output_pid_roll, -60.0, 60.0, 80.0, 200.0)
    ServoAng2 = map(output_pid_pitch, -60.0, 60.0, 195.0, 75.0)
    #ServoAng1 = 140 - rollDegree
    #ServoAng2 = 135 + pitchDegree

    #RunServo1(ServoAng1)
    RunServo2(ServoAng2)

    endTime = time.time()
    period = endTime - startTime

    #print("ServoAng1:%f" %ServoAng1)
    print("ServoAng2:%f" %ServoAng2)
    print("Roll:%f" %rollDegree,"Pitch:%f" %pitchDegree )
    print("Period: %f" %period)
    print("-------------------------------------------------")

