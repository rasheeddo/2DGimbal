import serial
import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
'''
def SetOperatingMode(MODE):

    TorqueOff()

    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, MODE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, MODE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_OPERATING_MODE, MODE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_OPERATING_MODE, MODE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_OPERATING_MODE, MODE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_OPERATING_MODE, MODE)


    present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE)
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

def TorqueOn():
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)

def TorqueOff():
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    #dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL7_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)

def IsMoving1():
    Moving1, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MOVING)
    return Moving1

def IsMoving2():
    Moving2, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MOVING)
    return Moving2

def MovingStatus1():
    MovingStat1, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_MOVING_STATUS)
    
    if MovingStat1 > 48:
        print("Motor1 is in Trapezodal Profile")
    elif MovingStat1 < 35 and MovingStat1 > 20:
        print("Motor1 is in Triangular Profile")
    elif MovingStat1 < 20 and MovingStat1 > 3:
        print("Motor1 is in Rectangular Profile")
    elif MovingStat1 < 3:
        print("Motor1 is in Step Mode (No Profile)")
    else:
        print("UNKNOWN Profile...")

    return MovingStat1

def MovingStatus2():
    MovingStat2, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_MOVING_STATUS)
    
    if MovingStat2 > 48:
        print("Motor2 is in Trapezodal Profile")
    elif MovingStat2 < 35 and MovingStat2 > 20:
        print("Motor2 is in Triangular Profile")
    elif MovingStat2 < 20 and MovingStat2 > 3:
        print("Motor2 is in Rectangular Profile")
    elif MovingStat2 < 3:
        print("Motor2 is in Step Mode (No Profile)")
    else:
        print("UNKNOWN Profile...")

    return MovingStat2

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



####################################################### Set Servo Configuration #############################################################
# Control table address
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
DXL7_ID                      = 7

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
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

SetOperatingMode(VELOCITY_CONTROL)
SetVelocityLimit2(700)

TorqueOn()
'''
ser = serial.Serial('/dev/ttyACM0',500000)

s = []
data=[]
lastS0 = 1490
lastS1 = 1490

while True:
    '''
    read_serial=ser.readline()
    s[0] = str(ser.readline())
    #S = int(s[0])
    #Smap = map(S,-3,105,0,100)
    print s
    #print("INT: %d" %int(s[0]))
    #DriveServoSpeed2(int(s[0]))

    '''
    startTime = time.time()
    data = str(ser.readline())
    S = data.split(',')

    if len(S) == 2:
        stick = S[0]
        switch = S[1]
    else:
        stick = lastS0
        switch = lastS1

    stick_pwm = int(float(stick))
    #switch_pwm = int(float(switch))
    endTime = time.time()
    period = endTime - startTime
    print S
    print stick
    print switch
    control_ang = map(stick_pwm,1083,1905,-1.0,1.0)
    print control_ang
    lastS0 = stick
    lastS1 = switch
    print ("Period %f" %period)


    '''
    else:
        print "There is some trash data..."
        print stick
        print switch
    '''
