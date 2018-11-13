import serial
import time
from dynamixel_sdk import *                    # Uses Dynamixel SDK library

def map(val, in_min, in_max, out_min, out_max):

    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

ser = serial.Serial('/dev/ttyACM0',9600)

data=[]

while True:

    startTime = time.time()
    data = str(ser.readline())
    DATA = list(data)
    print(DATA)
    print len(DATA)
    if len(DATA) == 11:
    	DataCh1 = [DATA[0],DATA[1],DATA[2],DATA[3]]
    	DataCh2 = [DATA[5],DATA[6],DATA[7],DATA[8]]
    	Ch1 = "".join(DataCh1)
    	Ch2 = "".join(DataCh2)
    	print int(Ch1)
    	print int(Ch2)
    else:
    	print ("Trash Data")

    endTime = time.time()
    period = endTime - startTime
    print ("Period %f" %period)
