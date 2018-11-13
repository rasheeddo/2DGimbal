from dronekit import connect, VehicleMode
import math
import time

vehicle = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', wait_ready=True, baud=115200)
Buff_P = list()
Buff_R = list()
while True:


	#print " Ch1: %s" % vehicle.channels['7']
	#print " Ch2: %s" % vehicle.channels['8']
	#print vehicle.channels

	startTime = time.time()
	print vehicle.attitude.pitch*180/math.pi
	print vehicle.attitude.roll*180/math.pi
	endTime = time.time()
	period = endTime - startTime
	print period	
	print("----------------------------------------")

	'''
	startTime = time.time()
	for i in range(0,10):
		Pitch = vehicle.attitude.pitch*180/math.pi
		Roll = vehicle.attitude.roll*180/math.pi
		Buff_P.append(Pitch)
		Buff_R.append(Roll)

		if i == 9:
			Ave_Pitch = sum(Buff_P)/len(Buff_P)
			Ave_Roll = sum(Buff_R)/len(Buff_R)
			Buff_P = list()
			Buff_R = list()

	endTime = time.time()
	period = endTime - startTime
	print Ave_Pitch
	print Roll
	print period
	print("--------------------")
	'''

