# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# MPU-6000
# This code is designed to work with the MPU-6000_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/content/Accelorometer?sku=MPU-6000_I2CS#tabs-0-product_tabset-2

import smbus
import time
from time import sleep 
import datetime

# Get I2C bus
bus = smbus.SMBus(1)

# MPU-6000 address, 0x68(104)
# Select gyroscope configuration register, 0x1B(27)
#		0x18(24)	Full scale range = 2000 dps
bus.write_byte_data(0x68, 0x1B, 0x18)
# MPU-6000 address, 0x68(104)
# Select accelerometer configuration register, 0x1C(28)
#		0x18(24)	Full scale range = +/-16g
bus.write_byte_data(0x68, 0x1C, 0x18)
# MPU-6000 address, 0x68(104)
# Select power management register1, 0x6B(107)
#		0x01(01)	PLL with xGyro reference
bus.write_byte_data(0x68, 0x6B, 0x01)
#Enable Magnetometer
#bus.write_byte_data(0x68, 0x36, 0xff)
ret =bus.read_i2c_block_data(0x68, 0x37)
print(ret)
bus.write_byte_data(0x68, 0x37, 0x32)
sleep(0.5)
ret =bus.read_i2c_block_data(0x68, 0x37)
print(ret)
for i in range(0,10):
	sleep(1)
	bus.write_byte_data(0x68, 0x37, 0x02)
	sleep(0.5)
	bus.write_byte_data(0x68, 0x38, 0x01)
	bus.write_byte_data(0x68, 0x39, 0xff)
	sleep(1)
	ret =bus.read_i2c_block_data(0x68, 0x37)
	print(ret)
#bus.write_byte_data(0x68, 0x38, 0xff)



#bus.write_byte_data(0x0C, 0x0A7, 0x00)
#ret = bus.read_i2c_block_data(0x0C, 0x75)

#print ("Who am I %d" %(ret))
time.sleep(0.8)
xGyroPrev= 0
yGyroPrev= 0
zGyroPrev= 0
xGyroAngle =0.0
yGyroAngle =0.0
zGyroAngle =0.0
xGyroOffset=37
yGyroOffset=5
zGyroOffset=-59
TotalAngle=0

count = 0
xCum =0.0
yCum =0.0
zCum =0.0

xGyroAnglePrev=0

dtPrev = datetime.datetime.now()

while True:
	# MPU-6000 address, 0x68(104)
	# Read data back from 0x3B(59), 6 bytes
	# Accelerometer X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
	data = bus.read_i2c_block_data(0x68, 0x3B, 6)
	dtNow = datetime.datetime.now()
	dtDiff = dtNow -dtPrev

	# Convert the data
	xAccl = data[0] * 256 + data[1]
	if xAccl > 32767 :
		xAccl -= 65536

	yAccl = data[2] * 256 + data[3]
	if yAccl > 32767 :
		yAccl -= 65536

	zAccl = data[4] * 256 + data[5]
	if zAccl > 32767 :
		zAccl -= 65536

	# MPU-6000 address, 0x68(104)
	# Read data back from 0x43(67), 6 bytes
	# Gyrometer X-Axis MSB, X-Axis LSB, Y-Axis MSB, Y-Axis LSB, Z-Axis MSB, Z-Axis LSB
	data = bus.read_i2c_block_data(0x68, 0x43, 6)

	# Convert the data
	xGyro = data[0] * 256 + data[1]
	if xGyro > 32767 :
		xGyro -= 65536
	xCum +=xGyro
	xGyro -= xGyroOffset

	yGyro = data[2] * 256 + data[3]
	if yGyro > 32767 :
		yGyro -= 65536
	yGyro -= yGyroOffset

	zGyro = data[4] * 256 + data[5]
	if zGyro > 32767 :
		zGyro -= 65536
	zGyro -= zGyroOffset

	
	yCum +=yGyro
	zCum +=zGyro
	# Output data to screen
	""" print("Acceleration in X-Axis : %d" %xAccl)
	print("Acceleration in Y-Axis : %d" %yAccl) 
	print("Acceleration in Z-Axis : %d" %zAccl)
	print("X-Axis of Rotation : %d" %xGyro)
	print("Y-Axis of Rotation : %d" %yGyro)
	print("Z-Axis of Rotation : %d" %zGyro) """
	xGyroAngleTemp=(xGyro*dtDiff.microseconds/(16.4*1000000))
	if abs(xGyro) > 60:
		xGyroAngle +=(xGyroAngleTemp)
		TotalAngle +=abs(xGyroAngleTemp)
	#print (dtDiff.microseconds)
	#xGyroAngle = xGyroAngle % 180
	yGyroAngle+=(yGyroPrev-yGyro)*180.0/32767.0
	yGyroAngle = yGyroAngle % 180
	zGyroAngle+=(zGyroPrev-zGyro)*180.0/32767.0
	zGyroAngle = zGyroAngle % 180
	if abs(xGyro) > 60 :
		print("X : %f Prev: %f xGyro %d" % ( xGyroAngle, TotalAngle,xGyro))
	#print("X : %d Y: %d Z %d" % ( xGyroAngle,yGyroAngle,zGyroAngle))
	xGyroAnglePrev=xGyroAngle
	xGyroPrev= xGyro
	yGyroPrev= yGyro
	zGyroPrev= zGyro
	count +=1
	if count > 3000:
		break
	dtPrev =dtNow

	sleep(0.01)

print ("XAvg : %d YAvg: %d ZAvg %d" % ( xCum/count, yCum/count, zCum/count))