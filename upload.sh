#!/bin/sh

stm32flash -b 115200 -w Debug/ReadSensor2.hex /dev/ttyUSB0 -g 0x0
if [ $? -eq 0 ]; then
	echo "first upload success"
else
	echo "first upload fail, sending R for reset"
	stty -F /dev/ttyUSB0 921600
	echo 'R' > /dev/ttyUSB0
	sleep 0.1
	stm32flash -b 115200 -w Debug/ReadSensor2.hex /dev/ttyUSB0 -g 0x0
fi