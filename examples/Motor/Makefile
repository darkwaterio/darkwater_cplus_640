CC = g++
DW = ../../darkwater
INCLUDES = -I ../..

all:
	$(CC) $(INCLUDES) Motor.cpp $(DW)/DW640.cpp $(DW)/PCA9685.cpp $(DW)/I2Cdev.cpp $(DW)/gpio.cpp $(DW)/Util.cpp -o Motor

clean:
	rm Motor
