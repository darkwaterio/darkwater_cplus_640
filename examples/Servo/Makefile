CC = g++
DW = ../../darkwater
INCLUDES = -I ../..

all:
	$(CC) $(INCLUDES) Servo.cpp $(DW)/PCA9685.cpp $(DW)/I2Cdev.cpp $(DW)/gpio.cpp $(DW)/Util.cpp -o Servo

clean:
	rm Servo
