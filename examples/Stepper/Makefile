CC = g++
DW = ../../darkwater
INCLUDES = -I ../..

all:
	$(CC) $(INCLUDES) Stepper.cpp $(DW)/DW640.cpp $(DW)/PCA9685.cpp $(DW)/I2Cdev.cpp $(DW)/gpio.cpp $(DW)/Util.cpp -o Stepper

clean:
	rm Stepper
