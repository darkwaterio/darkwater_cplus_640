CC = g++
DW = ../../darkwater

PIGPIO_PATH := $(PIGPIO_PATH)

LIB = -L$(PIGPIO_PATH)
INCLUDES = -I ../.. -I$(PIGPIO_PATH)


all:
	$(CC) $(INCLUDES) $(LIB)  PPM.cpp $(DW)/PCA9685.cpp $(DW)/I2Cdev.cpp $(DW)/gpio.cpp $(DW)/Util.cpp -o PPM -lrt -lpthread -lpigpio

clean:
	rm PPM
