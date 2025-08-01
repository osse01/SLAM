CC = g++
CFLAGS = -Wall -std=c++11
RPLIDAR_SDK_PATH =  $(shell echo ~/rplidar_sdk)

INCLUDES = -I$(RPLIDAR_SDK_PATH)/sdk/include \
           -I$(RPLIDAR_SDK_PATH)/sdk/src \
           -Iinclude

LIBS = -L$(RPLIDAR_SDK_PATH)/output/Linux/Release \
       -lsl_lidar_sdk

SRCDIR = src
INCDIR = include
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)
TARGET = lidar_test

$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ $(LIBS)

%.o: %.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)

.PHONY: clean