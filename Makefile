GRIPPROJ=GearLift

# Use the FRC-provided toolchain for building C++ binaries to run on the RoboRIO
CXX=/usr/local/bin/arm-frc-linux-gnueabi-c++
CC=/usr/local/bin/arm-frc-linux-gnueabi-gcc

RIODIR=$(HOME)/wpilib/cpp/current
NTLIBS=-lntcore
CSLIBS=-lcscore -lwpilibc
OPENCVLIBS=-lopencv_imgproc -lopencv_imgcodecs -lopencv_core
LDLIBS=-Wl,-rpath-link,$(RIODIR)/lib -L$(RIODIR)/lib $(OPENCVLIBS) $(CSLIBS) $(NTLIBS) -lstdc++
CPPFLAGS=-I$(GRIPPROJ) -I$(RIODIR)/include
CXXFLAGS=-std=c++11

SRCS=GRIPonRoboRIO.cpp $(GRIPPROJ)/GripPipeline.cpp
OBJS=$(subst .cpp,.o,$(SRCS))

all: GRIPonRoboRIO

GRIPonRoboRIO: $(OBJS)

GRIPonRoboRIO.o: GRIPonRoboRIO.cpp $(GRIPPROJ)/GripPipeline.h

$(GRIPPROJ)/GripPipeline.o: $(GRIPPROJ)/GripPipeline.cpp $(GRIPPROJ)/GripPipeline.h

clean:
	$(RM) $(OBJS) GRIPonRoboRIO
