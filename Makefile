#############################################################################
# Makefile for building: Evolutionary Learning
#############################################################################

PROJECT      ?= edgeflow

####### Compiler, tools and options

CC            = gcc
CXX           = g++ 
DEFINES	      = -DCOMPILE_ON_LINUX
CFLAGS        = -pipe -g -Wall -W $(DEFINES) $(shell pkg-config --cflags opencv) -fpermissive
CXXFLAGS      = -pipe -g -Wall -W $(DEFINES) -MMD -std=c++11 -fpermissive
#CXXFLAGS      += $(shell pkg-config --cflags opencv)
LINK          = g++
#LFLAGS        = $(shell pkg-config --libs opencv)
LIBS          = $(SUBLIBS) -lrt -pthread -L../../../opencv/install/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lopencv_videoio -Wl,-rpath,${CURDIR}/../../../opencv/install/lib
DEL_FILE      = rm -f

CV_PATH		  = ../drone_vision
INC_PATH	  = -I../../ -I../../common -I../../stereoboard -I../../stereoboard/drivers/inc -I../../stereoboard/math -I../../../opencv/install/include
CXXFLAGS	  +=-I${CV_PATH} -I${CV_PATH}/cv ${INC_PATH}

####### Output directory

OBJECTS_DIR   = ./

####### Files

TARGET  = testing

SOURCES = $(CV_PATH)/cv/image.c $(wildcard ../../stereoboard/math/*.c)

ifeq ($(PROJECT), gate)
	SOURCES += gate_detection.cpp \
            ../../stereoboard/gate_detection.c \
            ../../stereoboard/stereo_image.c
            
    DEFINES += -DGATE_DETECTION_GRAPHICS=1
else
    SOURCES += main.cpp \
            ../../stereoboard/edgeflow.c \
            $(CV_PATH)/cv/encoding/jpeg.c
endif

$(info $(SOURCES))

CPP_OBJECTS = $(SOURCES:.cpp=.o)
OBJECTS = $(CPP_OBJECTS:.c=.o)

$(info $(OBJECTS))

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) -o "$@" "$<"

.c.o:
	$(CC)  -c $(CFLAGS) -o "$@" "$<"

####### Build rules
.PHONY : all clean

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)
	{ test -n "$(DESTDIR)" && DESTDIR="$(DESTDIR)" || DESTDIR=.; } && test $$(gdb --version | sed -e 's,[^0-9]\+\([0-9]\)\.\([0-9]\).*,\1\2,;q') -gt 72 && gdb --nx --batch --quiet -ex 'set confirm off' -ex "save gdb-index $$DESTDIR" -ex quit '$(TARGET)' && test -f $(TARGET).gdb-index && objcopy --add-section '.gdb_index=$(TARGET).gdb-index' --set-section-flags '.gdb_index=readonly' '$(TARGET)' '$(TARGET)' && rm -f $(TARGET).gdb-index || true

debug:
	gdb $(TARGET)
  
clean: 
	$(MAKE) -C ../../stereoboard clean
	$(DEL_FILE) $(wildcard *.o) $(wildcard *.d) $(TARGET)
	$(DEL_FILE) $(OBJECTS)

####### Compile

%.o: %.c 
	$(CXX) -c $(CXXFLAGS) $(DEPFLAGS) -o $@ $<
	
####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

-include $(OBJECTS:.o=.d)
