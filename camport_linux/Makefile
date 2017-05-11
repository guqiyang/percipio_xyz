TOPDIR  := $(shell pwd)

CROSS_COMPILE	:= $(CROSS_COMPILE)
CC 	:= $(CROSS_COMPILE)gcc

SYS	:= $(shell $(CC) -dumpmachine)

COMMON_CFLAGS := -g -O0 -Wall

CFLAGS	:= $(COMMON_CFLAGS) -fPIC
CFLAGS	+= -I$(TOPDIR)/include

LDFLAGS := -lstdc++ -lpthread -ljpeg  -lusb-1.0 -lopencv_core -lm -lboost_system
LDFLAGS	+= -lopencv_imgproc -lopencv_highgui -lopencv_contrib -lpcl_io -lpcl_common

LIBCAM	  := libcamm.so

CAM_TEST  := camtest
CB_TEST   := cbtest
TRIG_TEST := trigtest

CAMTEST_SRC := $(wildcard $(TOPDIR)/cam_test/*.cpp)
CBTEST_SRC  := $(wildcard $(TOPDIR)/callback_test/*.cpp)
TRIGTEST_SRC:= $(wildcard $(TOPDIR)/trigger_test/*.cpp)

SOURCES := $(CAMTEST_SRC) $(CBTEST_SRC) $(TRIGTEST_SRC)
OBJS	:= $(addsuffix .o , $(basename $(SOURCES)))

all: $(CAM_TEST) $(CB_TEST) $(TRIG_TEST)

$(CAM_TEST): $(CAMTEST_SRC)
	$(CC) $(CFLAGS) $(CAMTEST_SRC) -o $(CAM_TEST)  $(LDFLAGS) -L ./lib  -lcamm

$(CB_TEST): $(CBTEST_SRC)
	$(CC) $(CFLAGS) $(CBTEST_SRC) -o $(CB_TEST)  $(LDFLAGS) -L ./lib  -lcamm

$(TRIG_TEST): $(TRIGTEST_SRC)
	$(CC) $(CFLAGS) $(TRIGTEST_SRC) -o $(TRIG_TEST)  $(LDFLAGS) -L ./lib  -lcamm

clean:
	rm -rf $(OBJS) $(CAM_TEST) $(CB_TEST) $(TRIG_TEST)
