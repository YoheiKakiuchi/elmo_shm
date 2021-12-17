# Executable name
NL =		lpms_imu_shm

LPDIR = LpSensor-1.3.5-Linux-x86-64
NEW_LIBDIRS =	$(LPDIR)/lib/libLpSensor.so

# Release configuration
CFLAGS =	-c -O2 -Wall -Wextra -std=c++11 -std=gnu++11 -D_CRT_SECURE_NO_DEPRECATE

INCDIRS =	-I../include -Isrc -I$(LPDIR)/include
LIBS =		-lLpSensor -lm -lpthread -lrt -ldl
CC =		gcc
CXX=		g++
LD =		$(CXX)

#ifeq ($(USE_AMP),ON)
#SHMOBJ = ../controller/amp_shm.o
#else
#SHMOBJ = ../controller/system_shm.o
#endif

OBJ =		lpms_imu_shm.o $(SHMOBJ)

all:		chkLibrary $(NL)

$(NL):		$(OBJ)
	$(LD) $(OBJ) $(LIBS) -o $(NL)

lpms_imu_shm.o: src/lpms_imu_shm.cpp
	$(CXX) $(CFLAGS) $(INCDIRS) $<

clean:
	rm -f *.o
	rm -f $(NL)

chkLibrary:
	@if [ ! -e $(NEW_LIBDIRS) ]; then \
	echo "You should install smb://andromeda/share1/STARO/downloads/liblpsensor-1.3.5-Linux.deb"; \
	exit; \
	fi

