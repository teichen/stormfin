SHELL = /bin/sh

# make clean
# make all
# make run

CHIBB_OBJS = ChibbComm.o

STORMFIN_OBJS = StormFin.o MockData.o Controller.o DataStore.o KalmanFilter.o ComplementaryFilter.o Utilities.o GSLWrappers.o Acoustics.o Sensors.o Thrusters.o LaminarModel.o Collocation.o

CFLAGS = -g -O0
CC = clang++
INCLUDES = -I/usr/local/include
LIBS = -L/usr/local/lib -L/usr/local/Cellar/gperftools/2.10/lib -lgsl -lgslcblas -lm -lprofiler -lwiringPi

all:run_stormfin run_chibb

run_stormfin:${STORMFIN_OBJS}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${STORMFIN_OBJS} ${LIBS}

run_chibb:${CHIBB_OBJS}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${CHIBB_OBJS} ${LIBS}

clean:
	-rm -f *.o core *.core *.dat *.dylib

.cpp.o:
	${CC} ${CFLAGS} ${INCLUDES} -c $<


