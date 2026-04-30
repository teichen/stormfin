SHELL = /bin/sh

# make clean
# make all
# make run

OBJS = main.o Controller.o KalmanFilter.o ComplementaryFilter.o Utilities.o Sensors.o Thrusters.o LaminarModel.o Collocation.o
SRC = main.cpp Controller.cpp KalmanFilter.cpp ComplementaryFilter.cpp Utilities.cpp Sensors.cpp Thrusters.cpp LaminarModel.cpp Collocation.cpp
CFLAGS = -g -O0
CC = clang++
INCLUDES = -I/usr/local/include
LIBS = -L/usr/local/lib -L/usr/local/Cellar/gperftools/2.10/lib -lgsl -lgslcblas -lm -lprofiler

all:run

run:${OBJS}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS} ${LIBS}

clean:
	-rm -f *.o core *.core *.dat *.dylib

.cpp.o:
	${CC} ${CFLAGS} ${INCLUDES} -c $<


