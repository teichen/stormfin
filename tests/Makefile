SHELL = /bin/sh

# make all
# make sensors_test
# make clean

OBJS_SENSORS_TEST = sensors_test.o ../Controller.o ../Filter.o ../Sensors.o ../Thrusters.o ../LaminarModel.o ../Model.o ../RungeKutta.o ../Collocation.o
CFLAGS = -g -O0
CC = clang++
INCLUDES =
LIBS = -L/usr/local/lib -L/usr/local/Cellar/gperftools/2.10/lib -lgsl -lgslcblas -lprofiler

all:sensors_test

sensors_test:${OBJS_SENSORS_TEST}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS_SENSORS_TEST} ${LIBS}

clean:
	-rm -f *.o core *.core *.dat

.cpp.o:
	${CC} ${CFLAGS} ${INCLUDES} -c $<


