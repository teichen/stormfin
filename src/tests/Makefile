SHELL = /bin/sh

# make all
# make run_test
# make sensors_test
# make utilities_test
# make filter_test
# make clean

OBJS_RUN_TEST = run_test.o ../RunGNC.o ../Sensors.o
OBJS_SENSORS_TEST = sensors_test.o ../Sensors.o
OBJS_UTILITIES_TEST = utilities_test.o ../Utilities.o ../GSLWrappers.o ../LaminarModel.o
OBJS_FILTER_TEST = filter_test.o ../KalmanFilter.o ../ComplementaryFilter.o ../Utilities.o ../GSLWrappers.o ../LaminarModel.o
CFLAGS = -g -O0
CC = clang++
INCLUDES =
LIBS = -L/usr/local/lib -L/usr/local/Cellar/gperftools/2.10/lib -lgsl -lgslcblas -lprofiler

all:run_test sensors_test utilities_test filter_test

run_test:${OBJS_RUN_TEST}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS_RUN_TEST} ${LIBS}
	./run_test

sensors_test:${OBJS_SENSORS_TEST}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS_SENSORS_TEST} ${LIBS}
	./sensors_test

utilities_test:${OBJS_UTILITIES_TEST}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS_UTILITIES_TEST} ${LIBS}
	./utilities_test

filter_test:${OBJS_FILTER_TEST}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS_FILTER_TEST} ${LIBS}
	./filter_test

clean:
	-rm -f *.o core *.core *.dat

.cpp.o:
	${CC} ${CFLAGS} ${INCLUDES} -c $<


