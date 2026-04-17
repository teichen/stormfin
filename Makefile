SHELL = /bin/sh

# make clean
# make all
# make run

OBJS = main.o Controller.o Filter.o Sensors.o Drivers.o Propagate.o Update.o
SRC = main.cpp Controller.cpp Filter.cpp Sensors.cpp Drivers.cpp Propagate.cpp Update.cpp
CFLAGS = -g -O0
CC = clang++
INCLUDES = -I/usr/local/include
LIBS = -L/usr/local/lib -L/usr/local/Cellar/gperftools/2.10/lib -lgsl -lgslcblas -lprofiler

all:run

run:${OBJS}
	${CC} ${CFLAGS} ${INCLUDES} -o $@ ${OBJS} ${LIBS}

clean:
	-rm -f *.o core *.core *.dat *.dylib

.cpp.o:
	${CC} ${CFLAGS} ${INCLUDES} -c $<


