CC = g++

OPTIMIZE = -O

CFLAGS = $(OPTIMIZE) -Wno-unused-result `pkg-config bullet --cflags ` -g -O0
LFLAGS =  `pkg-config bullet_robotics --libs` -lBulletExampleBrowserLib -lOpenGLWindow -lBullet3Common

SRCS = main.cpp Mattress.cpp BedFrame.cpp Nursing.cpp
OBJS = $(SRCS:.cpp=.o)

TARGETS = main

.cpp.o:
	$(CC) $(CFLAGS) -c $<

all: $(TARGETS)

clean::
	rm -f *~ *.o core

distclean::
	rm -f *~ *.o core ${TARGETS}

main: $(OBJS) 
	$(CC) -o $@ $^ $(LFLAGS)
	strip $@

