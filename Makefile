CC = gcc
CFLAGS = -Wall -O3
INCLUDES = -I/opt/local/include
LFLAGS = -L/opt/local/lib
LIBS = -lpng -ltiff -ljpeg -lm 

OBJS = fish2sphere.o bitmaplib.o

all: fish2sphere

fish2sphere: $(OBJS)
	$(CC) $(INCLUDES) $(CFLAGS) -o fish2sphere $(OBJS) $(LFLAGS) $(LIBS)

fish2sphere.o: fish2sphere.c fish2sphere.h
	$(CC) $(INCLUDES) $(CFLAGS) -c fish2sphere.c
 
bitmaplib.o: bitmaplib.c bitmaplib.h
	$(CC) $(INCLUDES) $(CFLAGS) -c bitmaplib.c

clean:
	rm -rf core fish2sphere $(OBJS)

