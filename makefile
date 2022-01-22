# compiler
cc = gcc
# CFLAGS
CFLAGS =-c -lm -Wall
OFLAGS = -lm -Wall -o
# complilation into object file & linking object file into excutable
# dependencies
all: simulate

simulate: main.o libpid.o
	$(cc) main.o libpid.o $(OFLAGS) sim
	
main.o: main.c libpid.h
	$(cc) $(CFLAGS) main.c
	
libpid.o: libpid.h
	$(cc) $(CFLAGS) libpid.c

clean:
	rm -rf *.o
run:
	./sim
