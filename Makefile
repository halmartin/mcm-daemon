CC?=gcc
CFLAGS?=-I. -I/usr/include
LIBS=-liniparser
DEPS = mcm.h
OBJ = mcm-daemon.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

mcm-daemon: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

clean:
	rm -f *.o
