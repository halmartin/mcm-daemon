ifeq ($(DEB_HOST_GNU_TYPE),)
	CC     ?= gcc
	AR     ?= ar
	STRIP  ?= strip
else
	CC    = $(DEB_HOST_GNU_TYPE)-gcc
	AR    = $(DEB_HOST_GNU_TYPE)-ar
	STRIP = $(DEB_HOST_GNU_TYPE)-strip
endif

CFLAGS += -I. -Iiniparser/src -I/usr/include
LIBS   +=-Liniparser -liniparser

DEPS = mcm.h iniparser/src/iniparser.h
OBJ  = mcm-daemon.o

INSTALL = install

ETC_DIR=$(DESTDIR)/etc
SYS_DIR=$(DESTDIR)/etc/systemd/system/
BIN_DIR=$(DESTDIR)/sbin

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

mcm-daemon: $(OBJ) iniparser/libiniparser.a
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS)

iniparser/libiniparser.a: iniparser
	make -C iniparser CC=$(CC) AR=$(AR) STRIP=$(STRIP)

iniparser/src/iniparser.h:
	git clone http://github.com/ndevilla/iniparser

clean:
	make -C iniparser veryclean
	rm -f *.o mcm-daemon
	

install:
	$(INSTALL) -d $(ETC_DIR)
	$(INSTALL) -d $(SYS_DIR)
	$(INSTALL) -d $(BIN_DIR)
	$(INSTALL) -t $(ETC_DIR) mcm-daemon.ini
	$(INSTALL) -t $(SYS_DIR) mcm-daemon.service
	$(INSTALL) -t $(BIN_DIR) mcm-daemon
