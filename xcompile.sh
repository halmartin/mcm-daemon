#!/bin/bash
make clean

if [ ! -d iniparser ] ; then
	git clone http://github.com/ndevilla/iniparser
	cd iniparser 
	make CC=arm-linux-gnueabihf-gcc AR=arm-linux-gnueabihf-ar RANLIB=arm-linux-gnueabihf-ranlib
	cd ..
fi
make CC=arm-linux-gnueabihf-gcc CFLAGS="-I. -I iniparser/src" LIBS="-L iniparser -liniparser"
