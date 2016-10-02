CC=gcc
PROG=mikes
SRCS=mikes.c \
     public_relations.c \
     range_sensor.c \
     mikes_logs.c \
     navigation.c \
     gui.c \
     windows.c \
     config/config.c \
     config_mikes.c
OBJS=${SRCS:.c=.o}
CFLAGS=-std=c11 -D_BSD_SOURCE -I/usr/include/cairo -g
LDFLAGS=-lpthread -lcairo -lX11 -lm
PREFIX=/usr/local

all: ${OBJS}
	${CC} ${OBJS} -o ${PROG} ${LDFLAGS}

install:
	mv ${PROG} ${PREFIX}/bin
	cp mikes.sh /etc/init.d/mikes
	chmod a+x /etc/init.d/mikes
	update-rc.d mikes defaults
	cp config_mikes.cfg ${PREFIX}/etc

uninstall:
	update-rc.d -f mikes remove
	rm -f ${PREFIX}/bin/${PROG}

clean:
	rm -f *.o ${PROG}
