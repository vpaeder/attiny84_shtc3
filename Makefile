MCU=attiny84
F_CPU=1000000L
CC=avr-g++
LD=avr-ld
OBJCOPY=avr-objcopy
CFLAGS+=-Wall -g -Os -ffunction-sections -fdata-sections -mmcu=${MCU} -DF_CPU=${F_CPU} -I. -I/opt/local/avr/include
LDFLAGS+=-Wall -g -Os -Wl,--gc-sections -Wl,-u,vfprintf -lprintf_flt -mmcu=${MCU} 
TARGET=attiny_shtc3
SRCS=attiny_shtc3.cpp
OBJS=attiny_shtc3.o

all:
	${CC} ${CFLAGS} -c ${SRCS}
	${CC} ${LDFLAGS} -o ${TARGET}.bin ${OBJS}
	${OBJCOPY} -j .text -j .data -O ihex ${TARGET}.bin ${TARGET}.hex

flash:
	avrdude -p ${MCU} -c usbasp -U flash:w:${TARGET}.hex:i -F -P usb

clean:
	rm -f *.bin *.hex
