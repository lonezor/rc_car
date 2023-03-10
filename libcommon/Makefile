### HW config ####

# Arduino MEGA 1280
#CFLAGS_HW_CFG := -mmcu=atmega1280 -DF_CPU=16000000
#AVRDUDE_HW_CFG := -p m1280 -c stk500v1 -P/dev/ttyUSB0 -b57600

# Arduino MEGA 2560
CFLAGS_HW_CFG := -mmcu=atmega2560 -DF_CPU=16000000
AVRDUDE_HW_CFG := -p m2560 -c stk500v2 -P/dev/ttyACM0

CC=avr-g++
CPP=avr-g++
AR=avr-ar
LD=avr-ld
OBJCOPY=avr-objcopy
CFLAGS=-c $(CFLAGS_HW_CFG) -Wall -Os -I ../libarduino
AVRDUDE_VERBOSITY=
STRIP=avr-strip

SRC := ${wildcard *.c}

INC := ${wildcard *.h}

OBJ := $(SRC:.c=.o)

all: common

common: $(OBJ) $(SRC) $(INC)
	make -C ../libarduino
	$(AR) rcs libcommon.a $(OBJ)

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex
	rm -f *~
