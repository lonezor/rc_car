### HW config ####

# Arduino MEGA 1280
#CFLAGS_HW_CFG := -mmcu=atmega1280 -DF_CPU=16000000
#AVRDUDE_HW_CFG := -p m1280 -c stk500v1 -P/dev/ttyUSB0 -b57600

# Arduino MEGA 2560
CFLAGS_HW_CFG := -mmcu=atmega2560 -DF_CPU=16000000
AVRDUDE_HW_CFG := -p m2560 -c stk500v2 -P/dev/ttyACM0

# Build mode
#CFLAGS_BUILD_MODE := -DPCU_BUILD
#CFLAGS_BUILD_MODE := -DSCU_BUILD
CFLAGS_BUILD_MODE := -DVMU_BUILD

CC=avr-g++
CPP=avr-g++
AR=avr-ar
LD=avr-ld
OBJCOPY=avr-objcopy
CFLAGS_ELF_OUTPUT=$(CFLAGS_HW_CFG) $(CFLAGS_BUILD_MODE) -Wall -Os -L ../libarduino -L ../libcommon
CFLAGS=-c $(CFLAGS_HW_CFG) $(CFLAGS_BUILD_MODE) -Wall -Os -I ../libarduino -I ../libcommon
AVRDUDE_VERBOSITY=
STRIP=avr-strip

SRC := ${wildcard *.c}
INC := ${wildcard *.h}
OBJ := $(SRC:.c=.o)

all: vehicle

vehicle: vehicle.hex

vehicle.hex: vehicle.elf
	@cp vehicle.elf vehicle_unstripped.elf
	$(STRIP) -s vehicle.elf
	$(OBJCOPY) -O ihex -R .eeprom vehicle.elf vehicle.hex

vehicle.elf: $(OBJ) $(SRC) $(INC)
	make -C ../libarduino
	make -C ../libcommon
	$(CC) $(CFLAGS_ELF_OUTPUT) *.o -o vehicle.elf -lcommon -larduino

upload: vehicle
	avrdude $(AVRDUDE_VERBOSITY) -C /etc/avrdude.conf -q -q $(AVRDUDE_HW_CFG) -D -Uflash:w:vehicle.hex:i

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex
	rm -f *~

mrproper: clean
	make -C ../libarduino
	make -C ../libcommon
