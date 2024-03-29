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
CFLAGS_ELF_OUTPUT=$(CFLAGS_HW_CFG) -Wall -Os -L ../libarduino -L ../libcommon
CFLAGS=-c $(CFLAGS_HW_CFG) -Wall -Os -I ../libarduino -I ../libcommon
AVRDUDE_VERBOSITY=
STRIP=avr-strip

SRC := ${wildcard *.c}

INC := ${wildcard *.h}

OBJ := $(SRC:.c=.o)

all: controller

controller: controller.hex

controller.hex: controller.elf
	@cp controller.elf controller_unstripped.elf
	$(STRIP) -s controller.elf
	$(OBJCOPY) -O ihex -R .eeprom controller.elf controller.hex

controller.elf: $(OBJ) $(SRC) $(INC)
	make -C ../libarduino
	make -C ../libcommon
	$(CC) $(CFLAGS_ELF_OUTPUT) *.o -o controller.elf -lcommon -larduino

upload: controller
	avrdude $(AVRDUDE_VERBOSITY) -C /etc/avrdude.conf -q -q $(AVRDUDE_HW_CFG) -D -Uflash:w:controller.hex:i

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex
	rm -f *~

mrproper: clean
	make -C ../libarduino
	make -C ../libcommon

