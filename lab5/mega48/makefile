PRG            =lab5_mega48

OBJ            = $(PRG).o lm73_functions.o twi_master.o uart_functions.o

SRCS	       = $(PRG).c lm73_functions.c twi_master.c uart_functions.c

MCU_TARGET     = atmega48
PROGRAMMER_TARGET = m48
OPTIMIZE       = -O2    # options are 1, 2, 3, s
CC             = avr-gcc

CPU_SPEED = 8000000UL

LFUSE_VAL = E2
HFUSE_VAL = DF
EFUSE_VAL = 01

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS) -DF_CPU=$(CPU_SPEED)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean: 
	rm -rf *.o $(PRG).elf *.bin *.hex *.srec *.bak  
	rm -rf $(PRG)_eeprom.bin $(PRG)_eeprom.hex $(PRG)_eeprom.srec
	rm -rf *.lst *.map 

#setup for for USB programmer
#may need to be changed depending on your programmer
program: clean $(PRG).hex
	sudo avrdude -p $(PROGRAMMER_TARGET) -c osuisp2 -e -U flash:w:$(PRG).hex  -v

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@

.PHONY : fuses
fuses:
	sudo avrdude -p $(PROGRAMMER_TARGET) -c osuisp2 -e -U lfuse:w:0x$(LFUSE_VAL):m -U hfuse:w:0x$(HFUSE_VAL):m -U efuse:w:0x$(EFUSE_VAL):m -v
