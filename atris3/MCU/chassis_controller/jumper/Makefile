TARGET=jumper.hex
EXECUTABLE=jumper.elf
BIN_IMAGE=jumper.bin

ifeq ($(OS),Windows_NT)     # is Windows_NT on XP, 2000, 7, Vista, 10...
    detected_OS := Windows
else
    detected_OS := $(shell uname)  # same as "uname -s"  
    export PATH += :../../toolchain/gcc-arm-none-eabi-6_2-2016q4/bin
endif

# $(info $(detected_OS) )

CC=arm-none-eabi-gcc
#LD=arm-none-eabi-ld 
LD=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy 
OD=arm-none-eabi-objdump

BIN=$(CP) -O ihex 

DEFS = -DUSE_STDPERIPH_DRIVER -DSTM32F4XX -DHSE_VALUE=25000000

MCU = cortex-m4
MCFLAGS = -mcpu=$(MCU) -mlittle-endian -std=c99 -lgcc -lc -lm -lrdimon \
-ffunction-sections -fdata-sections -Wl,--gc-sections -mthumb-interwork -mthumb

STM32_INCLUDES = -I./Libraries/CMSIS/ST/STM32F4xx/Include/ \
	-I./Libraries/CMSIS/Include/ \
	-I./Libraries/STM32F4xx_StdPeriph_Driver/inc/ \
	-I./app/


OPTIMIZE = -O2 -g 

CFLAGS	= $(MCFLAGS) $(OPTIMIZE) $(DEFS) -I./ -I./ $(STM32_INCLUDES)  -Wl,-T,stm32_flash.ld
#-mapcs-float use float regs. small increase in code size


SRC = $(wildcard ./app/*.c) \
	./Libraries/CMSIS/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c \
	./Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c \
	./Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
	./Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
	./Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
#	./Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c \


STARTUP = ./Libraries/CMSIS/ST/STM32F4xx/Source/Templates/gcc_ride7/startup_stm32f4xx.s

#OBJDIR = .
#OBJ = $(SRC:%.c=$(OBJDIR)/%.o) 
#OBJ += Startup.o

all: $(TARGET) $(BIN_IMAGE)
	

$(BIN_IMAGE): $(EXECUTABLE)
	$(CP) -O binary $^ $@

$(TARGET): $(EXECUTABLE)
	$(CP) -O ihex $^ $@

$(EXECUTABLE): $(SRC) $(STARTUP)
	$(CC) $(CFLAGS) $^ -lm -lc -o $@

flash:
	st-flash write $(BIN_IMAGE) 0x08000000

clean:
	rm  jumper.hex jumper.elf jumper.bin

