INCLUDES = -I../STM32L1_Discovery_Firmware_Pack_V1.0.3/Libraries/CMSIS/Device/ST/STM32L1xx/Include/
INCLUDES += -I../STM32L1_Discovery_Firmware_Pack_V1.0.3/Libraries/CMSIS/Include/
all:
	arm-none-eabi-gcc -O0 -g -mcpu=cortex-m3 -c -o main.o main.c -Wall -mthumb ${INCLUDES} -fno-common
	arm-none-eabi-ld -TSTM32L152xB.ld -nostartfiles -o demo.elf main.o
	arm-none-eabi-objcopy -Obinary demo.elf demo.bin

install:
	~/Workspace/ucontroller/stlink/st-flash write ./demo.bin 0x8000000
