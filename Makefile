CC=arm-none-eabi-gcc

INCLUDE_DIRS += inc
INCLUDE_DIRS += drivers/CMSIS/Include
INCLUDE_DIRS += drivers/CMSIS/Device/ST/STM32L4xx/Include

ARCH=cortex-m4
CFLAGS= -mcpu=$(ARCH) -mthumb -Wall -DSTM32L476xx $(foreach D, $(INCLUDE_DIRS), -I$(D))
LDFLAGS= -Wl,-Map=$(BUILD_DIR)/output.map -nostdlib 

BUILD_DIR=build

OBJECTS += startup.o
OBJECTS += main.o
OBJECTS += rcc.o gpio.o

OBJECT_PATHS=$(foreach O, $(OBJECTS), $(BUILD_DIR)/$(O))

LINK_SCRIPT=link.ld

.PHONY: all clean flash

all: $(BUILD_DIR)/firmware.elf

$(BUILD_DIR)/firmware.bin: $(BUILD_DIR)/firmware.elf
	arm-none-eabi-objcopy -O binary $^ $@

$(BUILD_DIR)/firmware.elf: $(OBJECT_PATHS)
	$(CC) $(LDFLAGS) -T $(LINK_SCRIPT) -o $@ $^


$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD_DIR)/startup.o: startup.s | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c -o $@ $<


flash: $(BUILD_DIR)/firmware.elf
	STM32_Programmer_CLI -c port=SWD mode=UR -d $< 0x08000000 -rst

$(BUILD_DIR):
	mkdir -p $@

clean:
	rm -rf $(BUILD_DIR)
