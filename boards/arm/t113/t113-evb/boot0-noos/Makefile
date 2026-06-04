# Standalone build for T113 boot0 (bare-metal, no NuttX)
#
# Produces boot0.bin with eGON header, ready for:
#   xfel spinand splwrite 2048 1048576 boot0.bin
#
# Usage:
#   make                  Build boot0.bin
#   make clean            Remove build artifacts
#   make flash            Flash to SPI NAND via xfel (must be in FEL mode)
#   make CROSS=xxx-       Use a different cross-compiler prefix

NUTTX_DIR ?= $(shell git -C $(dir $(lastword $(MAKEFILE_LIST))) rev-parse --show-toplevel)
ARCH_DIR   = $(NUTTX_DIR)/arch/arm/src/t113
SCRIPT_DIR = $(NUTTX_DIR)/boards/arm/t113/t113-evb/scripts
TOOLS_DIR  = $(NUTTX_DIR)/tools

CROSS  ?= arm-none-eabi-
CC      = $(CROSS)gcc
AS      = $(CROSS)gcc
LD      = $(CROSS)ld
OBJCOPY = $(CROSS)objcopy
SIZE    = $(CROSS)size

ASFLAGS  = -mcpu=cortex-a7 -marm -mfloat-abi=hard -mfpu=neon-vfpv4
DEFINES  = -DCONFIG_T113_BOOT0_NOOS
CFLAGS   = -mcpu=cortex-a7 -marm -mfloat-abi=hard -mfpu=neon-vfpv4 \
           -O3 -fno-lto -fno-stack-protector -fno-common \
           -ffunction-sections -fdata-sections -ffreestanding -nostdlib \
           -Wall -Wextra -Wno-unused-parameter $(DEFINES)
LDFLAGS  = -T $(SCRIPT_DIR)/sram.ld -nostdlib \
           --defsym CONFIG_IDLETHREAD_STACKSIZE=0 \
           --defsym CONFIG_ARCH_INTERRUPTSTACK=0

SRCS_S = $(ARCH_DIR)/t113_boot0.S
SRCS_C = $(ARCH_DIR)/t113_load.c

OBJS = boot0_asm.o boot0_load.o

BUILD_DIR = build

all: boot0.bin

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/boot0_asm.o: $(SRCS_S) | $(BUILD_DIR)
	$(AS) $(ASFLAGS) $(DEFINES) -c $< -o $@

$(BUILD_DIR)/boot0_load.o: $(SRCS_C) | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/boot0.elf: $(BUILD_DIR)/boot0_asm.o $(BUILD_DIR)/boot0_load.o
	$(LD) $(LDFLAGS) $^ -o $@
	$(SIZE) $@

boot0.bin: $(BUILD_DIR)/boot0.elf
	$(OBJCOPY) -O binary $< $@
	python3 $(TOOLS_DIR)/mksunxi.py --nand $@

flash: boot0.bin
	xfel spinand splwrite 2048 1048576 $<

clean:
	rm -rf $(BUILD_DIR) boot0.bin

.PHONY: all flash clean
