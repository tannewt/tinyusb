CROSS_COMPILER = lm32-none-

CFLAGS = \
	-DCFG_TUSB_MCU=OPT_MCU_LM32 \

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/metro_m0_express/samd21g18a_flash.ld

LD_FLAGS +=

SRC_C += \
	-I$(TOP)/hw/mcu/valentyusb/main.c

INC += \
	-I$(TOP)/hw/mcu/valentyusb/

VENDOR = valentyusb
CHIP_FAMILY = valentyusb
