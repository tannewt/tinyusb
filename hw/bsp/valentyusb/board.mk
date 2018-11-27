include $(TOP)/../../../../build/tinyfpga_bx_usb_lm32.minimal/software/include/generated/variables.mak

CROSS_COMPILE = $(TRIPLE)-

CFLAGS = \
	-DCFG_TUSB_MCU=OPT_MCU_LM32 \
	-Wno-error=int-to-pointer-cast \
	-Wno-error=shadow \
	$(CPUFLAGS) \
	-g3 \
	-fomit-frame-pointer\
	-fno-builtin \
	-flto \
	-fuse-linker-plugin \
	-Os \
	-ffunction-sections \
	-fdata-sections \
	-flto \
	-nostartfiles \
	-nostdlib \
	-nodefaultlibs

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/valentyusb/linker.ld

LDFLAGS += \
	-L$(BUILDINC_DIRECTORY) \
	-L$(BUILDINC_DIRECTORY)/../libbase \
	-L$(BUILDINC_DIRECTORY)/../libcompiler_rt \
	$(BUILDINC_DIRECTORY)/../libbase/crt0-$(CPU)-xip.o \
	$(BUILDINC_DIRECTORY)/../libbase/libbase-nofloat.a \
	$(BUILDINC_DIRECTORY)/../libcompiler_rt/libcompiler_rt.a \
	-lbase-nofloat \
	-lcompiler_rt \

LIBS = \
	-lbase-nofloat \
	-lcompiler_rt \

#SRC_C += \
#	-I$(TOP)/hw/mcu/valentyusb/main.c

INC += \
	-I$(TOP)/hw/mcu/valentyusb/ \
	-I$(BUILDINC_DIRECTORY) \
	-I$(SOC_DIRECTORY)/software/include \
	-I$(SOC_DIRECTORY)/software/include/base \

VENDOR = #valentyusb
CHIP_FAMILY = valentyusb
