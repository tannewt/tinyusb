include ../../../tools/top.mk

# Select the board to build for.
ifeq ($(BOARD),)
  $(error You must provide a BOARD parameter)
else
  ifeq ($(wildcard $(TOP)/hw/bsp/$(BOARD)/.),)
    $(error Invalid BOARD specified)
  endif
endif

# If the build directory is not given, make it reflect the board name.
BUILD ?= build-$(BOARD)

CROSS_COMPILE = arm-none-eabi-

include $(TOP)/hw/bsp/$(BOARD)/board.mk

CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
MKDIR = mkdir
SED = sed
CP = cp
RM = rm

INC += -Isrc \
	-I$(TOP)/hw \
	-I$(TOP)/src

CFLAGS += \
	-fsingle-precision-constant \
	-fno-strict-aliasing \
	-Wdouble-promotion \
	-Wno-endif-labels \
	-Wstrict-prototypes \
	-Werror-implicit-function-declaration \
	-Wpointer-arith \
	-Wfloat-equal \
	-Wundef \
	-Wshadow \
	-Wwrite-strings \
	-Wsign-compare \
	-Wmissing-format-attribute \
	-Wno-deprecated-declarations \
	-Wpacked \
	-Wnested-externs \
	-Wunreachable-code \
	-Wcast-align \
	-Wno-error=lto-type-mismatch \
	-ffunction-sections \
	-fdata-sections

#Debugging/Optimization
ifeq ($(DEBUG), 1)
  CFLAGS += -O0 -ggdb
else
  CFLAGS += -flto -Os
endif

CFLAGS += $(INC) -Wall -Werror -std=gnu11 -nostdlib -DBOARD_$(shell echo $(BOARD) | tr a-z A-Z)

LDFLAGS = $(CFLAGS) -nostartfiles -fshort-enums -Wl,-nostdlib -Wl,-T,$(LD_FILE) -Wl,-Map=$@.map -Wl,-cref -Wl,-gc-sections -specs=nano.specs
LIBS := -lgcc -lc

# Use toolchain libm if we're not using our own.
ifndef INTERNAL_LIBM
LIBS += -lm
endif

EXAMPLE_SOURCE += \
	src/main.c \
	src/msc_app.c \
	src/msc_disc_ram.c

SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

OBJ = $(addprefix $(BUILD)/obj/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/obj/, $(SRC_S:.s=.o))

all: $(BUILD)/$(BOARD)-firmware.elf

OBJ_DIRS = $(sort $(dir $(OBJ)))
$(OBJ): | $(OBJ_DIRS)
$(OBJ_DIRS):
	$(MKDIR) -p $@

$(info $(OBJ))
$(BUILD)/$(BOARD)-firmware.elf: $(OBJ)
	@$(CC) -o $@ $(LDFLAGS) $^ -Wl,--start-group $(LIBS) -Wl,--end-group

$(BUILD)/$(BOARD)-firmware.bin: $(BUILD)/$(BOARD)-firmware.elf
	$(OBJCOPY) -O binary -j .vectors -j .text -j .data $^ $@

# We set vpath to point to the top of the tree so that the source files
# can be located. By following this scheme, it allows a single build rule
# to be used to compile all .c files.
vpath %.c . $(TOP)
$(BUILD)/obj/%.o: %.c
	@echo CC $@
	@$(CC) $(CFLAGS) -c -MD -o $@ $<
	@# The following fixes the dependency file.
	@# See http://make.paulandlesley.org/autodep.html for details.
	@# Regex adjusted from the above to play better with Windows paths, etc.
	@$(CP) $(@:.o=.d) $(@:.o=.P); \
	  $(SED) -e 's/#.*//' -e 's/^.*:  *//' -e 's/ *\\$$//' \
	      -e '/^$$/ d' -e 's/$$/ :/' < $(@:.o=.d) >> $(@:.o=.P); \
	  $(RM) $(@:.o=.d)

clean:
	rm -rf build-$(BOARD)
