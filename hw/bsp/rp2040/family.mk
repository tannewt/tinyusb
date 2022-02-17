JLINK_DEVICE = rp2040_m0_0
PYOCD_TARGET = rp2040

ifeq ($(DEBUG), 1)
CMAKE_DEFSYM += -DCMAKE_BUILD_TYPE=Debug
endif

# Default to native port. Ports >1 use PIO.
PORT ?= 0

ifeq ($(PORT), 0)
  $(info "Native USB port")
else
  $(info "PIO USB port")
endif

$(BUILD):
	cmake -S . -B $(BUILD) -DBOARD_DEVICE_RHPORT_NUM=$(PORT) -DFAMILY=$(FAMILY) -DBOARD=$(BOARD) -DPICO_BUILD_DOCS=0 $(CMAKE_DEFSYM)

all: $(BUILD)
	$(MAKE) -C $(BUILD)

clean:
	$(RM) -rf $(BUILD)

flash: flash-pyocd
flash-uf2:
	@$(CP) $(BUILD)/$(PROJECT).uf2 /media/$(USER)/RPI-RP2
