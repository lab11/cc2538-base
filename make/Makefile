################################################################################
## Makefile for compiling apps against the nRF51 SDK
################################################################################

################################################################################
## Usage
##
## In your application Makefile, set the following variables:
##   - APPLICATION_SRCS   : List of all .c files to be compiled.
##   - LIBRARY_PATHS      : List of directories with .h files
##   - SOURCE_PATHS       : List of directories with .c files
##   - SOFTDEVICE_MODEL   : s110 | s120 | s130 | s210 | s310 | or do not set for no softdevice
##
## Optional
##   - SOFTDEVICE_VERSION : Full version number of the softdevice.
##   - SOFTDEVICE         : Path to the softdevice to use
##   - START_CODE         : .s file to execute first
##   - SYSTEM_FILE        : Base nRF .c file.
##   - NRF_MODEL          : nrf51 | nrf52  : Set by the softdevice used
##   - BOARD              : Hardware board being used. Will get set as #define.
##   - RAM_KB             : Size of RAM on chip    : Defaults to 16
##   - FLASH_KB           : Size of flash on chip  : Defaults to 256
##   - SDK_VERSION        : Major version number of the SDK to use. Defaults to 10
##   - GDB_PORT_NUMBER    : Defaults to 2331
##
##
## Then at the end of the Makefile do something like:
##    NRF_BASE_PATH ?= ../../nrf5x-base
##    include $(NRF_BASE_PATH)/make/Makefile

# Helper for relative paths
BASE_DIR := $(dir $(lastword $(MAKEFILE_LIST)))..

# Setup the compilers to use
TOOLCHAIN_PREFIX ?= arm-none-eabi
AS      = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-as
CC      = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-gcc
LD      = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-gcc
OBJCOPY = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-objcopy
OBJDUMP = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-objdump
SIZE    = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-size
GDB     = $(TOOLCHAIN_PATH)$(TOOLCHAIN_PREFIX)-gdb

# Set default board
BOARD ?= BOARD_NULL

# Set hardware memory sizes
RAM_KB   ?= 16
FLASH_KB ?= 256

# Set this for using GDB
GDB_PORT_NUMBER ?= 2331

# Required files to compile
STARTUP_FILE ?= startup_gcc.c

# Set the path to the softdevice
SOFTDEVICE ?= $(BASE_DIR)/softdevice/$(SOFTDEVICE_MODEL)_$(NRF_MODEL)_$(SOFTDEVICE_VERSION)_softdevice.hex

# Need some common options for interacting with the chip over JTAG
JLINK_OPTIONS = -device $(NRF_IC) -if swd -speed 1000

# If not set in app makefile, lets optimize for size
CFLAGS ?= -Os

# Add useful paths from nRF5x-base
SOURCE_PATHS += $(BASE_DIR)/startup/
SOURCE_PATHS += $(BASE_DIR)/bsp/

LIBRARY_PATHS += $(BASE_DIR)/bsp/

FF_PATH ?= $(BASE_DIR)/foundation-firmware/1.0.1.0

LIBRARY_PATHS += $(FF_PATH)/driverlib/cc2538/source
LIBRARY_PATHS += $(FF_PATH)/driverlib/cc2538/inc
LIBRARY_PATHS += $(FF_PATH)/usblib/cc2538/source/class_hid
# LIBRARY_PATHS += $(FF_PATH)/usblib/cc2538/source/class_cdc
LIBRARY_PATHS += $(FF_PATH)/usblib/cc2538/source/library
LIBRARY_PATHS += $(FF_PATH)/usblib/cc2538/source/library/cc2538

SOURCE_PATHS += $(FF_PATH)/driverlib/cc2538/source
SOURCE_PATHS += $(FF_PATH)/usblib/cc2538/source/class_hid
SOURCE_PATHS += $(FF_PATH)/usblib/cc2538/source/class_cdc
SOURCE_PATHS += $(FF_PATH)/usblib/cc2538/source/library
SOURCE_PATHS += $(FF_PATH)/usblib/cc2538/source/library/cc2538


# # Location for BLE Address if stored in Flash
# ifeq ($(FLASH_KB), 256)
# 	# 256 KB Flash
# 	CFLAGS += -DBLEADDR_FLASH_LOCATION=0x0003FFF8
# 	BLEADDR_FLASH_LOCATION ?= 0x0003FFF8
# else
# 	# Assume 128 KB Flash
# 	CFLAGS += -DBLEADDR_FLASH_LOCATION=0x0001FFF8
# 	BLEADDR_FLASH_LOCATION ?= 0x0001FFF8
# endif

print-% : ; @echo $* = $($*)

LINKER_SCRIPT ?= $(BASE_DIR)/make/cc2538.ld
OUTPUT_NAME ?= $(addsuffix _$(SOFTDEVICE_MODEL), $(PROJECT_NAME))

LIBRARY_INCLUDES = $(addprefix -I,$(LIBRARY_PATHS))
# CMSIS_INCLUDE = $(addprefix -I,$(CMSIS_INCLUDE_PATH))

VPATH = $(SOURCE_PATHS)

OUTPUT_PATH = _build/

CPUFLAGS = -mcpu=cortex-m3 -mthumb -mlittle-endian
CFLAGS += -std=gnu99 -c $(CPUFLAGS) -Wall -D$(BOARD)
CFLAGS += -s -ffunction-sections -fdata-sections
# CFLAGS += $(CMSIS_INCLUDE) $(LIBRARY_INCLUDES) -MD
CFLAGS += $(LIBRARY_INCLUDES) -MD
LDFLAGS += $(CPUFLAGS) -L $(BASE_DIR)/make -T $(LINKER_SCRIPT) -Wl,--gc-sections -Wl,-Map=$(OUTPUT_PATH)$(OUTPUT_NAME).Map
LDFLAGS += -nostartfiles

HEX = $(OUTPUT_PATH)$(OUTPUT_NAME).hex
ELF = $(OUTPUT_PATH)$(OUTPUT_NAME).elf
BIN = $(OUTPUT_PATH)$(OUTPUT_NAME).bin

SRCS = $(STARTUP_FILE) $(notdir $(APPLICATION_SRCS))
OBJS = $(addprefix $(OUTPUT_PATH), $(SRCS:.c=.o)) $(addprefix $(OUTPUT_PATH),$(APPLICATION_LIBS))
DEPS = $(addprefix $(OUTPUT_PATH), $(SRCS:.c=.d))
# SRCS_AS = $(START_CODE)
# OBJS_AS = $(addprefix $(OUTPUT_PATH), $(SRCS_AS:.s=.os))

MAKE_BUILD_FOLDER = mkdir -p $(OUTPUT_PATH)

### Verbosity control. Use  make V=1  to get verbose builds.

ifeq ($(V),1)
  PRINT_CC  =
  PRINT_LD  =
  PRINT_AR  =
  PRINT_AS  =
  PRINT_HEX =
  PRINT_BIN =
  PRINT_SIZ =
  Q=
else
  PRINT_CC  = @echo "  CC       " $<
  PRINT_LD  = @echo "  LD       " $(ELF)
  PRINT_AR  = @echo "  AR       " $@
  PRINT_AS  = @echo "  AS       " $<
  PRINT_HEX = @echo "  HEX      " $(HEX)
  PRINT_BIN = @echo "  BIN      " $(BIN)
  PRINT_SIZ = @echo "  SIZE     " $(ELF)
  Q=@
endif


all: $(OBJS) $(OBJS_AS) $(HEX)

debug: CFLAGS += -g
debug: $(OBJS) $(OBJS_AS) $(HEX)


rebuild: clean all

BSL = $(BASE_DIR)/tools/cc2538-bsl.py

ifdef ID
  BSL_FLAGS += -i $(ID)
endif

ifdef PORT
  BSL_FLAGS += -p $(PORT)
endif

install: all
	python2 $(BSL) -b 500000 -e -w -v $(BSL_FLAGS) $(BIN)


$(HEX): $(OBJS) $(OBJS_AS)
	$(PRINT_LD)
	$(Q)$(LD) $(LDFLAGS) $(OBJS_AS) $(OBJS) -o $(ELF)
	$(PRINT_HEX)
	$(Q)$(OBJCOPY) -Oihex $(ELF) $(HEX)
	$(PRINT_BIN)
	$(Q)$(OBJCOPY) -Obinary $(ELF) $(BIN)
	$(PRINT_SIZ)
	$(Q)$(SIZE) $(ELF)

size: $(ELF)
	$(SIZE) $(ELF)

$(OUTPUT_PATH)%.o: %.c
	@$(MAKE_BUILD_FOLDER)
	$(PRINT_CC)
	$(Q)$(CC) $(CFLAGS) $< -o $@

$(OUTPUT_PATH)%.os: %.s
	@$(MAKE_BUILD_FOLDER)
	$(PRINT_AS)
	$(Q)$(AS) $< -o $@

-include $(DEPS)

.PHONY: all clean rebuild size
