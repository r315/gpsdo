######################################
# target
######################################
TARGET ?=gpsdo
BOARD  ?=BOARD_DK415
#######################################
# Features
#######################################
FEATURE_ENABLE +=ENABLE_I2C
FEATURE_ENABLE +=ENABLE_DAC
FEATURE_ENABLE +=ENABLE_SPI
FEATURE_ENABLE +=ENABLE_ADC
FEATURE_ENABLE +=ENABLE_CLI ENABLE_CLI_COLOR
#FEATURE_ENABLE +=ENABLE_GPS
FEATURE_ENABLE +=ENABLE_PPS
FEATURE_ENABLE +=ENABLE_IOEXPANDER
FEATURE_ENABLE +=ENABLE_COUNT ENABLE_COUNT_RECIPROCAL
#FEATURE_ENABLE +=ENABLE_BUZZER
FEATURE_ENABLE +=ENABLE_DEBUG
FEATURE_ENABLE +=ENABLE_DEBUG_BOARD
FEATURE_ENABLE +=ENABLE_DEBUG_APP
#######################################
# paths
#######################################

PROJECT_DIR ?=.
TARGET_PATH :=$(CURDIR)
BUILD_DIR   :=$(PROJECT_DIR)/build/$(TARGET)
#APP_PATH    :=$(PROJECT_DIR)/app
#LIBEMB_PATH :=$(PROJECT_DIR)/lib/libemb
LIB_PATH        =$(LIBEMB_PATH)/lib
DRIVER_PATH 	=$(LIBEMB_PATH)/drv
DRIVER_COMPONENT_PATH =$(LIBEMB_PATH)/component
CMSIS_PATH 		=$(LIBEMB_PATH)/CMSIS

MIDDLEWARE_PATH =../../Middlewares

#######################################
# Includes
#######################################
C_INCLUDES = \
$(LIBEMB_PATH)/lib/inc \
$(LIBEMB_PATH)/drv/inc \
$(LIBEMB_PATH)/CMSIS/Core/Include \
$(LIBEMB_PATH)/CMSIS/Device/ARTERY/AT32F4xx/inc \
$(LIBEMB_PATH)/component/display \
$(TARGET_PATH)/inc \
$(DRIVER_SOC_PATH)/inc \
$(DRIVER_COMPONENT_PATH) \
$(LIB_MULTIPROTOCOL_PATH) \
$(APP_PATH) \
$(MIDDLEWARE_PATH)/cli-simple \

AS_INCLUDES =\

######################################
# Sources
######################################
TARGET_SOURCES = \
$(CMSIS_PATH)/Device/ARTERY/AT32F4xx/src/startup_at32f415cbt7.c\
$(CMSIS_PATH)/Device/ARTERY/AT32F4xx/src/system_at32f4xx.c\
$(TARGET_PATH)/src/board_dk415.c \

DRV_SOURCES = \
$(DRIVER_PATH)/clock/clock_at32f4xx.c \
$(DRIVER_PATH)/uart/uart_at32f4xx.c \
$(DRIVER_PATH)/gpio/gpio_at32f4xx.c \
$(DRIVER_PATH)/wdt/wdt_at32f4xx.c \
$(DRIVER_PATH)/dma/dma_at32f4xx.c \
$(DRIVER_PATH)/flash/fls_at32f4xx.c \

LIB_SOURCES = \
$(APP_PATH)/syscalls.c \
$(LIBEMB_PATH)/lib/src/debug.c \
$(LIBEMB_PATH)/lib/src/fifo.c \
$(LIBEMB_PATH)/lib/src/strfunc.c \
$(LIBEMB_PATH)/lib/src/nvdata.c \
$(LIBEMB_PATH)/lib/src/stimer.c \

APP_SOURCES = \
$(APP_PATH)/app.c \

ifneq ($(filter ENABLE_I2C, $(FEATURE_ENABLE)),)
DRV_SOURCES +=$(DRIVER_PATH)/i2c/i2c_at32f4xx.c
endif

ifneq ($(filter ENABLE_SPI, $(FEATURE_ENABLE)),)
DRV_SOURCES +=$(DRIVER_PATH)/spi/spi_at32f4xx.c
endif

ifneq ($(filter ENABLE_CLI, $(FEATURE_ENABLE)),)
LIB_SOURCES +=$(MIDDLEWARE_PATH)/cli-simple/cli_simple.c
endif

ifneq ($(filter ENABLE_GPS, $(FEATURE_ENABLE)),)
APP_SOURCES +=$(APP_PATH)/gps.c
endif

ifneq ($(filter ENABLE_IOEXPANDER, $(FEATURE_ENABLE)),)
DRV_SOURCES +=$(DRIVER_COMPONENT_PATH)/io_expander/pcf8574.c
endif


C_SOURCES = \
$(TARGET_SOURCES) \
$(DRV_SOURCES) \
$(LIB_SOURCES) \
$(APP_SOURCES) \

CPP_SOURCES = \

LDSCRIPT :=AT32F415xB_FLASH.ld
#######################################
# Misc
#######################################

C_DEFS +=\
$(BOARD) \
$(DEVICE) \
$(FEATURE_ENABLE) \
UART_RX_MODE=UART_MODE_FIFO \
UART_TX_MODE=UART_MODE_FIFO \
FIFO_SIZE=128 \
FIFO_INTERRUPT_SAFE \
CONSOLE_WIDTH=128 \
CONSOLE_SIMPLE_COMMAND \
HSE_VALUE=12000000UL \
$(VERSION) \

#######################################
# CFLAGS
#######################################

DEVICE =AT32F415CBT7
CORE =cortex-m4
MCU =-mcpu=$(CORE) -mthumb

ifdef RELEASE
GIT_TAG :=$(shell git describe --abbrev=0 --tags 2>/dev/null || true)
VERSION :=VERSION_STRING=\"$(GIT_TAG)\"
OPT   =-O2
else
OPT   =-O0 -g -gdwarf-2 -Wall -Wextra
endif

ifndef V
VERBOSE =@
endif

ASFLAGS  =$(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -fdata-sections -ffunction-sections
CFLAGS   =$(MCU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -fdata-sections -ffunction-sections -fno-builtin -std=gnu11
CPPFLAGS =$(MCU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -fdata-sections -ffunction-sections -fno-unwind-tables -fno-exceptions -fno-rtti -fno-builtin
LDFLAGS  =$(MCU) $(SPECS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,-gc-sections,-cref

#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))
#######################################
# Libraries
#######################################

LIBDIR =
ifdef SEMIHOSTING
SPECS =--specs=rdimon.specs
LDLIBS =-nostartfiles -lc -lrdimon #-nostdlib
else
SPECS=--specs=nano.specs #--specs=nosys.specs
LIBS =
endif

#######################################
# Objects
#######################################

OBJECTS = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/, $(notdir $(CPP_SOURCES:.cpp=.obj)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

#OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
#vpath %.s $(sort $(dir $(ASM_SOURCES)))

#######################################
# Tool binaries
#######################################
PREFIX = arm-none-eabi-
CC  =$(PREFIX)gcc
CPP =$(PREFIX)g++
AS  =$(PREFIX)gcc -x assembler-with-cpp
LD  =$(PREFIX)gcc
AR  =$(PREFIX)ar
SZ  =$(PREFIX)size
OBJCOPY =$(PREFIX)objcopy
OBJDUMP =$(PREFIX)objdump
HEX = $(OBJCOPY) -O ihex
BIN = $(OBJCOPY) -O binary -S

ifeq ($(shell uname -s), Linux)
JLK =JLinkExe
else
JLK =JLink
endif

#######################################
# Rules
#######################################
all: $(BUILD_DIR)/$(TARGET).elf size
#@echo $(CMSIS_PATH)

bin: $(BUILD_DIR)/$(TARGET).bin

program-jlink: $(BUILD_DIR)/$(TARGET).jlink
	$(VERBOSE)$(JLK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $<

$(BUILD_DIR)/$(TARGET).jlink: $(BUILD_DIR)/$(TARGET).bin
	@echo "Creating Jlink configuration file"
	@echo "loadfile $< 0x08000000" > $@
	@echo "r" >> $@
	@echo "q" >> $@

program-stlink: $(BUILD_DIR)/$(TARGET).cfg $(BUILD_DIR)/$(TARGET).elf
	$(VERBOSE)openocd -f $(BUILD_DIR)/$(TARGET).cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

$(BUILD_DIR)/$(TARGET).cfg:
	@echo "Creating openocd configuration file"
	@echo "source [find interface/stlink.cfg]" > $@
	@echo "transport select hla_swd" >> $@
	@echo "source [find target/gd32e23x.cfg]" >> $@
	@echo "adapter speed 4000" >> $@
#ERASE_CMD =openocd -f $(PRG_CFG) -c "init" -c "halt" -c "stm32f1x unlock 0" -c "stm32f1x mass_erase 0" -c "exit"

test:
#@echo $(VERSION)
#@echo ""; $(foreach d, $(OBJECTS), echo $(d);)
#@echo ""; $(foreach d, $(FEATURE_ENABLE), echo $(d);)
#@echo $(sort $(C_INCLUDES))
#@echo $(FEATURE_ENABLE)

#######################################
# build rules
#######################################
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo "[CC]  $<"
	$(VERBOSE)$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.obj: %.cpp Makefile | $(BUILD_DIR)
	@echo "[CP]  $<"
	$(VERBOSE)$(CPP) -c $(CPPFLAGS)  $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "[AS]  $<"
	$(VERBOSE)$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) | Makefile
	@echo "[LD]  $@"
	$(VERBOSE)$(LD) $(LIBDIR) -T$(LDSCRIPT) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(VERBOSE)$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(VERBOSE)$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

size: $(BUILD_DIR)/$(TARGET).elf
	@echo "--- Size ---"
	$(VERBOSE)$(SZ) -A -x $<
	$(VERBOSE)$(SZ) -B $<

#######################################
# clean up
#######################################
clean:
	$(VERBOSE)-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
#-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)
ifeq ($(GCC_COLORS), )
export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'
#unexport GCC_COLORS
endif

# *** EOF ***
