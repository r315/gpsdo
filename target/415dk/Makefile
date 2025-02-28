######################################
# target
######################################
TARGET =gpsdo

#######################################
# Features
#######################################

FEATURE_TFT_DISPLAY =yes
FEATURE_CLI			=yes

#######################################
# paths
#######################################

BUILD_DIR 		:=build
TARGET_PATH 	=$(CURDIR)

APP_PATH        =../../app
MIDDLEWARE_PATH =../../Middlewares
LIB_PATH        =$(LIBEMB_PATH)/lib
DRIVERS_PATH 	=$(LIBEMB_PATH)/drv
CMSIS_PATH 		=$(LIBEMB_PATH)/CMSIS
DRIVERS_CORE    =$(CMSIS_PATH)/Device/ARTERY/AT32F4xx
DRIVERS_SOC 	=$(DRIVERS_PATH)/SOC/at32f4xx
COMPONENT_PATH 	=$(LIBEMB_PATH)/component
DRIVERS_APP 	=$(APP_PATH)
#######################################
# Includes
#######################################
C_INCLUDES = \
$(CMSIS_PATH)/Core/Include \
$(CMSIS_PATH)/Device/ARTERY/AT32F4xx/inc \
$(DRIVERS_CORE)/inc \
$(DRIVERS_SOC)/inc \
$(DRIVERS_PATH)/inc \
$(TARGET_PATH)/inc \
$(LIB_PATH)/inc \
$(MIDDLEWARE_PATH)/cli-simple \
$(APP_PATH) \
$(COMPONENT_PATH)/ds1086 \

AS_INCLUDES =\

######################################
# Sources
######################################
DRIVER_SOURCES = \
$(DRIVERS_CORE)/src/startup_at32f415cbt7.c\
$(DRIVERS_CORE)/src/system_at32f4xx.c\
$(DRIVERS_SOC)/src/at32f4xx_rcc.c\
$(DRIVERS_SOC)/src/at32f4xx_i2c.c\
$(DRIVERS_SOC)/src/at32f4xx_i2c_ex.c\
$(DRIVERS_PATH)/gpio/gpio_at32f4xx.c \
$(DRIVERS_PATH)/uart/uart_at32f4xx.c \
$(DRIVERS_PATH)/i2c/i2c_at32f4xx.c \

COMPONENT_SOURCES = \
$(COMPONENT_PATH)/ds1086/ds1086.c \

C_SOURCES = \
$(DRIVER_SOURCES) \
$(COMPONENT_SOURCES) \
$(LIB_PATH)/src/fifo.c \
$(TARGET_PATH)/src/board_415dk.c \
$(APP_PATH)/main.c \
$(APP_PATH)/gpsdo.c \
$(MIDDLEWARE_PATH)/cli-simple/cli_simple.c \
$(MIDDLEWARE_PATH)/cli-simple/syscalls.c \

CPP_SOURCES = \

LDSCRIPT :=$(TARGET_PATH)/AT32F415xB_FLASH.ld
#######################################
# Misc
#######################################

OCD_CONFIG =$(TARGET_PATH)/at32f415.cfg

# C defines
C_DEFS +=\
$(DEVICE) \
BOARD_415DK \
USE_STDPERIPH_DRIVER \

#######################################
# CFLAGS
#######################################

DEVICE =AT32F415CBT7
CPU =-mcpu=cortex-m4 -mthumb

# fpu
#at43f415 does not have FPU
#FPU =-mfloat-abi=hard -mfpu=fpv4-sp-d16
#FPU =-mfloat-abi=soft

# float-abi
FLOAT_ABI =#-u_printf_float

# mcu
MCU = $(CPU) $(FPU) $(FLOAT_ABI)

ifdef RELEASE
OPT   =-O2
else
OPT   =-Og -g -gdwarf-2 -Wall #-Wextra
endif

ifndef V
VERBOSE =@
else
VERBOSE =
endif

# Generate dependency information
#CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst))

ASFLAGS  =$(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS   =$(MCU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -std=gnu11 -fdata-sections -ffunction-sections #-fstack-usage
CPPFLAGS =$(CPU) $(OPT) $(addprefix -D, $(C_DEFS)) $(addprefix -I, $(C_INCLUDES)) -fdata-sections -ffunction-sections -fno-unwind-tables -fno-exceptions -fno-rtti
LDFLAGS  =$(MCU) $(SPECS) -Wl,--gc-sections,-Map=$(BUILD_DIR)/$(TARGET).map,--cref

#######################################
# Libraries
#######################################

LIBDIR =
ifdef SEMIHOSTING
SPECS =--specs=rdimon.specs
LDLIBS =-nostartfiles -lc -lrdimon
else
SPECS=--specs=nosys.specs --specs=nano.specs
#-nostdlib -lstd++ -lnosys -lm
LIBS =-lstdc++
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

#VPATH +=$(sort $(dir $(C_SOURCES)))

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
JLK ="/opt/SEGGER/JLink/JLinkExe"
PRG_CFG = $(TARGET).elf
PRG_CMD = openocd -f at32f415.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"
else
JLK =jlink
PRG_CFG = $(TARGET).jlink
PRG_CMD = $(VERBOSE)$(JLK) -device $(DEVICE) -if SWD -speed auto -CommanderScript $(BUILD_DIR)/$(PRG_CFG)
#-device CORTEX-M4
endif

#######################################
# Rules
#######################################
all: $(BUILD_DIR) $(BUILD_DIR)/$(TARGET).elf size
#@echo $(CMSIS_PATH)

bin: $(BUILD_DIR)/$(TARGET).bin

program: all $(BUILD_DIR)/$(PRG_CFG)
	$(PRG_CMD)

$(BUILD_DIR)/$(TARGET).jlink: $(BUILD_DIR)/$(TARGET).bin
	@echo "Creating Jlink configuration file"
	@echo "loadfile $< 0x08000000" > $@
	@echo "r" >> $@
	@echo "q" >> $@
#######################################
# build rules
#######################################
$(BUILD_DIR)/%.d: %.c
	$(VERBOSE)$(CC) $(CFLAGS) -MF"$@" -MG -MM -MP -MT"$@" -MT"$(<:.c=.o)" "$<"

$(BUILD_DIR)/%.o: %.c
	@echo "[CC]  $<"
	$(VERBOSE)$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.obj: %.cpp
	@echo "[CP]  $<"
	$(VERBOSE)$(CPP) -c $(CPPFLAGS)  $< -o $@

$(BUILD_DIR)/%.o: %.s
	@echo "[AS]  $<"
	$(VERBOSE)$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS)
	@echo "[LD]  $@"
	$(VERBOSE)$(LD) $(LIBDIR) -T$(LDSCRIPT) $(LDFLAGS) $^ $(LIBS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(VERBOSE)$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
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
