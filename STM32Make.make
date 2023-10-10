##########################################################################################################################
# File automatically-generated by STM32forVSCode
##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = fenice-bms-lv


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Core/Lib/bms_monitor/monitor_int.c \
Core/Lib/can-comm/can_comm.c \
Core/Lib/can-lib/lib/bms/bms_network.c \
Core/Lib/can-lib/lib/bms/bms_watchdog.c \
Core/Lib/can-lib/lib/inverters/inverters_network.c \
Core/Lib/can-lib/lib/inverters/inverters_watchdog.c \
Core/Lib/can-lib/lib/primary/primary_network.c \
Core/Lib/can-lib/lib/primary/primary_watchdog.c \
Core/Lib/can-lib/lib/secondary/secondary_network.c \
Core/Lib/can-lib/lib/secondary/secondary_watchdog.c \
Core/Lib/cli_bms_lv/cli_bms_lv.c \
Core/Lib/current_transducer/current_transducer.c \
Core/Lib/dac_pump/dac_pump.c \
Core/Lib/errors/error.c \
Core/Lib/errors/error_list_ref.c \
Core/Lib/health_signals/health_signals.c \
Core/Lib/inverters/inverters.c \
Core/Lib/lv_watchdog/can_watchdog.c \
Core/Lib/mcp23017/mcp23017.c \
Core/Lib/measurements/measurements.c \
Core/Lib/micro-libs/blink/blink.c \
Core/Lib/micro-libs/bms-monitor/ltc6811/ltc6811.c \
Core/Lib/micro-libs/bms-monitor/volt/volt.c \
Core/Lib/micro-libs/circ-buf/circ-buf.c \
Core/Lib/micro-libs/cli/cli.c \
Core/Lib/micro-libs/ctrl-nwk-utils/ctrl-nwk-utils.c \
Core/Lib/micro-libs/error-utils/error-utils.c \
Core/Lib/micro-libs/invlib/invlib.c \
Core/Lib/micro-libs/llist/llist.c \
Core/Lib/micro-libs/pid/pid.c \
Core/Lib/micro-libs/priority-queue/priority_queue_fast_insert.c \
Core/Lib/micro-libs/pwm/pwm.c \
Core/Lib/micro-libs/timer-utils/timer_utils.c \
Core/Lib/notes_buzzer/notes_buzzer.c \
Core/Lib/radiators/radiator.c \
Core/Lib/thermocouple/thermocouple.c \
Core/Src/adc.c \
Core/Src/can.c \
Core/Src/common.c \
Core/Src/current_sensor.c \
Core/Src/dac.c \
Core/Src/dma.c \
Core/Src/gpio.c \
Core/Src/i2c.c \
Core/Src/main.c \
Core/Src/spi.c \
Core/Src/stm32f4xx_hal_msp.c \
Core/Src/stm32f4xx_it.c \
Core/Src/system_stm32f4xx.c \
Core/Src/template.c \
Core/Src/tim.c \
Core/Src/usart.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c


CPP_SOURCES = \


# ASM sources
ASM_SOURCES =  \
startup_stm32f446xx.s



#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
POSTFIX = "
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
GCC_PATH="/usr/bin
ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++$(POSTFIX)
CC = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX)
AS = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX) -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy$(POSTFIX)
SZ = $(GCC_PATH)/$(PREFIX)size$(POSTFIX)
else
CXX = $(PREFIX)g++
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DERROR_LIB_IMPLEMENTATION \
-DLTC_COUNT=1 \
-DNOLOGGER \
-DSTM32F446xx \
-DUSE_HAL_DRIVER


# CXX defines
CXX_DEFS =  \
-DSTM32F446xx \
-DUSE_HAL_DRIVER


# AS includes
AS_INCLUDES = \

# C includes
C_INCLUDES =  \
-ICore/Inc \
-ICore/Lib/bms_monitor \
-ICore/Lib/can-comm \
-ICore/Lib/can-lib/lib/bms \
-ICore/Lib/can-lib/lib/inverters \
-ICore/Lib/can-lib/lib/primary \
-ICore/Lib/can-lib/lib/secondary \
-ICore/Lib/can-lib/proto/bms \
-ICore/Lib/can-lib/proto/inverters \
-ICore/Lib/can-lib/proto/primary \
-ICore/Lib/can-lib/proto/secondary \
-ICore/Lib/cli_bms_lv \
-ICore/Lib/current_transducer \
-ICore/Lib/dac_pump \
-ICore/Lib/errors \
-ICore/Lib/health_signals \
-ICore/Lib/inverters \
-ICore/Lib/lv_watchdog \
-ICore/Lib/mcp23017 \
-ICore/Lib/measurements \
-ICore/Lib/micro-libs/blink \
-ICore/Lib/micro-libs/bms-monitor \
-ICore/Lib/micro-libs/bms-monitor/ltc6811 \
-ICore/Lib/micro-libs/bms-monitor/volt \
-ICore/Lib/micro-libs/circ-buf \
-ICore/Lib/micro-libs/cli \
-ICore/Lib/micro-libs/ctrl-nwk-utils \
-ICore/Lib/micro-libs/error-utils \
-ICore/Lib/micro-libs/invlib \
-ICore/Lib/micro-libs/llist \
-ICore/Lib/micro-libs/pid \
-ICore/Lib/micro-libs/priority-queue \
-ICore/Lib/micro-libs/pwm \
-ICore/Lib/micro-libs/timer-utils \
-ICore/Lib/notes_buzzer \
-ICore/Lib/radiators \
-ICore/Lib/thermocouple \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CXXFLAGS = $(MCU) $(CXX_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -feliminate-unused-debug-types

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf -ggdb
CXXFLAGS += -g -gdwarf -ggdb
endif

# Add additional flags
CFLAGS += 
ASFLAGS += -Wall -fdata-sections -ffunction-sections 
CXXFLAGS += 

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F446RETx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = \


# Additional LD Flags from config file
ADDITIONALLDFLAGS = -specs=nano.specs -u_printf_float 

LDFLAGS = $(MCU) $(ADDITIONALLDFLAGS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of cpp program objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of C objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
# list of ASM program objects
UPPER_CASE_ASM_SOURCES = $(filter %.S,$(ASM_SOURCES))
LOWER_CASE_ASM_SOURCES = $(filter %.s,$(ASM_SOURCES))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(UPPER_CASE_ASM_SOURCES:.S=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(LOWER_CASE_ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cxx STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cxx=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c STM32Make.make | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) STM32Make.make
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# flash
#######################################
flash: $(BUILD_DIR)/$(TARGET).elf
	"/usr/bin/openocd" -f ./openocd.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

#######################################
# erase
#######################################
erase: $(BUILD_DIR)/$(TARGET).elf
	"/usr/bin/openocd" -f ./openocd.cfg -c "init; reset halt; stm32f4x mass_erase 0; exit"

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# custom makefile rules
#######################################


	
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***