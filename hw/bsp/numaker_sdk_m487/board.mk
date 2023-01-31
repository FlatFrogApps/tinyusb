


CFLAGS += \
  -fshort-enums \
  -flto \
  -mthumb \
  -mabi=aapcs-linux \
  -mcpu=cortex-m4 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -DCFG_TUSB_MCU=OPT_MCU_NUM487

CFLAGS += -Wno-error=undef -Wno-error=redundant-decls

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/gcc_arm.ld

SRC_C += \
  hw/mcu/ff_nuvoton/m480/Device/Nuvoton/M480/Source/system_M480.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/clk.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/gpio.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/i2c.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/i2s.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/rtc.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/spi.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/spim.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/sys.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/timer.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/uart.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/wdt.c \
  hw/mcu/ff_nuvoton/m480/StdDriver/src/wwdt.c \
  src/portable/nuvoton/num487/dcd_num487.c \
  src/portable/nuvoton/num487/dcd_num487_fs.c \
  src/portable/nuvoton/num487/dcd_num487_hs.c \

SRC_S += \
  hw/mcu/ff_nuvoton/m480/Device/Nuvoton/M480/Source/GCC/startup_M480.S

INC += \
  $(TOP)/hw/mcu/ff_nuvoton/m480/Device/Nuvoton/M480/Include \
  $(TOP)/hw/mcu/ff_nuvoton/m480/StdDriver/inc \
  $(TOP)/hw/mcu/ff_nuvoton/m480/CMSIS/Include \
  $(TOP)/hw/bsp/numaker_sdk_m487

# For freeRTOS port source
FREERTOS_PORT = ARM_CM4F

# For flash-jlink target
JLINK_DEVICE = M487JIDAE 

OPENOCD_PATH=~/local/OpenOCD

OPENOCD=$(OPENOCD_PATH)/bin/openocd
OPENOCD_FLAGS  = -f $(OPENOCD_PATH)/scripts/interface/nulink.cfg
OPENOCD_FLAGS += -f $(OPENOCD_PATH)/scripts/target/numicroM4.cfg

# Note
# To be able to program the SPI flash, it need to boot with ICP mode "1011". 
# However, in ICP mode, opencod cannot establish connection to the mcu. 
# Therefore, there is no easy command line flash for NUC505
# It is probably better to just use Nuvoton NuMicro ICP programming on windows to program the board
# - 1111 "SPI" (run from internal flash)
# - 1110 "USB" (mass storage emulator that accepts a .bin file)
# - 0111 "ICE-SPI" (allow external debugger access, but may not be programmable)
# - 1011 ICP mode (programmable via NuMicro ICP programming tool)

# Flash using Nuvoton's openocd fork at https://github.com/OpenNuvoton/OpenOCD-Nuvoton
# Please compile and install it from github source
flash: $(BUILD)/$(PROJECT).elf
	$(OPENOCD) $(OPENOCD_FLAGS) -c "program $< verify reset exit"
