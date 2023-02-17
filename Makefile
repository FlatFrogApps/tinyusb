# Base makefile for tinyusb

TINYUSB_DIR := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

TINYUSB_SRC =

ifeq ($(USE_TINYUSB_USB), Yes)
TINYUSB_SRC += $(TINYUSB_DIR)/src/tusb.c \
               $(TINYUSB_DIR)/src/device/usbd.c \
               $(TINYUSB_DIR)/src/device/usbd_control.c \
               $(TINYUSB_DIR)/src/common/tusb_fifo.c
endif

ifeq ($(USE_TINYUSB_DFU), Yes)
TINYUSB_SRC += $(TINYUSB_DIR)/src/class/dfu/dfu_device.c
endif

ifeq ($(USE_TINYUSB_DFU_RT), Yes)
TINYUSB_SRC += $(TINYUSB_DIR)/src/class/dfu/dfu_rt_device.c
endif

ifeq ($(USE_TINYUSB_VENDOR), Yes)
TINYUSB_SRC += $(TINYUSB_DIR)/src/class/vendor/vendor_device.c
endif

ifeq ($(USE_TINYUSB_HID), Yes)
TINYUSB_SRC += $(TINYUSB_DIR)/src/class/hid/hid_device.c
endif

ifeq ($(USE_TINYUSB_DRIVER), Yes)
TINYUSB_SRC += $(TINYUSB_DIR)/src/portable/nuvoton/m487/dcd_m487.c
endif

ifeq ($(USE_TINYUSB_HS_FS_DRIVER), Yes)
TINYUSB_SRC += $(TINYUSB_DIR)/src/portable/nuvoton/num487/dcd_num487.c \
               $(TINYUSB_DIR)/src/portable/nuvoton/num487/dcd_num487_fs.c \
               $(TINYUSB_DIR)/src/portable/nuvoton/num487/dcd_num487_hs.c
endif
