include ../../../tools/top.mk
include ../../make.mk

INC += \
	src \
	$(TOP)/src/class/dfu \
	$(TOP)/hw \

# CFLAGS += -DNDEBUG

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

include ../../rules.mk
