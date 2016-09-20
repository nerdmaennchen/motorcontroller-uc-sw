
TARGET = bldc

# compiler
CROSS_COMPILE_PREFIX = arm-none-eabi-
CC      = $(CROSS_COMPILE_PREFIX)gcc
CPP     = $(CROSS_COMPILE_PREFIX)g++
LD      = $(CROSS_COMPILE_PREFIX)g++
SIZE    = $(CROSS_COMPILE_PREFIX)size
GDB     = $(CROSS_COMPILE_PREFIX)gdb
OBJ_CPY = $(CROSS_COMPILE_PREFIX)objcopy

LIBS = m nosys gcc
LIB_PATHS =
INCLUDES = src/ \
	src/target/include \
	src/framework/include

LINKER_SCRIPT_FILE = src/target/stm32f4/stm32.ld
MAP_FILE = $(TARGET).map


INCLUDE_CMD = $(addprefix -I, $(INCLUDES))
LIB_CMD = $(addprefix -l, $(LIBS))
LIB_PATH_CMD = $(addprefix -L, $(LIB_PATHS))

# Flags

DEFINES		+= -DSTM32F4

FP_FLAGS       ?=  -mhard-float -mfpu=fpv4-sp-d16
COMMON_FLAGS	+= $(DEFINES) $(FP_FLAGS)
COMMON_FLAGS	+= -mthumb -mcpu=cortex-m4
COMMON_FLAGS	+= -O0 -g3
COMMON_FLAGS	+= $(INCLUDE_CMD)

###############################################################################
# C flags

CFLAGS		+= $(COMMON_FLAGS)
CFLAGS		+= -Wextra -Wshadow -Wimplicit-function-declaration
CFLAGS		+= -Wredundant-decls -Wmissing-prototypes -Wstrict-prototypes
CFLAGS		+= -fno-common -ffunction-sections -fdata-sections

###############################################################################
# C++ flags

CPPFLAGS	+= $(COMMON_FLAGS)
CPPFLAGS	+= -Wextra -Wshadow -Wredundant-decls
CPPFLAGS	+= -fno-common -ffunction-sections -fdata-sections
CPPFLAGS	+= -std=c++11 -fvisibility=hidden -frepo -fno-rtti
CPPFLAGS	+= -MD
CPPFLAGS	+= -Wall -Wundef
CPPFLAGS	+= -I$(INCLUDE_DIR)

###############################################################################
# Linker flags

LINKERFLAGS += --static -nostartfiles -mthumb  $(FP_FLAGS)
LINKERFLAGS += -Wl,--gc-sections
LINKERFLAGS += -Wl,-Map=$(TARGET).map
LINKERFLAGS += -Wl,--start-group
LINKERFLAGS += -T$(LINKER_SCRIPT_FILE)
LINKERFLAGS += -mthumb -mcpu=cortex-m4 -fvisibility=hidden 
LINKERFLAGS +=  -fvisibility=hidden -frepo -fno-rtti

CPP_SUFFIX = .cpp
C_SUFFIX   = .c
OBJ_SUFFIX = .o
DEP_SUFFIX = .d
OBJ_DIR    = obj/

# files that should be listed first when linking
IMPORTANT_ORDER_FILES = vectorISR.c
IGNORE_STRINGS = */archive/*


IGNORE_STRINGS       += $(IMPORTANT_ORDER_FILES)
CPP_FILES            += $(sort $(filter-out $(IGNORE_STRINGS), $(shell find src -name "*$(CPP_SUFFIX)" | grep -v $(addprefix -e, $(IGNORE_STRINGS)))))
IMPORTANT_CPP_FILES  += $(sort $(filter-out $(IGNORE_STRINGS), $(shell find src -name "*$(CPP_SUFFIX)" | grep $(addprefix -e, $(IGNORE_STRINGS)))))
ALL_CPP_FILES         = $(CPP_FILES) $(IMPORTANT_CPP_FILES)

C_FILES              += $(sort $(filter-out $(IGNORE_STRINGS), $(shell find src -name "*$(C_SUFFIX)" | grep -v $(addprefix -e, $(IGNORE_STRINGS)))))
IMPORTANT_C_FILES    += $(sort $(filter-out $(IGNORE_STRINGS), $(shell find src -name "*$(C_SUFFIX)" | grep $(addprefix -e, $(IGNORE_STRINGS)))))
ALL_C_FILES           =  $(C_FILES) $(IMPORTANT_C_FILES)

CPP_OBJ_FILES        += $(addsuffix $(OBJ_SUFFIX), $(addprefix $(OBJ_DIR), $(CPP_FILES) $(IMPORTANT_CPP_FILES)))
C_OBJ_FILES          += $(addsuffix $(OBJ_SUFFIX), $(addprefix $(OBJ_DIR), $(C_FILES) $(IMPORTANT_C_FILES)))


DEP_FILES            += $(addprefix $(OBJ_DIR), $(addsuffix $(DEP_SUFFIX), $(ALL_CPP_FILES) $(ALL_C_FILES)))

ifndef VERBOSE
SILENT = @
endif


.phony: all clean flash

all: $(TARGET).elf

clean:
	$(SILENT) rm -rf $(OBJ_DIR) $(TARGET).elf $(TARGET).map $(TARGET).bin

dbg:
	@echo $(CPP_OBJ_FILES)
	@echo $(C_OBJ_FILES)


$(TARGET).elf: $(CPP_OBJ_FILES) $(C_OBJ_FILES)
	$(SILENT) echo linking $(target)
	$(SILENT) $(LD) -o $@ $^ $(LINKERFLAGS) $(LIB_PATH_CMD) $(LIB_CMD)
	$(SIZE) $@
	@ echo done

$(OBJ_DIR)%$(C_SUFFIX)$(OBJ_SUFFIX): %$(C_SUFFIX) $(LINKER_SCRIPT_FILE)
	@echo building $<
	@ mkdir -p $(dir $@)
	@ $(CC) $(CFLAGS) $(INCLUDE_CMD) -MM -MF $(OBJ_DIR)$<.d -c $<
	@ mv -f $(OBJ_DIR)$<.d $(OBJ_DIR)$<.d.tmp
	@ sed -e 's|.*:|$@:|' < $(OBJ_DIR)$<.d.tmp > $(OBJ_DIR)$<.d
	@ rm -f $(OBJ_DIR)$<.d.tmp
	
	$(SILENT) $(CC) $(CFLAGS) $(INCLUDE_CMD) -o $@ -c $<
	
	
$(OBJ_DIR)%$(CPP_SUFFIX)$(OBJ_SUFFIX): %$(CPP_SUFFIX) $(LINKER_SCRIPT_FILE)
	@echo building $<
	@ mkdir -p $(dir $@)
	@ $(CPP) $(CPPFLAGS) $(INCLUDE_CMD) -MM -MF $(OBJ_DIR)$<.d -c $<
	@ mv -f $(OBJ_DIR)$<.d $(OBJ_DIR)$<.d.tmp
	@ sed -e 's|.*:|$@:|' < $(OBJ_DIR)$<.d.tmp > $(OBJ_DIR)$<.d
	@ rm -f $(OBJ_DIR)$<.d.tmp
	
	$(SILENT) $(CPP) $(CPPFLAGS) $(INCLUDE_CMD) -o $@ -c $<
	
	
docu: $(TARGET).doxyfile $(C_FILES)
	doxygen $(TARGET).doxyfile

flash: $(TARGET).elf
	$(SILENT) $(GDB) -batch -x bin/flashGDB $<
	
$(TARGET).bin: $(TARGET).elf
	@ $(OBJ_CPY) -O binary $< $@

usb-flash: $(TARGET).bin
	@ dfu-util -s 0x08000000 -D $< -a 0 -R

-include $(DEP_FILES)
