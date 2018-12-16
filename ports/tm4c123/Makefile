# Makefile for Micropython Tiva Lauchpad (TM4C123GH6PM) Port
# by Raphael Kriegl

DEBUG = 1

# Include variabes for Python and Compiler
include ../../py/mkenv.mk

#
# Defines the part type that this project uses.
#
PART=TM4C123GH6PM

# OpenOCD paths
#OPENOCD_PATH = /usr/local/share/openocd
OPENOCD_PATH = openocd

#OPENOCD_BOARD_CFG = $(OPENOCD_PATH)/scripts/board/ek-tm4c123gxl.cfg
OPENOCD_BOARD_CFG = openocd/ek-tm4c123gxl.cfg
OPENOCD_PROG_CFG = openocd/tiva_flash.cfg


# Name of the Project (and the resulting binary file)
PROJECT_NAME = uPY_TM4C123

# Location of the TivaWare library
# Can be pointed to a newer version if desired
TIVAWARE_LIB = /mnt/c/ti/TivaWare
#TIVAWARE_LIB = /home/raphael/ti/tivaware

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h $(BUILD)/pins_qstr.h
QSTR_GLOBAL_DEPENDENCIES = board/mpconfigboard.h

# include py core make definitions
include $(TOP)/py/py.mk

# Cross compilation prefix
CROSS_COMPILE = arm-none-eabi-
# Cross compilation flag
CROSS = 1

OD = $(CROSS_COMPILE)objdump

ODFLAGS = -S
LFLAGS = --gc-sections
CPFLAGS = -O binary

LINKER_FILE = tm4c123gh6pm.ld

# Included directories
INC += -I.
INC += -I./mods
INC += -I./misc
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(BUILD)/genhdr
INC += -I$(TIVAWARE_LIB)
INC += -I$(TIVAWARE_LIB)/driverlib
INC += -I$(PY_SRC)
INC += -I./board
INC += -I../../

DFU = $(TOP)/tools/dfu.py
PYDFU = $(TOP)/tools/pydfu.py
CFLAGS_CORTEX_M4 = -mthumb -mtune=cortex-m4 -mabi=aapcs-linux -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fsingle-precision-constant -Wdouble-promotion
CFLAGS = $(INC) -Wall -Werror -std=c99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT)
#Preprocessor Flags
CFLAGS += -DPART_$(PART) -c -D$(PART) -DTARGET_IS_TM4C123_RB1 -DMICROPY_PIN_DEFS_PORT_H=\"mods/pin_defs_tm4c123.h\" #-DMICROPY_MPHALPORT_H=\"mphalport.h\"

LDFLAGS = -nostdlib -T $(LINKER_FILE) --entry ResetISR -Map=$@.map --cref --gc-sections

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb -DDEBUG
else
CFLAGS += -Os -DNDEBUG
CFLAGS += -fdata-sections -ffunction-sections
endif

# Libraries
LIBS = $(TIVAWARE_LIB)/driverlib/gcc/libdriver.a
LICGCC := ${shell $(CC) $(CFLAGS) -print-libgcc-file-name}
LIBC := ${shell ${CC} ${CFLAGS} -print-file-name=libc.a}
LIBM := ${shell ${CC} ${CFLAGS} -print-file-name=libm.a}

# Sources	


# Module sources
MODS_SRC_C = $(addprefix mods/,\
	mphalport.c \
	pin.c \
	pin_named_pins.c \
	modmachine.c \
	)
	#	modmachine.c \

# QSTR sources
SRC_QSTR += $(MODS_SRC_C)

# Core sources
CORE_SRC_C = \
	main.c \
	startup_gcc.c \
	uart_core.c \
	gccollect.c 
	
ASM_SRC_S = \
	gchelper.s

PY_SRC_C = \
	lib/utils/stdout_helpers.c \
	lib/utils/pyexec.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c \
	_frozen_mpy.c 
		#lib/utils/printf.c 
		
SRC_C = $(CORE_SRC_C) $(MODS_SRC_C) $(GEN_PINS_SRC) $(PY_SRC_C) 

ASM_OBJ = $(addprefix $(BUILD)/, $(ASM_SRC_S:.s=.o))

C_OBJ = $(addprefix $(BUILD)/, $(SRC_C:.c=.o))

OBJ = $(PY_O) $(ASM_OBJ) $(C_OBJ) 


all: $(OBJ) ${PROJECT_NAME}.axf ${PROJECT_NAME}

$(BUILD)/_frozen_mpy.c: frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
	$(ECHO) "MISC freezing bytecode"
	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@
	
MAKE_PINS = board/make-pins.py
BOARD_PINS = board/pins.csv
AF_FILE = board/tm4c123_af.csv
PREFIX_FILE = board/tm4c123_prefix.c
GEN_PINS_SRC = $(BUILD)/pins.c
GEN_PINS_HDR = $(HEADER_BUILD)/pins.h
GEN_PINS_QSTR = $(BUILD)/pins_qstr.h

# Making OBJ use an order-only dependency on the generated pins.h file
# has the side effect of making the pins.h file before we actually compile
# any of the objects. The normal dependency generation will deal with the
# case when pins.h is modified. But when it doesn't exist, we don't know
# which source files might need it.
$(OBJ): | $(GEN_PINS_HDR)

# Call make-pins.py to generate both pins_gen.c and pins.h
$(GEN_PINS_SRC) $(GEN_PINS_HDR) $(GEN_PINS_QSTR): $(BOARD_PINS) $(MAKE_PINS) $(AF_FILE) $(PREFIX_FILE) | $(HEADER_BUILD)
	$(ECHO) "Create $@"
	$(Q)$(PYTHON) $(MAKE_PINS) --board $(BOARD_PINS) --af $(AF_FILE) --prefix $(PREFIX_FILE) --hdr $(GEN_PINS_HDR) --qstr $(GEN_PINS_QSTR) > $(GEN_PINS_SRC)

$(BUILD)/pins.o: $(BUILD)/pins.c
	$(call compile_c)	

${PROJECT_NAME}.axf: $(OBJ)
	@echo
	@echo Making driverlib
	$(MAKE) -C ${TIVAWARE_LIB}/driverlib/
	@echo
	@echo Linking...
	$(LD) -T $(LINKER_FILE) $(LFLAGS) -o ${PROJECT_NAME}.axf $(OBJ) $(LIBS)  $(LIBM) $(LIBC) $(LIBGCC)

${PROJECT_NAME}: ${PROJECT_NAME}.axf
	@echo
	@echo Copying...
	$(OBJCOPY) $(CPFLAGS) ${PROJECT_NAME}.axf ${PROJECT_NAME}.bin
	@echo
	@echo Creating list file...
	$(OD) $(ODFLAGS) ${PROJECT_NAME}.axf > ${PROJECT_NAME}.lst

# Move this to .sh file??
load:
	sudo openocd --file $(OPENOCD_BOARD_CFG)
#-f $(OPENOCD_PROG_CFG) -c "tiva_flash `basename $(CURID) `$(PROJECT_NAME).bin" -c shutdown
#gdb-multiarch
#target extended-remote :3333
#monitor reset halt
#load
#monitor reset init

CLEAN_EXTRA += *.axf *.bin *.lst #$(TIVAWARE_LIB)/driverlib/gcc/* 

include $(TOP)/py/mkrules.mk

