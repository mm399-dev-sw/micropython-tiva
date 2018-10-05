# Makefile for Micropython Tiva Lauchpad (TM4C123GH6PM) Port
# by Raphael Kriegl

#Check for OS
ifdef SYSTEMROOT
#Windows / MinGW
   RM = del /q
   RD = rd /s /q
   FixPath = $(subst /,\,$1)
else
#Linux
   ifeq ($(shell uname), Linux)
      RM = rm -f
      RD = rm -rf
      FixPath = $1
   endif
endif

DEBUG = 1

# Include variabes for Python and Compiler
include ../../py/mkenv.mk

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

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
TIVAWARE_LIB = TivaWare

# The base directory for TivaWare.
#ROOT = $(TIVAWARE_LIB)

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
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(TIVAWARE_LIB)
INC += -I$(TIVAWARE_LIB)/driverlib
INC += -I$(PY_SRC)
INC += -I../../

DFU = $(TOP)/tools/dfu.py
PYDFU = $(TOP)/tools/pydfu.py
CFLAGS_CORTEX_M4 = -mthumb -mtune=cortex-m4 -mabi=aapcs-linux -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fsingle-precision-constant -Wdouble-promotion
CFLAGS = $(INC) -Wall -Werror -std=c99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT)
#Preprocessor Flags
CFLAGS += -DPART_$(PART) -c -DTARGET_IS_TM4C123_RB1

LDFLAGS = -nostdlib -T $(LINKER_FILE) --entry ResetISR -Map=$@.map --cref --gc-sections

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb -D DEBUG
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
SRC_C = \
	main.c \
	startup_gcc.c \
	uart_core.c \
	lib/utils/printf.c \
	lib/utils/stdout_helpers.c \
	lib/utils/pyexec.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c \
	_frozen_mpy.c 

OBJ = $(PY_CORE_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o))


all: $(OBJ) ${PROJECT_NAME}.axf ${PROJECT_NAME}

$(BUILD)/_frozen_mpy.c: frozentest.mpy $(BUILD)/genhdr/qstrdefs.generated.h
	$(ECHO) "MISC freezing bytecode"
	$(Q)$(TOP)/tools/mpy-tool.py -f -q $(BUILD)/genhdr/qstrdefs.preprocessed.h -mlongint-impl=none $< > $@

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


include $(TOP)/py/mkrules.mk

clean:
	$(RD) $(BUILD)
	$(RM) $(TIVAWARE_LIB)/driverlib/gcc/*.d
	$(RM) $(TIVAWARE_LIB)/driverlib/gcc/*.o
	$(RM) *.o *.d *.axf *.bin *.lst
