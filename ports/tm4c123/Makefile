include ../../py/mkenv.mk

CROSS = 1

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

#
# Defines the part type that this project uses.
#
PART=TM4C123GH6PM

PROJECT_NAME = uPY_TM4C123

TIVAWARE_LIB = TivaWare/

#
# The base directory for TivaWare.
#
ROOT = $(TIVAWARE_LIB)

# include py core make definitions
include $(TOP)/py/py.mk

ifeq ($(CROSS), 1)
CROSS_COMPILE = arm-none-eabi-
endif

OD = $(CROSS_COMPILE)objdump

ODFLAGS = -S
LFLAGS = --gc-sections
CPFLAGS = -OBinary

LINKER_FILE = tm4c123gh6pm.ld

INC += -I.
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(TIVAWARE_LIB)
INC += -I$(TIVAWARE_LIB)/inc
INC += -I$(PY_SRC)
INC += -I../../

ifeq ($(CROSS), 1)
DFU = $(TOP)/tools/dfu.py
PYDFU = $(TOP)/tools/pydfu.py
CFLAGS_CORTEX_M4 = -mthumb -mtune=cortex-m4 -mabi=aapcs-linux -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -fsingle-precision-constant -Wdouble-promotion
CFLAGS = $(INC) -Wall -Werror -std=gnu99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT)
#Preprocessor Flags
CFLAGS += -DPART_$(PART) -c -DTARGET_IS_TM4C123_RB1

LDFLAGS = -nostdlib -T $(LINKER_FILE) -Map=$@.map --cref --gc-sections
else
LD = gcc
#CFLAGS = -m32 $(INC) -Wall -Werror -std=gnu99 $(COPT)
LDFLAGS = -m32 -Wl,-Map=$@.map,--cref -Wl,--gc-sections
endif

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb -D DEBUG
else
CFLAGS += -Os -DNDEBUG
CFLAGS += -fdata-sections -ffunction-sections
endif


LIBS = $(TIVAWARE_LIB)/driverlib/gcc/libdriver.a


LICGCC := ${shell $(CC) $(CFLAGS) -print-libgcc-file-name}

LIBC:=${shell ${CC} ${CFLAGS} -print-file-name=libc.a}

LIBM:=${shell ${CC} ${CFLAGS} -print-file-name=libm.a}


SRC_C = \
	main.c \
	startup_gcc.c \
	uart_core.c \
	lib/utils/printf.c \
	lib/utils/stdout_helpers.c \
	lib/utils/pyexec.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c \
	$(BUILD)/_frozen_mpy.c 

OBJ = $(PY_CORE_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o))


all: $(OBJ) ${PROJECT_NAME}.axf ${PROJECT_NAME}

%.o: %.c
	@echo
	@echo Compiling $<...
	$(CC) -c $(CFLAGS) ${<} -o ${@}

${PROJECT_NAME}.axf: $(OBJ)
	@echo
	@echo Making driverlib
	$(MAKE) -C ${TIVAWARE_LIB}driverlib/
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


# Rule to load the project to the board
# I added a sudo because it's needed without a rule.
load:
	sudo ${FLASHER} ${PROJECT_NAME}.bin ${FLASHER_FLAGS}


test: $(BUILD)/firmware.elf
	$(Q)/bin/echo -e "print('hello world!', list(x+1 for x in range(10)), end='eol\\\\n')\\r\\n\\x04" | $(BUILD)/firmware.elf | tail -n2 | grep "^hello world! \\[1, 2, 3, 4, 5, 6, 7, 8, 9, 10\\]eol"



include $(TOP)/py/mkrules.mk
