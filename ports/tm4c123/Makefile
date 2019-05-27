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
#TIVAWARE_LIB = /mnt/c/ti/TivaWare
TIVAWARE_LIB = /home/raphael/ti/tivaware

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h $(BUILD)/pins_qstr.h
QSTR_GLOBAL_DEPENDENCIES = board/mpconfigboard.h

MICROPY_FLOAT_IMPL=none#single
LONGINT_IMPL=none#mpz

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
STARTUP_FILE = startup_TM4C123.c
SYSTEM_FILE = misc/system_TM4C123.c

USBDEV_DIR=usbdev

# Included directories
INC += -I.
INC += -I./mods
INC += -I./misc
INC += -I./cmsis
INC += -I$(TOP)
INC += -I$(BUILD)
INC += -I$(BUILD)/genhdr
INC += -I$(TIVAWARE_LIB)
INC += -I$(TIVAWARE_LIB)/driverlib
INC += -I$(PY_SRC)
INC += -I./board
INC += -I../../
INC += -I$(USBDEV_DIR)/core/inc -I$(USBDEV_DIR)/class/inc

DFU = $(TOP)/tools/dfu.py
PYDFU = $(TOP)/tools/pydfu.py
FATFS_DIR=lib/oofatfs
CFLAGS_CORTEX_M4 = -mthumb -mtune=cortex-m4 -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mabi=aapcs-linux
#Warnings
CFLAGS = $(INC) -Wall -Werror -Wpointer-arith
#Compiler
CFLAGS += -std=gnu99 -nostdlib $(CFLAGS_CORTEX_M4) $(COPT) -fshort-enums #-fno-short-enums
#Preprocessor Flags
CFLAGS += -DPART_$(PART) -c -D$(PART) -DARMCM4_FP -DTARGET_IS_TM4C123_RB1 -DMICROPY_PIN_DEFS_PORT_H=\"mods/pin_defs_tm4c123.h\" #-DMICROPY_MPHALPORT_H=\"mphalport.h\"
CFLAGS += -DFFCONF_H=\"lib/oofatfs/ffconf.h\"

ifeq ($(LONGINT_IMPL), mpz)
CFLAGS += -DMICROPY_LONGINT_IMPL=MICROPY_LONGINT_IMPL_MPZ
else
CFLAGS += -DMICROPY_LONGINT_IMPL=MICROPY_LONGINT_IMPL_NONE
endif

ifeq ($(MICROPY_FLOAT_IMPL),double)
CFLAGS += -DMICROPY_FLOAT_IMPL=MICROPY_FLOAT_IMPL_DOUBLE
else ifeq ($(MICROPY_FLOAT_IMPL),single)
CFLAGS += -DMICROPY_FLOAT_IMPL=MICROPY_FLOAT_IMPL_FLOAT
CFLAGS += -fsingle-precision-constant -Wdouble-promotion
else 
CFLAGE += -DMICROPY_FLOAT_IMPL=MICROPY_FLOAT_IMPL_NONE
endif
LDFLAGS = -nostdlib -T $(LINKER_FILE) -Map=$@.map --cref --gc-sections

# Tune for Debugging or Optimization
ifeq ($(DEBUG), 1)
CFLAGS += -O0 -ggdb -DDEBUG
else
CFLAGS += -Os -DNDEBUG
CFLAGS += -fdata-sections -ffunction-sections
endif

# Libraries
LIBS = $(TIVAWARE_LIB)/driverlib/gcc/libdriver.a
LIBGCC := ${shell $(CC) $(CFLAGS) --print-libgcc-file-name}
LIBC := ${shell ${CC} ${CFLAGS} --print-file-name=libc.a}
LIBM := ${shell ${CC} ${CFLAGS} --print-file-name=libm.a}

# Sources	

#lib
SRC_LIB = $(addprefix lib/,\
	libc/string0.c \
	mp-readline/readline.c \
	netutils/netutils.c \
	timeutils/timeutils.c \
	utils/interrupt_char.c \
	utils/pyexec.c \
	utils/sys_stdio_mphal.c \
	oofatfs/ff.c \
	oofatfs/option/unicode.c \
	)
	#oofatfs/ff.c 
	#oofatfs/option/unicode.c 

# Module sources
MODS_SRC_C = $(addprefix mods/,\
	mphalport.c \
	pin.c \
	pin_named_pins.c \
	modmachine.c \
	modpyb.c \
	uart.c \
	rtc.c \
	systick.c \
	handlers.c \
	spi.c \
	sdcard.c \
	moduos.c \
	pendsv.c \
	pybthread.c \
	modutime.c \
	irq.c \
)
	# fatfs_port.c \
	# irq.c \
	# modutime.c \	
	# moduos.c \
	# pybthread.c \
	# pendsv.c \
	# flashbdev.c \
	# storage.c \
	# flash.c \
	# )
	#	extint.c see forum

# QSTR sources
SRC_QSTR += $(MODS_SRC_C)

# Core sources
CORE_SRC_C = \
	$(STARTUP_FILE) \
	main.c \
	uart_core.c \
	gccollect.c \
	$(SYSTEM_FILE) \
	
ASM_SRC_S = \
	gchelper.s 


ifeq ($(MICROPY_FLOAT_IMPL),double)
SRC_LIBM = $(addprefix lib/libm_dbl/,\
	__cos.c \
	__expo2.c \
	__fpclassify.c \
	__rem_pio2.c \
	__rem_pio2_large.c \
	__signbit.c \
	__sin.c \
	__tan.c \
	acos.c \
	acosh.c \
	asin.c \
	asinh.c \
	atan.c \
	atan2.c \
	atanh.c \
	ceil.c \
	cos.c \
	cosh.c \
	erf.c \
	exp.c \
	expm1.c \
	floor.c \
	fmod.c \
	frexp.c \
	ldexp.c \
	lgamma.c \
	log.c \
	log10.c \
	log1p.c \
	modf.c \
	nearbyint.c \
	pow.c \
	rint.c \
	scalbn.c \
	sin.c \
	sinh.c \
	sqrt.c \
	tan.c \
	tanh.c \
	tgamma.c \
	trunc.c \
	)
else
SRC_LIBM = $(addprefix lib/libm/,\
	math.c \
	thumb_vfp_sqrtf.c \
	acoshf.c \
	asinfacosf.c \
	asinhf.c \
	atan2f.c \
	atanf.c \
	atanhf.c \
	ef_rem_pio2.c \
	erf_lgamma.c \
	fmodf.c \
	kf_cos.c \
	kf_rem_pio2.c \
	kf_sin.c \
	kf_tan.c \
	log1pf.c \
	nearbyintf.c \
	sf_cos.c \
	sf_erf.c \
	sf_frexp.c \
	sf_ldexp.c \
	sf_modf.c \
	sf_sin.c \
	sf_tan.c \
	wf_lgamma.c \
	wf_tgamma.c \
	)
endif

# Additional sources
# ADD_SRC_C = 
# 	drivers/bus/softspi.c 
		
SRC_C = $(CORE_SRC_C) $(MODS_SRC_C) $(GEN_PINS_SRC) $(SRC_LIB) $(SRC_LIBM) $(ADD_SRC_C)
$(SRC_LIBM): COPT += -Os

ASM_OBJ = $(addprefix $(BUILD)/, $(ASM_SRC_S:.s=.o))

C_OBJ = $(addprefix $(BUILD)/, $(SRC_C:.c=.o))

OBJ = $(PY_O) $(ASM_OBJ) $(C_OBJ) 

# This file contains performance critical functions so turn up the optimisation
# level.  It doesn't add much to the code size and improves performance a bit.
# Don't use -O3 with this file because gcc tries to optimise memset in terms of itself.
$(BUILD)/lib/libc/string0.o: COPT += -O2

# We put several files into the first 16K section with the ISRs.
# If we compile these using -O0 then it won't fit. So if you really want these
# to be compiled with -O0, then edit boards/common.ld (in the .isr_vector section)
# and comment out the following lines.
$(BUILD)/$(FATFS_DIR)/ff.o: COPT += -Os
$(filter $(PY_BUILD)/../extmod/vfs_fat_%.o, $(PY_O)): COPT += -Os
$(PY_BUILD)/formatfloat.o: COPT += -Os
$(PY_BUILD)/parsenum.o: COPT += -Os
$(PY_BUILD)/mpprint.o: COPT += -Os


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

INSERT_USB_IDS = $(TOP)/tools/insert-usb-ids.py
FILE2H = $(TOP)/tools/file2h.py

USB_IDS_FILE = mods/usb.h
CDCINF_TEMPLATE = pybcdc.inf_template
GEN_CDCINF_FILE = $(HEADER_BUILD)/pybcdc.inf
GEN_CDCINF_HEADER = $(HEADER_BUILD)/pybcdc_inf.h

# List of sources for qstr extraction
SRC_QSTR += $(SRC_C) $(SRC_MOD) $(SRC_LIB) $(EXTMOD_SRC_C)
# Append any auto-generated sources that are needed by sources listed in
# SRC_QSTR
SRC_QSTR_AUTO_DEPS += $(GEN_CDCINF_HEADER)

# Making OBJ use an order-only depenedency on the generated pins.h file
# has the side effect of making the pins.h file before we actually compile
# any of the objects. The normal dependency generation will deal with the
# case when pins.h is modified. But when it doesn't exist, we don't know
# which source files might need it.
$(OBJ): | $(GEN_PINS_HDR)

# With conditional pins, we may need to regenerate qstrdefs.h when config
# options change.
$(HEADER_BUILD)/qstrdefs.generated.h: board/mpconfigboard.h

# main.c can't be even preprocessed without $(GEN_CDCINF_HEADER)
main.c: $(GEN_CDCINF_HEADER)

# Call make-pins.py to generate both pins_gen.c and pins.h
$(GEN_PINS_SRC) $(GEN_PINS_HDR) $(GEN_PINS_QSTR): $(BOARD_PINS) $(MAKE_PINS) $(AF_FILE) $(PREFIX_FILE) | $(HEADER_BUILD)
	$(ECHO) "Create $@"
	$(Q)$(PYTHON) $(MAKE_PINS) --board $(BOARD_PINS) --af $(AF_FILE) --prefix $(PREFIX_FILE) --hdr $(GEN_PINS_HDR) --qstr $(GEN_PINS_QSTR) > $(GEN_PINS_SRC)

$(BUILD)/pins.o: $(BUILD)/pins.c
	$(call compile_c)	

${PROJECT_NAME}.axf: $(OBJ)
	@echo
	@echo Used these C-Flags: $(CFLAGS)
	#$(CC) $(CFLAGS) -E build/pins.c	-o build/pins.e
	@echo
	@echo Making driverlib
	$(MAKE) -C ${TIVAWARE_LIB}/driverlib/
	@echo
	@echo Linking...
	@echo AXF:
	$(LD) -T $(LINKER_FILE) $(LFLAGS) -o ${PROJECT_NAME}.axf $(OBJ) $(LIBS) $(LIBM) $(LIBC) $(LIBGCC)
	@echo ELF:
	$(LD) -T $(LINKER_FILE) $(LFLAGS) -o ${PROJECT_NAME}.elf $(OBJ) $(LIBS) $(LIBM) $(LIBC) $(LIBGCC)

${PROJECT_NAME}: ${PROJECT_NAME}.axf
	@echo
	@echo Copying...
	$(OBJCOPY) $(CPFLAGS) ${PROJECT_NAME}.axf ${PROJECT_NAME}.bin
	@echo
	@echo Creating list file...
	$(OD) $(ODFLAGS) ${PROJECT_NAME}.axf > ${PROJECT_NAME}.lst

CMSIS_DIR = ./cmsis

GEN_STMCONST_HDR = $(HEADER_BUILD)/modstm_const.h
GEN_STMCONST_QSTR = $(BUILD)/modstm_qstr.h
GEN_STMCONST_MPZ = $(HEADER_BUILD)/modstm_mpz.h
CMSIS_MCU_HDR = $(CMSIS_DIR)/core_cm4.h
# Use a pattern rule here so that make will only call make-stmconst.py once to
# make both modstm_const.h and modstm_qstr.h
$(HEADER_BUILD)/%_const.h $(BUILD)/%_qstr.h: $(CMSIS_MCU_HDR) make-stmconst.py | $(HEADER_BUILD)
	$(ECHO) "GEN stmconst $@"
	$(Q)$(PYTHON) make-stmconst.py --qstr $(GEN_STMCONST_QSTR) --mpz $(GEN_STMCONST_MPZ) $(CMSIS_MCU_HDR) > $(GEN_STMCONST_HDR)

$(GEN_CDCINF_HEADER): $(GEN_CDCINF_FILE) $(FILE2H) | $(HEADER_BUILD)
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(FILE2H) $< > $@

$(GEN_CDCINF_FILE): $(CDCINF_TEMPLATE) $(INSERT_USB_IDS) $(USB_IDS_FILE) | $(HEADER_BUILD)
	$(ECHO) "GEN $@"
	$(Q)$(PYTHON) $(INSERT_USB_IDS) $(USB_IDS_FILE) $< > $@

CLEAN_EXTRA += *.axf *.bin *.lst *.elf #$(TIVAWARE_LIB)/driverlib/gcc/* 

include $(TOP)/py/mkrules.mk

