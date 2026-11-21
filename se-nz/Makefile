# NijiN SE-NZ Converter Firmware Build & Host SIL Makefile

ARM_CC      ?= arm-none-eabi-gcc
ARM_OBJCOPY ?= arm-none-eabi-objcopy
ARM_OBJDUMP ?= arm-none-eabi-objdump
ARM_SIZE    ?= arm-none-eabi-size
ARM_NM      ?= arm-none-eabi-nm
HOST_CC     ?= $(shell command -v clang >/dev/null 2>&1 && echo clang || echo gcc)
PYTHON      ?= python3

UNAME_S     := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
  SHLIB_EXT   := dylib
  SHLIB_FLAGS := -dynamiclib
else
  SHLIB_EXT   := so
  SHLIB_FLAGS := -shared -fPIC
endif

BUILD_DIR   := build
INC_DIR     := include
LD_SCRIPT   := ld/hc32f460jeua_boryoza.ld

COMMON_WARN := -Wall -Wextra -Werror -Wno-unused-parameter
MCU_ARCH    := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
MCU_CFLAGS  := -std=c17 -O2 -g3 $(COMMON_WARN) -ffunction-sections -fdata-sections -fno-common -ffreestanding -I$(INC_DIR) $(MCU_ARCH)
CORE_OPT    := -O3 -ffp-contract=fast -fno-math-errno -fno-trapping-math -fno-signed-zeros -fomit-frame-pointer
NOFPU_FLAGS := -mgeneral-regs-only -fno-tree-vectorize -fno-builtin
MCU_LDFLAGS := $(MCU_ARCH) -nostartfiles -nostdlib -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/BORYOZA_V200.map -T $(LD_SCRIPT) -lgcc

HAL_ASM     := src/00_hal/startup_hc32f460.S
HAL_SRCS    := \
	src/00_hal/sys_clock_sram.c \
	src/00_hal/gpio_pads.c \
	src/00_hal/tmra1_clock.c \
	src/00_hal/efm_ramfunc.c

TRANS_SRCS  := \
	src/01_transport/z_spi3_slave.c \
	src/01_transport/z_spi1_hotline.c \
	src/01_transport/e_usart1_dma.c \
	src/01_transport/e_tmra4_vsync.c

CORE_SRCS   := \
	src/02_core/lens_session.c \
	src/02_core/af_servo_bridge.c \
	src/02_core/vr_duty_controller.c \
	src/02_core/aperture_controls.c \
	src/02_core/optics_xlate_fpu.c \
	src/02_core/z_snapshot_builder.c

DIAG_SRCS   := \
	src/03_diag_storage/blackbox_recorder.c \
	src/03_diag_storage/journal_dual_sector.c

MAIN_SRC    := src/main.c

HAL_OBJS    := $(BUILD_DIR)/startup_hc32f460.o $(patsubst src/00_hal/%.c,$(BUILD_DIR)/00_hal_%.o,$(HAL_SRCS))
TRANS_OBJS  := $(patsubst src/01_transport/%.c,$(BUILD_DIR)/01_trans_%.o,$(TRANS_SRCS))
CORE_OBJS   := $(patsubst src/02_core/%.c,$(BUILD_DIR)/02_core_%.o,$(CORE_SRCS))
DIAG_OBJS   := $(patsubst src/03_diag_storage/%.c,$(BUILD_DIR)/03_diag_%.o,$(DIAG_SRCS))
MAIN_OBJ    := $(BUILD_DIR)/main.o

ALL_MCU_OBJS := $(HAL_OBJS) $(TRANS_OBJS) $(CORE_OBJS) $(DIAG_OBJS) $(MAIN_OBJ)

ELF_OUT     := $(BUILD_DIR)/BORYOZA_V200.elf
BIN_OUT     := $(BUILD_DIR)/BORYOZA_V200.bin
PUB_OUT     := $(BUILD_DIR)/BORYOZA_V200.PUB
PUB_V224    := $(BUILD_DIR)/BORYOZA_V224.PUB
SIL_LIB     := $(BUILD_DIR)/libboryoza_core.$(SHLIB_EXT)
GEN_HDR     := $(INC_DIR)/lens_db_generated.h

.PHONY: all mcu sil check test clean

all: mcu sil

$(BUILD_DIR)/startup_hc32f460.o: $(HAL_ASM) | $(BUILD_DIR)
	$(ARM_CC) $(MCU_ARCH) -g3 -c $< -o $@

$(BUILD_DIR)/00_hal_%.o: src/00_hal/%.c $(INC_DIR)/hc32f460_regs.h $(INC_DIR)/boryoza_types.h | $(BUILD_DIR)
	$(ARM_CC) $(MCU_CFLAGS) $(NOFPU_FLAGS) -c $< -o $@

$(BUILD_DIR)/01_trans_%.o: src/01_transport/%.c $(INC_DIR)/hc32f460_regs.h $(INC_DIR)/boryoza_types.h | $(BUILD_DIR)
	$(ARM_CC) $(MCU_CFLAGS) $(NOFPU_FLAGS) -c $< -o $@

$(BUILD_DIR)/02_core_%.o: src/02_core/%.c $(GEN_HDR) $(INC_DIR)/boryoza_types.h $(INC_DIR)/lens_db.h | $(BUILD_DIR)
	$(ARM_CC) $(MCU_CFLAGS) $(CORE_OPT) -c $< -o $@

$(BUILD_DIR)/03_diag_%.o: src/03_diag_storage/%.c $(INC_DIR)/hc32f460_regs.h $(INC_DIR)/boryoza_types.h | $(BUILD_DIR)
	$(ARM_CC) $(MCU_CFLAGS) $(NOFPU_FLAGS) -c $< -o $@

$(MAIN_OBJ): $(MAIN_SRC) $(GEN_HDR) $(INC_DIR)/hc32f460_regs.h $(INC_DIR)/boryoza_types.h | $(BUILD_DIR)
	$(ARM_CC) $(MCU_CFLAGS) $(CORE_OPT) -c $< -o $@

$(ELF_OUT): $(ALL_MCU_OBJS) $(LD_SCRIPT)
	$(ARM_CC) $(ALL_MCU_OBJS) $(MCU_LDFLAGS) -o $@
	$(ARM_SIZE) $@

$(PUB_OUT) $(BIN_OUT): $(ELF_OUT) tools/pack_pub.py
	$(PYTHON) tools/pack_pub.py --elf $(ELF_OUT) --bin $(BIN_OUT) --pub $(PUB_OUT) --objcopy $(ARM_OBJCOPY) --objdump $(ARM_OBJDUMP)
	cp -f $(PUB_OUT) $(PUB_V224)

mcu: $(PUB_OUT)

$(SIL_LIB): $(CORE_SRCS) $(GEN_HDR) $(INC_DIR)/boryoza_types.h $(INC_DIR)/lens_db.h | $(BUILD_DIR)
	$(HOST_CC) -std=c17 -O2 -g $(COMMON_WARN) -fsanitize=address,undefined $(SHLIB_FLAGS) -I$(INC_DIR) $(CORE_SRCS) -lm -o $@

sil: $(SIL_LIB)

check: all
	$(PYTHON) tools/check_source_comments.py --selftest
	@if $(ARM_NM) -u $(ELF_OUT) | grep -qE '__aeabi_u?ldivmod|__udivmoddi4'; then \
		echo "ERROR: soft 64-bit division helpers found in $(ELF_OUT)"; exit 1; \
	fi
	$(PYTHON) tools/pack_pub.py --selftest || true
	$(PYTHON) tools/decode_blackbox.py --selftest
	$(PYTHON) tools/dump_usb_blackbox.py --selftest
	$(PYTHON) tools/flash_usb_bor.py --selftest

test: check
	$(PYTHON) tests/test_core_sil.py

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
