PROGRAM    := tinyuwb
SRCS       := $(wildcard *.c) $(wildcard decadriver/*.c) $(wildcard decartls/*.c) $(wildcard os/*.c)
CROSS      ?= arm-none-eabi-

# NOTE: TICK_HZ has to be large enough for ranging to be accurate
CONFIG_TICK_HZ     := 500
CONFIG_DEEP_SLEEP  := 1
CONFIG_ENABLE_LEDS := 1


###############################################################################

# Clear implicit rules
.SUFFIXES:

AR         := $(CROSS)ar
CC         := $(CROSS)gcc
LD         := $(CROSS)ld
OBJCOPY    := $(CROSS)objcopy
OBJDUMP    := $(CROSS)objdump
SIZE       := $(CROSS)size
NM         := $(CROSS)nm
GDB        := gdb-multiarch

ELF        := $(PROGRAM).elf
BIN        := $(PROGRAM).bin
HEX        := $(PROGRAM).hex
MAP        := $(PROGRAM).map
DMP        := $(PROGRAM).out

TOPDIR     := $(shell pwd)
DEPDIR     := $(TOPDIR)/.dep
OBJS       := $(SRCS:.c=.o)

# Debugging
CFLAGS     += -Wall -Wdouble-promotion -g3 -gdwarf-4
# Optimizations
# NOTE: without optimization everything will fail... Computational power is marginal.
CFLAGS     += -O2 -fbranch-target-load-optimize -fipa-pta -frename-registers -fgcse-sm -fgcse-las -fsplit-loops -fstdarg-opt
# Selected flags from -O3
CFLAGS     += -funswitch-loops -fpredictive-commoning -fgcse-after-reload \
              -fsplit-paths -ftree-slp-vectorize -fvect-cost-model -ftree-partial-pre -fipa-cp-clone
#              -ftree-loop-vectorize -ftree-loop-distribution -ftree-loop-distribute-patterns -floop-interchange \
# Use these for debugging-friendly binary
#CFLAGS     += -O0
#CFLAGS     += -Og
# Disabling aggressive loop optimizations since it does not work for loops longer than certain iterations
CFLAGS     += -fno-aggressive-loop-optimizations
# Aggressive optimizations (unstable or causes huge binaries, e.g. peel-loops gives huge gpio_mode_setup, which is rarely used)
#CFLAGS     += -finline-functions -fpeel-loops -funroll-loops -floop-unroll-and-jam -fbranch-target-load-optimize2
# Includes
CFLAGS     += -Ilibopencm3/include/ -I$(TOPDIR)
# Config
CONFIG     := TICK_HZ=$(CONFIG_TICK_HZ) DEEP_SLEEP=$(CONFIG_DEEP_SLEEP) ENABLE_LEDS=$(CONFIG_ENABLE_LEDS)
CFLAGS     += $(addprefix -D,$(CONFIG))

# Generate dependency information
CFLAGS     += -MT $@ -MMD -MP -MF $(DEPDIR)/$*.Td
$(shell mkdir -p $(DEPDIR) >/dev/null)
PRECOMPILE  = mkdir -p $(dir $(DEPDIR)/$*.Td)
POSTCOMPILE = mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d && touch $@

# Architecture-dependent
LDSCRIPT   := libopencm3/lib/stm32/l0/stm32l0xx6.ld
ARCH_FLAGS := -DSTM32L0 -mthumb -mcpu=cortex-m0plus -msoft-float -fsingle-precision-constant -ffast-math -flto --specs=nano.specs
OPENCM3_MK := lib/stm32/l0
LIBOPENCM3 := libopencm3/lib/libopencm3_stm32l0.a
CFLAGS     += $(ARCH_FLAGS)
CFLAGS     += -fno-common -ffunction-sections -fdata-sections
# LDPATH is required for libopencm3's ld scripts to work.
LDPATH     := libopencm3/lib/
# NOTE: the rule will ensure CFLAGS are added during linking
LDFLAGS    += -nostdlib -L$(LDPATH) -T$(LDSCRIPT) -Wl,-Map -Wl,$(MAP) -Wl,--gc-sections -Wl,--relax
LDLIBS     += $(LIBOPENCM3) -lc -lgcc


default: $(BIN) $(HEX) $(DMP) size

$(ELF): $(LDSCRIPT) $(OBJS) $(LIBOPENCM3) Makefile
	$(CC) -o $@ $(CFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS)

$(DMP): $(ELF)
	$(OBJDUMP) -d $< > $@

%.hex: %.elf
	$(OBJCOPY) -S -O ihex   $< $@

%.bin: %.elf
	$(OBJCOPY) -S -O binary $< $@

%.o: %.c $(LIBOPENCM3) Makefile
	$(PRECOMPILE)
	$(CC) $(CFLAGS) -c $< -o $@
	$(POSTCOMPILE)

# NOTE: libopencm3's Makefile is unaware of top-level Makefile changes, so force remake
# NOTE: update ld script modified time to avoid unnecessary remaking
$(LIBOPENCM3) $(LDSCRIPT): Makefile
	git submodule update --init
	make -B -C libopencm3 CFLAGS="$(CFLAGS)" PREFIX=$(patsubst %-,%,$(CROSS)) $(OPENCM3_MK)
	if [ -e $(LDSCRIPT) ]; then touch $(LDSCRIPT); fi


.PHONY: clean distclean size symbols symbols_bss flash erase debug monitor check

clean:
	rm -f $(OBJS) $(ELF) $(HEX) $(BIN) $(MAP) $(DMP)
	rm -rf $(DEPDIR)

distclean: clean
	make -C libopencm3 clean
	rm -f *.o *~ *.swp *.hex

# These targets want clean terminal output
size: $(ELF)
	@echo ""
	@$(SIZE) $<
	@echo ""

symbols: $(ELF)
	@$(NM) --demangle --size-sort -S $< | grep -v ' [bB] '

symbols_bss: $(ELF)
	@$(NM) --demangle --size-sort -S $< | grep ' [bB] '

flash: $(HEX)
	@killall st-util || echo
	@st-flash --reset --format ihex write $<

erase:
	@killall st-util || echo
	@st-flash erase

debug: $(ELF)
	@killall st-util || echo
	@setsid st-util &
	@-$(GDB) $< -q -ex 'target extended-remote localhost:4242' -ex 'load'

# Dependencies
$(DEPDIR)/%.d:
.PRECIOUS: $(DEPDIR)/%.d
include $(wildcard $(patsubst %,$(DEPDIR)/%.d,$(basename $(SRCS))))
