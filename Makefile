##################################################################################################
# CH32X035 Makefile
##################################################################################################
# Automatische build voor CH32X035 projecten met GCC toolchain.
# Auteur: Jij (met hulp van AI)
# Doel: Compileer, link en flash een CH32X035 applicatie.

##################################################################################################
# PROJECT CONFIGURATIE
##################################################################################################

# Naam van het uiteindelijke bestand
TARGET = CH32X035C8T6_mf1

# Directories
SRC_DIRS = User Core Debug Peripheral/src Startup
INC_DIRS = Peripheral/inc Core Debug User
BUILD_DIR = build
BIN_DIR = bin

# Microcontroller instellingen
MCU = CH32X035
F_CPU = 48000000
LDSCRIPT = Ld/Link.ld

##################################################################################################
# TOOLCHAIN
##################################################################################################

# Gebruik de specifieke toolchain die je hebt opgegeven
PREFIX = /usr/share/MRS2/MRS-linux-x64/resources/app/resources/linux/components/WCH/Toolchain/RISC-V\ Embedded\ GCC12/bin/riscv-wch-elf
CC = $(PREFIX)-gcc
OBJCOPY = $(PREFIX)-objcopy
OBJDUMP = $(PREFIX)-objdump
OBJSIZE = $(PREFIX)-size

# ISP Tool (pas aan als je een andere gebruikt)
ISPTOOL = chprog

##################################################################################################
# COMPILER FLAGS
##################################################################################################

# Architectuur en ABI
ARCH_FLAGS = -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore

# C Compiler Flags
CFLAGS = -g -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections
CFLAGS += -fno-common -fmax-errors=20
CFLAGS += $(ARCH_FLAGS) -DF_CPU=$(F_CPU)UL
CFLAGS += -Wall -Wextra -Wunused -Wuninitialized
CFLAGS += -std=gnu99
CFLAGS += -MMD -MP # Genereer dependency files

# Include Paths (automatisch gegenereerd)
INCLUDES = $(foreach dir,$(INC_DIRS),-I$(dir))

# Linker Flags
LDFLAGS = $(ARCH_FLAGS) $(CFLAGS)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -nostartfiles
LDFLAGS += -Xlinker --gc-sections
LDFLAGS += -Xlinker --print-memory-usage
LDFLAGS += -Wl,-Map,$(BIN_DIR)/$(TARGET).map
LDFLAGS += --specs=nano.specs --specs=nosys.specs

# Assembly Flags
ASMFLAGS = $(ARCH_FLAGS)

##################################################################################################
# FILE DISCOVERY
##################################################################################################

# Vind alle C en Assembly source bestanden in de opgegeven directories
C_SOURCES = $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.c))
ASM_SOURCES = $(foreach dir,$(SRC_DIRS),$(wildcard $(dir)/*.S))

# Maak een lijst van object files in de build directory
OBJECTS = $(C_SOURCES:%.c=$(BUILD_DIR)/%.o)
OBJECTS += $(ASM_SOURCES:%.S=$(BUILD_DIR)/%.o)

##################################################################################################
# BUILD TARGETS
##################################################################################################

# Standaard target
.PHONY: all
all: $(BIN_DIR)/$(TARGET).elf $(BIN_DIR)/$(TARGET).hex $(BIN_DIR)/$(TARGET).bin

# Link het ELF bestand
$(BIN_DIR)/$(TARGET).elf: $(OBJECTS)
	@echo "Linking $@"
	@mkdir -p $(BIN_DIR)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
	$(OBJSIZE) $@

# Genereer Intel HEX bestand
$(BIN_DIR)/$(TARGET).hex: $(BIN_DIR)/$(TARGET).elf
	@echo "Creating HEX file $@"
	$(OBJCOPY) -O ihex $< $@

# Genereer binary bestand
$(BIN_DIR)/$(TARGET).bin: $(BIN_DIR)/$(TARGET).elf
	@echo "Creating binary file $@"
	$(OBJCOPY) -O binary $< $@

# Compileer C source bestanden naar object files
$(BUILD_DIR)/%.o: %.c
	@echo "Compiling $<"
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Assembleer Assembly source bestanden naar object files
$(BUILD_DIR)/%.o: %.S
	@echo "Assembling $<"
	@mkdir -p $(dir $@)
	$(CC) $(ASMFLAGS) $(INCLUDES) -c $< -o $@

##################################################################################################
# UTILITY TARGETS
##################################################################################################

# Flash het binary bestand naar de microcontroller
.PHONY: flash
flash: $(BIN_DIR)/$(TARGET).bin
	@echo "Flashing $(BIN_DIR)/$(TARGET).bin to $(MCU)..."
	$(ISPTOOL) $(BIN_DIR)/$(TARGET).bin

# Start een seriële monitor (vereist 'screen' of 'minicom')
.PHONY: monitor
monitor:
	@echo "Starting serial monitor on /dev/ttyUSB0 at 115200 8N1..."
	screen /dev/ttyUSB0 115200

# Maak de build directories aan
.PHONY: directories
directories:
	@mkdir -p $(BUILD_DIR) $(BIN_DIR)

# Ruim alle gegenereerde bestanden op
.PHONY: clean
clean:
	@echo "Cleaning project..."
	rm -rf $(BUILD_DIR)
	rm -rf $(BIN_DIR)

# Toon informatie over het project
.PHONY: info
info:
	@echo "Project: $(TARGET)"
	@echo "MCU: $(MCU)"
	@echo "CPU Freq: $(F_CPU) Hz"
	@echo "Toolchain Prefix: $(PREFIX)"
	@echo "Source Dirs: $(SRC_DIRS)"
	@echo "Include Dirs: $(INC_DIRS)"
	@echo "Sources Found:"
	@echo "  C: $(C_SOURCES)"
	@echo "  ASM: $(ASM_SOURCES)"

##################################################################################################
# AUTOMATISCHE DEPENDENCIES
##################################################################################################
# Include de automatisch gegenereerde dependency files (.d)
# Dit zorgt ervoor dat alleen de bestanden die een header hebben gewijzigd, opnieuw worden gecompileerd.
-include $(OBJECTS:.o=.d)
