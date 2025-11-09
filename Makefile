# 1. Store compiler info
CC 		 = $(PREFIX)-gcc 		# C compiler
AS 		 = $(PREFIX)-gcc 		# Assembler
LD	 	 = $(PREFIX)-gcc 		# Linker
SIZE	 = $(PREFIX)-size 		# Final size of the program
OBJCOPY	 = $(PREFIX)-objcopy 	# Convert .elf to .bin

# 2. Set Project Name
TARGET = Canards # final name of .elf file

# 3. Retrieve File paths
C_SOURCES=
S_SOURCES=
INC_PATHS=
CFLAGS=
LDFLAGS=
BUILD_DIR=

# 4. Build Logic
C_OBJS = $(patsubst %.c, $(BUILD_DIR)/%.o, $(C_SOURCES))
S_OBJS = $(patsubst %.s, $(BUILD_DIR)/%.o, $(S_SOURCES))
OBJS   = $(C_OBJS) $(S_OBJS) # combine list of C obj and Assembly obj into one giant list

# 5. Targets
all : $(BUILD_DIR)/$(TARGET).elf

clean: 
	@echo "Cleaning build directory"
	@rm -rf build

# 6. Build .elf by linking everything
$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	@echo "Linking Firmware"
	@$(LD) $(LDFLAGS) $(OBJS) -o $@
	@echo "Created $@"

# 7. Compile .c files
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
# 	$< is the .c file
# 	$@ is the .o file
	@echo "Compiling $<" 
	@$(CC) -c $(CFLAGS) $(INC_PATHS) $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "Assembling"
	@$(AS) -c $(CFLAGS) $< -o $@

# 8. Declare all clean are "phony" targets
.PHONY: all clean