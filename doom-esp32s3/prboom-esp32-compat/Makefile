SRC_DIR := .
PREFIX ?= .
OBJ_DIR ?= $(PREFIX)/obj
LIB_DIR ?= $(PREFIX)/lib
INC_DIR := ./include ../prboom/include ../prboom-wad-tables/include

# Specific file to be excluded
EXCLUDE_FILES := psxcontroller.c

# Create a list of files to exclude prefixed with $(SRC_DIR)/
EXCLUDE_FILES_FULL := $(addprefix $(SRC_DIR)/, $(EXCLUDE_FILES))

# Glob all .c files in the source directory
SRC_FILES := $(filter-out $(SRC_DIR)/$(EXCLUDE_FILES), $(wildcard $(SRC_DIR)/*.c))
OBJ_FILES := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC_FILES))

LIBRARY := $(LIB_DIR)/libprboom_esp32s3.a

# Include paths, convert each INC_DIR to -I$(INC_DIR)
INC_FLAGS := $(addprefix -I,$(INC_DIR))
# CFlags
CFLAGS += -Wno-error=char-subscripts -Wno-error=unused-value -Wno-error=parentheses -Wno-error=int-to-pointer-cast -Wno-pointer-sign \
		-Wno-unused-but-set-variable -Wno-maybe-uninitialized -Wno-misleading-indentation

.PHONY: all clean

all: $(LIBRARY)

$(LIBRARY): $(OBJ_FILES)
	mkdir -p $(LIB_DIR)
	$(AR) -rcs $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) $(INC_FLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(LIB_DIR) $(TARGET)