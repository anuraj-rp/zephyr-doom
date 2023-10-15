SRC_DIR := .
PREFIX ?= .
OBJ_DIR ?= $(PREFIX)/obj
LIB_DIR ?= $(PREFIX)/lib
INC_DIR := ./include

# Glob all .c files in the source directory
SRC_FILES := $(wildcard $(SRC_DIR)/*.c)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC_FILES))

LIBRARY := $(LIB_DIR)/libprboom_wad_tables.a

# Include paths
INC_FLAGS := -I$(INC_DIR)
# CFLAGS
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
