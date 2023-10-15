SRC_DIR := .
PREFIX ?= .
OBJ_DIR ?= $(PREFIX)/obj
LIB_DIR ?= $(PREFIX)/lib
INC_DIR := ./include ../prboom-wad-tables/include/

# Glob all .c files in the source directory
SRC_FILES := $(wildcard $(SRC_DIR)/*.c)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC_FILES))

LIBRARY := $(LIB_DIR)/libprboom.a
# Include paths
INC_FLAGS := -I$(INC_DIR)
# CFlags
CFLAGS += -Wno-error=char-subscripts -Wno-error=unused-value -Wno-error=parentheses -Wno-error=int-to-pointer-cast -Wno-pointer-sign \
		-Wno-unused-but-set-variable -Wno-maybe-uninitialized
		
.PHONY: all clean

all: $(LIBRARY)

$(LIBRARY): $(OBJ_FILES)
	mkdir -p $(LIB_DIR)
	$(AR) -rcs $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(OBJ_DIR)
	$(CC) $(INC_FLAGS) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(LIB_DIR) $(TARGET)
