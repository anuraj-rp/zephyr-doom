SRC_DIR := .
PREFIX ?= .
OBJ_DIR ?= $(PREFIX)/obj
LIB_DIR ?= $(PREFIX)/lib

# Glob all .c files in the source directory
SRC_FILES := $(wildcard $(SRC_DIR)/*.c)
OBJ_FILES := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC_FILES))

LIBRARY := $(LIB_DIR)/libprboom_wad_tables.a

.PHONY: all clean

all: $(LIBRARY)

$(LIBRARY): $(OBJ_FILES)
	mkdir -p $(LIB_DIR)
	$(AR) -rcs $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR) $(LIB_DIR) $(TARGET)
