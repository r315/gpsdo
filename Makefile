BUILD_DIR  	:=$(CURDIR)/build
APP_DIR 	:=$(CURDIR)/App

all: dk230

norule:
	@echo "specify board"
	@echo "Available boards:"
	@echo "	gd - 230dk"

clean:
	@${RM} -rf build

dk230:
	@"$(MAKE)" -C target/230dk BUILD_DIR=$(BUILD_DIR)/230dk APP_DIR=$(APP_DIR)

dk230-program:
	@"$(MAKE)" -C target/230dk BUILD_DIR=$(BUILD_DIR)/230dk APP_DIR=$(APP_DIR) program

.PHONY: