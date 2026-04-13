# HMTL_Fire_Control Makefile
#
# Targets:
#   make all       — build all projects
#   make build     — build HMTL_Fire_Control + Wickerman (default envs)
#   make build-all — build all projects including Old

PIO ?= pio

.PHONY: all build 

all: build

build:
	@echo "=== Building HMTL_Fire_Control_Wickerman (firecontroller, touchcontroller) ==="
	cd platformio/HMTL_Fire_Control_Wickerman && $(PIO) run -e firecontroller
	cd platformio/HMTL_Fire_Control_Wickerman && $(PIO) run -e touchcontroller
