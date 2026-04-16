# HMTL_Fire_Control Makefile
#
# Targets:
#   make all         — build all firmware + run all tests
#   make build       — build HMTL_Fire_Control_Wickerman (default envs)
#   make test        — run all tests (Python unit + native C++)
#   make test-python — run Python unit tests only (no hardware required)
#   make test-native — run native C++ unit tests only (no hardware required)

PIO    ?= pio
PYTHON ?= python3

.PHONY: all build test test-python test-native

all: build test

build:
	@echo "=== Building HMTL_Fire_Control_Wickerman (firecontroller, touchcontroller) ==="
	cd platformio/HMTL_Fire_Control_Wickerman && $(PIO) run -e firecontroller
	cd platformio/HMTL_Fire_Control_Wickerman && $(PIO) run -e touchcontroller

test: test-python test-native

test-python:
	@echo "=== Running Python unit tests ==="
	cd python && $(PYTHON) -m pytest tests/unit/ -v

test-native:
	@echo "=== Running native C++ unit tests ==="
	cd platformio/HMTL_Fire_Control_Test && $(PIO) test -e native
