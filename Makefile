# HMTL_Fire_Control Makefile
#
# Targets:
#   make all              — build all firmware + run all tests + coverage
#   make build            — build HMTL_Fire_Control_Wickerman (default envs)
#   make test             — run all tests (Python unit + native C++)
#   make test-python      — run Python unit tests only (no hardware required)
#   make test-native      — run native C++ unit tests only (no hardware required)
#   make coverage         — run Python + C++ coverage and print summaries
#   make coverage-python  — Python coverage via pytest-cov
#   make coverage-native  — C++ coverage via LLVM instrumentation + llvm-cov

PIO    ?= pio
PYTHON ?= $(HOME)/.platformio/penv/bin/python3

NATIVE_DIR       := platformio/HMTL_Fire_Control_Test
COVERAGE_PROFRAW := /tmp/hmtl_fc_coverage
COVERAGE_PROFDATA := /tmp/hmtl_fc_coverage.profdata
COVERAGE_BINARY  := $(NATIVE_DIR)/.pio/build/native_coverage/program

.PHONY: all build test test-python test-native coverage coverage-python coverage-native

all: build test coverage

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
	cd $(NATIVE_DIR) && $(PIO) test -e native

coverage-python:
	@echo "=== Python coverage ==="
	cd python && $(PYTHON) -m pytest tests/unit/ --cov=. --cov-report=term-missing

coverage-native:
	@echo "=== C++ native coverage ==="
	rm -f $(COVERAGE_PROFRAW)-*.profraw
	cd $(NATIVE_DIR) && LLVM_PROFILE_FILE="$(COVERAGE_PROFRAW)-%p.profraw" $(PIO) test -e native_coverage
	xcrun llvm-profdata merge -sparse $(COVERAGE_PROFRAW)-*.profraw -o $(COVERAGE_PROFDATA)
	@echo "--- C++ Coverage Summary ---"
	xcrun llvm-cov report $(COVERAGE_BINARY) \
	    -instr-profile=$(COVERAGE_PROFDATA) \
	    -ignore-filename-regex='stubs/|test/|unity'

coverage: coverage-python coverage-native
	@echo ""
	@echo "============================================================"
	@echo "=== Combined Coverage Summary: All Files               ==="
	@echo "============================================================"
	@echo "--- Python ---"
	cd python && $(PYTHON) -m coverage report
	@echo "--- C++ ---"
	xcrun llvm-cov report $(COVERAGE_BINARY) \
	    -instr-profile=$(COVERAGE_PROFDATA) \
	    -ignore-filename-regex='stubs/|test/|unity'
