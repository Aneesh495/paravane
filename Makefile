.PHONY: all build test clean lint check-style sim help

JANET ?= janet
PYTHON ?= python3
CC ?= clang
CFLAGS ?= -O3 -Wall -Wextra -fPIC

SRC_JANET = $(shell find src -name "*.janet" 2>/dev/null)
TEST_JANET = $(shell find test -name "*.janet" 2>/dev/null)

all: build test

build:
	@echo "==> Verifying Janet syntax..."
	@for f in $(SRC_JANET); do \
		$(JANET) -c $$f /dev/null || exit 1; \
	done
	@echo "==> Janet syntax verification passed."

test:
	@echo "==> Running test suite..."
	@$(JANET) test/runner.janet

sim:
	@echo "==> Running autonomous subsea mission simulation..."
	@$(JANET) src/paravane/main.janet run-mission examples/survey_deep_ridge.janet

clean:
	@rm -rf build bin *.o *.so *.dylib *.log output/

check-style:
	@echo "==> Verifying code hygiene and character invariants..."
	@$(PYTHON) tools/check_style.py

help:
	@echo "Paravane Build System"
	@echo "  make build       : Verify syntax across all Janet modules"
	@echo "  make test        : Execute all unit, structure, and integration tests"
	@echo "  make sim         : Run end-to-end subsea survey simulation"
	@echo "  make check-style : Enforce clean style and zero em-dash compliance"
	@echo "  make clean       : Remove generated artifacts"
