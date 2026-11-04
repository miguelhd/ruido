CXX ?= clang++
CLANG_FORMAT := $(shell command -v clang-format 2>/dev/null || xcrun --find clang-format 2>/dev/null)
NODE ?= $(shell command -v node 2>/dev/null || ls -1 $$HOME/.nvm/versions/node/*/bin/node 2>/dev/null | tail -n 1)

# Mandatory embedded C++17 hygiene flags (AGENTS.md & src/dsp/AGENTS.md):
# -Wdouble-promotion + -Werror mathematically forbids accidental float64 usage in firmware code.
DSP_CXXFLAGS := \
	-std=c++17 \
	-O2 \
	-Wall \
	-Wextra \
	-Werror \
	-Wdouble-promotion \
	-Wshadow \
	-fno-exceptions \
	-fno-rtti \
	-fno-math-errno \
	-ffp-contract=off \
	-MMD -MP \
	-Isrc

TEST_CXXFLAGS := \
	-std=c++17 \
	-O2 \
	-Wall \
	-Wextra \
	-Werror \
	-fno-exceptions \
	-fno-rtti \
	-MMD -MP \
	-Isrc

BUILD_DIR := build

DSP_SRCS := \
	src/dsp/SchmittOscillatorBank.cpp \
	src/dsp/VoiceLfoBank.cpp \
	src/dsp/Ms20Filter.cpp \
	src/dsp/HalfBandDecimator.cpp \
	src/dsp/DroneEngine.cpp

UI_SRCS := \
	src/core/GlobalControl.cpp \
	src/core/KeyboardLiveMixer.cpp \
	src/core/KeyboardEngine.cpp \
	src/core/LedEngine.cpp \
	src/core/SnapshotBank.cpp \
	src/core/RuidoController.cpp \
	src/ui/ScopeTraceEngine.cpp \
	src/ui/HudAmpGraph.cpp \
	src/ui/HudGraphs.cpp \
	src/ui/HudGlobalViews.cpp \
	src/ui/HudKeybedView.cpp \
	src/ui/HudChrome.cpp \
	src/ui/ScopeHudRenderer.cpp

ALL_ENGINE_SRCS := $(DSP_SRCS) $(UI_SRCS)

DSP_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(DSP_SRCS))
UI_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(UI_SRCS))

UBSAN_DSP_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/ubsan/%.o,$(DSP_SRCS))
UBSAN_UI_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/ubsan/%.o,$(UI_SRCS))
TSAN_DSP_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/tsan/%.o,$(DSP_SRCS))

TEST_DSP_BIN := $(BUILD_DIR)/test_dsp
TEST_UI_BIN := $(BUILD_DIR)/test_ui
TEST_DSP_UBSAN_BIN := $(BUILD_DIR)/test_dsp_ubsan
TEST_UI_UBSAN_BIN := $(BUILD_DIR)/test_ui_ubsan
TEST_DSP_TSAN_BIN := $(BUILD_DIR)/test_dsp_tsan
WASM_OUT := platforms/sim/web/ruido.wasm
MACOS_SIM_BIN := $(BUILD_DIR)/ruido_sim_macos

.PHONY: all check format test test-ui test-dsp wasm sim-macos serve tsan ubsan ubsan-ui clean

all: $(TEST_DSP_BIN) $(TEST_UI_BIN) $(WASM_OUT) $(MACOS_SIM_BIN)

check:
	python3 tools/check_hygiene.py

format:
	@"$(CLANG_FORMAT)" -i $(wildcard src/core/*.h src/core/*.cpp src/dsp/*.h src/dsp/*.cpp src/ui/*.h src/ui/*.cpp tests/*.h tests/*.cpp)

$(BUILD_DIR)/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(DSP_CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/tests/%.o: tests/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(TEST_CXXFLAGS) -c $< -o $@

$(TEST_DSP_BIN): $(DSP_OBJS) $(BUILD_DIR)/tests/test_dsp.o
	$(CXX) $(TEST_CXXFLAGS) -pthread $^ -o $@

$(TEST_UI_BIN): $(DSP_OBJS) $(UI_OBJS) $(BUILD_DIR)/tests/test_ui.o
	$(CXX) $(TEST_CXXFLAGS) -pthread $^ -o $@

test-ui: check $(TEST_UI_BIN) $(WASM_OUT)
	./$(TEST_UI_BIN)
	@if [ -n "$(NODE)" ]; then "$(NODE)" tests/verify_wasm.mjs; fi

test-dsp: check $(TEST_DSP_BIN)
	./$(TEST_DSP_BIN)

test: check $(TEST_DSP_BIN) $(TEST_UI_BIN) $(WASM_OUT) $(MACOS_SIM_BIN)
	./$(TEST_DSP_BIN)
	./$(TEST_UI_BIN)
	./$(MACOS_SIM_BIN) --headless-check
	@if [ -n "$(NODE)" ]; then "$(NODE)" tests/verify_wasm.mjs; fi

wasm: $(WASM_OUT)

$(WASM_OUT): platforms/sim/wasm_bridge.cpp $(ALL_ENGINE_SRCS) \
	$(wildcard src/core/*.h src/dsp/*.h src/ui/*.h platforms/sim/wasm_libc/*)
	@mkdir -p $(dir $@)
	$(CXX) --target=wasm32 -std=c++17 -O3 \
		-Wall -Wextra -Werror -Wdouble-promotion -Wshadow \
		-fno-exceptions -fno-rtti -fno-math-errno -ffp-contract=off \
		-nostdinc -nostdinc++ -nostdlib \
		-isystem platforms/sim/wasm_libc -Isrc \
		-c platforms/sim/wasm_bridge.cpp -o $@

sim-macos: $(MACOS_SIM_BIN)

$(MACOS_SIM_BIN): $(DSP_OBJS) $(UI_OBJS) platforms/sim/macos_sim.cpp
	@mkdir -p $(dir $@)
	$(CXX) -std=c++17 -O2 -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
		-Isrc $(DSP_OBJS) $(UI_OBJS) platforms/sim/macos_sim.cpp \
		-framework AudioToolbox -framework CoreFoundation -pthread -o $@

serve: $(WASM_OUT)
	@echo "Ruido Web Simulator running at http://127.0.0.1:8080 (Cache-Control: no-store)"
	python3 tools/serve_web.py 8080

$(BUILD_DIR)/tsan/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(DSP_CXXFLAGS) -O1 -g -fsanitize=thread -c $< -o $@

$(BUILD_DIR)/tsan/tests/%.o: tests/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=thread -c $< -o $@

$(TEST_DSP_TSAN_BIN): $(TSAN_DSP_OBJS) $(BUILD_DIR)/tsan/tests/test_dsp.o
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=thread -pthread $^ -o $@

tsan: $(TEST_DSP_TSAN_BIN)
	./$(TEST_DSP_TSAN_BIN)

$(BUILD_DIR)/ubsan/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(DSP_CXXFLAGS) -O1 -g -fsanitize=undefined -c $< -o $@

$(BUILD_DIR)/ubsan/tests/%.o: tests/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -c $< -o $@

$(TEST_DSP_UBSAN_BIN): $(UBSAN_DSP_OBJS) $(BUILD_DIR)/ubsan/tests/test_dsp.o
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -pthread $^ -o $@

$(TEST_UI_UBSAN_BIN): $(UBSAN_DSP_OBJS) $(UBSAN_UI_OBJS) $(BUILD_DIR)/ubsan/tests/test_ui.o
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -pthread $^ -o $@

ubsan-ui: $(TEST_UI_UBSAN_BIN)
	./$(TEST_UI_UBSAN_BIN)

ubsan: $(TEST_DSP_UBSAN_BIN) $(TEST_UI_UBSAN_BIN)
	./$(TEST_DSP_UBSAN_BIN)
	./$(TEST_UI_UBSAN_BIN)

clean:
	rm -rf $(BUILD_DIR) $(WASM_OUT)

-include $(DSP_OBJS:.o=.d) $(UI_OBJS:.o=.d) $(BUILD_DIR)/tests/test_dsp.d $(BUILD_DIR)/tests/test_ui.d \
	$(UBSAN_DSP_OBJS:.o=.d) $(UBSAN_UI_OBJS:.o=.d) $(BUILD_DIR)/ubsan/tests/test_dsp.d $(BUILD_DIR)/ubsan/tests/test_ui.d \
	$(TSAN_DSP_OBJS:.o=.d) $(BUILD_DIR)/tsan/tests/test_dsp.d
