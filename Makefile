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
	src/core/LedEngine.cpp \
	src/core/RuidoController.cpp \
	src/ui/ScopeTraceEngine.cpp \
	src/ui/HudGraphs.cpp \
	src/ui/HudGlobalViews.cpp \
	src/ui/HudChrome.cpp \
	src/ui/ScopeHudRenderer.cpp

ALL_ENGINE_SRCS := $(DSP_SRCS) $(UI_SRCS)

DSP_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(DSP_SRCS))
UI_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(UI_SRCS))

TEST_DSP_BIN := $(BUILD_DIR)/test_dsp
TEST_UI_BIN := $(BUILD_DIR)/test_ui
WASM_OUT := platforms/sim/web/ruido.wasm
MACOS_SIM_BIN := $(BUILD_DIR)/ruido_sim_macos

.PHONY: all check format test wasm sim-macos serve tsan ubsan clean

all: $(TEST_DSP_BIN) $(TEST_UI_BIN) $(WASM_OUT) $(MACOS_SIM_BIN)

check:
	python3 tools/check_hygiene.py

format:
	"$(CLANG_FORMAT)" -i $(wildcard src/core/*.h src/core/*.cpp src/dsp/*.h src/dsp/*.cpp src/ui/*.h src/ui/*.cpp tests/*.h tests/*.cpp)

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

tsan:
	@mkdir -p $(BUILD_DIR)/tsan/core $(BUILD_DIR)/tsan/dsp $(BUILD_DIR)/tsan/ui $(BUILD_DIR)/tsan/tests
	@for f in $(ALL_ENGINE_SRCS); do \
		$(CXX) $(DSP_CXXFLAGS) -O1 -g -fsanitize=thread -c $$f -o $(BUILD_DIR)/tsan/$${f#src/}.o || exit 1; \
	done
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=thread -c tests/test_dsp.cpp -o $(BUILD_DIR)/tsan/tests/test_dsp.o
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=thread -pthread \
		$(patsubst src/%.cpp,$(BUILD_DIR)/tsan/%.cpp.o,$(DSP_SRCS)) \
		$(BUILD_DIR)/tsan/tests/test_dsp.o -o $(BUILD_DIR)/test_dsp_tsan
	./$(BUILD_DIR)/test_dsp_tsan

ubsan:
	@mkdir -p $(BUILD_DIR)/ubsan/core $(BUILD_DIR)/ubsan/dsp $(BUILD_DIR)/ubsan/ui $(BUILD_DIR)/ubsan/tests
	@for f in $(ALL_ENGINE_SRCS); do \
		$(CXX) $(DSP_CXXFLAGS) -O1 -g -fsanitize=undefined -c $$f -o $(BUILD_DIR)/ubsan/$${f#src/}.o || exit 1; \
	done
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -c tests/test_dsp.cpp -o $(BUILD_DIR)/ubsan/tests/test_dsp.o
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -c tests/test_ui.cpp -o $(BUILD_DIR)/ubsan/tests/test_ui.o
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -pthread \
		$(patsubst src/%.cpp,$(BUILD_DIR)/ubsan/%.cpp.o,$(DSP_SRCS)) \
		$(BUILD_DIR)/ubsan/tests/test_dsp.o -o $(BUILD_DIR)/test_dsp_ubsan
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -pthread \
		$(patsubst src/%.cpp,$(BUILD_DIR)/ubsan/%.cpp.o,$(ALL_ENGINE_SRCS)) \
		$(BUILD_DIR)/ubsan/tests/test_ui.o -o $(BUILD_DIR)/test_ui_ubsan
	./$(BUILD_DIR)/test_dsp_ubsan
	./$(BUILD_DIR)/test_ui_ubsan

clean:
	rm -rf $(BUILD_DIR) $(WASM_OUT)

-include $(DSP_OBJS:.o=.d) $(UI_OBJS:.o=.d) $(BUILD_DIR)/tests/test_dsp.d $(BUILD_DIR)/tests/test_ui.d
