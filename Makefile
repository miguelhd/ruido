CXX ?= clang++

# Mandatory embedded C++17 hygiene flags (AGENTS.md & src/dsp/AGENTS.md):
# -Wdouble-promotion + -Werror mathematically forbids accidental float64 usage in DSP code.
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
	src/dsp/Ms20Filter.cpp \
	src/dsp/HalfBandDecimator.cpp \
	src/dsp/DroneEngine.cpp

DSP_OBJS := $(patsubst src/%.cpp,$(BUILD_DIR)/%.o,$(DSP_SRCS))
TEST_BIN := $(BUILD_DIR)/test_dsp

.PHONY: all test tsan ubsan clean

all: $(TEST_BIN)

$(BUILD_DIR)/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(DSP_CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/tests/test_dsp.o: tests/test_dsp.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(TEST_CXXFLAGS) -c $< -o $@

$(TEST_BIN): $(DSP_OBJS) $(BUILD_DIR)/tests/test_dsp.o
	$(CXX) $(TEST_CXXFLAGS) -pthread $^ -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

tsan:
	@mkdir -p $(BUILD_DIR)/tsan/dsp $(BUILD_DIR)/tsan/tests
	@for f in $(DSP_SRCS); do \
		$(CXX) $(DSP_CXXFLAGS) -O1 -g -fsanitize=thread -c $$f -o $(BUILD_DIR)/tsan/$${f#src/}.o || exit 1; \
	done
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=thread -c tests/test_dsp.cpp -o $(BUILD_DIR)/tsan/tests/test_dsp.o
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=thread -pthread \
		$(patsubst src/%.cpp,$(BUILD_DIR)/tsan/%.cpp.o,$(DSP_SRCS)) \
		$(BUILD_DIR)/tsan/tests/test_dsp.o -o $(BUILD_DIR)/test_dsp_tsan
	./$(BUILD_DIR)/test_dsp_tsan

ubsan:
	@mkdir -p $(BUILD_DIR)/ubsan/dsp $(BUILD_DIR)/ubsan/tests
	@for f in $(DSP_SRCS); do \
		$(CXX) $(DSP_CXXFLAGS) -O1 -g -fsanitize=undefined -c $$f -o $(BUILD_DIR)/ubsan/$${f#src/}.o || exit 1; \
	done
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -c tests/test_dsp.cpp -o $(BUILD_DIR)/ubsan/tests/test_dsp.o
	$(CXX) $(TEST_CXXFLAGS) -O1 -g -fsanitize=undefined -pthread \
		$(patsubst src/%.cpp,$(BUILD_DIR)/ubsan/%.cpp.o,$(DSP_SRCS)) \
		$(BUILD_DIR)/ubsan/tests/test_dsp.o -o $(BUILD_DIR)/test_dsp_ubsan
	./$(BUILD_DIR)/test_dsp_ubsan

clean:
	rm -rf $(BUILD_DIR)

-include $(DSP_OBJS:.o=.d) $(BUILD_DIR)/tests/test_dsp.d
