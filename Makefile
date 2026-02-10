CXX = clang++
# Unused: warn, but annoying to block compilation on
# Sign compare: noisy
# Command line arg: noisy, not relevant to students
CXXFLAGS = \
	-Wall -Wextra -Werror \
	-Wno-error=unused-function \
	-Wno-error=unused-parameter \
	-Wno-error=unused-variable \
	-Wno-error=unused-but-set-variable \
	-Wno-error=unused-value \
	-Wno-sign-compare \
	-Wno-unused-command-line-argument \
	-std=c++2a -I. -O2 -g -fno-omit-frame-pointer \
	-fsanitize=address,undefined

ENV_VARS = ASAN_OPTIONS=detect_leaks=1 LSAN_OPTIONS=suppressions=suppr.txt:print_suppressions=false

# On Ubuntu and WSL, googletest is installed to /usr/include or
# /usr/local/include, which are used by default.

# On Mac, we need to manually include them in our path. Brew installs to
# different locations on Intel/Silicon, so ask brew where things live.
UNAME := $(shell uname)
ifeq ($(UNAME), Darwin)
	GTEST_PREFIX := $(shell brew --prefix googletest)
	LLVM_PREFIX := $(shell brew --prefix llvm)
	CXX := $(LLVM_PREFIX)/bin/clang++
	CXXFLAGS += -I$(GTEST_PREFIX)/include
	CXXFLAGS += -L$(GTEST_PREFIX)/lib
	CXXFLAGS += -L$(LLVM_PREFIX)/lib/c++
	CXXFLAGS += -Wno-vector-conversion
endif

build/%.o: tests/%.cpp graph.h
	mkdir -p build && $(CXX) $(CXXFLAGS) -c $< -o $@

build/%.o: %.cpp graph.h
	mkdir -p build && $(CXX) $(CXXFLAGS) -c $< -o $@

HEADERS := $(wildcard *.h)

TEST_FILES := $(wildcard tests/*.cpp)
TEST_NAMES := $(basename $(notdir $(TEST_FILES)))
TEST_OBJS := $(addprefix build/,$(addsuffix .o,$(TEST_NAMES)))

SOURCE_FILES := $(wildcard *.cpp)
SOURCES := $(filter-out main server,$(basename $(SOURCE_FILES)))
SOURCE_OBJS := $(addprefix build/,$(addsuffix .o,$(SOURCES)))

# Tests

osm_tests: $(TEST_OBJS) $(SOURCE_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(TEST_OBJS) $(SOURCE_OBJS)  -lgtest -lgmock -lgtest_main -o $@

test_graph: osm_tests
	$(ENV_VARS) ./$< --gtest_color=yes --gtest_filter="Graph*"

test_build_graph: osm_tests
	$(ENV_VARS) ./$< --gtest_color=yes --gtest_filter="BuildGraph*"

test_dijkstra: osm_tests
	$(ENV_VARS) ./$< --gtest_color=yes --gtest_filter="Dijkstra*"

test_all: osm_tests
	$(ENV_VARS) ./$< --gtest_color=yes

osm_main:  $(SOURCE_OBJS) build/main.o $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCE_OBJS) build/main.o -o $@

run_osm: osm_main
	$(ENV_VARS) ./$<

osm_server: $(SOURCE_OBJS) build/server.o $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCE_OBJS) build/server.o -o $@

run_server: osm_server
	$(ENV_VARS) ./$<

clean:
	rm -f osm_main osm_server osm_tests build/*
	# MacOS symbol cleanup
	rm -rf *.dSYM

.PHONY: clean test_all test_graph test_build_graph test_dijkstra run_osm
