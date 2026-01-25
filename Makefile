# Modular Makefile with ImGui Support

CXX := g++
STD := -std=c++17
CXXFLAGS = -O3 -Wall -Wextra -ffast-math $(shell sdl2-config --cflags)

# Include directories
INC_DIRS := $(shell find . -type d -name include 2>/dev/null | sed 's|^./||')

CPPFLAGS := $(patsubst %,-I%,$(INC_DIRS))

LDFLAGS = $(shell sdl2-config --libs)

# Sources
SRCS := $(shell find . -name '*.cpp' ! -path './build/*' ! -path './lib/*' -print | sed 's|^./||')

# Objects
OBJS := $(patsubst %.cpp,build/%.o,$(SRCS))

TARGET := sim

.PHONY: all clean show

all: $(TARGET)

$(TARGET): $(OBJS)
	@echo Linking $@
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS) $(LDFLAGS)

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo Compiling $<
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

show:
	@echo "Includes: $(INC_DIRS)"
	@echo "Sources: $(SRCS)"

clean:
	@rm -rf build/ $(TARGET)