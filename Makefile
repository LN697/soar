# Modular Makefile

CXX := g++
STD := -std=c++17
CXXFLAGS = $(STD) -O3 -Wall -Wextra -ffast-math

# Include directories
INC_DIRS := $(shell find . -type d -name include 2>/dev/null | sed 's|^./||')

CPPFLAGS := $(patsubst %,-I%,$(INC_DIRS))

# Sources
SRCS := $(shell find . -name '*.cpp' ! -path './build/*' ! -path './lib/*' ! -path './test/*' -print | sed 's|^./||')

# Objects
OBJS := $(patsubst %.cpp,build/%.o,$(SRCS))

TARGET := sim

.PHONY: all clean show
all: $(TARGET)

$(TARGET): $(OBJS)
	@echo Linking $@
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo Compiling $<
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) -c $< -o $@

show:
	@echo "Includes: $(INC_DIRS)"
	@echo "Sources: $(SRCS)"

clean:
	@rm -rf build/ $(TARGET)

.PHONY: test
test:
	@echo Tests Blocked
# 	@echo Testing Battery
# 	g++ -std=c++17 $(CPPFLAGS) drone/src/battery.cpp test/battery/test_runner.cpp -o build/battery_test
# 	python3 test/battery/stress_test.py build/battery_test
# 	@echo Testing Propulsion
# 	g++ -std=c++17 $(CPPFLAGS) drone/src/propeller.cpp drone/src/motor.cpp test/propulsion/test_runner.cpp -o build/propulsion_test
# 	python3 test/propulsion/stress_test.py build/propulsion_test
# 	@echo Testing ESC
# 	g++ -std=c++17 $(CPPFLAGS) drone/src/esc.cpp drone/src/propeller.cpp drone/src/motor.cpp drone/src/battery.cpp test/esc/test_runner.cpp -o build/esc_test
# 	python3 test/esc/stress_test.py build/esc_test
# 	@echo Testing FC
# 	g++ -std=c++17 $(CPPFLAGS) drone/src/controller.cpp pid/src/pid.cpp test/fc/test_runner.cpp -o build/fc_test
# 	python3 test/fc/stress_test.py build/fc_test
# 	@echo Testing IMU
# 	g++ -std=c++17 $(CPPFLAGS) drone/src/imu.cpp test/imu/test_runner.cpp -o build/imu_test
# 	python3 test/imu/stress_test.py build/imu_test