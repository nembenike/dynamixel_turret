CXX      ?= g++
CXXFLAGS ?= -O3 -Wall -Wextra -std=gnu++20 -Wno-unused-result
LIBS     ?= -lraylib -lm -lpthread -ldl -lX11 -ldxl_x64_c -lopencv_core -lopencv_videoio -lopencv_imgproc
INCLUDE  ?= -I/usr/local/include/dynamixel_sdk/ -I/usr/include/opencv4
LIBDIR   ?= -L/usr/local/lib

TARGET      ?= turret
SERIAL_PORT ?= /dev/ttyACM0
PYTHON      ?= python3

.PHONY: all clean run help install-python-deps

all: $(TARGET)

$(TARGET): turret.c
	$(CXX) $(CXXFLAGS) -x c++ turret.c -o $(TARGET) $(INCLUDE) $(LIBDIR) $(LIBS)

run: all
	@echo "Granting serial port access on $(SERIAL_PORT) (best effort)..."
	@sudo chmod a+rw $(SERIAL_PORT) 2>/dev/null || true
	./$(TARGET)

install-python-deps:
	$(PYTHON) -m pip install -r requirements.txt

clean:
	rm -f $(TARGET)
	rm -rf __pycache__

help:
	@echo "Targets:"
	@echo "  make all                 Build turret"
	@echo "  make run                 Build + run turret"
	@echo "  make install-python-deps Install Python requirements"
	@echo "  make clean               Remove build/cache artifacts"
