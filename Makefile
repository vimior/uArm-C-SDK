CC = gcc
CPP = g++
RM = rm -rf

LIBFLAGS = -fPIC -shared -lpthread

#SOURCES
CC_SOURCES = \
uarm/uarm.c
CPP_SOURCES = \
uarm/uarm.cpp

# TARGET
INCLUDE_PATH = './build/include/'
CC_LIB_PATH = ./build/lib/c
CPP_LIB_PATH = ./build/lib/c++
CC_LIB = libuarm.so
CPP_LIB = libuarm.so
COPY_INCLUDE_FILE = copy_include_file
BUILD_CC_LIB = cc_lib_so
BUILD_CPP_LIB = cpp_lib_so

all: $(BUILD_CC_LIB) $(BUILD_CPP_LIB) $(COPY_INCLUDE_FILE)

$(COPY_INCLUDE_FILE):
	mkdir -p $(INCLUDE_PATH)
	cp uarm/uarm.h $(INCLUDE_PATH)

$(BUILD_CC_LIB):
	mkdir -p $(CC_LIB_PATH)
	$(CC) $(CC_SOURCES) $(LIBFLAGS) -o $(CC_LIB_PATH)/$(CC_LIB)
$(BUILD_CPP_LIB):
	mkdir -p $(CPP_LIB_PATH)
	$(CPP) $(CPP_SOURCES) $(LIBFLAGS) -o $(CPP_LIB_PATH)/$(CPP_LIB)
