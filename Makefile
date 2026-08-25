CXX       ?= c++
CXXFLAGS  ?= -std=c++14 -O2 -Wno-deprecated
CPPFLAGS  ?=
LDFLAGS   ?=
LDLIBS    ?= -lboost_program_options
INCDIR     = inc
SRCDIR     = src

# Linux packages install Boost and dlib under the compiler's normal include
# paths, so no local path is needed.  Set these only for non-system installs:
#   make BOOST_ROOT=/opt/boost DLIB_ROOT=/opt/dlib \
#        BOOST_LIB_DIR=/opt/boost/stage/lib
BOOST_ROOT    ?= $(firstword $(wildcard /Users/jiaqit/local_deps/boost_1_84_0))
BOOST_LIB_DIR ?= $(firstword $(wildcard /Users/jiaqit/local_deps/boost_build/lib))
DLIB_ROOT     ?= $(firstword $(wildcard /Users/jiaqit/local_deps/dlib-19.24))

INCLUDES = -I$(INCDIR)
ifneq ($(strip $(BOOST_ROOT)),)
INCLUDES += -I$(BOOST_ROOT)
endif
ifneq ($(strip $(DLIB_ROOT)),)
INCLUDES += -I$(DLIB_ROOT)
endif
ifneq ($(strip $(BOOST_LIB_DIR)),)
LDFLAGS += -L$(BOOST_LIB_DIR)
endif

HEADERS  = $(wildcard $(INCDIR)/*.h)
LIB_SRCS = $(SRCDIR)/map_loader.cpp $(SRCDIR)/cbs.cpp $(SRCDIR)/path_planners.cpp $(SRCDIR)/simulation.cpp
LIB_OBJS = $(LIB_SRCS:.cpp=.o)
TARGET   = mapd

all: $(TARGET)

$(TARGET): driver.o $(LIB_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

driver.o: driver.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(SRCDIR)/%.o: $(SRCDIR)/%.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(SRCDIR)/*.o driver.o $(TARGET)

.PHONY: all clean
