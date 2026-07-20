CXX      = c++
CXXFLAGS = -std=c++14 -O2 -Wno-deprecated
INCDIR   = inc
SRCDIR   = src
BOOST    = /Users/jiaqit/local_deps/boost_1_84_0
BOOST_LIB = /Users/jiaqit/local_deps/boost_build/lib
DLIB     = /Users/jiaqit/local_deps/dlib-19.24

INCLUDES = -I$(INCDIR) -I$(BOOST) -I$(DLIB)
LDFLAGS  = -L$(BOOST_LIB) -lboost_program_options

HEADERS  = $(wildcard $(INCDIR)/*.h)
LIB_SRCS = $(SRCDIR)/map_loader.cpp $(SRCDIR)/cbs.cpp $(SRCDIR)/simulation.cpp $(SRCDIR)/ref_solve.cpp
LIB_OBJS = $(LIB_SRCS:.cpp=.o)
TARGET   = mapd

all: $(TARGET)

$(TARGET): driver.o $(LIB_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

driver.o: driver.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(SRCDIR)/%.o: $(SRCDIR)/%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(SRCDIR)/*.o driver.o $(TARGET)

.PHONY: all clean
