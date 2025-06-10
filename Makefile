LDFLAGS += -L./dcel -lmesh
INCLUDES += -I./dcel
CXX := g++
CXXFLAGS := -Wall -g
CPP_SRCS := main.cpp 

TARGET := convex_hull

OBJECTS := $(C_SRCS:.c=.o) $(CPP_SRCS:.cpp=.o)

all: $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $<

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(OBJECTS) $(TARGET)