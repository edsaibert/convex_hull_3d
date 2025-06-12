LDFLAGS += -L./dcel -lmesh -L./grafo -lgraph
INCLUDES += -I./dcel -I./grafo
CXX := g++
CXXFLAGS := -Wall -g
CPP_SRCS := main.cpp ConvexHull.cpp 

TARGET := convex_hull

OBJECTS := $(C_SRCS:.c=.o) $(CPP_SRCS:.cpp=.o)

all: $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $<

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(OBJECTS) $(TARGET)