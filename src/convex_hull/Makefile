INCLUDES := -I../dcel -I../grafo
CXX := g++
CXXFLAGS := -Wall -g
CPP_SRCS := ConvexHull.cpp

TARGET := libconvex_hull.a

OBJECTS := $(CPP_SRCS:.cpp=.o)

all: $(TARGET)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $<

$(TARGET): $(OBJECTS)
	ar rcs $@ $^

clean:
	rm -f $(OBJECTS) $(TARGET)