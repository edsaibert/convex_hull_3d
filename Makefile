CXX := g++
CXXFLAGS := -Wall -g

DCEL_DIR := src/dcel
GRAFO_DIR := src/grafo
CONVEX_HULL_DIR := src/convex_hull

INCLUDES := -I$(DCEL_DIR) -I$(GRAFO_DIR) -I$(CONVEX_HULL_DIR)
LDFLAGS := -L$(DCEL_DIR) -L$(GRAFO_DIR) -L$(CONVEX_HULL_DIR) -lconvex_hull -lmesh -lgraph

MAIN_SRC := main.cpp
TARGET := convex

.PHONY: all dcel grafo convex_hull clean

all: dcel grafo convex_hull $(TARGET)

dcel:
	$(MAKE) -C $(DCEL_DIR)

grafo:
	$(MAKE) -C $(GRAFO_DIR)

convex_hull:
	$(MAKE) -C $(CONVEX_HULL_DIR)

$(TARGET): $(MAIN_SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)
	$(MAKE) -C $(DCEL_DIR) clean
	$(MAKE) -C $(GRAFO_DIR) clean
	$(MAKE) -C $(CONVEX_HULL_DIR) clean