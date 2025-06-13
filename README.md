# convex_hull_3d

 g++ script.cpp $(python3-config --includes --ldflags) -o script 
 g++ script.cpp \
    $(python3-config --includes --ldflags) \
    -I$(python -c "import numpy; print(numpy.get_include())") \
    -o script