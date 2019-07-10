g++ -c soccer_lib.cpp
g++ -c debug.cpp
g++ -o debug debug.o soccer_lib.o -lrcsc_geom

rm -rf *.o
