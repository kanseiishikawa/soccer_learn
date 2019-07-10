BOOST_CPPFLAGS = -I/usr/local/include
BOOST_LDFLAGS = -L/usr/local/lib
LDFLAGS =  -L/usr/local/lib
LIBRCSCLIB = /usr/local/lib
IBS = -lrcsc_agent -lrcsc_ann -lrcsc_net -lrcsc_time -lrcsc_param -lrcsc_gz -lrcsc_rcg -lrcsc_geom -lz -lm

CXX = g++

debug:debug.cpp soccer_lib.cpp

clean:
