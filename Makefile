CC=gcc
CXX=g++
CPPFLAGS= -Wall -g -c -fPIC -std=c++14 
OPENCV=$(shell pkg-config opencv --cflags --libs)
OBJS= ./obj/*.o
TARGET= ./bin/*

./bin/camera_calibration : ./obj/camera_calibration.cpp.o
	$(CXX) -o $@ $^ $(OPENCV) 

./bin/stereo_camera_calibration : .obj/stereo_camera_calibration.cpp.o
	$(CXX) -o $@ $^ $(OPENCV) 

./obj/camera_calibration.cpp.o : ./src/camera_calibration.cpp
	$(CXX) $(CPPFLAGS) -o $@ $< $(OPENCV) 

./obj/stereo_camera_calibration.cpp.o : ./src/stereo_camera_calibration.cpp
	$(CXX) $(CPPFLAGS) -o $@ $< $(OPENCV) 


clean :
	rm -rf $(OBJS)
	rm $(TARGET)




