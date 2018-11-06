project(aush)

set(CMAKE_CXX_STANDARD 11)

cmake_minimum_required(VERSION 3.8)

find_package(OpenCV REQUIRED)

add_executable(
	aush 
	angslu.cpp
	imgprdo.cpp
	main.cpp
	serial.cpp
	angslu.hpp
	imgprdo.hpp
	serial.h
)

target_link_libraries(aush ${OpenCV_LIBS})