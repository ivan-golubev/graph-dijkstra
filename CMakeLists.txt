cmake_minimum_required(VERSION 3.30)
project(graphtests
	VERSION 0.1.0
	DESCRIPTION "Testing graph algos in console"
	LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(graphtests src/main.cpp)
