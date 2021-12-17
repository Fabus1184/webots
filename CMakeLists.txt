cmake_minimum_required(VERSION 3.20)
project(my_project C)
include_directories("/opt/webots/include/controller/c")

set(CMAKE_C_STANDARD 11)

add_executable(my_project controllers/lab4/lab4.c controllers/lab4/tools.c controllers/lab4/tools.h controllers/lab4/test.c controllers/lab5/test.cpp)
