cmake_minimum_required(VERSION 3.17)
project(DA_proj2)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(DA_proj2 main.cpp
        src/interface.h
        src/interface.cpp
        src/graph.h
        src/graphBuilder.h
        src/graphBuilder.cpp
        src/MutablePriorityQueue.h
        src/algorithms.cpp
        src/algorithms.h)