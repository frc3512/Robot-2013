cmake_minimum_required(VERSION 2.8)
include(FRCDeploy)

project(FRC_UserProgram)
find_package(WPILib)
include_directories(${WPILib_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-O3 -Wall -s -DVxWorks=1")
set(CMAKE_C_FLAGS "-O3 -Wall -s -DVxWorks=1")

set(DIRS
    src/*
)

file(GLOB SRC *.cpp *.c)

foreach(ELEM ${DIRS})
    file(GLOB_RECURSE FILES ${ELEM}.cpp ${ELEM}.c)
    list(APPEND SRC ${FILES})
endforeach(ELEM)

add_executable(FRC_UserProgram.out ${SRC})

target_link_libraries(FRC_UserProgram.out ${WPILib_LIBRARY})

add_make_deploy(FRC_UserProgram.out 10.35.12.2)
