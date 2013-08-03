cmake_minimum_required(VERSION 2.8)
include(FRCDeploy)

project(Cynisca)
find_package(WPILib)
include_directories(${WPILib_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-O3 -Wall -s -std=c++11 -DVxWorks=1")
set(CMAKE_C_FLAGS "-O3 -Wall -s -std=c11 -DVxWorks=1")

set(DIRS
    AutonomousModes/*
    KalmanGyro/*
    LiveGrapherHost/*
    SFMLNetwork/*
    SFMLSystem/*
    Subsystems/*
)

file(GLOB SRC *.cpp *.c)

foreach(ELEM ${DIRS})
    file(GLOB_RECURSE FILES ${ELEM}.cpp ${ELEM}.c)
    list(APPEND SRC ${FILES})
endforeach(ELEM)

add_executable(Cynisca.out ${SRC})

target_link_libraries(Cynisca.out ${WPILib_LIBRARY})

add_make_deploy(Cynisca.out 10.35.12.2)
