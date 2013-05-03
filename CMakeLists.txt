cmake_minimum_required(VERSION 2.8)
include(FRCDeploy)

project(Cynisca)
find_package(WPILib)
include_directories(${WPILib_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-O3 -s -std=c++11")

set(DIRS
    AutonomousModes/*
    KalmanGyro/*
    LiveGrapherHost/*
    Subsystems/*
    SFMLNetwork/*
    SFMLSystem/*
)

file(GLOB SRC *.cpp *.c)

foreach(ELEM ${DIRS})
    file(GLOB_RECURSE FILES ${ELEM}.cpp ${ELEM}.c)
    list(APPEND SRC ${FILES})
endforeach(ELEM)

add_executable(Cynisca.out ${SRC})

target_link_libraries(Cynisca.out ${WPILib_LIBRARY})

add_make_deploy(Cynisca.out 10.35.12.2)
