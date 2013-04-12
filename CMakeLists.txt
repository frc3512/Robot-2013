cmake_minimum_required(VERSION 2.8)
include(FRCDeploy)

project(Discobolus)
find_package(WPILib)
include_directories(${WPILib_INCLUDE_DIRS})

file(GLOB_RECURSE SRC *.cpp *.c)

add_executable(Discobolus.out ${SRC})

target_link_libraries(Discobolus.out ${WPILib_LIBRARY})

add_make_deploy(Discobolus.out 10.35.12.2)
