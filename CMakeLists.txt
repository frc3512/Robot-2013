cmake_minimum_required(VERSION 2.8)

project(Discobolus)

include_directories( "${WIND_BASE}/target/h/WPILib" )

add_subdirectory(SFMLNetwork)
add_subdirectory(SFMLSystem)

set(SRC
    AutonomousModes/AutonFeed.cpp
    AutonomousModes/AutonShoot.cpp

    Kinect/DriveKinect.cpp
    Kinect/TurretKinect.cpp
    Kinect/KinectBase.cpp

    AutonContainer.cpp
    Autonomous.cpp
    ButtonTracker.cpp
    Disabled.cpp
    DriverStationDisplay.cpp
    OperatorControl.cpp
    OurRobot.cpp
    Settings.cpp
)

add_executable(FRC_UserProgram.out ${SRC})

target_link_libraries(FRC_UserProgram.out SFMLNetwork SFMLSystem)
