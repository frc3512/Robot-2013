# FRC team 3512's 2013 robot

The source code for our 2013 FRC robot named Cynisca.

## Setup

Install the relevant FRC toolchain for your platform (see
https://github.com/wpilibsuite/allwpilib/releases). Make sure the toolchain is
placed in `~/wpilib/2020/roborio` (Linux) or
`C:\Users\Public\wpilib\2020\roborio` (Windows).

Install the following OS packages.

* gcc >= 7.3.0
* python >= 3.6
* rsync (optional, for CSV logging)

Install the following python packages via `pip3 install --user package_name`.

* wpiformat (optional)
  * https://github.com/wpilibsuite/styleguide/blob/main/wpiformat/README.rst

We use the latest version of clang-format with wpiformat for C++ formatting.

## Build options

### Build everything

* `./gradlew build`

This runs a roboRIO and desktop build and runs the desktop tests.

### Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at
10.35.12.2, and restarts it.

### Documentation

* `./gradlew doxygen`

This command generates HTML documentation for the robot code from in-source
Doxygen comments. The results are placed in a `docs/html` folder with an
`index.html` page as the root.

## Autonomous mode selection

Open shuffleboard and select the desired autonomous mode from the dropdown menu.
When the indicator next to the menu turns from a red X to a green checkmark, the
robot has confirmed the selection.

See
[this](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html)
for details on how the robot side works.
