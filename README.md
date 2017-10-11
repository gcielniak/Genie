# Genie
RGBN camera drivers using the GenICam interface

## Prerequisites
- [Boost](http://www.boost.org/users/history/version_1_65_1.html) (tested +1.60)
- [OpenCV](https://opencv.org/releases.html) (tested +3.x)
- [Common Vision Blox](https://www.commonvisionblox.com/en/cvb-download/) (tested +2016 SP1, 12.1)

## Building
The following setup was built using CMake (3.3.0) and Visual Studio 2015 on Windows 10:
- set up the project: `mkdir build & cd build & cmake .. -G "Visual Studio 14 2015 Win64"`;
- to quickly build the program from a command line: `cmake --build . --config release`.

## Testing
Connect your RGBN camera (AD-130GE, GenieNano) and set it up using the GenICamBrowser application. The example app is included in `Apps/GenieApp`.
