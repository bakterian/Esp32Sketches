# LoRa Mositure Meter
Simple moisture meter build using a Heltec Lora 32 MCU.

# Operation Modes
* Calibration Mode
  The moisture sensors must be calibrated from time to time.
 
* Conitnous measurment mode
  In this mode the moisture measurnents are sent using LoRa
  Contines a description of the measured plant and a moisture threshold value

* Ad-hoc mesurment mode (defult setting)
  In this mode the measurment results are frequently displayed on the OLED display

# Prerequisites
A Esp8266-Arduino-Makefile cloned fork is required.
After cloning the fork a installaton (as root) must be issued.
Some of the binaries that will likely not be present with a typical OS distrubtion are:
 - jq
 - make
 - cmake
In the final step of the build process you may run in to problem with python imports.
To overcome those problems install pip (for the python 2.7).
Afterwards install a python module called pyserial:
```bassh
$ pip install pyserial
```
Also check/adjust the config.json file in the Esp8266-Arduino-Makefile project as it
contains configuration data refernced by this project 
  
# Installation
Just run the installLibraries script which will fetch and unpack the third-party arduino libraries
```bassh
# bash installLibraries.bash
```

# Building
The build process is managed using cmake.
Run the following to generate, configure cmake (after navigating to this folder):
```bassh
$ cmake . 
```
Run the following command to build the application elf and bin files.
```bassh
$ cmake --build . --target AppBuild
```
If a adjustment or change of the arduino compilation aruments is needed just modify the CMakeLists.txt


# Unit Testing
To be added.


