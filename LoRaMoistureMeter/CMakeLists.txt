CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
project(appBuilder C CXX)

add_custom_target(AppBuild COMMAND make ARDUINO_ARCH=esp32 ARDUINO_VARIANT=esp32 CPU_FREQ=80 SERIAL_PORT=/dev/esp32 -f ../../Esp8266-Arduino-Makefile/espXArduino.mk
		  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}

)

add_subdirectory(test)

# could be done cleaner by 
# add_custom_target(UTBuild COMMAND test/build.sh 
# 		  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
# )
