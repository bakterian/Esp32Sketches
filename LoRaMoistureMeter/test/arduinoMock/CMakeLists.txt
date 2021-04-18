cmake_minimum_required(VERSION 2.8.8)
project(arduino_mock_builder C CXX)
include(ExternalProject)

ExternalProject_Add(arduinoMock
	URL ${CMAKE_CURRENT_SOURCE_DIR}/../../../../arduino-mock
	DOWNLOAD_NO_EXTRACT true
	PREFIX ${CMAKE_CURRENT_BINARY_DIR}/adruinoMock
	INSTALL_COMMAND ""
)

ExternalProject_Get_Property(arduinoMock source_dir)
set(ARDUINO_MOCK_INCLUDE_DIRS ${source_dir}/include/arduino-mock PARENT_SCOPE)

message(arduino-mock source_dir: ${source_dir})

ExternalProject_Get_Property(arduinoMock binary_dir)
set(ARDUINO_MOCK_LIBS_DIR ${binary_dir} PARENT_SCOPE)

message(arduino-mock binary_dir: ${binary_dir})
