cmake_minimum_required(VERSION 2.8.8)
project(arduino-mock-test-all)

find_package(Threads REQUIRED)
add_subdirectory(arduinoMock)

# =============================== CORE UNIT TEST TARGET ====================================

include_directories(
    ${ARDUINO_MOCK_INCLUDE_DIRS}
    ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest/googletest/include
    ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest/googlemock/include
    ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest/googlemock/include/gmock
    ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest/googletest/include/gtest
)

file(GLOB LIB_SRCS "../*.ino")
file(GLOB SRCS "*.cpp")
add_executable(UTBuild ${SRCS} ${LIB_SRCS})

target_link_libraries(UTBuild
    ${ARDUINO_MOCK_LIBS_DIR}/dist/lib/libarduino_mock.a
    ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest-build/googlemock/gtest/libgtest.a
    ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest-build/googlemock/gtest/libgtest_main.a
    ${ARDUINO_MOCK_LIBS_DIR}/lib/gtest/gtest/src/gtest-build/googlemock/libgmock.a
    ${CMAKE_THREAD_LIBS_INIT}
)

add_dependencies(UTBuild arduinoMock)

install(TARGETS UTBuild DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/build)

# ============================================================================================


enable_testing()
add_test(TestAll UTBuild)
