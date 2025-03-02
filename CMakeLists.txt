cmake_minimum_required(VERSION 3.10)
project(wt901c485_cpp)

# Set the Cpp standard
set(CMAKE_CXX_STANDARD 20)



set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Threads REQUIRED)
find_library(SERIAL_LIB serial)

include_directories(${CMAKE_SOURCE_DIR}/include)
# Add the executable
add_executable(wt901c485_cpp src/main.cpp src/main_driver.cpp src/driver.cpp)
# add_executable(wt901c485_cpp_driver src/main_driver.cpp)

# Link the serial library
# target_link_libraries(wt901c485_cpp serial)


target_include_directories(wt901c485_cpp PRIVATE /usr/include include/)
target_link_libraries(wt901c485_cpp PRIVATE ${SERIAL_LIB} ${CMAKE_THREAD_LIBS_INIT})