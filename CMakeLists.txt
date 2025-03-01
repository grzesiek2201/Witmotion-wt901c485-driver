cmake_minimum_required(VERSION 3.10)
project(wt901c485_cpp)

# Set the Cpp standard
set(CMAKE_CXX_STANDARD 20)



set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#
# Prefer -pthread compiler and linker flag when using libpthread. This must
# be set before call to find_package(Threads).
#
set(THREADS_HAVE_PTHREAD_ARG 1)
find_package(Threads REQUIRED)
#
# If you've installed LibSerial in a non-standard location, please add it
# to PKG_CONFIG_PATH first. For example, if LibSerial is installed under
# /opt/libserial, set PKG_CONFIG_PATH environment variable to 
# /opt/libserial/lib/pkgconfig/ before running cmake for this example.
#
# export PKG_CONFIG_PATH=/opt/libserial/lib/pkgconfig/
#
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)



# Add the executable
add_executable(wt901c485_cpp src/main.cpp)

# Link the serial library
# target_link_libraries(wt901c485_cpp serial)

target_include_directories(wt901c485_cpp PRIVATE ${SERIAL_INCLUDE_DIRS})
target_link_libraries(wt901c485_cpp PRIVATE ${SERIAL_LDFLAGS} ${CMAKE_THREAD_LIBS_INIT})