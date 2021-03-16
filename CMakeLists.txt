cmake_minimum_required(VERSION 2.8.12)

project(elmo_shm CXX)

##enable_language(CXX)
add_subdirectory(SOEM)

find_package(Boost REQUIRED
             COMPONENTS program_options)
##message("## Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
##message("## Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
## TODO find_package(yaml-cpp)
## TODO find_package(ncurses)
## TODO tinfo-dev??
set(SOURCES src/elmo_shm.cpp src/elmo_common.cpp src/display_shm.cpp src/list_display.cpp src/driver_parameter.cpp src/IIRFilter.cpp)
set(CMAKE_C_FLAGS "-O3 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare -std=c++11")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare -std=c++11")
include_directories(include ${Boost_INCLUDE_DIRS} SOEM/soem SOEM/osal SOEM/osal/linux SOEM/oshw/linux)
link_directories(${Boost_LIBRARY_DIRS})

add_executable(elmo_shm ${SOURCES})
target_link_libraries(elmo_shm soem rt pthread ncurses tinfo yaml-cpp boost_program_options)
install(TARGETS elmo_shm DESTINATION bin)
## apt
# libyaml-cpp-dev
# libncurses5-dev
# libtinfo-dev
# libboost-program-options-dev
