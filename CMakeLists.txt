cmake_minimum_required(VERSION 2.8.12)

project(elmo_shm CXX)

##enable_language(CXX)

find_package(Boost REQUIRED
             COMPONENTS program_options)
##message("## Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
##message("## Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")

set(SOURCES src/elmo_shm.cpp src/elmo_common.cpp src/display_shm.cpp src/list_display.cpp src/driver_parameter.cpp)
set(CMAKE_C_FLAGS "-O3 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare -std=c++11")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter -Wno-sign-compare -std=c++11")
include_directories(include soem_lib/include/soem ${Boost_INCLUDE_DIRS})
link_directories(soem_lib/lib ${Boost_LIBRARY_DIRS})

add_executable(elmo_shm ${SOURCES})
target_link_libraries(elmo_shm soem rt pthread ncurses tinfo yaml-cpp boost_program_options)
install(TARGETS elmo_shm DESTINATION bin)
## apt
# libyaml-cpp-dev
# libncurses5-dev
# libtinfo-dev
# libboost-program-options-dev

#add_library(eus_shm_interface SHARED euslisp/eus_shm_interface.cpp src/servo_shm.cpp)
#target_link_libraries(eus_shm_interface pthread)
#set_target_properties(eus_shm_interface PROPERTIES COMPILE_FLAG "-fPIC" PREFIX "" SUFFIX ".so" LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)

# TODO
## boost option parser
## simulation-mode without EtherCAT
