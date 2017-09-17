cmake_minimum_required(VERSION 2.8)
project(read_imu)
#set complier
#set(CMAKE_CXX_COMPILER "x86_64-poky-linux-g++")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -march=core2 -mtune=core2 -msse3 -mfpmath=sse --sysroot=/opt/poky/1.8.1/sysroots/core2-64-poky-linux")
####  new
set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -I ../new-mavlink/common -I ../Eigen -I ../assimp")

add_definitions(-DCOMPILEDWITHC11)

##############################
#find_package(Eigen3 REQUIRED)
#find_package(PkgConfig)
#pkg_search_module(Eigen3 REQUIRED eigen3)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST}) 

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS -pthread LINK_FLAGS -pthread)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
#######new new
target_link_libraries(${PROJECT_NAME} glut)
target_link_libraries(${PROJECT_NAME} GL)
target_link_libraries(${PROJECT_NAME} GLU)
target_link_libraries(${PROJECT_NAME} GLEW)
target_link_libraries(${PROJECT_NAME} assimp)
