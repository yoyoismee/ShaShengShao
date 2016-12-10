cmake_minimum_required(VERSION 2.6)
project(HitAlert)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(VERSION_MAJOR 0)
set(VERSION_MINOR 9)
set(VERSION_PATCH 0)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "build")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(OpenCV REQUIRED)

include_directories("inc")
include_directories("$ENV{OPENCV_BUILD}/include")

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "inc/*.hpp")

add_executable(HitAlert ${SOURCES} ${HEADERS})
target_link_libraries( DisplayImage ${OpenCV_LIBS} )

#target_link_libraries(HitAlert debug C:/opencv310/opencv/build/x64/vc14/lib/opencv_world310d.lib)
#target_link_libraries(HitAlert general C:/opencv310/opencv/build/x64/vc14/lib/opencv_world310.lib)
#target_link_libraries(HitAlert optimized C:/opencv310/opencv/build/x64/vc14/lib/opencv_world310.lib)


#FIND_LIBRARY(OCV NAMES opencv_world310.lib PATHS "$ENV{OPENCV_BUILD}/x64/vc14/lib")
#FIND_LIBRARY(OCV_D NAMES opencv_world310d.lib PATHS "$ENV{OPENCV_BUILD}/x64/vc14/lib")

#link_directories("$ENV{OPENCV_BUILD}/x64/vc14/lib")

#target_link_libraries(HitAlert debug $ENV{OPENCV_BUILD}/opencv_world310d.lib)
#target_link_libraries(HitAlert optimized $ENV{OPENCV_BUILD}/opencv_world310.lib)

