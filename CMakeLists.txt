cmake_minimum_required (VERSION 2.6)
project (Upsurge CXX)

enable_language(C)

# The version number.
# configure a header file to pass some of the CMake settings
# to the source code

# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h
include_directories("${PROJECT_SOURCE_DIR}")
find_package(Threads REQUIRED)
set(Boost_USE_MULTITHREADED ON)
find_package(OpenCV REQUIRED) 
find_package(realsense2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_executable(upsurge "${PROJECT_SOURCE_DIR}/main.cpp" "${PROJECT_SOURCE_DIR}/Upsurge.cpp" "${PROJECT_SOURCE_DIR}/base64.cpp")

add_executable(hsv "${PROJECT_SOURCE_DIR}/HSV.cpp")

target_include_directories(hsv PRIVATE ${realsense_INCLUDE_DIR})

target_link_libraries(hsv ${OpenCV_LIBS} ${realsense2_LIBRARY})

target_include_directories(upsurge PRIVATE ${realsense_INCLUDE_DIR})
target_include_directories(upsurge PRIVATE ${Boost_INCLUDE_DIRS})

target_link_libraries(upsurge ${OpenCV_LIBS} ${realsense2_LIBRARY} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
