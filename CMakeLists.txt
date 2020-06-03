cmake_minimum_required(VERSION 2.8.3)
project(delta_1a_lidar)

set(DELTA_1A_LIDAR_SDK_PATH "./src/")

FILE(GLOB DELTA_1A_LIDAR_SDK_SRC 
  "${DELTA_1A_LIDAR_SDK_PATH}/src/*.cpp"
)

include_directories(
  ${DELTA_1A_LIDAR_SDK_PATH}/include
  ${DELTA_1A_LIDAR_SDK_PATH}/src
)

add_executable(empty app/empty.cpp ${DELTA_1A_LIDAR_SDK_SRC})
target_link_libraries(empty -lrt -lpthread)

add_executable(detect app/detect.cpp ${DELTA_1A_LIDAR_SDK_SRC})
target_link_libraries(detect -lrt -lpthread)



