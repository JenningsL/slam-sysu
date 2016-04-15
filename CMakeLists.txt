# This is demo on how to link libpointmatcher to an
# external project.

cmake_minimum_required (VERSION 2.8)
project(slam)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake) # add FindMatlab module
find_package(Matlab REQUIRED MX_LIBRARY)
find_package(libpointmatcher REQUIRED)

add_definitions(/DMATLAB_MEX_FILE) #define matlab macros
add_definitions(/DMX_COMPAT_32)

include_directories(${Matlab_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${libpointmatcher_INCLUDE_DIRS})
include_directories(include)

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

add_subdirectory(octomap)
add_subdirectory(octomap/math)
add_subdirectory(dbscan)
add_subdirectory(ransac)
add_subdirectory(tracking)

add_executable(${PROJECT_NAME} ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME} ${libpointmatcher_LIBRARIES})
target_link_libraries(${PROJECT_NAME} octomap)
target_link_libraries(${PROJECT_NAME} dbscan)
target_link_libraries(${PROJECT_NAME} ransac)
target_link_libraries(${PROJECT_NAME} tracking)
target_link_libraries(${PROJECT_NAME} ${Matlab_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Matlab_MX_LIBRARY})
