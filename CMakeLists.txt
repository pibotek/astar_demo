cmake_minimum_required(VERSION 3.5.1)

project(planner_test)

set(CMAKE_CXX_STANDARD 11)

option(USE_MINI_SAMPLE "use mini sample as input" OFF)
option(DEBUG_PLANNER_DETAIL "debug planner detail" OFF)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if (USE_MINI_SAMPLE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_MINI_SAMPLE=1)
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_MINI_SAMPLE=0)
endif()

if (DEBUG_PLANNER_DETAIL)
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_PLANNER_DETAIL=1)
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_PLANNER_DETAIL=0)
endif()


target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})