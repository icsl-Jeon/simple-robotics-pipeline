project(simple_robotics_frontend)
cmake_minimum_required(VERSION 3.22)

include_directories(include)

file(GLOB_RECURSE FRONTEND_SRCS "src/my_robotics_pipeline/frontend/*.cc")
file(GLOB_RECURSE BACKEND_SRCS "src/my_robotics_pipeline/backend/*.cc")

add_library(my_robotics_library STATIC ${FRONTEND_SRCS} ${BACEND_SRCS})
target_include_directories(my_robotics_library
    PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include DESTINATION .)
install(TARGETS my_robotics_library EXPORT my_robotics_libraryConfig)
install(EXPORT my_robotics_libraryConfig DESTINATION share/my_robotics_library/cmake)
