project(simple_robotics_frontend)
cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)

include_directories(include)

file(GLOB_RECURSE FRONTEND_SRCS "src/frontend/*.cc")
file(GLOB_RECURSE BACKEND_SRCS "src/backend/*.cc")
message(${BACKEND_SRCS})

add_library(my_robotics_library STATIC ${FRONTEND_SRCS} ${BACKEND_SRCS})
target_include_directories(my_robotics_library
        PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include DESTINATION .)
install(TARGETS my_robotics_library EXPORT my_robotics_libraryConfig)
install(EXPORT my_robotics_libraryConfig DESTINATION share/my_robotics_library/cmake)

enable_testing()
find_package(GTest REQUIRED)
add_executable(pipe_line_test test/pipeline_test.cc test/pipeline_test.cc)
target_link_libraries(pipe_line_test my_robotics_library GTest::GTest GTest::Main)
add_test(pipe_line_test pipe_line_test)