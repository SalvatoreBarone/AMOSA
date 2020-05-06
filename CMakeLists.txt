project(AMOSA LANGUAGES CXX)
cmake_minimum_required(VERSION 3.13)

set (CMAKE_CXX_STANDARD 11)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/libs)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Threads)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -ggdb3 -O0 ")

set (AMOSA_SRC_DIR				${CMAKE_CURRENT_SOURCE_DIR}/src)
set (AMOSA_HDR_INSTALL_DIR		/usr/include/)

install(FILES ${AMOSA_SRC_DIR}/amosa.hpp DESTINATION include)

set (EXAMPLE1_NAME	example1)
set (EXAMPLE2_NAME	example2)
set (EXAMPLE3_NAME	example3)
set (EXAMPLE_SRC	${CMAKE_CURRENT_SOURCE_DIR}/example)

add_executable(
	${EXAMPLE1_NAME}
	${EXAMPLE_SRC}/example.cpp
	${EXAMPLE_SRC}/problem.cpp
)

target_include_directories (
	${EXAMPLE1_NAME}
	PRIVATE ${AMOSA_SRC_DIR}
)

target_link_directories(
	${EXAMPLE1_NAME}
	PRIVATE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

target_link_libraries(
	${EXAMPLE1_NAME}
	${CMAKE_THREAD_LIBS_INIT}
)

add_executable(
	${EXAMPLE2_NAME}
	${EXAMPLE_SRC}/example2.cpp
	${EXAMPLE_SRC}/problem2.cpp
)

target_include_directories (
	${EXAMPLE2_NAME}
	PRIVATE ${AMOSA_SRC_DIR}
)

target_link_directories(
	${EXAMPLE2_NAME}
	PRIVATE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

target_link_libraries(
	${EXAMPLE2_NAME}
	${CMAKE_THREAD_LIBS_INIT}
)

add_executable(
	${EXAMPLE3_NAME}
	${EXAMPLE_SRC}/example3.cpp
	${EXAMPLE_SRC}/problem3.cpp
)

target_include_directories (
	${EXAMPLE3_NAME}
	PRIVATE ${AMOSA_SRC_DIR}
)

target_link_directories(
	${EXAMPLE3_NAME}
	PRIVATE ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

target_link_libraries(
	${EXAMPLE3_NAME}
	${CMAKE_THREAD_LIBS_INIT}
)
