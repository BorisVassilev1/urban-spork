cmake_minimum_required(VERSION 3.0.0)

project(RTInOneWeekend)

set(HEADERS
)

set(SOURCES
	src/Material.h
	src/Material.cpp

	src/Image.hpp

	src/Primitive.h
	src/Primitive.cpp
	src/Accelerators.cpp

	src/Utils.hpp
	src/Threading.hpp
	src/Mesh.h
	src/Mesh.cpp

	src/Mesh.cpp
	src/main.cpp

	src/third_party/stb_image_write.h
	src/third_party/tiny_obj_loader.h
)

add_executable(${PROJECT_NAME} "${SOURCES};${HEADERS}")
target_compile_definitions(${PROJECT_NAME} PRIVATE MESH_FOLDER="${CMAKE_SOURCE_DIR}/mesh")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
set(CMAKE_CXX_FLAGS "-O3 -std=c++17")

target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)


add_library("urban-spork" STATIC ${SOURCES})
target_link_libraries("urban-spork" PRIVATE Threads::Threads)
target_compile_definitions("urban-spork" PRIVATE MESH_FOLDER="${CMAKE_SOURCE_DIR}/mesh")

