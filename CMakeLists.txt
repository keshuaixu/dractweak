cmake_minimum_required(VERSION 2.8)
project(dractweak)

set(IMGUI_PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui")
set(AMP1394_PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/mechatronics-software")


add_subdirectory("${AMP1394_PATH}")
set (CMAKE_PREFIX_PATH "${Amp1394_BINARY_DIR}" ${CMAKE_PREFIX_PATH})
find_package(Amp1394 REQUIRED)
include_directories("${Amp1394_INCLUDE_DIR};${Amp1394_BINARY_DIR}")
link_directories (${Amp1394_LIBRARY_DIR})

find_package(glfw3 REQUIRED)
include_directories(${GLFW_INCLUDE_DIRS})
# link_libraries(${GLFW_LIBRARY_DIRS})

cmake_policy (SET CMP0072 NEW)
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

add_library("gl3w" "${IMGUI_PATH}/examples/libs/gl3w/GL/gl3w.c")
include_directories("${IMGUI_PATH}/examples/libs/gl3w")

add_executable(dractweak dractweak.cpp)
set_property(TARGET dractweak PROPERTY CXX_STANDARD 20)


include_directories("${IMGUI_PATH};${IMGUI_PATH}/backends")
target_sources(dractweak 
    PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/Amp1394Console.cpp"
    "${IMGUI_PATH}/imgui.cpp"
    "${IMGUI_PATH}/imgui_draw.cpp"
    "${IMGUI_PATH}/imgui_demo.cpp"
    "${IMGUI_PATH}/imgui_tables.cpp"
    "${IMGUI_PATH}/imgui_widgets.cpp"
    "${IMGUI_PATH}/backends/imgui_impl_opengl3.cpp"
    "${IMGUI_PATH}/backends/imgui_impl_glfw.cpp"
    )
target_link_libraries(dractweak Amp1394 OpenGL::GL OpenGL::GLU glfw gl3w ${CMAKE_DL_LIBS})
install(TARGETS dractweak DESTINATION bin)
