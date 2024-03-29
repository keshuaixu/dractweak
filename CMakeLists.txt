cmake_minimum_required(VERSION 2.8)
project(dractweak)

include_directories("include")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(IMGUI_PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/imgui")
set(IMPLOT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/external/implot")
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

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})

add_executable(dractweak dractweak.cpp)
set_property(TARGET dractweak PROPERTY CXX_STANDARD 20)


include_directories("${IMGUI_PATH};${IMGUI_PATH}/backends;${IMGUI_PATH}/misc/cpp")
include_directories("${IMPLOT_PATH}")

set(COMMON_FILES
"${IMGUI_PATH}/imgui.cpp"
"${IMGUI_PATH}/imgui_draw.cpp"
"${IMGUI_PATH}/imgui_demo.cpp"
"${IMGUI_PATH}/imgui_tables.cpp"
"${IMGUI_PATH}/imgui_widgets.cpp"
"${IMGUI_PATH}/backends/imgui_impl_opengl3.cpp"
"${IMGUI_PATH}/backends/imgui_impl_glfw.cpp"
"${IMGUI_PATH}/misc/cpp/imgui_stdlib.cpp"
"${IMPLOT_PATH}/implot.cpp"
"${IMPLOT_PATH}/implot_items.cpp"
)

target_sources(dractweak PUBLIC ${COMMON_FILES})
target_link_libraries(dractweak Amp1394 OpenGL::GL OpenGL::GLU glfw GLEW::GLEW ${CMAKE_DL_LIBS})

add_executable(dractest dractest.cpp)
target_sources(dractest PUBLIC ${COMMON_FILES})
target_link_libraries(dractest Amp1394  OpenGL::GL OpenGL::GLU glfw GLEW::GLEW ${CMAKE_DL_LIBS})
set_property(TARGET dractest PROPERTY CXX_STANDARD 20)

add_executable(dractune dractune.cpp)
target_sources(dractune PUBLIC ${COMMON_FILES})
target_link_libraries(dractune Amp1394  OpenGL::GL OpenGL::GLU glfw GLEW::GLEW ${CMAKE_DL_LIBS})
set_property(TARGET dractune PROPERTY CXX_STANDARD 20)


install(TARGETS dractweak DESTINATION bin)
install(TARGETS dractest DESTINATION bin)
install(TARGETS dractune DESTINATION bin)