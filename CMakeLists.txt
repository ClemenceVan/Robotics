cmake_minimum_required(VERSION 3.10)
project(Robot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Enable ccache if available
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# --- Fetch FTXUI --------------------------------------------------------------
include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED TRUE)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG v4.0.0
)
FetchContent_GetProperties(ftxui)
if(NOT ftxui_POPULATED)
  FetchContent_Populate(ftxui)
  add_subdirectory(${ftxui_SOURCE_DIR} ${ftxui_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# Set the build directory to ./build
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Set the source directory
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)

# Collect all .cpp files in the src/ directory
file(GLOB_RECURSE SOURCES ${SOURCE_DIR}/*.cpp)

# Add an executable with the collected sources
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(Robot PRIVATE src)

# Fetch and include OpenCV
find_package(OpenCV REQUIRED)
target_link_libraries(Robot PRIVATE ${OpenCV_LIBS})

target_link_libraries(Robot
  PRIVATE ftxui::screen
  PRIVATE ftxui::dom
  PRIVATE ftxui::component
  PRIVATE m
  PRIVATE pthread
  PRIVATE robotic_gcc
  PRIVATE wiringPi
)

# Append additional linker flags
string(APPEND CMAKE_EXE_LINKER_FLAGS " /home/pi/Documents/Labs/Lab\\ 2/2/libraries/ina219.so")

# Move the compiled binary to the root directory
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# Use precompiled headers
target_precompile_headers(Robot PRIVATE src/pch.h)
