set(CMAKE_CXX_STANDARD 17)
cmake_minimum_required(VERSION 3.10)
find_program(CMAKE_C_COMPILER NAMES $ENV{CC} gcc PATHS ENV PATH NO_DEFAULT_PATH)
find_program(CMAKE_CXX_COMPILER NAMES $ENV{CXX} g++ PATHS ENV PATH NO_DEFAULT_PATH)
project(Robot)

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

# --- Fetch Eigen --------------------------------------------------------------
FetchContent_Declare(eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen
  GIT_TAG 3.3.9
)

FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
  add_library(Eigen INTERFACE)
  target_include_directories(Eigen INTERFACE ${eigen_SOURCE_DIR})
endif()

# --- Fetch Standalone Asio ---------------------------------------------------
FetchContent_Declare(asio
  GIT_REPOSITORY https://github.com/chriskohlhoff/asio
  GIT_TAG asio-1-18-2
)

FetchContent_GetProperties(asio)
if(NOT asio_POPULATED)
  FetchContent_Populate(asio)
  add_library(Asio INTERFACE)
  target_include_directories(Asio INTERFACE ${asio_SOURCE_DIR}/asio/include)
endif()
# ------------------------------------------------------------------------------

# Set the build directory to ./build
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Set the source directory
set(SOURCE_DIR ${CMAKE_SOURCE_DIR}/src)

# Collect all .cpp files in the src/ directory
file(GLOB_RECURSE SOURCES ${SOURCE_DIR}/*.cpp)
set (SOURCES ${SOURCES} ${CMAKE_SOURCE_DIR}/libraries/ina219.so)
# set(SOURCES
#   ${SOURCE_DIR}/main.cpp
#   ${SOURCE_DIR}/screentest.cpp
# )

# Add an executable with the collected sources
add_executable(${PROJECT_NAME} ${SOURCES})

# Include libraries from the ./libraries directory
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/libraries)

# Link libraries from the ./libraries directory
# target_link_libraries(${PROJECT_NAME} PRIVATE Eigen)

# Link with ftxui libraries
target_link_libraries(${PROJECT_NAME}
  PRIVATE ftxui::screen
  PRIVATE ftxui::dom
  PRIVATE ftxui::component # Not needed for this example.
  PRIVATE Eigen # Added Eigen library
  PRIVATE Asio # Added Asio library
  # PRIVATE m uncomment for pi
  # PRIVATE pthread
  # PRIVATE robotic_gcc
  # PRIVATE wiringPi
)

if (EMSCRIPTEN) 
  string(APPEND CMAKE_CXX_FLAGS " -s USE_PTHREADS") 
  string(APPEND CMAKE_EXE_LINKER_FLAGS " -s ASYNCIFY") 
  string(APPEND CMAKE_EXE_LINKER_FLAGS " -s PROXY_TO_PTHREAD") 

  foreach(file "index.html" "run_webassembly.py")
    configure_file("src/${file}" ${file})
  endforeach(file)
endif()

# Append additional linker flags
# set(INA219_LIB ${CMAKE_SOURCE_DIR}/libraries/ina219.so)
# string(APPEND CMAKE_EXE_LINKER_FLAGS " ${INA219_LIB}")

# Move the compiled binary to the root directory
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}
)
