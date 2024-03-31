cmake_minimum_required(VERSION 3.8)
project(framework)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)
if(LIBUSB1_FOUND)
  include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})
else()
  message(FATAL_ERROR "Failed to find libusb-1.0" )
endif()

file(GLOB_RECURSE SOURCES "src/*.cc" "src/*.c")

add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${LIBUSB1_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC include)

# install libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)