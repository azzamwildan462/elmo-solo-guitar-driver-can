cmake_minimum_required(VERSION 3.0)

project(ElmoCan)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src) 

add_library(ElmoCan src/elmo_can.cpp)

set(CMAKE_INSTALL_INCLUDEDIR /usr/local/include)
set(CMAKE_INSTALL_LIBDIR /usr/local/lib)

target_include_directories(ElmoCan PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# Install headers
install(DIRECTORY src/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# Install library
install(TARGETS ElmoCan
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(ElmoCan_INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}-targets
  DESTINATION ${ElmoCan_INSTALL_CONFIGDIR}
)

