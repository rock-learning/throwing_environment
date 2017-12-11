cmake_minimum_required(VERSION 2.6)

project(throwing_environment)
set(PROJECT_VERSION 1.0)
set(PROJECT_DESCRIPTION "An environment with a robot that throws a ball.")

include(FindPkgConfig)
find_package(lib_manager)
lib_defaults()
define_module_info()


if(WIN32)
  # this fixes the error 998 from the LibManager
  SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--enable-auto-import -Wall")
  SET(CMAKE_MODULE_LINKER_FLAGS "-Wl,--enable-auto-import -Wall")
else(WIN32)
  SET(CMAKE_CXX_FLAGS "-fPIC -Wall")
endif(WIN32)


pkg_check_modules(BOLERO "bolero")
include_directories(${BOLERO_INCLUDE_DIRS})

pkg_check_modules(MARS_ENV "mars_environment")
include_directories(${MARS_ENV_INCLUDE_DIRS})
link_directories(${MARS_ENV_LIBRARY_DIRS})

pkg_check_modules(MARS_UTILS "mars_utils")
include_directories(${MARS_UTILS_INCLUDE_DIRS})
link_directories(${MARS_UTILS_LIBRARY_DIRS})

pkg_check_modules(CONFIGMAPS "configmaps")
include_directories(${CONFIGMAPS_INCLUDE_DIRS})
link_directories(${CONFIGMAPS_LIBRARY_DIRS})

include_directories(src)

set(SOURCES src/ThrowingEnvironment.cpp)
set(HEADERS src/ThrowingEnvironment.h)

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME}
    ${MARS_ENV_LIBRARIES}
    ${MARS_UTILS_LIBRARIES}
    ${CONFIGMAPS_LIBRARIES})

if(WIN32)
  set(LIB_INSTALL_DIR bin) # .dll are in PATH, like executables
else(WIN32)
  set(LIB_INSTALL_DIR lib)
endif(WIN32)


set(_INSTALL_DESTINATIONS
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION ${LIB_INSTALL_DIR}
  ARCHIVE DESTINATION lib
)


IF (WIN32)
  SET (POSTLIB ".dll")
  SET (PRELIB "lib")
  SET (LIBPATH "bin")
ELSE (WIN32)
  IF (APPLE)
    SET (POSTLIB ".dylib")
    SET (PRELIB "lib")
    SET (LIBPATH "lib")
  ELSE (APPLE)
    SET (POSTLIB ".so")
    SET (PRELIB "lib")
    SET (LIBPATH "lib")
  ENDIF (APPLE)
ENDIF (WIN32)

# Install the library into the lib folder
install(TARGETS ${PROJECT_NAME} ${_INSTALL_DESTINATIONS})

# Install headers into mars include directory
install(FILES ${HEADERS} DESTINATION include/bolero/${PROJECT_NAME})

# Prepare and install necessary files to support finding of the library 
# using pkg-config
configure_file(${PROJECT_NAME}.pc.in ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

install(FILES configuration/throwing.smurfs
        DESTINATION configuration/${PROJECT_NAME}/)
install(DIRECTORY configuration/robot
        DESTINATION configuration/${PROJECT_NAME})
install(DIRECTORY configuration/target
        DESTINATION configuration/${PROJECT_NAME})
