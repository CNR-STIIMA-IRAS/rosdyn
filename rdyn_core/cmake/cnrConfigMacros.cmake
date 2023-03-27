#
# get_project_name
#
macro(get_project_name filename extracted_name extracted_version)
  # Read the package manifest.
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/${filename}" package_xml_str)

  # Extract project name.
  if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
    message(FATAL_ERROR "Could not parse project name from package manifest (aborting)")
  else()
    set(extracted_name ${CMAKE_MATCH_1})
  endif()

  # Extract project version.
  if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
    message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
  else()
    set(extracted_version ${CMAKE_MATCH_1})
  endif()

  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
    cmake_policy(SET CMP0048 OLD)
  else()  
    cmake_policy(SET CMP0048 NEW)
  endif()

endmacro()

#
# cnr_set_flags
#
macro(cnr_set_flags)
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "/W3" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  endif()

  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()

  if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()



  if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
    
    set(CMAKE_CXX_FLAGS "-std=c++11")

  else()
    
    set(LOCAL_CXX_STANDARD 14)
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      
      add_compile_options(-Wall -Wextra -Wpedantic -D_TIME_BITS=64 -D_FILE_OFFSET_BITS=64)

      if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_VERSION GREATER 12)
        set(LOCAL_CXX_STANDARD 17)
      elseif(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION GREATER 10)
        set(LOCAL_CXX_STANDARD 17)
      endif()

    endif()

    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
      message(STATUS "CMAKE CXX STANDARD: ${LOCAL_CXX_STANDARD}")
      set(CMAKE_CXX_STANDARD ${LOCAL_CXX_STANDARD})
    endif()

    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()

  if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
    set(THREADS_PREFER_PTHREAD_FLAG ON)
  endif()

endmacro()

#
# cnr_target_compile_options
#
macro(cnr_target_compile_options TARGET_NAME)

if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")

  target_compile_options(${TARGET_NAME} 
    PRIVATE -Wall -Wextra -Wunreachable-code -Wpedantic
    PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  target_compile_options(${TARGET_NAME} 
    PRIVATE -Wweak-vtables -Wexit-time-destructors -Wglobal-constructors -Wmissing-noreturn -Wno-gnu-zero-variadic-macro-arguments
    PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")

  target_compile_options(${TARGET_NAME} PRIVATE /W3)

  endif()
endmacro()

#
# cnr_enable_testing
#
macro(cnr_enable_testing ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)
  if(${ENABLE_TESTING})
    message(STATUS "Enable testing")
    if(${USE_ROS1})
      find_package(rostest REQUIRED)
      find_package(roscpp REQUIRED)  
    else()
      enable_testing()
      find_package(GTest REQUIRED)
    endif()

    if(${ENABLE_COVERAGE_TESTING} AND NOT WIN32)
      message(STATUS "Enable Coverage")
      if(${USE_ROS1})
        find_package(code_coverage REQUIRED)
        APPEND_COVERAGE_COMPILER_FLAGS()
      else() 
        set(CMAKE_CXX_FLAGS "-Wno-deprecated-register ${CMAKE_CXX_FLAGS}")
        set(CMAKE_CXX_FLAGS_DEBUG "-Wno-deprecated-register -O0 -g -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS_DEBUG}")
      endif()
    endif()
  endif()
endmacro()

#
# cnr_install_directories
#
macro(cnr_install_directories USE_ROS1 CNR_INSTALL_INCLUDE_DIR CNR_INSTALL_LIB_DIR CNR_INSTALL_BIN_DIR CNR_INSTALL_SHARE_DIR)
if(USE_ROS1)
  set(${CNR_INSTALL_INCLUDE_DIR}    ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  set(${CNR_INSTALL_LIB_DIR}        ${CATKIN_PACKAGE_LIB_DESTINATION})
  set(${CNR_INSTALL_BIN_DIR}        ${CATKIN_GLOBAL_BIN_DESTINATION})
  set(${CNR_INSTALL_SHARE_DIR}      ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
  set(${CNR_INSTALL_INCLUDE_DIR}    "${CMAKE_INSTALL_PREFIX}/include")
  set(${CNR_INSTALL_LIB_DIR}        "${CMAKE_INSTALL_PREFIX}/lib")
  set(${CNR_INSTALL_BIN_DIR}        "${CMAKE_INSTALL_PREFIX}/bin")
  set(${CNR_INSTALL_SHARE_DIR}      "${CMAKE_INSTALL_PREFIX}/share")
endif()
endmacro()
