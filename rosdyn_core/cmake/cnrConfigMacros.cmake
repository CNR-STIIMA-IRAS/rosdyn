####
#
#
####
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

  IF("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
    set(CMAKE_CXX_FLAGS "-std=c++11")
  ELSE()
    # Default to C++14
    if(NOT CMAKE_CXX_STANDARD)
      set(CMAKE_CXX_STANDARD 14)
    endif()

    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
  endif()

endmacro()

####
#
#
####
macro(cnr_target_compile_options TARGET_NAME)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")

  target_compile_options(${TARGET_NAME} 
    PRIVATE -Wall -Wextra -Wunreachable-code -Wpedantic
    PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  target_compile_options(${TARGET_NAME} 
    PRIVATE -Wweak-vtables -Wexit-time-destructors -Wglobal-constructors -Wmissing-noreturn
    PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")

  target_compile_options(${TARGET_NAME} PRIVATE /W3  )

  endif()
endmacro()
