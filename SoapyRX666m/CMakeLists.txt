########################################################################
# Build Soapy SDR support module for RTL-SDR Devices
########################################################################
cmake_minimum_required(VERSION 2.8.7)
project(SoapyRX666m CXX)

find_package(SoapySDR "0.4.0" NO_MODULE REQUIRED)
if (NOT SoapySDR_FOUND)
    message(FATAL_ERROR "Soapy SDR development files not found...")
endif ()


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Threads)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#enable c++11 features
if(CMAKE_COMPILER_IS_GNUCXX)

    #C++11 is a required language feature for this project
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
    if(HAS_STD_CXX11)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	else()
		message(FATAL_ERROR "expected C++11")
    endif()
	SET(CMAKE_CXX_FLAGS "-O2")
	SET(CMAKE_C_FLAGS "-O2")

    #Thread support enabled (not the same as -lpthread)
	list(APPEND RX666M_LIBRARIES -pthread)

    #disable warnings for unused parameters
    add_definitions(-Wno-unused-parameter)

endif(CMAKE_COMPILER_IS_GNUCXX)

# check if rtlsdr library includes support for bias-tee
unset(CMAKE_REQUIRED_LIBRARIES)

set(OTHER_LIBS "" CACHE STRING "Other libraries")

SOAPY_SDR_MODULE_UTIL(
    TARGET RX666mSupport
    SOURCES
        SoapyRX666m.hpp
        Registration.cpp
        Settings.cpp
        Streaming.cpp
		LowLevel.cpp
		tuner_r82xx.cpp
    LIBRARIES
        ${RX666M_LIBRARIES}
        ${OTHER_LIBS}
)

ADD_EXECUTABLE(rx666m_util rx666m_util.cpp LowLevel.cpp tuner_r82xx.cpp)
TARGET_LINK_LIBRARIES(rx666m_util pthread boost_program_options)

ADD_EXECUTABLE(rx666m_convert rx666m_convert.cpp)
TARGET_LINK_LIBRARIES(rx666m_convert boost_program_options)

