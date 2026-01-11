cmake_minimum_required(VERSION 3.10)

project(RTKLIB C)

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Global Definitions
add_definitions(
    -DENAGLO
    -DENAQZS
    -DENACMP
    -DENAGAL
    -DENAIRN
    -DENALEO
    -DNFREQ=5
    -DTRACE
)

if(WIN32)
    add_definitions(
        -DWIN32
        -D_WINSOCK_DEPRECATED_NO_WARNINGS
        -D_CRT_SECURE_NO_WARNINGS
    )
endif()

if(UNIX)
    add_definitions(-DSVR_REUSEADDR)
endif()

# Include directories
include_directories(src)

# Source files list for RTKLIB library
set(RTKLIB_SOURCES
    src/convgpx.c
    src/convkml.c
    src/convrnx.c
    src/datum.c
    src/download.c
    src/ephemeris.c
    src/geoid.c
    src/gis.c
    src/ionex.c
    src/lambda.c
    src/options.c
    src/osnma.c
    src/pntpos.c
    src/postpos.c
    src/ppp_ar.c
    src/ppp.c
    src/ppprtk.c
    src/preceph.c
    src/rcvraw.c
    src/rinex.c
    src/rtcm.c
    src/rtcm2.c
    src/rtcm3.c
    src/rtcm3e.c
    src/rtkcmn.c
    src/rtkpos.c
    src/rtksvr.c
    src/sbas.c
    src/solution.c
    src/stream.c
    src/streamsvr.c
    src/tides.c
    src/tle.c
    
    # Receiver drivers
    src/rcv/binex.c
    src/rcv/crescent.c
    src/rcv/javad.c
    src/rcv/novatel.c
    src/rcv/nvs.c
    src/rcv/rt17.c
    src/rcv/septentrio.c
    src/rcv/skytraq.c
    src/rcv/ss2.c
    src/rcv/ublox.c
)

# Create the static library target
add_library(rtklib STATIC ${RTKLIB_SOURCES})

# Platform specific link libraries
if(WIN32)
    target_link_libraries(rtklib PUBLIC ws2_32 winmm)
else()
    target_link_libraries(rtklib PUBLIC m pthread)
endif()

# ------------------------------------------------------------------------------
# Console Applications
# ------------------------------------------------------------------------------

# rnx2rtkp
add_executable(rnx2rtkp app/consapp/rnx2rtkp/rnx2rtkp.c)
target_link_libraries(rnx2rtkp rtklib)

# pos2kml
add_executable(pos2kml app/consapp/pos2kml/pos2kml.c)
target_link_libraries(pos2kml rtklib)

# convbin
add_executable(convbin app/consapp/convbin/convbin.c)
target_link_libraries(convbin rtklib)

# str2str
if(NOT MSVC)
    add_executable(str2str app/consapp/str2str/str2str.c)
    target_link_libraries(str2str rtklib)
endif()

# rtkrcv
# rtkrcv needs vt.c which is in the same directory
if(NOT MSVC)
    add_executable(rtkrcv 
        app/consapp/rtkrcv/rtkrcv.c
        app/consapp/rtkrcv/vt.c
    )
    target_include_directories(rtkrcv PRIVATE app/consapp/rtkrcv)
    target_link_libraries(rtkrcv rtklib)
endif()

# Tests (Optional, adapted from original src/CMakeLists.txt)
enable_testing()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/test/test_rtklib.c")
    add_executable(test_rtklib src/test/test_rtklib.c)
    target_link_libraries(test_rtklib rtklib)
    add_test(NAME test_rtklib COMMAND test_rtklib)
endif()

# Unit tests from test/utest
# Excluded: t_corrperf.c (missing sdr.h), t_filter.c (missing stdout.h)
set(UTEST_SOURCES
    test/utest/t_atmos.c
    test/utest/t_coord.c
    test/utest/t_geoid.c
    test/utest/t_gloeph.c
    test/utest/t_ionex.c
    test/utest/t_lambda.c
    test/utest/t_matrix.c
    test/utest/t_misc.c
    test/utest/t_ppp.c
    test/utest/t_preceph.c
    test/utest/t_rinex.c
    test/utest/t_time.c
    test/utest/t_tle.c
)

foreach(test_src ${UTEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} rtklib)
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/utest")
endforeach()
