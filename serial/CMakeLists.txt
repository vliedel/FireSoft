
cmake_minimum_required(VERSION 2.6)
project(TEST)

## Target
#set(TEST_SRCS main.cpp AsyncSerial.cpp BufferedAsyncSerial.cpp)
#add_executable(async ${TEST_SRCS})



## Link libraries
#set(Boost_NO_SYSTEM_PATHS ON)
#set(Boost_DEBUG ON)
#set(Boost_DIR "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi")
#set(BOOST_ROOT "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi")
#set(BOOST_LIBRARYDIR "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi/usr/local/lib")

set(BOOST_LIBS thread date_time system chrono)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)

#SET(LIBS ${LIBS} "-lboost_thread.so")
#SET(LIBS ${LIBS} "-lboost_date_time.so")
#SET(LIBS ${LIBS} "-lboost_system.so")

#SET(Boost_LIBRARIES "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi/lib/libboost_thread.so.1.51.0")
#SET(Boost_LIBRARIES ${Boost_LIBRARIES} "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi/lib/libboost_date_time.so.1.51.0")
#SET(Boost_LIBRARIES ${Boost_LIBRARIES} "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi/lib/libboost_system.so.1.51.0")

find_package(Threads REQUIRED)

#MESSAGE("boost root dir: ${Boost_DIR}")
#MESSAGE("boost lib dir: ${BOOST_LIBRARYDIR}")
#MESSAGE("boost inc dir: ${BOOST_INCLUDEDIR}")

ADD_LIBRARY(serial AsyncSerial.cpp BufferedAsyncSerial.cpp)
TARGET_LINK_LIBRARIES(serial ${Boost_LIBRARIES})
