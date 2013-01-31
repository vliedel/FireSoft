# Find Serial lib
#
# Sets:
# SERIAL_INCLUDE_DIR
# SERIAL_LIBS

# Add boost_libs
#set(Boost_NO_SYSTEM_PATHS ON)
#set(BOOST_LIBRARYDIR "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi/usr/lib")
#set(Boost_DIR "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi")

SET(BOOST_LIBS thread date_time system chrono)
find_package(Boost COMPONENTS ${BOOST_LIBS} REQUIRED)
find_package(Threads REQUIRED)

#SET(Boost_LIBRARIES "boost_thread")
#SET(Boost_LIBRARIES ${Boost_LIBRARIES} "boost_date_time")
#SET(Boost_LIBRARIES ${Boost_LIBRARIES} "boost_system")

#SET(Boost_LIBRARIES "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi/lib/libboost_thread.so.1.51.0")
#SET(Boost_LIBRARIES ${Boost_LIBRARIES} "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi/lib/libboost_date_time.so.1.51.0")
#SET(Boost_LIBRARIES ${Boost_LIBRARIES} "$ENV{OVEROTOP}/tmp/sysroots/x86_64-linux/usr/armv7a/arm-angstrom-linux-gnueabi/lib/libboost_system.so.1.51.0")

STRING(REGEX REPLACE ".+\\/" "" DIR_UP ${CMAKE_BINARY_DIR})

SET(SERIAL_LIBS ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} "${ROOT_DIR}/serial/${DIR_UP}/libserial.a")

SET(SERIAL_INCLUDE_DIR ${ROOT_DIR}/serial)
