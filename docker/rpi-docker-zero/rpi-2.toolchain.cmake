set( CMAKE_SYSTEM_NAME Linux )
set( CMAKE_SYSTEM_VERSION 1 )
set( CMAKE_SYSTEM_PROCESSOR arm )
set( CMAKE_LIBRARY_ARCHITECTURE arm-linux-gnueabihf )
set( FLOAT_ABI_SUFFIX "hf" )

SET(CMAKE_CROSSCOMPILING True)
set(CMAKE_COMPILER_IS_RASPBERRY_CROSS_COMPILER ON)

SET(GCC_COMPILER_VERSION "" CACHE STRING "GCC Compiler version")
SET(GNU_MACHINE "arm-linux-gnueabihf" CACHE STRING "GNU compiler triple")

# cross compiler tools
set(CC_RPI_GCC arm-linux-gnueabihf-gcc)
set(CC_RPI_GXX arm-linux-gnueabihf-g++)
SET(PKG_CONFIG_LIBDIR /usr/lib/arm-linux-gnueabihf/pkgconfig)

set(COMMON_FLAGS "-I/usr/include/arm-linux-gnueabihf -I/usr/include -L/lib/arm-linux-gnueabihf -Wl,-rpath-link,/lib/arm-linux-gnueabihf -L/usr/lib/arm-linux-gnueabihf -Wl,-rpath-link,/usr/lib/arm-linux-gnueabihf")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS}")

# specify the cross compiler
SET(CMAKE_C_COMPILER   ${CC_RPI_GCC})
SET(CMAKE_CXX_COMPILER ${CC_RPI_GXX})

SET(BOOST_ROOT /root/boost_1_72_0)
set(Boost_ARCHITECTURE "-a32")

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH} /usr/lib/arm-linux-gnueabihf /lib/arm-linux-gnueabihf /root/opencv/build ${BOOST_ROOT}/stage)

SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
