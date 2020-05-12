SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_VERSION 1)
SET(CMAKE_SYSTEM_PROCESSOR aarch64)
SET(CMAKE_CROSSCOMPILING True)

SET(FLOAT_ABI_SUFFIX "hf")
set(ARM_LINKER_FLAGS "-Wl,--no-undefined -Wl,--gc-sections -Wl,-z,noexecstack -Wl,-z,relro -Wl,-z,now")

SET(GCC_COMPILER_VERSION "" CACHE STRING "GCC Compiler version")
SET(GNU_MACHINE "aarch64-linux-gnu" CACHE STRING "GNU compiler triple")

# cross compiler tools
set(CC_RPI_GCC /usr/bin/aarch64-linux-gnu-gcc)
set(CC_RPI_GXX /usr/bin/aarch64-linux-gnu-g++)
set(CC_RPI_LIBS /usr/lib/aarch64-linux-gnu)
SET(PKG_CONFIG_LIBDIR /usr/lib/aarch64-linux-gnu/pkgconfig)

# specify the cross compiler
SET(CMAKE_C_COMPILER   ${CC_RPI_GCC})
SET(CMAKE_CXX_COMPILER ${CC_RPI_GXX})

SET(BOOST_ROOT /root/boost_1_72_0)
set(Boost_ARCHITECTURE "-arm")

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH ${CMAKE_FIND_ROOT_PATH} /root/dldt/inference_engine/share /root/dldt/inference_engine /root/opencv/build ${BOOST_ROOT}/stage)
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

