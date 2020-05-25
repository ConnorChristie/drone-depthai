FROM debian:buster

RUN apt-get update
RUN apt-get install -y build-essential gawk gcc g++ gfortran git texinfo bison libncurses-dev
RUN apt-get install -y --no-install-recommends wget

WORKDIR /root

RUN wget --no-check-certificate https://sourceforge.net/projects/raspberry-pi-cross-compilers/files/Raspberry%20Pi%20GCC%20Cross-Compiler%20Toolchains/Buster/GCC%208.3.0/Raspberry%20Pi%201%2C%20Zero/cross-gcc-8.3.0-pi_0-1.tar.gz
RUN apt-get install -y --no-install-recommends bsdtar
RUN bsdtar xf cross-gcc-8.3.0-pi_0-1.tar.gz && rm cross-gcc-8.3.0-pi_0-1.tar.gz

ENV PATH=/root/cross-pi-gcc-8.3.0-0/bin:$PATH

RUN apt-get install -y --no-install-recommends cmake unzip

RUN dpkg --add-architecture armhf && apt-get update
RUN apt-get install -y --no-install-recommends \
    libusb-1.0-0-dev:armhf

COPY rpi.toolchain.cmake .

# Build OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.4.zip --no-check-certificate
RUN unzip opencv.zip && mv opencv-3.4.4 opencv
RUN mkdir -p ~/opencv/build
RUN cd ~/opencv/build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_TOOLCHAIN_FILE=/root/rpi.toolchain.cmake \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_DOCS=OFF \
          -DBUILD_PERF_TESTS=OFF \
          -DBUILD_TESTS=OFF \
          -DWITH_GSTREAMER=OFF \
          -DWITH_GTK=OFF ..
RUN cd ~/opencv/build && make --jobs=$(nproc --all)

# Install Boost
RUN wget --no-check-certificate https://sourceforge.net/projects/boost/files/boost/1.72.0/boost_1_72_0.tar.bz2
RUN tar xjf boost_1_72_0.tar.bz2

RUN cd boost_1_72_0 && \
    ./bootstrap.sh --prefix=/root/boost && \
    echo "using gcc : arm : arm-linux-gnueabihf-g++ ;" > ~/user-config.jam && \
    ./b2 -j $(nproc --all) --with-atomic --with-chrono --with-date_time --with-filesystem --with-regex --with-thread --with-system --no-samples --no-tests toolset=gcc-arm threading=multi link=static runtime-link=static --layout=tagged --build-type=complete stage

COPY rpi-2.toolchain.cmake .

WORKDIR /root/build
