FROM debian:buster

USER root

RUN dpkg --add-architecture armhf && apt-get update
RUN apt-get install -y --no-install-recommends \
    curl \
    zip \
    unzip \
    git \
    wget \
    cmake \
    pkg-config \
    build-essential \
    crossbuild-essential-armhf \
    libusb-1.0-0-dev:armhf \
    libgtk-3-dev:armhf \
    ffmpeg:armhf \
    libpango1.0-dev:armhf

WORKDIR /root

# Install CMake
RUN wget --no-check-certificate https://www.cmake.org/files/v3.14/cmake-3.14.3.tar.gz && \
    tar xf cmake-3.14.3.tar.gz && \
    (cd cmake-3.14.3 && ./bootstrap --parallel=$(nproc --all) && make --jobs=$(nproc --all) && make install) && \
    rm -rf cmake-3.14.3 cmake-3.14.3.tar.gz

RUN git config --global http.sslVerify false

RUN apt-get install -y --no-install-recommends \
    libavcodec-dev:armhf \
    libavformat-dev:armhf \
    libswscale-dev:armhf \
    libgstreamer1.0-0:armhf \
    gstreamer1.0-plugins-base:armhf \
    gstreamer1.0-plugins-good:armhf \
    gstreamer1.0-plugins-bad:armhf \
    gstreamer1.0-plugins-ugly:armhf \
    gstreamer1.0-libav:armhf \
    gstreamer1.0-doc:armhf \
    gstreamer1.0-tools:armhf \
    libgstreamer1.0-dev:armhf \
    libgstreamer-plugins-base1.0-dev:armhf \
    libgtk2.0-dev:armhf

COPY arm-openvino.toolchain.cmake .

ENV PKG_CONFIG_LIBDIR /usr/lib/arm-linux-gnueabihf/pkgconfig

# Build OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.4.zip --no-check-certificate
RUN unzip opencv.zip && mv opencv-3.4.4 opencv
RUN mkdir -p ~/opencv/build
RUN cd ~/opencv/build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_TOOLCHAIN_FILE=/root/arm-openvino.toolchain.cmake \
          -DBUILD_EXAMPLES=ON \
          -DBUILD_DOCS=OFF \
          -DBUILD_PERF_TESTS=OFF \
          -DBUILD_TESTS=OFF \
          -DWITH_GSTREAMER=ON \
          -DWITH_GTK=ON ..
RUN cd ~/opencv/build && make --jobs=$(nproc --all)

# Install Boost
RUN wget --no-check-certificate https://sourceforge.net/projects/boost/files/boost/1.72.0/boost_1_72_0.tar.bz2
RUN tar xjf boost_1_72_0.tar.bz2

RUN cd boost_1_72_0 && \
    ./bootstrap.sh --prefix=/root/boost && \
    echo "using gcc : arm : arm-linux-gnueabihf-g++ ;" > ~/user-config.jam && \
    ./b2 -j $(nproc --all) --with-atomic --with-chrono --with-date_time --with-filesystem --with-regex --with-thread --with-system --no-samples --no-tests toolset=gcc-arm threading=multi link=static runtime-link=static --layout=tagged --build-type=complete stage

WORKDIR /root/build
