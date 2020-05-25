FROM debian:stretch

USER root

RUN dpkg --add-architecture arm64 && apt-get update
RUN apt-get install -y --no-install-recommends \
    curl \
    zip \
    unzip \
    git \
    wget \
    cmake \
    pkg-config \
    build-essential \
    crossbuild-essential-arm64 \
    libusb-1.0-0-dev:arm64 \
    libgtk-3-dev:arm64 \
    ffmpeg:arm64 \
    libpango1.0-dev:arm64

WORKDIR /root

# Install CMake
RUN wget --no-check-certificate https://www.cmake.org/files/v3.14/cmake-3.14.3.tar.gz && \
    tar xf cmake-3.14.3.tar.gz && \
    (cd cmake-3.14.3 && ./bootstrap --parallel=$(nproc --all) && make --jobs=$(nproc --all) && make install) && \
    rm -rf cmake-3.14.3 cmake-3.14.3.tar.gz

RUN git config --global http.sslVerify false

RUN apt-get install -y --no-install-recommends \
    libavcodec-dev:arm64 \
    libavformat-dev:arm64 \
    libswscale-dev:arm64 \
    libgstreamer1.0-0:arm64 \
    gstreamer1.0-plugins-base:arm64 \
    gstreamer1.0-plugins-good:arm64 \
    gstreamer1.0-plugins-bad:arm64 \
    gstreamer1.0-plugins-ugly:arm64 \
    gstreamer1.0-libav:arm64 \
    gstreamer1.0-doc:arm64 \
    gstreamer1.0-tools:arm64 \
    libgstreamer1.0-dev:arm64 \
    libgstreamer-plugins-base1.0-dev:arm64

RUN apt-get install -y --no-install-recommends libgtk2.0-dev:arm64

COPY arm-openvino.toolchain.cmake .

ENV PKG_CONFIG_LIBDIR /usr/lib/aarch64-linux-gnu/pkgconfig

# Build OpenCV
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.4.zip --no-check-certificate
RUN unzip opencv.zip && mv opencv-3.4.4 opencv
RUN mkdir -p ~/opencv/build
RUN cd ~/opencv/build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_TOOLCHAIN_FILE=/root/arm-openvino.toolchain.cmake \
          -DBUILD_EXAMPLES=OFF \
          -DBUILD_DOCS=OFF \
          -DBUILD_PERF_TESTS=OFF \
          -DBUILD_TESTS=OFF \
          -DWITH_GSTREAMER=ON \
          -DWITH_GTK=ON ..
RUN cd ~/opencv/build && make --jobs=$(nproc --all)

# Install Inference Engine
RUN git clone https://github.com/opencv/dldt.git
RUN cd ~/dldt && git checkout 2019_R3
RUN cd ~/dldt/inference-engine && \
    git submodule init && \
    git submodule update --recursive
RUN mkdir ~/dldt/inference-engine/build
RUN cd ~/dldt/inference-engine/build && \
    OpenCV_DIR=/root/opencv/build \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_TOOLCHAIN_FILE="/root/arm-openvino.toolchain.cmake" \
          -DENABLE_MKL_DNN=OFF \
          -DENABLE_CLDNN=OFF \
          -DENABLE_GNA=OFF \
          -DENABLE_SSE42=OFF \
          -DTHREADING=SEQ ..
RUN cd ~/dldt/inference-engine/build && \
    make --jobs=$(nproc --all)

# Install Boost
RUN wget --no-check-certificate https://sourceforge.net/projects/boost/files/boost/1.72.0/boost_1_72_0.tar.bz2
RUN tar xjf boost_1_72_0.tar.bz2

RUN cd boost_1_72_0 && \
    ./bootstrap.sh --prefix=/root/boost && \
    echo "using gcc : arm : aarch64-linux-gnu-g++ ;" > ~/user-config.jam && \
    ./b2 -j $(nproc --all) --with-atomic --with-chrono --with-date_time --with-filesystem --with-regex --with-thread --with-system --no-samples --no-tests toolset=gcc-arm threading=multi link=static runtime-link=static --layout=tagged --build-type=complete stage

# Install Python 3.7
RUN apt-get install -y --no-install-recommends \
    libffi-dev \
    zlib1g-dev

RUN wget --no-check-certificate https://www.python.org/ftp/python/3.7.3/Python-3.7.3.tar.xz
RUN tar -xf Python-3.7.3.tar.xz
RUN cd Python-3.7.3 && ./configure && make --jobs=$(nproc --all) build_all && make install

WORKDIR /root/build
