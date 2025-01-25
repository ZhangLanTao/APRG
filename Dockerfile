# This dockerfile is mainly from hybridgroup, some modifications are made to fit the needs of the project
# hybridgroup: 
# to build this docker image:
#   docker build -f Dockerfile.opencv-ubuntu-22.04 -t ghcr.io/hybridgroup/opencv:4.8.1-ubuntu-22.04 .
#   docker build --build-arg OPENCV_VERSION="4.x" --build-arg OPENCV_FILE="https://github.com/opencv/opencv/archive/refs/heads/4.x.zip" --build-arg OPENCV_CONTRIB_FILE="https://github.com/opencv/opencv_contrib/archive/refs/heads/4.x.zip" -f Dockerfile.opencv-ubuntu-20.04 -t ghcr.io/hybridgroup/opencv:4.8.1-dev-ubuntu-20.04 .

FROM ubuntu:22.04
LABEL maintainer="zlt"

ENV TZ=Asia/Shanghai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install build dependencies
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y --no-install-recommends --fix-missing \
    build-essential binutils \
    ca-certificates cmake cmake-qt-gui curl \
    dbus-x11 \
    ffmpeg \
    gdb gcc g++ gfortran git \
    tar \
    lsb-release \
    procps \
    manpages-dev \
    unzip \
    zip \
    wget \
    xauth \
    swig \
    python3-pip python3-dev python3-numpy python3-distutils \
    python3-setuptools python3-pyqt5 python3-opencv \
    libboost-python-dev libboost-thread-dev libatlas-base-dev libavcodec-dev \
    libavformat-dev libavutil-dev libcanberra-gtk3-module libeigen3-dev \
    libglew-dev libgl1-mesa-dev libgl1-mesa-glx libglib2.0-0 libgtk2.0-dev \
    libgtk-3-dev libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev liblapack-dev \
    liblapacke-dev libopenblas-dev libopencv-dev libpng-dev libpostproc-dev \
    libpq-dev libsm6 libswscale-dev libtbb-dev libtbb2 libtesseract-dev \
    libtiff-dev libtiff5-dev libv4l-dev libx11-dev libxext6 libxine2-dev \
    libxrender-dev libxvidcore-dev libx264-dev libgtkglext1 libgtkglext1-dev \
    libvtk9-dev libdc1394-dev libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev libopenexr-dev \
    openexr \
    pkg-config \
    qv4l2 \
    v4l-utils \
    zlib1g-dev \
    locales \
    && locale-gen en_US.UTF-8 \
    && LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# OpenCV
ARG OPENCV_VERSION="4.8.1"
ENV OPENCV_VERSION $OPENCV_VERSION

ARG OPENCV_FILE="https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip"
ENV OPENCV_FILE $OPENCV_FILE

ARG OPENCV_CONTRIB_FILE="https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip"
ENV OPENCV_CONTRIB_FILE $OPENCV_CONTRIB_FILE

RUN curl -Lo opencv.zip ${OPENCV_FILE} && \
    unzip -q opencv.zip && \
    curl -Lo opencv_contrib.zip ${OPENCV_CONTRIB_FILE} && \
    unzip -q opencv_contrib.zip && \
    rm opencv.zip opencv_contrib.zip && \
    cd opencv-${OPENCV_VERSION} && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D WITH_IPP=OFF \
    -D WITH_OPENGL=OFF \
    -D WITH_QT=OFF \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-${OPENCV_VERSION}/modules \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D WITH_JASPER=OFF \
    -D WITH_TBB=ON \
    -D BUILD_JPEG=ON \
    -D WITH_SIMD=ON \
    -D ENABLE_LIBJPEG_TURBO_SIMD=ON \
    -D BUILD_DOCS=OFF \
    -D BUILD_EXAMPLES=OFF \
    -D BUILD_TESTS=OFF \
    -D BUILD_PERF_TESTS=ON \
    -D BUILD_opencv_java=NO \
    -D BUILD_opencv_python=NO \
    -D BUILD_opencv_python2=NO \
    -D BUILD_opencv_python3=NO \
    -D OPENCV_GENERATE_PKGCONFIG=ON .. && \
    make -j $(nproc --all) && \
    make preinstall && make install && ldconfig && \
    cd / && rm -rf opencv*

# When using Dev Container in VSCode, open APRG folder in the container, data volume is auto mounted, so no need to copy data
# COPY ./include /APRG/include
# COPY ./src /APRG/src
# COPY ./CMakeLists.txt /APRG/CMakeLists.txt
# WORKDIR /APRG
# RUN mkdir build && cd build && cmake .. && make -j $(nproc --all)
