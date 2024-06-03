# 베이스 이미지 설정
FROM ubuntu:20.04

# 비대화식 설치 모드 설정 및 패키지 소스 변경
ARG DEBIAN_FRONTEND=noninteractive
RUN cd /etc/apt && sed -i 's/kr.archive.ubuntu.com/mirror.kakao.com/g' sources.list \
    && sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' sources.list \
    && sed -i 's/security.ubuntu.com/mirror.kakao.com/g' sources.list

# 기본 패키지 설치
RUN apt update && apt install -y sudo wget git curl locales zip gpg tzdata keyboard-configuration console-setup vim gedit

# 시간대 및 키보드 설정
RUN unlink /etc/localtime && ln -s /usr/share/zoneinfo/Asia/Seoul /etc/localtime \
    && echo 'keyboard-configuration keyboard-configuration/layoutcode string kr' | debconf-set-selections \
    && echo 'keyboard-configuration keyboard-configuration/layout select Korean' | debconf-set-selections \
    && echo 'keyboard-configuration keyboard-configuration/modelcode string pc105' | debconf-set-selections \
    && echo 'keyboard-configuration keyboard-configuration/model select Generic 105-key PC (intl.)' | debconf-set-selections \
    && dpkg-reconfigure --frontend=noninteractive keyboard-configuration console-setup

# 로케일 설정
RUN locale-gen ko_KR.UTF-8 && update-locale LANG=ko_KR.UTF-8

# C++ 개발 환경 설치
RUN apt update && apt install -y build-essential gcc g++ cmake

# ROS noetic 설치
RUN apt update && apt install -y gnupg curl lsb-release openssl && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \
    apt install -y ros-noetic-desktop-full ros-noetic-teleop-twist-keyboard ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs ros-noetic-hector-trajectory-server ros-noetic-cv-bridge ros-noetic-tf ros-noetic-message-filters ros-noetic-image-transport ros-noetic-pcl-ros && \
    apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-pip && \
    rosdep init && rosdep update

RUN apt autoremove -y

# ORB-SLAM3 설치 위한 종속성
RUN apt install -y gnupg2 curl lsb-core vim wget python3-pip libpng16-16 libjpeg-turbo8 libtiff5
RUN apt install -y \
        # Base tools
        cmake \
        build-essential \
        git \
        unzip \
        pkg-config \
        python3-dev \
        # OpenCV dependencies
        python3-numpy \
        # Pangolin dependencies
        libgl1-mesa-dev \
        libglew-dev \
        libpython3-dev \
        libeigen3-dev \
        apt-transport-https \
        ca-certificates\
        software-properties-common

# Build OpenCV
RUN apt install -y python3-dev python3-numpy python-dev python-numpy libavcodec-dev libavformat-dev libswscale-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libgtk-3-dev

RUN cd /tmp && git clone https://github.com/opencv/opencv.git && git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv_contrib && \
    git checkout 4.4.0 && cd .. && \
    cd opencv && \
    git checkout 4.4.0 && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
          -D BUILD_opencv_python3=OFF \
          -D BUILD_opencv_python2=OFF \
          -D BUILD_DOCS=OFF \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_TESTS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D BUILD_opencv_java=OFF \
          -D BUILD_opencv_js=OFF \
          -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/opencv /tmp/opencv_contrib

# Build Pangolin
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && git checkout v0.6 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/Pangolin

# Build Ceres
RUN apt install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev

# RUN cd /tmp && git clone https://github.com/ceres-solver/ceres-solver.git && \
#     cd ceres-solver && git checkout 1.14.x && mkdir build && cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
#     make -j$(nproc) && make install && \
#     cd / && rm -rf /tmp/ceres-solver

# Build DSO
RUN apt install -y libsuitesparse-dev libeigen3-dev libboost-all-dev zlib1g-dev

RUN cd /tmp && git clone https://github.com/Lee-hwansoo/dso.git && \
    cd dso && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j12

# Build opengv
RUN cd /tmp && git clone https://github.com/laurentkneip/opengv.git && \
    cd opengv && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-std=c++11 -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/opengv

# 새로운 사용자 생성 및 홈 디렉토리 설정
ARG USERNAME
ARG PASSWORD
ARG WORKSPACE
ARG USER_UID
ARG USER_GID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo "$USERNAME:$PASSWORD" | chpasswd \
    && usermod -aG sudo $USERNAME

# sudo 권한 부여 및 비밀번호 없이 sudo 사용 가능하도록 설정
RUN echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME
