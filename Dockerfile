# 베이스 이미지 설정
FROM ubuntu:20.04

# 비대화식 설치 모드 설정 및 패키지 소스 변경
ENV DEBIAN_FRONTEND=noninteractive
RUN cd /etc/apt && sed -i 's/kr.archive.ubuntu.com/mirror.kakao.com/g' sources.list \
    && sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' sources.list \
    && sed -i 's/security.ubuntu.com/mirror.kakao.com/g' sources.list

# 기본 패키지 설치
RUN apt update && apt upgrade -y && apt install -y sudo wget git curl locales zip gpg tzdata keyboard-configuration console-setup vim gedit

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
    apt install -y ros-noetic-desktop-full ros-noetic-teleop-twist-keyboard ros-noetic-ackermann-msgs ros-noetic-derived-object-msgs && \
    apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-pip && \
    rosdep init && rosdep update

RUN apt autoremove -y

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
