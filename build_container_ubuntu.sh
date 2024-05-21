#!/bin/bash

set -e

if [ "$EUID" -ne 0 ]; then
  echo "이 스크립트는 root 권한이 필요합니다. 'sudo'를 사용하여 실행하십시오."
  exit 1
else
  echo "root 권한으로 실행 중입니다."
fi

# 도커가 설치되어 있는지 확인
if ! command -v docker &> /dev/null
then
    echo "도커가 설치되어 있지 않습니다. 도커를 설치해 주세요."
    exit 1
fi

# 컨테이너 생성 및 실행 변수 설정
CONTAINER_NAME="q2"
IMAGE_NAME="ubuntu_q2:20.04"
DOCKERFILE_PATH=$(pwd)/Dockerfile

USERNAME=ubuntu
PASSWORD=ubuntu
USER_UID=1000
USER_GID=1000

LOCAL_WORKSPACE_DIR=$(pwd)/catkin_ws
DOCKER_WORKSPACE_DIR=/home/$USERNAME/catkin_ws
LOCAL_SIM_DIR=$(pwd)/km7_v2
DOCKER_SIM_DIR="/home/$USERNAME/km7_v2"

# UI permissions
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
xhost +local:docker

# Dockerfile이 있는지 확인
if [ ! -f "$DOCKERFILE_PATH" ]; then
    echo "Dockerfile을 찾을 수 없습니다. 올바른 경로에 Dockerfile을 배치해 주세요."
    exit 1
fi

# 이미지 빌드
echo "Docker 이미지 빌드 중: $IMAGE_NAME"
docker build --build-arg USERNAME=$USERNAME --build-arg PASSWORD=$PASSWORD --build-arg WORKSPACE=$DOCKER_WORKSPACE_DIR --build-arg USER_UID=$USER_UID --build-arg USER_GID=$USER_GID -t $IMAGE_NAME -f $DOCKERFILE_PATH .

# 기존 컨테이너 제거
docker rm -f $CONTAINER_NAME &>/dev/null

# 새 컨테이너 생성 및 실행
echo "새 우분투 20.04 컨테이너 생성 및 실행"
docker run -td --privileged --net=host --ipc=host \
    --name="$CONTAINER_NAME" \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "$XSOCK:$XSOCK:rw" \
    -e "XAUTHORITY=$XAUTH" \
    -e "ROS_IP=127.0.0.1" \
    --cap-add=SYS_PTRACE \
    -v $LOCAL_WORKSPACE_DIR:$DOCKER_WORKSPACE_DIR \
    -v $LOCAL_SIM_DIR:$DOCKER_SIM_DIR \
    -w $DOCKER_WORKSPACE_DIR \
    --user=$USER_UID:$USER_GID \
    $IMAGE_NAME bash

echo "$CONTAINER_NAME 컨테이너가 성공적으로 생성되었습니다."

# 시뮬레이터 디펜던시 설치
docker exec -it --user=root $CONTAINER_NAME bash -c "find $DOCKER_SIM_DIR/deb/ -name '*.deb' -exec apt install -y {} \; && \
    cd $DOCKER_SIM_DIR/Linux/DivineCarla-Linux && \
    pip3 install --ignore-installed -r requirements.txt && \
    pip3 uninstall carla && \
    pip3 install ./carla-0.9.10-cp38-cp38-linux_x86_64.whl"

# 워크스페이스 설정
docker exec -it --user=root $CONTAINER_NAME bash -c "chown -R $USERNAME:$PASSWORD $DOCKER_WORKSPACE_DIR"
docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

docker exec -it $CONTAINER_NAME bash -c "echo '' >> /home/$USERNAME/.bashrc && \
    echo 'alias cm=\"cd $DOCKER_WORKSPACE_DIR && catkin_make\"' >> /home/$USERNAME/.bashrc && \
    echo 'alias ccm=\"cd $DOCKER_WORKSPACE_DIR && rm -rf build/ devel/ && catkin_make\"' >> /home/$USERNAME/.bashrc && \
    echo 'source /opt/ros/noetic/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'source $DOCKER_WORKSPACE_DIR/devel/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'export ROS_MASTER_URI=http://localhost:11311' >> /home/$USERNAME/.bashrc && \
    echo 'export ROS_HOSTNAME=localhost' >> /home/$USERNAME/.bashrc && \
    echo 'export HOSTIP=$(grep -m1 "nameserver" /etc/resolv.conf | awk "{print \$2}")' >> /home/$USERNAME/.bashrc && \
    echo 'alias start=\"roslaunch integration temp.launch\"' >> /home/$USERNAME/.bashrc"

echo "프로젝트 설정이 성공적으로 완료되었습니다."
