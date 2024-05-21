# 제7회 국민대학교 자율주행 경진대회 예선2

## Project Tree (WSL2)

``` bash
.
├── Dockerfile
├── README.md
├── build_container_ubuntu.sh
├── catkin_ws
│   └── src
│       ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
│       ├── integration
│       │   ├── CMakeLists.txt
│       │   ├── launch
│       │   │   └── temp.launch
│       │   ├── package.xml
│       │   └── rviz
│       └── preparation
│           ├── CMakeLists.txt
│           ├── package.xml
│           └── src
│               └── test_control.py
└── sim_dependency
    ├── carla-0.9.10-cp38-cp38-linux_x86_64.whl
    ├── deb
    │   ├── libcarla-0.9.10.deb
    │   ├── recast-1.5.1.deb
    │   ├── ros-noetic-carla-ctl_0.0.0-0focal_amd64.deb
    │   ├── ros-noetic-xycar-msgs_0.0.0-0focal_amd64.deb
    │   └── rpclib-2.2.1-3.deb
    └── requirements.txt
```

## Installation (WSL2)

``` bash
$ cd ~/
$ git clone https://github.com/sangmyung-hwansoo-competitions/qualifying2.git
$ cd qualifying2
$ sudo ./build_container_ubuntu.sh
```


## Usage (WSL2)

``` bash
# CarlaUE4 디렉토리 속 퍼블릭키 입력 후 윈도우 상에서 CarlaUE4.exe 실행
$ sudo docker exec -it q2 bash
$ roslaunch integration temp.launch
or
$ start
```
