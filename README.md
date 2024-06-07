# 제7회 국민대학교 자율주행 경진대회 예선2

## Project Tree (WSL2)

``` bash
.
└── src
    ├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
    ├── control
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src
    │       ├── lane_keeping_control.py
    │       ├── pid_controller.py
    │       └── teleop_keyboard.py
    ├── integration
    │   ├── CMakeLists.txt
    │   ├── bag
    │   ├── config
    │   │   ├── integration.yaml
    │   │   ├── ov2slam.yaml
    │   │   └── sensors_info.yaml
    │   ├── launch
    │   │   ├── play.launch
    │   │   └── record.launch
    │   └── package.xml
    ├── perception
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src
    │       ├── image_pre_processor.py
    │       ├── lane_detector.py
    │       ├── moving_average_filter.py
    │       ├── perception_utils.py
    │       ├── sig_sync.py
    │       └── start_signal.py
    ├── start.launch
    ├── test.launch
    └── visualization
        ├── CMakeLists.txt
        ├── package.xml
        ├── rviz
        │   └── start.rviz
        └── src
            └── show_image.py
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
$ cd src
$ roslaunch test.launch
```
