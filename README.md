# 제7회 국민대학교 자율주행 경진대회 예선2

## Installation(WSL2)

``` bash
$ cd ~/
$ git clone https://github.com/sangmyung-hwansoo-competitions/qualifying2.git
$ cd qualifying2
$ sudo ./build_container_ubuntu.sh
```

## Usage

``` bash
# CarlaUE4 디렉토리 속 퍼블릭키 입력 후 윈도우 상에서 CarlaUE4.exe 실행
$ sudo docker exec -it q2 bash
$ roslaunch integration temp.launch
```
