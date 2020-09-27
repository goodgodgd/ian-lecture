---
layout: post
title:  "ROS Setup"
date:   2020-09-27 09:00:13
categories: 2020-2-robotics
---



## ROS 설치

SBC에 ROS Melodic을 설치하는 과정은 Remote PC와 유사하지만 조금씩 다르니 아래 가이드를 따라 설치한다.



### 1. ROS 패키지 저장소 추가

ROS 패키지들은 `apt`를 통해서 설치할 수 있는데 그러려면 ROS 저장소를 추가해야 한다.

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
> /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt update
```

ROS 패키지를 신뢰할만한 패키지로 검증하는데 필요한 키를 등록한다.

```
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
Executing: /tmp/apt-key-gpghome.BoHU3zyXWt/gpg.1.sh --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
gpg: key F42ED6FBAB17C654: public key "Open Robotics <info@osrfoundation.org>" imported
gpg: Total number processed: 1
gpg:               imported: 1
```

"Open Robotics"가 키 리스트에 추가됐음을 확인한다.

```
$ sudo apt-key list | grep "Open Robotics"
Warning: apt-key output should not be parsed (stdout is not a terminal)
uid           [ unknown] Open Robotics <info@osrfoundation.org>
```



### 2. 패키지 /설치

PC들 사이에 통신을 하려면 시간이 서버와 동기화 되어있어야 한다. 동기화에 필요한 `ntpdate`와 원격 접속에 필요한 `openssh` 등 필요한 패키지들을 미리 설치한다.

```
$ sudo apt install -y build-essential chrony ntpdate net-tools gedit git openssh-server openssh-client
$ sudo ntpdate ntp.ubuntu.com
```



### 3. ROS 패키지 설치

`ros-melodic-desktop-full`은 ROS, [rqt](http://wiki.ros.org/rqt), [rviz](http://wiki.ros.org/rviz), robot-generic libraries, 2D/3D simulators, navigation and 2D/3D perception 등의 패키지들을 한꺼번에 설치할 수 있는 메타패키지다. `python-catkin-tools`는 catkin workspace를 효과적으로 관리할 수 있는 툴이다.

```
$ sudo apt install ros-melodic-desktop-full python-catkin-tools
```



### 4. rosdep 초기화

`rosdep`은 소스 코드로부터 컴파일시 필요한 시스템 dependency를 자동으로 설치해주는 유틸이다.

```
$ sudo rosdep init
$ rosdep update
```



### 5. Catkin Workspace 초기화

패키지를 만들고 빌드할 워크스페이스 디렉토리를 만들고 워크스페이스를 초기화한다. 초기화에 사용되는 `catkin` 유틸에 대한 설명은 다음 시간에 한다.

```
$ source /opt/ros/melodic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
~/catkin_ws$ catkin init
~/catkin_ws$ catkin build
~/catkin_ws$ ls -al
```




### 6. 초기화 스크립트 추가

`~/.bashrc` 파일은 bash 터미널을 열때 자동으로 실행되는 스크립트다. 여기에 ROS를 위한 기본 세팅을 추가하면 터미널에서 직접 명령을 실행하지 않아도 된다. 특히 `source /opt/ros/melodic/setup.bash`를 해야만 터미널에서 ros 명령어를 쓰고 ros 패키지 빌드도 할 수 있다.  

```
$ gedit ~/.bashrc
# .bashrc 아래에 다음 텍스트 추가 후 저장하고(ctrl+s) 닫기
alias cw='cd ~/catkin_ws'
alias cm='cd ~/catkin_ws && catkin_make'
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export TURTLEBOT3_MODEL=burger

$ source ~/.bashrc
```




### 7. 터틀봇 패키지 설치

터틀봇 자율주행을 하기 위해서는 로봇에 속도 명령을 내리고 현재 이동량을 읽고 LiDAR 센서 값을 받아올 수 있어야 한다. 이러한 기능을 하기 위해 아래 패키지들을 설치한다.

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws/src/turtlebot3
$ sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ sudo apt install ros-melodic-rosserial-python ros-melodic-tf
$ source /opt/ros/melodic/setup.bash
$ cd ~/catkin_ws
$ catkin build
```

OpenCR이 루트 권한을 얻지 않아도 rosrun를 이용할 수 있도록 설정한다.

```
$ rosrun turtlebot3_bringup create_udev_rules
```



### 8. ROS 동작 테스트

아래 명령어를 각각 다른 터미널에서 실행하여 거북이를 조종해본다. 터미널을 열고(ctrl+alt+T) 한 명령을 실행한 후 새 탭을 열어(ctrl+shift+T) 다음 명령어를 실행한다.

```
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtle_teleop_key
$ rosrun rqt_graph rqt_graph
$ rostopic echo /turtle1/cmd_vel
```



### 9. 펌웨어 업데이트

혹시 펌웨어 버전이 오래되어 업데이트를 해야한다면 SBC에서 아래 명령어를 통해 업데이트를 해준다.

출처: https://discourse.ros.org/t/announcing-turtlebot3-software-v1-0-0-and-firmware-v1-2-0-update/4888

```bash
#opencr 업데이트
$ export OPENCR_PORT=/dev/ttyACM0
$ export OPENCR_MODEL=burger
$ rm -rf ./opencr_update.tar.bz2

#한 줄의 명령어
$ wget https://github.com/ROBOTIS-GIT/OpenCR/raw/master/arduino/opencr_release/shell_update/opencr_update.tar.bz2 && tar -xvf opencr_update.tar.bz2 && cd ./opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr && cd ..
#여기까지

#패키지 소스 지우고 다시 다운로드
$ cd ~/catkin_ws/src/
$ rm -rf turtlebot3/ turtlebot3_msgs/ hls_lfcd_lds_driver/
$ git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git

#터틀봇에 불필요한 노드 삭제
$ cd ~/catkin_ws/src/turtlebot3
$ sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/

#기존 빌드파일 삭제 후 다시 빌드
$ cd ~/catkin_ws/
$ rm -rf build/ devel/
$ cd ~/catkin_ws && catkin_make -j1
```

