---
layout: post
title:  "Turtlebot Setup"
date:   2019-09-23 09:00:13
categories: 2019-2-robotics
---



# 터틀봇(Turtlebot) 소개

터틀봇은 Willow Garage 사의 Melonee Wise와 Tully Foote 내부 프로젝트로부터 시작했다. 2010년 Melonee는 새로 나온 Kinect 센서를 이용한 주행알고리즘을 시험하기 위해 iRobot Create에 붙여보았는데 바로 잘 작동하진 않았다. ROS로 작동시킬 수 있는 주행 플랫폼은 PR2와 Lego NXT 등이 있었지만 PR2는 너무 크고 비쌌고 NXT는 다양한 센서를 장착하기 어려웠다. Melonee와 Tully는 Create을 ROS를 위한 주행용 플랫폼으로 만들기 위해 작은 사내 프로젝트를 만들었다. 처음엔 ROS용 Create의 드라이버를 만들고 로봇의 전원이 Kinect 센서를 작동시키기엔 부족해서 전원 보드를 직접 만들었고 Odometry가 정확하지 않아서 Gyro 센서를 장착했다. 그러다 우연히 구글의 클라우드 로봇 프로젝트와 연결이 되어 개발용으로 만들었던 Create 기반 주행 플랫폼을 구글에 판매하게 되었다. 이를 계기로 터틀봇을 상품화시켜 2011년부터 판매하게 되었다.  

기존에는 주행연구를 하려면 Pioneer 3DX 같은 수백~수천만원짜리 로봇이 있어야 로봇을 제어하고 이동정보를 받을수 있었다. 반면 터틀봇은 가격이 저렴하고 다양한 PC나 센서에 연결할 수 있어 확장성이 좋았다. 무엇보다도 ROS를 이용하면 편리하게 제어할 수 있었다. 사람들은 직접 드라이버를 설치하고 로봇을 제어하는 API를 공부하지 않아도 ROS에서 제공하는 드라이버와 표준 메시지를 사용하면 로봇을 제어하고 센서 정보를 얻을 수 있게 되었다. 이러한 장점 덕분에 터틀봇은 많은 연구소에 널리 퍼졌고 이용자가 많아질수록 관련 소프트웨어가 풍부해지고 터틀봇을 사용한 논문도 많이 발표되었다.

2012년에는 국내의 유진 로봇에서 자사의 로봇 청소기 플랫폼을 기반으로 Willow Garage와 협업해 거북이(Kobuki)를 만들었다. 거북이는 터틀봇3(Turtlebot 2)로도 불렸으며 기존 터틀봇보다 플랫폼을 다양하게 확장할 수 있었다.

2017년에는 국내 로보티즈라는 회사에서 터틀봇3(Turtlebot 3)를 출시했다. 터틀봇3는 모듈형 판넬을 이용하여 조립식으로 층을 쌓아갈 수 있어 기존의 정형화된 확장 프레임을 벗어나 사용자가 자유롭게 원하는 모양을 조립할 수 있도록 설계했다. 저렴한 Raspberry Pi와 360도 Lidar 센서가 기본 장착되어 주행 어플리케이션을 더 쉽게 만들수 있게되었다.

![turtlebot](../assets/robotics-ros/turtlebot_family.png)

**출처**

<https://spectrum.ieee.org/automaton/robotics/diy/interview-turtlebot-inventors-tell-us-everything-about-the-robot>  

<https://www.turtlebot.com/>



# Raspberry Pi Setup

터틀봇3에는 기본적으로 Raspberry Pi가 장착되어있다. 우선 용어의 혼동을 줄이기 위해 용어들을 정리한다.

- Raspberry Pi: 원래는 개발도상국에서의 컴퓨터 교육을 위한 저가형 PC로 개발되었으나 오히려 연구용이나 취미용으로 많이 쓰이면서 판매가 크게 늘었다. 터틀봇3에 장착된 모델은 Raspberry Pi 3 Model B다.
- SBC (Single Board Computer): Raspberry Pi처럼 하나의 보드에 CPU, 메모리, 저장장치 등이 모두 장착되어 하나의 컴퓨터를 구성한 것을 말한다. 로봇공학 수업에서 "SBC"는 Raspberry Pi를 지칭한다.
- Raspbian: Raspberry Pi를 위한 데비안 계열 리눅스 배포판이다. 공식 홈페이지: <https://www.raspberrypi.org/>
- Ubuntu MATE: 우분투 메이트가 아니다! **우분투 마테**다. 기본 우분투는 SBC에서 사용하기에는 GUI 환경이 무겁기 때문에 데스크톱 환경을 가벼운 MATE로 바꾼것이다. Raspberry Pi를 지원하여 Raspberry Pi에서도 많이 쓰인다. 공식 홈페이지: <https://ubuntu-mate.org/>

Raspberry Pi에는 Raspbian과 Ubuntu MATE 둘 다 사용가능하다. ROS Kinetic Kame을 쓸 때는 Raspbian을 쓰는게 문제 없었지만 Melodic Morenia를 쓰려면 ROS를 소스에서부터 빌드해야 한다고 한다. 아무래도 ROS가 Ubuntu와 버전을 맞추다보니 Melodic Morenia와는 호환성 문제가 생긴듯 하다. 따라서 Melodic Morenia를 쓰기 위해서는 Ubuntu MATE 18.04 버전을 쓰는 것이 편할 것이다.

## 1. Ubuntu MATE 설치

우분투 마테를 쓰려면 SBC에 장착된 SD 카드를 꺼내서 PC에서 우분투 마테 부팅 디스크로 만들어야 한다. 먼저 아래 링크에서 Raspberry Pi 버전을 다운 받는다.  

<https://ubuntu-mate.org/download/>  

이후 PC에서 Rufus를 이용해 부팅 디스크를 만든다.

<https://rufus.ie/>  

SD카드를 Raspberry Pi에 끼운후 전원을 넣어서 켜면 설치과정이 시작된다. 우분투 데스크탑과 마찬가지로 언어와 키보드를 영어로 선택하고 사용자 계정을 다음과 같이 정한다.

- Your name, computer's name, username: turtle-xx (xx는 임의로 정함)
- Password: robot

![account](../assets/robotics-ros/mate-account.jpg)

설치가 완료되면 다음과 같은 화면이 나와야한다.

![mate-desktop](../assets/robotics-ros/mate-desktop.png)

그런데 직접 설치해본 결과 상단의 메뉴 바가 보이지 않는 경우가 있다. 이런 경우에는 다음과 같이 해결한다.

1. 터미널을 열고(ctrl+alt+T)  `mate-tweak` 실행
2. "Panel"에서 "Familar"를 다른 것으로 바꾸면 메뉴바가 활성화 된다.
3. 다시 "Familiar"로 돌아오면 기본 테마의 메뉴바가 활성화 된다.

정상작동하면 업그레이드를 먼저 진행한다.

```
sudo apt-get update
sudo apt-get upgrade -y
```



### 1.1 디스크 복사를 통한 설치

위 과정을 거치는 것은 번거롭기도 하고 시간이 많이 걸린다. 그러므로 미리 저 과정을 통해 SD카드를 하나 만들어 놓고 SD카드를 복제하는 것이 편할수 있다. 이를 위해서는 두 개의 저장장치가 필요하다. 하나는 (A) 우분투 마테가 세팅된 것이고 (B) 다른 하나는 용량이 비슷한 다른 저장장치다. 복사는 다음 순서대로 진행한다.

1. A를 연결하고 `lsblk` 명령어를 통해 저장장치의 마운트 위치를 알아낸다. 통상 `/dev/sdx` 위치에 마운트가 되고 `x`만 순서대로 변한다. **마운트 위치를 헷갈리면 호스트 시스템이 날아갈수 있습니다. 잘 확인하세요!**

2. 다음 명령어를 통해 A를 파일로 백업한다. A가 `/dev/sda`에 마운트 되었다면

   ```
   sudo dd if=/dev/sda of=~/rp_mate_18.04.iso bs=1024k
   ```

3. 그 다음 B를 연결하고 `lsblk` 명령어를 통해 B의 마운트 위치를 알아낸다.

4. 다음 명령어를 통해 이미지 파일을 B에 복사한다. B가 `/dev/sdb`에 마운트 되었다면

   ```
   sudo dd if=~/rp_mate_18.04.iso of=/dev/sdb bs=1024k
   ```



## 2. ROS 설치 (SBC)

SBC에 ROS Melodic을 설치하는 과정은 데스크탑과 유사하지만 조금씩 다르니 아래 가이드를 따라 설치한다.



## 1. ROS 패키지 저장소 추가

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



## 2. 시간 동기화

PC들 사이에 통신을 하려면 시간이 서버와 동기화 되어있어야 한다. 동기화에 필요한 `ntpdate` 및 필요한 다른 패키지들을 설치한다.

```
$ sudo apt install -y build-essential chrony ntpdate net-tools
$ sudo ntpdate ntp.ubuntu.com
```



## 3. ROS 패키지 설치

`ros-melodic-ros-base`는 GUI 관련 패키지를 제외한 최소 설치 세트다. SBC에서는 단순히 로봇과 센서 정보를 데스크탑에 입출력해주는 기능만 하기 때문에 많은 기능이 필요하지 않다.

```
$ sudo apt install ros-melodic-ros-base
```



## 4. rosdep 초기화

`rosdep`은 소스 코드로부터 컴파일시 필요한 시스템 dependency를 자동으로 설치해주는 유틸이다.

```
$ sudo rosdep init
$ rosdep update
```



## 5. Catkin Workspace 초기화

패키지를 만들고 빌드할 워크스페이스 디렉토리를 만들고 워크스페이스를 초기화한다. 여기서는 ROS에서 기본 제공하는 catkin 시스템을 사용한다.

```
$ source /opt/ros/melodic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
~/catkin_ws$ catkin_init_workspace
~/catkin_ws$ catkin_make
```



## 6. IP 확인

ROS는 마스터를 중심으로 통신을 하는데 여러 PC들 사이에 정보를 주고 받는 경우에는 마스터의 IP를 알고 있어야 한다. IP는 `ifconfig` 명령어로 찾을 수 있다. 여러 주소가 나오는데 그 중 `192.168.`로 시작하는 주소를 메모해둔다.

```
$ ifconfig
...
wlo1: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.219.189  netmask 255.255.255.0  broadcast 192.168.219.255
        inet6 fe80::e300:7b4b:5a8a:cc5f  prefixlen 64  scopeid 0x20<link>
        ether 0c:54:15:43:bb:0b  txqueuelen 1000  (Ethernet)
        RX packets 348844  bytes 501131238 (501.1 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 65528  bytes 9321861 (9.3 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```



## 7. 초기화 스크립트 추가

`~/.bashrc` 파일은 bash 터미널을 열때 자동으로 실행되는 스크립트다. 여기에 ROS를 위한 기본 세팅을 추가하면 터미널에서 직접 명령을 실행하지 않아도 된다. 특히 `source /opt/ros/melodic/setup.bash`를 해야만 터미널에서 ros 명령어를 쓰고 ros 패키지 빌드도 할 수 있다.  

```
$ gedit ~/.bashrc
# .bashrc 아래에 다음 텍스트 추가 후 저장하고(ctrl+s) 닫기
alias cw='cd ~/catkin_ws'
alias cm='cd ~/catkin_ws && catkin build'
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost

$ source ~/.bashrc
```

SBC는 처리속도가 느려서 자체적으로 알고리즘을 실행하기에는 부담스럽고 데스크탑으로 로봇 정보를 입출력해주는 역할을 한다. 따라서 데스크탑과 연동될 수 있도록 주소를 다음과 같이 세팅해야 한다.

- ROS_MASTER_URI: `localhost` 대신에 데스크탑의 IP를 적는다. 이때 데스크탑의 ROS_MASTER_URI도 동일하게 설정되어 있어야 한다.
- ROS_HOSTNAME: `localhost` 대신에 SBC의 IP를 적는다.



## 8. 터틀봇 패키지 설치

터틀봇 자율주행을 하기 위해서는 로봇에 속도 명령을 내리고 현재 이동량을 읽고 LiDAR 센서 값을 받아올 수 있어야 한다. 이러한 기능을 하기 위해 아래 패키지들을 설치한다.

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws/src/turtlebot3
$ sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
$ sudo apt-get install ros-kinetic-rosserial-python ros-kinetic-tf
$ source /opt/ros/kinetic/setup.bash
$ cd ~/catkin_ws
$ catkin_make
```

OpenCR이 루트 권한을 얻지 않아도 USB를 이용할 수 있도록 설정한다.

```
$ rosrun turtlebot3_bringup create_udev_rules
```

