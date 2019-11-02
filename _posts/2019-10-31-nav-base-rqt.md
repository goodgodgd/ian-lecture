---
layout: post
title:  "Navigation Base and Rqt"
date:   2010-10-27 09:00:13
categories: WIP
---



# Navigation Base

자율주행 프로젝트를 수행하는데 어디서부터 시작해야 할지 막막한 학생들을 위하여 기본 틀을 제공하고자 한다. 자율주행 패키지를 구현한다면 매 프레임마다 세 가지 일을 반복해야 한다.

- LDS 토픽을 입력으로 받기
- LDS 입력을 바탕으로 주변 상황을 파악하고 로봇의 이동 속도 결정 (알고리즘)
- 속도를 토픽으로 발행하여 로봇 제어

여기서는 중간의 알고리즘을 제외하고 입출력 부분만 구현한 예시를 보여준다. 

## Create Package

다음 명령어를 통해 패키지와 노드 스크립트를 생성한다.

```
$ cd ~/catkin_ws/src
$ catkin create pkg self_drive --catkin-deps roscpp std_msgs sensor_msgs geometry_msgs
$ cd self_drive/src
$ touch self_drive.py
$ chmod a+x self_drive.py
```



## Write Node Script

`package.xml`이나 `CMakeLists.txt`는 기존 강의자료를 참조하여 정리한다. 프로젝트에서는 `package.xml` 좀 더 상세히 작성해야 한다. 노드 스크립트는 다음과 같이 작성할 수 있다.

```python
#!/path/to/venv/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SelfDrive:
    def __init__(publisher):
        self.publisher = publisher

    def lds_callback(scan):
        # scan 분석 후 속도 결정
        # ...
        turtle_vel = Twist()
        # 전진 속도 및 회전 속도 지정
        turtle_vel.linear.x = 0.1
        turtle_vel.angular.z = 0.2
        # 속도 출력
        publisher.publish(turtle_vel)

rospy.init_node('self_drive')
publisher = rospy.Publisher('cmd_vel', TimePose, queue_size=1)
driver = SelfDrive(publisher)
subscriber = rospy.Subscriber('scan', LaserScan, 
                              lambda scan: driver.lds_callback(scan))
rospy.spin()
```







# Rqt







---

## TODO

- 11.5: 가상환경 및 pycharm 설정, 프로젝트 공지
- 11.6: ros name and roslaunch
- 11.12: navigation base, rviz, rqt
- 11.13: 
- 11.19: 2차원 좌표계 변환
- 11.20: 
- 11.26: 2차원 변환 코딩
- 11.27: 3차원 좌표계 변환
- 12.3: 
- 12.4: 3차원 변환 코딩

