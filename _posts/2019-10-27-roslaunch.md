---
layout: post
title:  "Name Remapping and Roslaunch"
date:   2010-10-27 09:00:13
categories: WIP
---



# ROS Name Remapping

ROS의 모든 구성 요소(노드, 토픽, 서비스, 파라미터) 등은 고유의 네임(Name)을 가진다. 예를 들어 각 노드는 고유의 네임을 가지기 때문에 같은 네임의 노드 두 개를 동시에 실행하지 못한다. 터틀심 패키지로 시험해보자.

```bash
$ roscore
# 새 탭
$ rosrun turtlesim turtlesim_node
# 새 탭
$ rosrun turtlesim turtlesim_teleop_key
# 거북이 조작 ...
# 새 탭
$ rosrun turtlesim turtlesim_node
```

마지막 명령어를 넣는 순간 기존 `turtlesim_node`는 종료되고 새로운 `turtlesim_node`가 실행되는 것을 볼 수 있다.



<http://wiki.ros.org/Remapping%20Arguments>



# Roslaunch





---

## TODO

- 10.30: pyenv 끝내
- 11.5: ros name and roslaunch, 프로젝트 공지
- 11.6: rviz, rqt
- 11.12: 2차원 좌표계 변환
- 11.13: 
- 11.19: 2차원 좌표계 변환 코딩
- 11.20:
- 11.26: 3차원 좌표계 변환
- 11.27:
- 12.3: 3차원 좌표계 변환 코딩
- 12.4: