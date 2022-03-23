---
layout: post
title:  "[Ro] ROS Python Package"
date:   2020-01-13 09:00:13
categories: draft
---



19,20. 표준 단위, 좌표계
\21. 시간은 다른거 하면서 녹여
\22. 파일시스템 설명은 예제 하나 빌드해보고 나서 설명
\23. 빌드시스템 23.1~5까지
\24. package.xml 레퍼런스 https://www.ros.org/reps/rep-0149.html
CMakeLists.txt, [setup.py](http://setup.py/) 설명

2부
\01. 프로그래밍 스타일
스타일지정: C_Cpp.clang_format_fallbackStyle
저장시 자동 포매팅: editor.formatOnSave
\02. 파이썬 프로그래밍
src/.vscode 남기고 나머지 싹지우고 다시 시작



1. minimal example: publisher, subscriber node 따로 구현
2. publisher + subscriber node 구현
3. custom message package와 이를 이용한 publisher, subscriber nodes package
4. 3번을 C++로 구현

# 1. ROS 프로그래밍 전 기초지식

19,20. 표준 단위, 좌표계

\23. 빌드시스템 23.1~5까지

\01. 프로그래밍 스타일

스타일지정: C_Cpp.clang_format_fallbackStyle
저장시 자동 포매팅: editor.formatOnSave

\02. 파이썬 프로그래밍
src/.vscode 남기고 나머지 싹지우고 다시 시작

# 2. Python 기반 Topic 통신 패키지

\21. 시간은 다른거 하면서 녹여
\22. 파일시스템 설명은 예제 하나 빌드해보고 나서 설명

하나의 패키지 안에 publisher, subscriber 구현

# 3. 메시지 타입 생성

한 폴더 안 세 개의 패키지에 message, publisher, subscriber 구현

1. turtlesim에서 cmd_vel 메시지 타입 발행
2. cmd_vel을 처리해서 내보낼 새로운 메시지 타입 정의
3. vel_classifier에서 cmd_vel을 구독하고 새로운 메시지 타입으로 발행
   1. 속도에 따라 직진, 좌회전, 우회전, 제자리좌회전, 제자리우회전 문자열 보내기
4. vel_class_printer에서 메시지를 받아서 출력



다음엔 C++로 2번과 같은 예제 해보기

---

# 1. ROS 2 프로그래밍 전 기초지식

본격적인 ROS 2 프로그래밍에 들어가기 전에 알아야할 몇 가지 기초지식을 익힌다. 

## 1.1. ROS 2의 표준 단위

> 1999년 9월 23일 미국항공우주국(NASA)에서 발사한 화성 기후 관측 위성(MCO·Mars Climate Orbiter)이 화성 궤도에 진입하다 사라지는 사고가 발생한다. 원인을 분석한 결과 로켓의 추력 단위를  국제 표준단위인 *N·s*가 아닌 미국식 단위인 *lb·s*를 사용했기 때문에 계산과정에 오차가 발생한 것으로 밝혀졌다. 1 N = 4.45 lb 인데 뉴턴 단위 값을 파운드 단위 값에 입력했다. 원래 프로그램 사양서에는 국제표준단위를 사용하도록 되어있었지만 지켜지지 않았다.

단위(unit)는 중요하다. 문화권에 따라 같은 물리량에 대해 다른 단위를 쓸 수도 있고 같은 단위 체계에서도 스케일에 따라 다양한 단위가 있다. (mm, m, km, g, kg, ton 등) 어떠한 시스템 내에서 서로 다른 여러가지 단위가 섞여서 쓰인다면 불필요한 변환 과정이 많아지고 코드를 읽기도 어려워 결국 치명적인 버그로 이어지게 된다. 그래서 모든 개발자들이 합의한 표준단위가 필요하다. 표준단위를 쓰면 불필요한 변환과정도 필요없고 수치의 의미를 확인하는데 드는 노력을 줄일수 있고 전체적으로 버그가 줄어든다.  

ROS 커뮤니티에서는 ROS 프로그래밍에 쓰이는 표준 단위로 세계적으로 가장 널리 사용되는 국제단위계인 SI 단위와 국제단위계의 7개 기본 단위를 조합해 만들어진 SI 유도 단위(SI derived unit)를 표준 단위로 정하였다. 로봇공학에서 주로 사용되는 단위는 다음 표와 같다.

| 물리량         | 단위(SI unit) | 물리량             | 단위(SI derived unit) |
| -------------- | ------------- | ------------------ | --------------------- |
| Length (길이)  | Meter (m)     | Angle (평면각)     | Radian (rad)          |
| Mass (질량)    | Kilogram (kg) | Frequency (주파수) | Hertz (Hz)            |
| Time (시간)    | Second (s)    | Force (힘)         | Newton (N)            |
| Current (전류) | Ampere (A)    | Power (일률)       | Watt (W)              |
|                |               | Voltage (전압)     | Volt (V)              |
|                |               | Temperature (온도) | Celsius (℃)           |
|                |               | Magnetism (자기장) | Tesla (T)             |



ROS에서는 표준 단위를 정해놓았으므로 다른 패키지에서 들어오는 데이터도 대부분 표준 단위로 읽을 수 있어서 편리하다. 만약 단위가 통일되어 있지 않다면 패키지 관련 문서를 뒤져서 데이터의 단위를 찾아봤어야 할 것이다. 앞으로 작성할 ROS 프로그램도 표준 단위로 작성하도록 하자.



## 1.2. ROS 2의 좌표 표현

ROS 2의 표준 단위가 프로그래밍에서 단위 불일치를 막기 위한 규칙이듯, 좌표 표현에 사용되는 좌표계(coordinate system)에도 통일된 규칙이 필요하다.  

예를 들어 로봇의 센서로 널리 사용되는 카메라의 경우 z forward, x right, y down을 기본 좌표계로 사용하는 경우가 많은데 로봇에서는 x forward, y left, z up이 기본 좌표계로 사용된다. 이 외에도 LiDAR, IMU 등의 센서들도 제조사마다 서로 다른 좌표계를 쓸 수 있다. 예를 들어 IMU에도 NED 타입(x North, y East, Z Down)과 ENU 타입(x East, y North, z Up)이 있다. 좌표계도 서로 다른 좌표계를 쓰게 되면 개발에 불편함이 생기고 버그를 유발할 수 있다. 

ROS 커뮤니티에서 표준 좌표 표현은 다음과 같다.

- 회전 방향은 오른손 법칙을 따른다. 좌표 축을 오른손 엄지를 향하게 했을 때 나머지 손가락이 향하는 방향이 회전의 정방향이다. (반시계 방향)  

- 좌표계의 축 방향은 **(x forward, y left, z up)**을 사용한다. ROS의 시각화 툴 RViz이나 시뮬레이터 Gazebo에서 각 축방향을 xyz 순서대로 Red, Green, Blue 색상으로 표현한다.  
- 지리적 위치는 ENU (x East, y North, z Up) 타입을 사용한다. 드론이나 실외 자율주행 로봇에 사용된다.
- 표준 좌표계를 벗어나는 좌표 표현은 접미사를 붙여 구분한다. 카메라 좌표계는 *_optical*, NED 좌표계는 *_ned*를 붙인다.



## 1.3. ROS 2의 빌드 시스템과 빌드 툴

빌드 시스템(build system)과 빌드 툴(build tool)의 차이는 빌드 대상의 범위다. 빌드 시스템은 단일 패키지를 빌드하고 빌드 툴은 빌드 시스템을 이용하여 전체 패키지를 빌드한다.  

**빌드 시스템(build system)**은 단일 패키지를 빌드하기 위해 해당 패키지가 의존하는 패키지를 찾아 빌드 과정에서 의존 패키지와 연결(link)하여 빌드해준다. 예를 들면 A 패키지에서 B 패키지에서 빌드한 라이브러리가 필요하다면 빌드 시스템에서 B 라이브러리를 찾아 A에서 사용할 수 있도록 해준다. ROS 2에서 C++ 패키지의 경우에는 CMake 기반의 **ament_cmake**를 사용하고 있다. Python 패키지는 setuptools 기반의 **ament_python**을 사용한다. ROS 1에서는 catkin 이라는 빌드 시스템이 사용됐는데, 참고로 catkin과 ament는 버드나무 화수(나뭇가지 끝에 기다랗게 무리지어 달리는 꽃송이)에 대한 이음동의어다. ROS 1의 개발 주체인 Willow Garage 뒷마당에 있던 버드나무 화수를 보고 지었다고 한다.

**빌드 툴(build tool)**은 다수의 패키지를 한꺼번에 빌드하기 위한 도구다. ROS는 코드의 재사용성을 위해 패키지와 노드 단위로 구성되어 있고 각 패키지는 다른 패키지와 의존성을 가지고 있다. 얽혀있는 의존성 때문에 다수의 패키지를 동시에 빌드해야 하는데 이때 빌드의 순서가 중요하다. A 패키지가 B 패키지에 의존하고 C 패키지는 A, B 패키지에 의존한다고 할 때 빌드 순서는 B → A → C가 되어야 한다. 실제 빌드 시에는 수십개의 패키지의 의존성이 복잡하게 얽혀 있을 수 있고 각 패키지마다 빌드 시스템도 다르므로 패키지들의 전체적인 의존관계를 파악한 후 순서대로 각 패키지에 대한 빌드 시스템을 호출해야한다. 이를 사람이 직접 하기에는 상당한 노력과 시간이 필요하므로 이를 자동화 한 것이 ROS의 빌드 툴이다. ROS 1에서는 catkin_make, catkin_tools 등이 사용됐지만 ROS 2에서는 colcon (collective construction)를 주로 사용한다. 

빌드 시스템과 빌드 툴의 사용법에 대해서는 실제 패키지를 개발하면서 알아보자.  



## 1.4. ROS 프로그래밍 규칙

### A. 코드 스타일 가이드

소프트웨어를 개발하기 위해 여러사람이 협업해 프로그래밍을 할 때는 일관된 규칙을 만들고 이를 준수해야 한다. 코드 스타일 가이드 준수는 처음에는 귀찮고 번거로울 수 있으나 소스코드 작업 시 빈번히 생기는 개발자의 선택을 줄여주고, 가독성을 높여 다른 개발자 및 이용자의 코드 이해도를 높여준다. ROS도 ROS 커뮤니티의 공동 작업의 결과물이다. ROS 커뮤니티에서는 협업 개발을 위해 [ROS 2 developer guide](https://docs.ros.org/en/foxy/Contributing/Developer-Guide.html), [ROS Enhancement Proposals(REPs)](https://ros.org/reps/rep-0001.html)와 같은 가이드와 규칙도 만들고, 일관된 코드 스타일을 지키기 위해 [ROS 2 Code style](https://docs.ros.org/en/foxy/Contributing/Code-Style-Language-Versions.html)을 만들어 이를 준수하고 있다. 일관된 코드 스타일을 관리하기 위해 린트(Lint)와 같은 자가 검토 툴을 제공하여 개발자가 프로그래밍 가이드를 준수했는지 확인할 수도 있다.  



### B. 기본 이름 규칙 (언어 공통)

이름 규칙에는 **snake_case, CamelCased, ALL_CAPITALS** 세 가지 종류가 있고 사용법은 다음과 같다.

- snake_case: 파일 이름, 변수명, 함수명
- CamelCased: 타입 및 클래스
- ALL_CAPITALS: 상수

단, ROS 인터페이스 파일은 /msg, /srv, /action 폴더에 위치시키며 인터페이스 파일명은 CamelCased 규칙을 따른다. 그 이유는 인터페이스 파일명이 소스코드에서 구조체 및 타입으로 사용되기 때문이다.  

그 외 특정 목적을 위해 만들어지는 고유의 파일 이름은 그대로 사용한다.

- 예시: CMakeLists.txt, README.md, LICENSE, CHANGELOG.rst, .gitignore, .travis.yaml



### C. C++ Style

ROS 2에서 사용되는 C++ 코드 스타일은 오픈소스 커뮤니티에서 가장 널리 사용되는 [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)를 바탕으로 일부 수정해 사용하고 있다. 다음에 기재되지 않은 부분은 REPs 및 Google C++ Style Guide를 참고한다.

1. 기본 규칙

   - C++14 Standard를 준수한다.

   - ROS 2에서는 C++14을 기본 컴파일러로 사용하며 가급적 플랫폼에 종속되는 비표준함수는 사용하지 않는다.

2. 라인 길이

   - 최대 100문자

3. 이름 규칙(Naming)

   - **CamelCased, snake_case, ALL_CAPITALS** 만을 사용한다.
     - CamelCased: 타입, 클래스, 구조체, 열거형
     - snake_case: 파일, 패키지, 인터페이스, 네임스페이스, 변수, 함수, 메소드
     - ALL_CAPITALS: 상수, 매크로
   - 소스 파일은 cpp 확장자를 사용한다.
   - 헤더 파일은 hpp 확장자를 사용한다.
   - 전역 변수를 반드시 사용해야 할 경우 접두어(**g_**)를 붙인다.
   - 클래스 멤버 변수에는 마지막에 밑줄(**_**)을 붙인다.

4. 공백 문자 vs 탭 (Spaces vs. Tabs)

   - 기본 들여쓰기(indent)는 공백 문자 2개를 사용한다. (탭 문자 사용 금지)
   - Class의 접근 지정자(public, protected, private)은 들여쓰지 하지 않는다.

5. 괄호 (Brace{})

   - if, else, do, while, for 구문에 괄호를 사용한다. (한 줄이라도 괄호를 생략하지 않는다.)

6. 주석 (Comments)

   - 문서 주석은 /** */을 사용한다.
   - 구현 주석은 //을 사용한다.

7. 린터 (Linters)

   - C++ 코드 스타일의 자동 오류 검출을 위하여 ament_cpplint, ament_uncrusify를 사용하고 정적 코드 분석이 필요한 경우 ament_cppcheck을 사용한다.

8. 기타

   - Boost 라이브러리의 사용은 가능한 피하고 어쩔 수 없을 경우에만 사용한다. (크고 복잡한 라이브러리인데 사용자끼리 버전 불일치 문제가 빈번할듯, Boost의 상당한 부분이 이미 C++14에 들어와 있음)
   - 포인터 구문은 `char * c;` 처럼 사용한다. (`char* c; char *c;` 불가)
   - 중첩 템플릿은 `set<list<string>>` 처럼 사용한다. (공백 없음)













