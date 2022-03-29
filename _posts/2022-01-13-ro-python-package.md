---
layout: post
title:  "[Ro] ROS Python Package (Basic)"
date:   2020-01-13 09:00:13
categories: draft
---



# 1. ROS 2 Programming Basic

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

1. 기본 규칙 : C++14 Standard를 준수한다.

   - ROS 2에서는 C++14을 기본 컴파일러로 사용하며 가급적 플랫폼에 종속되는 비표준함수는 사용하지 않는다.

2. 라인 길이 : 최대 100문자

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



### D. Python

파이썬 코드 스타일은 Python Enhancement Proposals (PEPs)의 [PEP 8](https://peps.python.org/pep-0008/)을 준수한다.

1. 기본 규칙 : 파이썬 3.5 이상을 사용한다.
2. 라인 길이 : 최대 100 문자
3. 이름 규칙(Naming)
   - **CamelCased, snake_case, ALL_CAPITALS** 만을 사용한다.
     - CamelCased: 타입, 클래스
     - snake_case: 파일, 패키지, 인터페이스, 모듈, 변수, 함수, 메소드
     - ALL_CAPITALS: 상수
4. 공백 문자 vs 탭 (Spaces vs. Tabs)

   - 기본 들여쓰기(indent)는 공백 문자 4개를 사용한다. (탭 문자 사용 금지)
5. 주석 (Comments)

   - 문서 주석은 """을 사용하며 Docstring Convention을 기술한 [PEP 257](https://peps.python.org/pep-0257/)을 준수한다.
   - 구현 주석은 #을 사용한다.
6. 린터 (Linters) : 파이썬 코드 스타일의 자동 오류 검출을 위하여 ament_flake8을 사용한다.

7. 기타: 모든 문자는 큰 따옴표(")가 아닌 **작은 따옴표(')**를 사용하여 표현한다.




# 2. Python Topic Comm. Package

첫 번째 ROS 프로그램을 파이썬으로 구현해보자. 간단한 토픽을 주고 받을 수 있는 퍼블리셔(Publisher)와 서브스크라이버(Subscriber) 노드를 만들고 동작시켜본다.  

## 2.1. 패키지 생성

ROS 2 패키지 생성 명령어는 다음과 같다.

```
$ ros2 pkg create [package name] --build-type [build type] --dependencies [dependent packages]
```

작업공간에 파이썬 기반 패키지를 하나 만들어보자.

```
$ cd ~/robot_ws/src
$ ros2 pkg create rclpy_topic_simple --build-type ament_python --dependencies rclpy std_msgs
```

파이썬을 쓰기 위해 [build type]을 ament_python으로 하고 [dependent packages]에 rclpy를 넣고 토픽 통신에 표준 메시지를 활용하기 위해 std_msgs도 추가했다.  

패키지를 생성하고 나면 다음과 같은 파일들이 만들어진다.  

```
.
├── package.xml
├── rclpy_topic_simple
│   └── __init__.py
├── resource
│   └── rclpy_topic_simple
├── setup.cfg
├── setup.py
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```



## 2.2. 패키지 설정

앞서 만든 package.xml, setup.py, setup.cfg 파일은 패키지 설정과 관련된 파일들이다. 하나씩 알아보고 수정해보자.  

### A. ROS 패키지 설정 파일 (package.xml)

ROS 패키지의 정보를 기술하는 필수 파일이다. 모든 ROS 패키지는 package.xml을 포함해야 한다. ROS는 오픈소스 커뮤니티에서 개발하기 때문에 패키지를 공유할 때 소스코드 뿐만 아니라 패키지와 관련된 정보를 함께 공유해야한다. 기술하는 내용으로는 패키지 이름, 저작자, 라이선스, 의존성 패키지 등이 있다. 빌드 할 때도 이 파일을 보고 빌드 툴이 작동한다.  

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rclpy_topic_simple</name>
  <version>0.1.0</version>
  <description>ROS 2 rclpy basic package to test topic communication</description>
  <author email="hyukdoo.choi@sch.ac.kr">ian</author>
  <maintainer email="hyukdoo.choi@sch.ac.kr">ian</maintainer>
  <license>MIT License</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

이 파일에 나올 수 있는 태그들은 다음과 같다.

- \<package\> : 여기부터  \</package\>까지가 패키지 설정 내용이다. format="3"은 패키지 설정 파일의 버전을 나타낸다. ROS 2에서는 3을 쓴다.
- \<name\> : 패키지 이름
- \<version\> : 패키지의 버전
- \<description\> : 패키지에 대한 간단한 설명
- \<maintainer\> : 패키지 관리자의 이름과 이메일
- \<license\> : [라이선스 종류](http://tech.inswave.com/2018/04/27/OpenSourceLicense/)
- \<url> : 패키지를 설명하는 웹 페이지 또는 소스코드 저장소 등의 주소
- \<author\> : 패키지 개발에 참여한 개발자의 이름과 메일 주소, 여러명이라면 개발자 수만큼 \<author\> 태그를 반복한다.
- \<build_depend\> : 패키지를 빌드할 때 필요한 의존 패키지 이름
- \<exec_depend\> : 패키지를 실행할 때 필요한 의존 패키지 이름
- \<depend> : \<build_depend\>와 \<exec_depend\>를 합친 것으로 두 가지 의존성을 한번에 지정
- \<test_depend\> : 패키지를 테스트 할 때 필요한 의존 패키지 이름
- \<export\> : 위에서 명시하지 않은 확장 태그명 사용
- \<build_tool\> : 패키지 생성시 --build-type 옵션으로 지정한 값이 들어가는데 파이썬은 ament_python, C++은 ament_cmake를 사용



### B. 파이썬 패키지 설정 파일 (setup.py)

이 파일은 순수 파이썬 패키지에서만 사용하는 배포를 위한 설정 파일이다. ROS 1에서는 파이썬 패키지에 대해서도 CMakeLists.txt가 필요했으나 ROS 2에서는 파이썬 패키지가 C++의 영향으로부터 벗어나 독자적인 설정 파일을 갖게 됐다. setup.py는 setuptools를 사용하여 배포를 위한 다양한 설정을 할 수 있다. package.xml과 중복되는 내용이 있으나 package.xml은 ROS 커뮤티니에 배포하기 위한 파일이고 setup.py는 파이썬 커뮤니티에 배포하기 위한 파일로 용도가 다르니 두 내용을 동일하게 맞춰주자.

```python
from setuptools import setup, find_packages

package_name = 'rclpy_topic_simple'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='ian',
    author_email='hyukdoo.choi@sch.ac.kr',
    maintainer='ian',
    maintainer_email='hyukdoo.choi@sch.ac.kr',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: MIT License',
        'Programming Language :: Python'
    ],
    description='ROS 2 rclpy basic package to test topic communication',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'helloworld_publisher = {package_name}.helloworld_publisher:main',
            f'helloworld_subscriber = {package_name}.helloworld_subscriber:main',
        ],
    },
)
```

- name: 패키지 이름
- version: 패키지 버전
- package: 의존하는 패키지, 하나씩 나열해도 되지만 find_packages()를 기입해주면 자동으로 의존 패키지를 찾아줌
- data_files: 이 패키지에서 사용되는 파일들을 기입하여 함께 배포 (파이썬 파일 제외)
- install_requires: 의존하는 패키지, 이 패키지를 pip를 통해 설치할 때 이곳에 기술된 패키지를 함께 설치함. ROS에서는 pip로 설치하지 않기 때문에 setuptools만을 기입
- zip_safe: 설치시 zip 파일로 아카이브할지 여부
- author, author_email, maintainer, maintainer_email: 저작자, 관리자의 이름과 이메일
- keywords: 패키지 키워드, PyPI 배포 시 이 패키지를 검색할 수 있는 키워드
- classifiers: PyPI에 등록될 메타 데이터 설정으로 PyPI 페이지 좌측 Meta 란에서 확인 가능
- description: 패키지 설명을 기입
- license: 라이선스 종류
- entry_points: 콘솔 스크립트를 설치하도록 콘솔 스크립트 이름과 호출 함수를 기입



### C. 패키지 환경 설정 파일 (setup.cfg)

순수 파이썬 패키지에서 사용하는 배포를 위한 구성 파일이다. setup.py에서 설정하지 못하는 기타 옵션을 여기서 설정할 수 있다. ROS 2에서는 [develop]과 [install] 옵션을 통해 스크립트의 저장 위치를 설정한다. 별로 수정할 내용은 없고 설정하는 경로에서 패키지 이름만 맞으면 된다.

```
[develop]
script-dir=$base/lib/rclpy_topic_simple
[install]
install-scripts=$base/lib/rclpy_topic_simple
```



## 2.3. 소스 코드

본 패키지는 최소한의 간단한 토픽 통신을 구현하는 것이 목표다. 토픽을 보내는 퍼블리셔(publisher) 역할을 하는 노드 하나와 서브스크라이버 역할의 노드 하나를 구현한다.

### A. 퍼블리셔 노드

String이라는 문자열 메시지 타입으로 토픽을 전송하는 퍼블리셔다. 코드는 다음과 같다.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class HelloworldPublisher(Node):
    def __init__(self):
        super().__init__('helloworld_publisher')
        qos_profile = QoSProfile(depth=10)
        self.helloworld_publisher = self.create_publisher(String, 'helloworld', qos_profile)
        self.timer = self.create_timer(1, self.publish_helloworld_msg)
        self.count = 0

    def publish_helloworld_msg(self):
        msg = String()
        msg.data = f'Hello world: {self.count}'
        self.helloworld_publisher.publish(msg)
        self.get_logger().info(f'published message: {msg.data}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = HelloworldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

부분별로 살펴보자.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
```

첫 번째 단락은 import 구문이다. 클래스 정의에서는 rclpy에 들어있는 Node, QoSProfile을 사용하고 main에서는 rclpy를 직접 사용한다. 전송하는 토픽 타입이 문자열이므로 String 클래스를 import 한다.

```python
class HelloworldPublisher(Node):
    def __init__(self):
        super().__init__('helloworld_publisher')
        qos_profile = QoSProfile(depth=10)
        self.helloworld_publisher = self.create_publisher(String, 'helloworld', qos_profile)
        self.timer = self.create_timer(1, self.publish_helloworld_msg)
        self.count = 0
```

Node 클래스를 상속한 HelloworldPublisher 클래스 선언과 생성자 정의다. 이 스크립트에서는 퍼블리셔 노드를 만드는게 목적이므로 노드 클래스를 정의해야 한다. 노드 클래스는 rclpy.node.Node 클래스를 상속해야 하고 이 클래스 객체를 생성해야 ROS에서 노드가 실행된다.  

생성자에서는 퍼블리셔 객체를 생성한다. 토픽을 발행하는 함수를 타이머 콜백(callback) 함수로 등록하여 주기적으로 1초에 한번 실행되게 한다. QoSProfile은 QoS (Quality of Service) 설정을 위한 것으로 depth=10 이면 서브스크라이버에서 데이터를 실시간으로 받지 못 할 경우에 데이터를 버퍼에 최대 10개까지 저장하는 설정이다.  

```python
    def publish_helloworld_msg(self):
        msg = String()
        msg.data = f'Hello world: {self.count}'
        self.helloworld_publisher.publish(msg)
        self.get_logger().info(f'published message: {msg.data}')
        self.count += 1
```

토픽 메시지를 퍼블리시 하는 함수다. 문자열 메시지 타입(String)으로 객체를 하나 만들고 데이터를 채워서 퍼블리셔로 메시지를 보낸다. 메시지를 구분하기 위해 카운트 값을 올린다.

```python
def main(args=None):
    rclpy.init(args=args)
    node = HelloworldPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

메인 함수다. rclpy를 직접 이용해 프로세스 전반을 관리한다. init() 함수로 ROS 통신을 준비하고, 이후 노드를 생성한다. spin() 함수로 타이머 콜백이 주기적으로 실행되도록 대기한다. (except) 키보드 입력으로 종료하면 메시지를 출력한다. (finally) spin() 함수가 어떻게 끝나건 상관없이 항상 노드 객체를 파괴하고 통신과 관련된 컨텍스트(context)를 종료한다.  



### B. 서브스크라이버 노드

서브스크라이버 노드는 퍼블리셔 노드에서 보낸 문자열 메시지를 받아서 터미널에 출력한다.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class HelloworldSubscriber(Node):
    def __init__(self):
        super().__init__('Helloworld_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.helloworld_subscriber = self.create_subscription(
            String,
            'helloworld',
            self.subscribe_topic_message,
            qos_profile)

    def subscribe_topic_message(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = HelloworldSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

소스코드의 구조는 퍼블리셔 노드와 유사하다. main() 함수는 똑같고 서브스크라이버 노드 클래스만 일부 차이가 있다.  

- 생성자에서 create_subscription() 함수를 통해 서브스크라이버 객체를 생성한다.
- 서브스크라이버 객체를 생성할 때 subscribe_topic_message() 함수를 콜백 함수로 등록한다.
- 콜백 함수는 토픽 메시지가 발행될 때마다 실행된다.
- subscribe_topic_message() 함수에서는 수신 받은 메시지를 터미널에 출력한다.



## 2.4. 빌드 및 실행

### A. colcon 빌드

ROS 2 패키지를 빌드 할 때는 먼저 경로를 워크스페이스로 이동한 후 colcon 빌드 툴을 사용한다. 빌드 옵션은 다음과 같다.

- --symlink-install : ROS 2 워크스페이스에는 build와 install 폴더가 있다. 빌드 과정에 필요한 부산물과 빌드 결과물 파일들이 생기는데 이 옵션을 넣으면 가급적 실제 파일을 복사하기 보다는 원본에 대한 심볼릭 링크만 생성해서 저장 공간을 아낄수 있다.
- --packages-select [package-names] : 특정 패키지만 빌드한다.
- --packages-up-to [package-names] : 특정 패키지와 의존 패키지까지 빌드한다.

다음 명령어를 통해 빌드를 실행해보자.

```
(워크스페이스로 이동)
$ cd ~/robot_ws
(워크스페이스 내 모든 패키지 빌드)
$ colcon build --symlink-install
(특정 패키지만 빌드)
$ colcon build --symlink-install --packages-select rclpy_topic_simple
```

특정 패키지의 첫 빌드 후에는 환경설정 파일을 불러와야 빌드된 노드를 실행할 수 있다.

```
$. ~/robot_ws/install/local_setup.bash
```



### B. 실행

ros2 run 명령어를 통해 퍼블리셔와 서브스크라이버 노드를 서로 다른 터미널에서 실행한다.

```
$ ros2 run rclpy_topic_simple helloworld_subscriber
```

```
$ ros2 run rclpy_topic_simple helloworld_publisher
```

서브스크라이버 노드에서 메시지를 잘 수신하는 것을 확인할 수 있다.

> $ ros2 run rclpy_topic_simple helloworld_subscriber 
> \[INFO]: Received message: Hello world: 0
> \[INFO]: Received message: Hello world: 1
> \[INFO]: Received message: Hello world: 2
> \[INFO]: Received message: Hello world: 3
> \[INFO]: Received message: Hello world: 4
> \[INFO]: Received message: Hello world: 5
> \[INFO]: Received message: Hello world: 6
> \[INFO]: Received message: Hello world: 7
> \[INFO]: Received message: Hello world: 8
> \[INFO]: Received message: Hello world: 9  
>
> ...

노드가 실행되기는 하는데 어떻게 실행되는걸까? 다시 생각해보면 helloworld_publisher.py와 helloworld_subscriber.py에 main() 함수는 있지만 main() 함수를 실행하는 코드는 없다. 즉 두 개의 스크립트 파일 자체는 실행되는 코드가 없다. 실행 명령어에도 파일 확장자 .py가 붙지 않았다.  

실마리는 위에서 설정했던 setup.py에 있다. setup() 함수의 입력인자로 entry_points를 설정할 수 있는데 여기서 ros2 run으로 실행하는 노드 실행 명령어와 이와 연결된 함수를 지정할 수 있다.  

```
    entry_points={
        'console_scripts': [
            f'helloworld_publisher = {package_name}.helloworld_publisher:main',
            f'helloworld_subscriber = {package_name}.helloworld_subscriber:main',
        ],
    },
```

위 예시에서는 rclpy_topic_simple/helloworld_publisher.py 파일의 main 함수를 "helloworld_publisher"라는 명령어로 사용할 수 있게 해준다. 서브스크라이버도 같은 방법으로 명령어를 지정했다.











