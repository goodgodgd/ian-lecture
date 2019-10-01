---
layout: post
title:  "Catkin Workspace"
date:   2010-09-28 09:00:13
categories: WIP Build System
---



# Linux Build System

로봇공학 수업에서는 파이썬을 이용한 로봇 프로그래밍을 할 것이지만 ROS는 기본적으로 C++을 기준으로 시스템이 구성되어있다. C/C++은 컴파일 언어기 때문에 코드로부터 실행 파일이나 라이브러리를 만들기 위해서는 두 단계를 거쳐야한다.

1. 컴파일 (Compile): 사람이 쓴 코드를 기계어로 번역하여 목적(object) 파일을 생성한다.
2. 빌드 (Build): 목적 파일을 라이브러리와 링크하여 실행 파일을 만든다.

이러한 과정을 처리해주는 시스템을 빌드 시스템 (Build System)이라 하는데 리눅스에서는 프로젝트의 복잡도에 따라 다양한 수준의 빌드 시스템이 존재한다.



## 1. GCC

gcc는 GNU Compiler Collection의 약자로 리눅스에서 기본적으로 사용되는 C/C++ 컴파일러다. 다음과 같은 간단한 코드를 작성하여 `hello.c`로 저장해보자.

```c
// hello.c
#include <stdio.h>
int main( void ){
    printf("HELLO WORLD !! \n");
    return 0;
}
```

이 코드를 gcc를 이용해 빌드해보자. 여기서는 간단한 소스 파일 하나지만 여러 개의 파일이 있는 일반적인 상황을 가정하고 빌드를 하였다.

```
$ gcc -c -o hello.o hello.c
$ gcc -o hello hello.o
$ ./hello
HELLO WORLD !! 
```

gcc를 이용한 빌드는 컴파일과 빌드를 따로 해야하고 중간의 목적 파일의 이름도 지정해줘야한다. 컴파일은 모든 소스파일마다 따로해줘야 하기 때문에 소스파일이 여러개일 경우 이를 일일히 컴파일하고 빌드하는 일이 매우 번거로워진다.  

참고자료: <https://ra2kstar.tistory.com/176>



## 2. make

make는 gcc를 좀 더 쉽게 쓰기 위해서 만들어진 툴이다. 여러 단계의 gcc 명령어를 빌드할 때마다 쓰는게 아니라 빌드 과정을 `Makefile`이라는 스크립트에 미리 적어놓고 `make`만 실행하면 자동으로 모든 과정이 실행되는 시스템이다. 위 예시 같은 경우 아래와 같은 Makefile을 작성해야 한다.

```makefile
# Makefile
hello: hello.o
	gcc -o hello hello.o
hello.o: hello.cpp
```

작성 후 `make` 명령어로 빌드한다.

```
$ rm hello.o hello
$ ls
hello.c  Makefile
$ make
cc    -c -o hello.o hello.c
gcc -o hello hello.o
```

gcc를 직접 이용하는 것보다는 `Makefile`을 미리 작성해놓고 make를 이용하는 것이 좀 더 쉽긴하다. 그리고 make는 여러개의 소스 파일이 있을 경우 수정된 소스파일만 빌드해주는 incremental build 기능도 있어서 효율적이다.  

하지만 프로젝트 규모가 조금 커지면 make도 여러가지 문제가 있다. `Makefile`을 작성할 때 중간 결과물을 지정해줘야 하는 점도 부담이고 소스가 수정되면서 변화하는 소스코드 사이의 의존관계를 `Makefile`에서 일일이 작성해줘야 한다. 프로젝트 규모가 커지면 `Makefile`이 너무 길어지고 복잡해져서 관리하기가 점점 어려워진다.



## 3. CMake

리눅스에서 대다수의 C/C++ 프로젝트는 CMake로 빌드를 한다. 앞서 말했다시피 복잡한 프로젝트의 `Makefile`을 직접 작성하는 것이 매우 까다롭기 때문에 간단한 문법의 `CMakeLists.txt` 파일을 작성하면 이를 바탕으로 소스코드를 분석하여 자동으로 `Makefile`을 만들어주는 툴이다. 위 예시의 경우 간단히 한 줄의 `CMakeLists.txt`를 작성하면 된다.

```cmake
# CMakeLists.txt
ADD_EXECUTABLE(hello hello.c)
```

CMake를 이용한 빌드 과정은 다음과 같다.

```
$ rm hello.o hello
$ ls
CMakeLists.txt  hello.c  Makefile
ian@ian:~/workspace/build-system$ cmake .
-- The C compiler identification is GNU 7.4.0
-- The CXX compiler identification is GNU 7.4.0
# ... 중략
-- Configuring done
-- Generating done
-- Build files have been written to: /home/ian/workspace/build-system
ian@ian:~/workspace/build-system$ make
Scanning dependencies of target hello
[ 50%] Building C object CMakeFiles/hello.dir/hello.c.o
[100%] Linking C executable hello
[100%] Built target hello
ian@ian:~/workspace/build-system$ ls
CMakeCache.txt  CMakeFiles  cmake_install.cmake  CMakeLists.txt  hello  hello.c  Makefile
ian@ian:~/workspace/build-system$ ./hello 
HELLO WORLD !! 
```

make 대비 CMake의 장점은 중간 결과물을 신경쓰지 않고 최종 결과물만 지정하면 되고, 소스코드를 분석해서 소스 파일 사이의 의존관계를 스스로 파악해주고, 사용자에게 유용한 다양한 변수와 매크로를 제공한다는 것이다. 그래서 일반적으로 `Makefile`에 비해 `CMakeLists.txt` 훨씬 짧고 관리하기도 간편하다.  

CMake는 리눅스에서 가장 널리 사용되는 빌드 시스템이긴 하지만 유일한 도구는 아니다. nmake, qmake 등 대안은 얼마든지 있다. 다만 CMake가 가장 많이 사용되기 때문에 ROS도 CMake를 기반으로 빌드 시스템을 구축한다.



#  Catkin Build System

## 1. Catkin 소개

ROS의 기본 철학이 최소 단위 기능만을 가지고 있는 여러 노드들의 조합으로 시스템을 구성하는 것이기 때문에 다수의 패키지를 동시에 개발하는 경우가 많다. 이때 여러 패키지의 코드를 수정한 후 이를 실행하기 위해서는 모든 패키지에서 따로 빌드를 해줘야한다. 즉 패키지마다 `cmake`와 `make`를 (옵션을 추가하여) 실행해야 한다는 것이다.  

이러한 과정이 번거롭기 때문에 ROS에는 모든 패키지를 하나의 워크스페이스(workspace) 경로에 담고 이들을 한번에 빌드할 수 있는 메타빌드 시스템(meta-build system)을 만들었는데 이것이 바로 캐킨(catkin)이다. 초기 ROS에서는 이러한 개념을 구현한 `rosbuild`가 있었으나 이후 `catkin`으로 발전하였다. 캐킨 시스템에서 사용자는 캐킨 명령어를 통해 워크스페이스 초기화, 패키지 생성, 전체 워크스페이스 빌드 등을 각각 한줄의 명령어를 통해 실행할 수 있게 되었다. 이것이 ROS 설치시 기본 설치되는 `catkin_`으로 시작하는 명령어들이다. 이 시스템에서는 하나의 최상위 `CMakeLists.txt`에서 하위 디렉토리에 들어있는 모든 패키지 내부의 `CMakeLists.txt`들을 자동 검색해서 여러 패키지들을 하나의 프로젝트처럼 빌드를 시킨다. 개발자는 각 패키지의 `CMakeLists.txt`만 잘 작성하고 나서 `catkin_make`만 실행하면 모든 패키지들이 병렬로 빌드되는 것이다.

하지만 이 시스템에도 몇 가지 문제가 있다. 

- 여러 패키지를 빌드할 때 하나의 패키지에서 cmake 에러가 나더라도 전체 빌드가 실패한다.
- 타겟 이름이나 cmake 변수가 중복되어 예상치 않은 버그나 에러가 발생할 수 있다.
- 오직 catkin으로 생성한 패키지만 빌드할 수 있다.

이러한 문제를 해결하고자 `catkin_make_isolated`라는 툴도 나왔으나 부작용으로 병렬 빌드가 되지 않는 등의 문제가 있었다. 그래서 `catkin` 이라는 툴이 새로 나오게 됐다. `catkin`이란 하나의 명령어로 verb에 따라 워크스페이스 초기화, 빌드, 패키지 생성 등을 할 수 있는 통합 툴이다. 특히 빌드를 하는 `catkin build`가 기존 `catkin_make`에 비해 크게 개선되었다. 기존에 모든 패키지를 하나의 프로젝트로 묶는 최상위 `CMakeLists.txt`가 사라지고 개별 패키지를 독립적으로 그러면서도 병렬로 빌드한다. 캐킨으로 생성하지 않은 plain CMake 프로젝트도 함께 빌드가 가능해지고 그 외에 다양한 사용자 인터페이스가 개선되었다.



## 2. ROS 기본 Catkin

우선 ROS에서 기본 제공하는 캐킨 빌드 시스템을 사용하는 법에 대해 간략히 알아본다. 아래 세 가지 명령어만 알면 캐킨을 사용할 수 있다.

> Note: 수업에서는 `~/catkin_ws` 경로를 기본 캐킨 워크스페이스로 쓰지만 꼭 `$HOME` 디렉토리에 `catkin_ws`란 이름을 쓸 필요는 없다. `~/catkin_ws`는 어디까지나 예시로서 흔하게 쓰이는 경로를 쓰는것 뿐이다.

- `catkin_init_workspace`: `~/catkin_ws/src` 폴더에서 이 명령어를 실행하면 `CMakeLists.txt`가 자동으로 생긴다. 이 파일에서 src 폴더 아래 모든 캐킨 패키지를 검색하여 한번에 빌드할 수 있게한다.
- `catkin_create_pkg <pakcage_name> <dependencies> `: `~/catkin_ws/src` 폴더에서 이 명령어를 실행하면 캐킨 패키지를 생성한다. 인자로 패키지 이름과 새로운 패키지가 의존하는 다른 ROS 패키지를 지정한다.
- `catkin_make`: 캐킨 워크스페이스 내부의 모든 패키지를 일괄 빌드한다. 내부적으로 cmake와 make를 자동 실행해준다. 처음 실행하면 `~/catkin_ws` 아래 자동으로 `build, devel` 디렉토리가 생긴다.

캐킨 빌드 시스템을 이해하기 위해서는 위 명령어로 자동 생성되는 워크스페이스 내부의 디렉토리 구성을 알아야 한다.

- src (source space): 패키지 소스코드가 있는 곳이다. `src` 바로 아래에는 `catkin_init_workspace`를 통해 만들어진 `CMakeLists.txt`가 있어야 하고 패키지들은 하위 디렉토리에 들어있어야 한다. 하위 디렉토리는 하나의 캐킨 패키지일 수도 있고 여러 패키지 디렉토리가 들어있을 수도 있다. 
- build (build space): `catkin_make`를 하면 자동으로 생성된다. 이곳은 빌드의 최종 결과물이 아닌 빌드에 필요한 설정 파일이나 중간 결과물들이 저장되는 곳이다.
- devel (development space): `catkin_make`를 하면 자동으로 생성된다. 이곳은 빌드의 결과물인 실행 파일, 메시지 헤더 파일, 라이브러리 등이 저장되는 곳이다. 노드의 실행에 직접적으로 필요한 파일들이라고 보면 된다. 또한 이 워크스페이스의 패키지를 어느 경로에서나 실행할 수 있게 해주는 `setup.bash`도 이곳에 있다.



### 2.1 패키지 만들기

새로운 캐킨 워크스페이스를 만들고 간단한 패키지를 만드는 것까지 실습을 해보자. 여기서는 기본적인 C++ 프로젝트를 만들어본다. `roscpp, std_msgs`에 의존하는 `hello_ros`라는 패키지를 만든다.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
~/catkin/src$ catkin_init_workspace
~/catkin/src$ catkin_create_pkg hello_ros roscpp std_msgs
```

`catkin_create_pkg` 명령어로 패키지를 생성하면 기본적으로 `CMakeLists.txt`와 `package.xml`이 생기고 주석을 통해 다양한 설정들을 가이드하고 있다.  

- package.xml: ROS에서 패키지를 관리하는데 필요한 정보를 담고 있다. 패키지를 배포하는데 필요한 저자, 라이센스 등의 정보와 패키지 의존성 등
- CMakeLists.txt: 로컬에서 소스를 빌드하는데 필요한 설정

이 두 개의 파일은 아래 내용처럼 작성할 수 있다.

#### package.xml

```xml
<?xml version="1.0"?>
<!-- packages.xml: ros에서 패키지를 관리하는데 필요한 정보 담음 -->
<!-- 공식 패키지로 업로드시 공개해야 할 정보들 포함: 저자, 저작권 등 -->
<!-- 의존 패키지들을 지정하면 빌드시 설치되지 않은 의존 패키지 자동 설치 -->
<!-- 필수 태그는 주석에 * 붙임 -->

<package format="2">
  <!-- *패키지 생성시 지정한 이름 -->
  <name>hello_ros</name>
  <!-- *기본 생성시 버전은 0.0.0 인데 원하는대로 수정 가능 -->
  <version>0.1.0</version>
  <!-- *패키지 소개글 -->
  <description>The hello_ros package</description>
  <!-- *관리자 정보, 여러명도 가능 -->
  <maintainer email="ian@todo.todo">ian</maintainer>
  <!-- *라이센스 지정 -->
  <license>BSD</license>
  <!-- 저자 정보 -->
  <author email="jane.doe@example.com">Jane Doe</author>
  
  <!-- 빌드 시스템 지정: catkin -->
  <buildtool_depend>catkin</buildtool_depend>
  <!-- 빌드에 필요한 패키지 -->
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <!-- 다른 패키지에서 이 프로젝트 결과물(라이브러리)을 링크하여 사용할 때 필요한 패키지 -->
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <!-- 실행에 필요한 패키지 -->
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```



#### CMakeLists.txt

`CMakeLists.txt`에는 다양한 설정이 들어갈 수 있으나 여기서는 최소한의 기능만 사용했고 `CMakeLists.txt`의 다양한 기능에 대해서는 따로 설명하기로 한다.

```cmake
# cmake 최소 버전: ubuntu 18.04에 설치되는 현재 버전은 3.10.2 이므로 만족
cmake_minimum_required(VERSION 2.8.3)
# package.xml에 지정된 이름과 동일한 프로젝트 이름
project(hello_ros)

# ROS Kinetic 이후로는 C++11을 사용할 수 있다.
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
# find_package(Boost REQUIRED COMPONENTS system)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/hello_ros_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
```



####  소스 코드 작성

`PKG_ROOT/src/hello_ros_node.cpp`를 다음과 같이 작성해보자. "hello ROS" 라는 메시지를 지속적으로 퍼블리시하는 노드다. 

```
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher publisher = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello ROS " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
```



### 2.2 빌드 및 실행

파일들이 준비됐다면 아래 명령어로 빌드를 한다. 명령을 실행하는 경로를 미리 맞춰야 한다.

```
~/catkin$ catkin_make
```

빌드가 다 되면 노드를 실행한다. 항상 그렇듯 `roscore`는 미리 켜져있어야 한다

```
$ roscore
# 새 탭 열기

ian@ian:~/catkin_ws$ rosrun hello_ros hello_ros_node 
[ INFO] [1569916580.188811753]: hello world 0
[ INFO] [1569916580.288903212]: hello world 1
[ INFO] [1569916580.389136994]: hello world 2
[ INFO] [1569916580.488957346]: hello world 3
[ INFO] [1569916580.589055009]: hello world 4
[ INFO] [1569916580.688950943]: hello world 5
[ INFO] [1569916580.788970498]: hello world 6
[ INFO] [1569916580.889171241]: hello world 7
[ INFO] [1569916580.989207716]: hello world 8
[ INFO] [1569916581.089128841]: hello world 9
[ INFO] [1569916581.189040936]: hello world 10
```



## 3. catkin tools

이번에는 기존 `~/catkin_ws`를 삭제하고 다시 만들어서 새로운 `catkin`을 써 볼것이다. 그리고 위에서 ROS 기본 catkin으로 했던 과정을 새로운 catkin으로 반복해본다.

```
$ cd ~
$ rm -rf ~/catkin_ws
$ mkdir ~/catkin_ws/src
$ cd ~/catkin_ws
```



### 3.1 catkin init

워크스페이스를 처음 만들었다면 워크스페이스를 초기화해야한다. 워크스페이스 디렉토리에서 `catkin init`을 실행하면 된다. 

```
~/catkin_ws$ catkin init
Initializing catkin workspace in `/home/ian/catkin_ws`.
--------------------------------------------------------
Profile:                     default
Extending:             [env] /opt/ros/melodic
Workspace:                   /home/ian/catkin_ws
--------------------------------------------------------
Build Space:       [missing] /home/ian/catkin_ws/build
Devel Space:       [missing] /home/ian/catkin_ws/devel
Install Space:      [unused] /home/ian/catkin_ws/install
Log Space:         [missing] /home/ian/catkin_ws/logs
Source Space:       [exists] /home/ian/catkin_ws/src
DESTDIR:            [unused] None
--------------------------------------------------------
Devel Space Layout:          linked
Install Space Layout:        None
--------------------------------------------------------
Additional CMake Args:       None
Additional Make Args:        None
Additional catkin Make Args: None
Internal Make Job Server:    True
Cache Job Environments:      False
--------------------------------------------------------
Whitelisted Packages:        None
Blacklisted Packages:        None
--------------------------------------------------------
Workspace configuration appears valid.
--------------------------------------------------------
ian@ian:~/catkin_ws$ ls -al
total 16
drwxr-xr-x  4 ian ian 4096 10월  1 18:24 .
drwxr-xr-x 26 ian ian 4096 10월  1 18:24 ..
drwxr-xr-x  2 ian ian 4096 10월  1 18:24 .catkin_tools
drwxr-xr-x  2 ian ian 4096 10월  1 18:24 src
```

`catkin`은 실행할 때마다 위와 같이 워크스페이스의 상태를 깔끔하게 정리해서 보여준다. `catkin init`을 실행해도 아무것도 변한 것 같지 않지만 `.catkin_tools`라는 디렉토리가 추가 되었음을 알 수 있다.

### 3.2 catkin create



### 3.3 catkin build



### 3.4 catkin clean



### 3.5 catkin config





## 4. CMakeLists.txt 주요 함수

`CMakeLists.txt`에 사용되는 주요 함수를 소개한다. 미리 다 알고 있을 필요는 없고 `CMakeLists.txt`를 작성할 때 필요한 부분을 찾아서 보면 된다. 패키지란 용어가 여러곳에서 쓰여 헷갈리므로 여기서 만들고 있는 패키지(hello_ros)는 **"THIS_PKG"**라 부르기로한다. 패키지의 루트 경로(~/catkin_ws/src/hello_ros)는 **"PKG_ROOT"**로 부른다.



### find_package()

THIS_PKG에서 사용할 외부 패키지를 검색해주는 함수다. 일반적으로는 아래 형식으로 쓴다.  

```cmake
find_package(<package_name1>, <package_name2>, ...)
```

`REQUIRED`를 붙이면 필수 패키지란 뜻으로 이 패키지를 찾지 못하면 에러가 난다. 의존 패키지가 있는지 확인하는 용도로도 많이 쓰인다.   

`COMPONENT`는 여러개의 구성요소를 가진 패키지에서 특정 요소만 요구할 때 쓴다. 예를 들어 `find_package(Boost REQUIRED COMPONENTS system)` 는 `Boost`라는 패키지에서 `system`이라는 패키지만 필요하다는 것이다.  

캐킨 패키지를 찾을 때도 아래 예시처럼 `catkin`이라는 패키지의 하위 구성요소로서 찾는다. 캐킨 패키지를 독립적인 패키지가 아니라 하위 요소로서 검색하는 이유는 이후의 편의를 위한 것이다. 이후의 설정에서 하위 패키지를 각각 설정해 줄 필요없이 `catkin` 하나만 설정해주면 하위 요소는 자동으로 설정이 된다.

```
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(foo ...)
target_link_libraries(foo ${catkin_LIBRARIES})
```

만약 아래와 같이 하위 요소를 패키지로서 따로 검색했다면 하위 패키지에 대한 설정을 다 따로 해줘야한다.

```
find_package(roscpp)
find_package(std_msgs)

include_directories(${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
add_executable(foo ...)
target_link_libraries(foo ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${std_msgs_LIBRARIES})
```



### catkin_python_setup()

패키지 내부에서 여러 스크립트 파일들이 엮여돌아는 경우를 생각해보자. 패키지 디렉토리 아래에 `PKG_ROOT/src/tutorial_package/hello.py`를 만들고 이를 `bin/hello`에서 import 하여 다음과 같이 사용하려고 한다.

```python
#! /usr/bin/env python
import tutorial_package.hello
if __name__ == '__main__':
    tutorial_package.hello.say('my friend!')
```

그런데 `bin/hello`를 실행하면 에러가 난다. `PKG_ROOT/src/tutorial_package/hello.py` 경로의 파일을 찾지 못하기 때문이다. `PKG_ROOT/src/tutorial_package`를 `PYTHONPATH`라는 시스템 변수에 추가하는 방법도 있지만 패키지 경로가 바뀌거나 다른 사람이 쓸때마다 설정해야 해서 권장할만한 방법은 아니다.  

이런 경우에 PKG_ROOT에 `setup.py`를 아래와 같이 작성한다.

```python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['tutorial_package'],
    package_dir={'': 'src'},
)
setup(**setup_args)
```

그리고 `CMakeLists.txt`에 다음 줄을 추가한다. 첫 번째 줄은 `setup.py`를 실행하여 파이썬경로를 추가해주고 두 번째 줄은 파이썬 스크립트의 설치 경로를 지정한다.

```cmake
catkin_python_setup()
catkin_install_python(PROGRAMS bin/hello
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

참고: <http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile>  

하지만 파이썬 기반 ROS 코딩을 할 때 항상 `setup.py`를 만들어야 하는 것은 아니다. 하나의 디렉토리에 모든 파일을 담으면 굳이 `setup.py`를 쓰거나 `CMakeLists.txt`에 위와 같은 두 줄을 추가할 필요가 없다.



### 메시지 추가 함수

토픽, 서비스, 액션 통신 방식에 따라 각기 다른 메시지 파일 형식이 존재하고 각각 따로 빌드한다. 사용자가 작성한 메시지 파일 형식에 맞는 함수로 메시지 파일을 추가하고 메시지를 생성한다.

- **add_message_files(FILES message_file.msg)**: 메시지 파일을 빌드 타겟에 추가한다. msg 파일은 반드시 `PKG_ROOT/msg` 디렉토리에 있어야 한다.
- **add_service_files(FILES service_file.srv)**: 서비스 파일을 빌드 타겟에 추가한다. srv 파일은 반드시 `PKG_ROOT/srv` 디렉토리에 있어야 한다.
- **add_action_files(FILES action_file.action)**: 서비스 파일을 빌드 타겟에 추가한다. action 파일은 반드시 `PKG_ROOT/action` 디렉토리에 있어야 한다.

모든 메시지를 추가한 후 아래 줄을 붙여야 한다. 새로운 메시지의 의존 타입을 적는데 주로 기본 메시지인 `std_msgs`를 쓰거나  다른 메시지가 들어있는 패키지 이름을 쓴다.

```cmake
generate_messages(DEPENDENCIES std_msgs)
```



### catkin_package()

`catkin_package()`는 다른 패키지가 THIS_PKG를 사용하려 할때 필요한 사항을 정리한다.

- INCLUDE_DIRS: THIS_PKG의 헤더 파일이 있는 경로
- LIBRARIES: THIS_PKG에서 만드는 라이브러리
- CATKIN_DEPENDS: THIS_PKG가 의존하는 캐킨 패키지로서 THIS_PKG를 사용하는 다른 패키지도 필요하게 되는 캐킨 패키지 지정
- DEPENDS: THIS_PKG가 의존하는 시스템 패키지로서 THIS_PKG를 사용하는 다른 패키지도 필요하게 되는 시스템 패키지 지정

```cmake
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_ros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
```



### include_directories()

C++에서 `#include "some_header.h" ` 를 컴파일 할 때 `some_header.h`를 검색하는 디렉토리를 추가한다. 패키지 내부에 헤더 파일이 있는 경로를 지정하고 (주로 `include`) 다른 ROS 패키지의 헤더 파일도 쓸 수 있도록 `${catkin_INCLUDE_DIRS}`도 추가한다. ROS외에 다른 라이브러리를 쓰는 경우에는 여기서 다른 라이브러리의 헤더 파일 경로를 추가해야한다.

```cmake
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
```



### add_library()

라이브러리 타겟을 추가한다. 라이브러리 파일을 만드는 경우에만 사용한다. 일반적으로 노드 실행 파일을 만드는 패키지에서는 쓰지 않는다. 아래 스크립트는 사용 방법과 예시다. 라이브러리 이름 다음에는 라이브러리를 빌드하는데 필요한 모든 소스파일을 나열해야 한다. 헤더 파일은 쓰지 않아도 된다. 파일이 많은 경우 소스 파일 목록을 변수로 만들고 변수를 함수에 입력한다.

```cmake
# add_library(<library_name> <src_file1> <src_file2> ...)
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/hello_ros.cpp)
```



### add_executable()

실행 파일 타겟을 추가한다. 타겟을 추가한다는 것은 cmake를 실행해서 생기는 `Makefile`이 타겟 결과물을 만들도록 자동 생성된다는 뜻이다. ROS에서 노드는 실행 파일으므로 노드를 만든다고 보면 된다. 아래 스크립트는 사용 방법과 예시다. 라이브러리 이름 다음에는 라이브러리를 빌드하는데 필요한 모든 소스파일을 나열해야 한다. 헤더 파일은 쓰지 않아도 된다. 파일이 많은 경우 소스 파일 목록을 변수로 만들고 변수를 함수에 입력한다.

```cmake
# add_executable(<executable_name> <src_file1> <src_file2> ...)
add_executable(${PROJECT_NAME}_node src/hello_ros_node.cpp)
```



### add_dependencies()

타겟에 필요한 다른 의존 타겟을 지정한다. 타겟은 빌드로 만들어지는 모든 최종 결과물을 말하는 것인데 실행파일, 라이브러리, 언어별 메시지 타겟도 포함된다. 그 중에서도 `EXPORTED_TARGET`은 언어별 메시지, 서비스, 액션의 타겟, dynamic configure 등과 같이 외부에 설치되는 타겟을 말한다. C++을 쓰는 경우에는 메시지 파일로부터 헤더 파일이 생성되고 파이썬을 쓰는 경우에는 메시지 파일로부터 파이썬 스크립트가 생성된다. 만약 내가 만든 메시지 타입으로 토픽을 퍼블리시하려면 먼저 메시지 헤더 파일이 빌드되어야 퍼블리시 노드를 빌드할 수 있다. 함수는 `add_dependencies(<target_name> <dependent_targets>)` 형식으로 사용하며 의미는 `<target_name>` 타겟에 대해 `<dependent_targets>`에 대한 의존성을 추가하는 것이다.  

의존성을 추가하는 이유는 빌드 순서를 정하기 위함이다. 의존하는 타겟을 먼저 빌드하고 의존 타겟을 사용하는 타겟을 나중에 빌드해야 빌드 과정에서 에러가 발생하지 않는다.  

그래서 ROS 패키지에서 `add_dependencies()`는 거의 모든 `add_library()`와 `add_executable()` 아래에 들어간다. 아래 예시는 라이브러리와 실행 파일 타겟을 만든 후 각각에 의존성을 추가하는 스크립트다. `${${PROJECT_NAME}_EXPORTED_TARGETS`는 THIS_PKG에서 만드는 `EXPORTED_TARGET`이고 `${catkin_EXPORTED_TARGETS}`는 캐킨 전체의 `EXPORTED_TARGET`을 말한다.

```cmake
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/hello_ros.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(${PROJECT_NAME}_node src/hello_ros_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```



### target_link_libraries()

특정 타겟을 위한 라이브러리를 링크한다. 실행파일이나 라이브러리를 빌드하는데 필요한 외부 라이브러리를 지정하는 것이다. 캐킨 패키지의 라이브러리는 `${catkin_LIBRARIES}`로 한번에 링크할 수 있다.

```cmake
# target_link_libraries(<target-name> <library1> <library2> ...)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
```



### install()

cmake로 `Makefile`을 만들고  `make`를 하여 타겟이 생성된 후 `make install`을 실행하면 타겟이 시스템에 설치된다. 설치란게 별게 아니고 타겟을 다른 곳에서도 쓸 수 있도록 소스 파일을 제외한 바이너리와 헤더 파일들을 다른 곳으로 복사하는 것을 말한다. 설치할 경로는 위에서 `CMAKE_INSTALL_PREFIX` 변수로 지정했다. 이 변수를 지정하지 않으면 기본적으로 `/usr/local`에 설치된다. `INSTALL()`은 복사할 파일의 종류에 따라 설치 경로에 대한 상대 경로를 지정한다.

```cmake
# install(TARGETS <target1> <target2> ...
        RUNTIME DESTINATION <executable-dir>
        LIBRARY DESTINATION <shared-lib-dir>
        ARCHIVE DESTINATION <static-lib-dir>
        PUBLIC_HEADER DESTINATION <header-dir>
        )
install(TARGETS ${OUTPUT_SHARED_LIB} ${OUTPUT_STATIC_LIB} ${OUTPUT_EXEC}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include
        )
```

위 함수의 인자들을 하나씩 알아보자

- TARGETS: 설치할 타겟들을 공백으로 구분하여 입력한다.
- RUNTIME DESTINATION: 실행 파일이 복사될 디렉토리를 지정한다.
- LIBRARY DESTINATION: "SHARED" 라이브러리 파일이 복사될 디렉토리를 지정한다.
- ARCHIVE DESTINATION: "STATIC" 라이브러리 파일이 복사될 디렉토리를 지정한다.
- PUBLIC_HEADER DESTINATION: `SET_TARGET_PROPERTIES()` 함수에서 지정한 PUBLIC_HEADER 파일을 복사할 디렉토리를 지정한다.

`INSTALL()` 함수에서 지정한 경로는 모두 `CMAKE_INSTALL_PREFIX` 혹은 `CATKIN_DEVEL_PREFIX`에 대한 상대 경로이다. 파일의 종류에 따라 저장될 디렉토리의 이름은 위와 같이 관습적으로 지정되어 있으니 가급적 저대로 쓰는 것이 좋다.  











- Catkin Workspace
    - 리눅스 빌드 시스템 gcc make cmake
    - 기존 catkin vs 새로운 catkin
    - catkin verbs
- 파이썬 가상환경: virutalenv, pyenv, pipenv



- 패키지 만들기
    - 토픽 노드: 기존 메시지 활용
    - 토픽 노드: 새로운 메시지 만들기
    - 서비스 노드: 기존 메시지 활용
    - 서비스 노드: 새로운 메시지 만들기
    - ROS launch 작성
- git을 이용한 패키지 관리

...

- 좌표계변환
- numpy, matplotlib
- ROS 패키지 활용 (SLAM, RViz)

