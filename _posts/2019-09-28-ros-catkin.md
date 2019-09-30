---
layout: post
title:  "Catkin Workspace"
date:   2010-09-28 09:00:13
categories: WIP
---



# Catkin Workspace



## 1. Build System

로봇공학 수업에서는 파이썬을 이용한 로봇 프로그래밍을 할 것이지만 ROS는 기본적으로 C++을 기준으로 시스템이 구성되어있다. C/C++은 컴파일 언어기 때문에 코드로부터 실행 파일이나 라이브러리를 만들기 위해서는 두 단계를 거쳐야한다.

1. 컴파일 (Compile): 사람이 쓴 코드를 기계어로 번역하여 목적(object) 파일을 생성한다.
2. 빌드 (Build): 목적 파일을 라이브러리와 링크하여 실행 파일을 만든다.

이러한 과정을 처리해주는 시스템을 빌드 시스템 (Build System)이라 하는데 리눅스에서는 프로젝트의 복잡도에 따라 다양한 수준의 빌드 시스템이 존재한다.

### 1.1. GCC

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

### 1.2 make

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

### 1.3 CMake

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
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
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



##  2. Catkin

### 2.1 Catkin 소개

ROS의 기본 철학이 최소 단위 기능만을 가지고 있는 여러 노드들의 조합으로 시스템을 구성하는 것이기 때문에 다수의 패키지를 동시에 개발하는 경우가 많다. 이때 여러 패키지의 코드를 수정한 후 이를 실행하기 위해서는 모든 패키지에서 따로 빌드를 해줘야한다. 즉 패키지마다 `cmake`와 `make`를 (옵션을 추가하여) 실행해야 한다는 것이다.  

이러한 과정이 번거롭기 때문에 ROS에는 모든 패키지를 하나의 워크스페이스(workspace) 경로에 담고 이들을 한번에 빌드할 수 있는 메타빌드 시스템(meta-build system)을 만들었는데 이것이 바로 캐킨(catkin)이다. 초기 ROS에서는 이러한 개념을 구현한 `rosbuild`가 있었으나 이후 `catkin`으로 발전하였다. 캐킨 시스템에서 사용자는 캐킨 명령어를 통해 워크스페이스 초기화, 패키지 생성, 전체 워크스페이스 빌드 등을 각각 한줄의 명령어를 통해 실행할 수 있게 되었다. 아래 세 가지 명령어만 알면 캐킨 시스템을 사용할 수 있다.

- `catkin_init_workspace`: `~/catkin_ws/src` 폴더에서 이 명령어를 실행하면 `CMakeLists.txt`가 자동으로 생긴다. 이 파일에서 src 폴더 아래 모든 캐킨 패키지를 검색하여 한번에 빌드할 수 있게한다.
- `catkin_create_pkg <pakcage name> <dependencies> `: 캐킨 패키지를 생성한다. 인자로 패키지 이름과 새로운 패키지가 의존하는 다른 ROS 패키지를 지정한다. 여기서 지정한 의존관계에 의해 빌드 순서가 결정된다.
- `catkin_make`: 캐킨 워크스페이스 내부의 모든 패키지를 일괄 빌드한다. 처음 실행하면 `~/catkin_ws` 아래 자동으로 `build, devel` 디렉토리가 생긴다.

> Note: 수업에서는 `~/catkin_ws` 경로를 기본 캐킨 워크스페이스로 쓰지만 꼭 `$HOME` 디렉토리에 `catkin_ws`란 이름을 쓸 필요는 없다. `~/catkin_ws`는 어디까지나 예시로서 흔하게 쓰이는 경로를 쓰는것 뿐이다.

캐킨 빌드 시스템을 이해하기 위해서는 워크스페이스 내부의 디렉토리 구성을 알아야 한다.

- src (source space): 패키지 소스코드가 있는 곳이다. `src` 바로 아래에는 `catkin_init_workspace`를 통해 만들어진 `CMakeLists.txt`가 있어야 하고 패키지들은 하위 디렉토리에 들어있어야 한다. 하위 디렉토리는 하나의 캐킨 패키지일 수도 있고 여러 패키지 디렉토리가 들어있을 수도 있다. 
- build (build space): `catkin_make`를 하면 자동으로 생성된다. 이곳은 빌드의 최종 결과물이 아닌 빌드에 필요한 설정 파일이나 중간 결과물들이 저장되는 곳이다.
- devel (development space): `catkin_make`를 하면 자동으로 생성된다. 이곳은 빌드의 결과물인 실행 파일, 메시지 헤더 파일, 라이브러리 등이 저장되는 곳이다. 노드의 실행에 직접적으로 필요한 파일들이라고 보면 된다. 또한 이 워크스페이스의 패키지를 어느 경로에서나 실행할 수 있게 해주는 `setup.bash`도 이곳에 있다.



### 2.2 Catkin 패키지 만들기

새로운 캐킨 워크스페이스를 만드는 것부터 간단한 패키지를 만드는 것까지 실습을 해보자. 여기서는 기본적인 C++ 프로젝트를 만들어본다.

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ..
$ catkin_create_pkg hello_ros roscpp std_msgs
```

`catkin_create_pkg` 명령어로 패키지를 생성하면 기본적으로 `CMakeLists.txt`와 `package.xml`이 생기고 주석을 통해 다양한 설정들을 가이드하고 있다.  

- package.xml: ROS에서 패키지를 관리하는데 필요한 정보를 담고 있다. 패키지를 배포하는데 필요한 저자, 라이센스 등의 정보와 패키지 의존성 등
- CMakeLists.txt: 로컬에서 소스를 빌드하는데 필요한 설정

두 파일에 다양한 설정 항목들이 있는데 모두 다 항상 필요한 것은 아니고 예제에 나오는 설정 위주로 의미와 사용법을 알아보자.  

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

```cmake
# cmake 최소 버전: ubuntu 18.04에 설치되는 현재 버전은 3.10.2 이므로 만족
cmake_minimum_required(VERSION 2.8.3)
# package.xml에 지정된 이름과 동일한 프로젝트 이름
project(hello_ros)

# ROS Kinetic 이후로는 C++11을 사용할 수 있다.
add_compile_options(-std=c++11)

# 캐킨 패키지 검색
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
## 시스템 패키지 검색 (리눅스 패키지)
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hello_ros
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hello_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/hello_ros_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
```



### 2.3 CMakeLists.txt 주요 함수

`CMakeLists.txt`에 사용되는 주요 함수를 소개한다. 미리 다 알고 있을 필요는 없고 `CMakeLists.txt`를 작성할 때 필요한 부분을 찾아서 보면 된다.

#### find_package()

현재 패키지에서 사용할 외부 패키지를 검색해주는 함수다. 일반적으로는 `find_package(<package_name1>, <package_name2>, ...)` 형식으로 쓴다.  

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



### add_message_files()





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

