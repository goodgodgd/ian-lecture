---
layout: post
title:  "ROS build system"
date:   2019-04-24 09:00:01
categories: Seminar
---

First write a post here, then move it to `_post` after completion.

# 1. Linux Build System

C언어를 처음 배울 때 보통 Visual Studio에서 시작하는 경우가 많다. Visual Studio(이하 VS)는 통합개발환경(IDE)라서 VS에서 코딩도하고 컴파일 빌드 디버깅 실행 모두가 가능하다. 하지만 몇 줄짜리 "hello world" 코드를 빌드하기 위해 IDE가 꼭 필요한 것일까? 당연히 코드 편집 따로 빌드 따로 할 수 있다. 메모장에서 간단한 코드를 작성하고 VS에 설치된 컴파일러를 이용해 command line에서 빌드하는 것도 가능~~하다고~~한다. [링크](<https://docs.microsoft.com/en-us/cpp/build/walkthrough-compiling-a-native-cpp-program-on-the-command-line?view=vs-2019>) 윈도우에서야 초보자건 개발자건 모두가 VS를 쓰지만 **리눅스**에서는 보통 편집기와 컴파일러가 분리되어있다. 리눅스에서의 통합개발환경이란 VS처럼 자체적인 컴파일러까지 포함하는게 아니고 리눅스에 기본 설치된 컴파일러인 **gcc**를 잘 가져다 쓰는 코드 편집기를 말한다. IDE 마다 각자의 방식으로 소스코드 목록과 의존성을 관리하는 프로젝트 파일을 만드는데 소프트웨어를 배포할 때 특정 IDE의 프로젝트 파일을 배포하게 되면 그 소프트웨어를 받아서 쓰는 사람도 특정 IDE를 설치해야 하는 번거로움이 있다. 그래서 리눅스에서 공통적으로 사용하는 빌드 시스템이 있는데 바로  `make`와 `cmake`다. 그래서 소프트웨어을 개발할 때는 편리한 IDE를 쓰고 배포하거나 남의 소프트웨어를 가져다 쓸 때는 command line에서 리눅스 빌드툴을 써서 직접 빌드를 한다. gcc, make, cmake의 관계를 다음과 같다.

- gcc: GNU Compiler Collection의 약자로 C/C++의 컴파일러이다.
- make: 여러 단계의 gcc 명령을 `Makefile` 이라는 스크립트로 만들어 한번에 실행하게 하고 Incremental Build를 지원한다.
- cmake: 중간 단계를 일일이 지정해줘야 하는 복잡한 `Makefile`을 좀 더 편리하게 만들어준다. 이를 위해 `CMakeLists.txt`라는 스크립트를 작성해야 한다.

gcc를 직접 쓰기 불편해서 make가 생겼는데 프로젝트가 복잡해지면 Makefile 작성하기도 어려워서 cmake가 생겼단다. 이번 장에서는 gcc와 make의 간단한 사용 예시를 보고 왜 cmake를 써야하는지 이해한 다음 cmake 사용법에 대해 알아본다.  

본 포스트는 다음 참고자료를 요약하여 작성하였다. 아래 포스트들에서 연결된 추가적인 포스트까지 참고하였다.

- [[Make 튜토리얼] Makefile 예제와 작성 방법 및 기본 패턴](<https://www.tuwlab.com/27193>)
- [[CMake 튜토리얼] 1. CMake 소개와 예제, 내부 동작 원리](<https://www.tuwlab.com/27234>)



## 1.1 gcc and make

예를 들어 다음과 같은 구조를 가진 소스 코드를 빌드하고자 한다. 

![source structure](/ian-lecture/assets/build-system/structure.png)

소스 코드는 다음과 같다.

```cpp
// bar.h
#ifndef BAR_H
#define BAR_H

#include <iostream>
void bar();

#endif // BAR_H
// bar.cpp
#include "bar.h"
void bar() {
	std::cout << "hello bar\n";
}
// foo.h
#ifndef FOO_H
#define FOO_H

#include <iostream>
void foo();

#endif // FOO_H
// foo.cpp
#include "foo.h"
void foo() {
	std::cout << "hello foo\n";
}
// main.cpp
#include <iostream>
#include "foo.h"
#include "bar.h"
int main()
{
	std::cout << "Hello main\n";
	foo();
	bar();
    return 0;
}
```



이를 빌드하려면 다음 명령어를 실행한다. C언어를 빌드할 때는 `gcc`를 쓰고 C++을 빌드할 때는 `g++`을 쓴다. 아래 명령어는 각각의 `*.c` 파일을 컴파일하여 object 파일(`*.o`)을 만들고 이들을 묶어 실행 파일 `myapp`을 빌드한다.  

```bash
$ g++ -c -o main.o main.cpp
$ g++ -c -o foo.o foo.cpp
$ g++ -c -o bar.o bar.cpp
$ g++ -o myapp main.o foo.o bar.o
$ ./myapp
> Hello main
> hello foo
> hello bar
```

겨우 몇 개의 파일을 결합하여 빌드하는데 네 번의 명령어를 입력해야 하고 더 싫은건 코드를 수정할 때마다 같은 과정을 반복해야 한다는 것이다. 물론 `foo`만 수정하는경우 두 번째와 네 번째 명령어만 쓰면 되지만 어쨌든 매우 번거로운 과정이다. 이 과정을 쉘 스크립트로 작성하는 방법도 있지만 그러면 수정할 때마다 전체 빌드를 다시 하게 되므로 수정사항을 확인하는데 시간이 오래 걸릴 것이다.   

그래서 나온 것이 `make`다. 방금 말한대로 빌드과정을 make의 문법에 맞춰 스크립트로 작성한 후 `make` 명령어를 통해 한 번에 빌드할 수 있다. 위의 `g++` 명령어를 그대로 쓴 것과의 차이점은 Incremental Build가 가능하다는 것이다. 즉 변경한 부분만 알아서 빌드를 해준다. 다음과 같이 `Makefile`을 작성하고 `make` 명령어를 실행해보자.  

```bash
$ rm *.o *.out
$ gedit Makefile
# Makefile 작성
myapp: main.o foo.o bar.o
    g++ -o myapp main.o foo.o bar.o
main.o: foo.h bar.h main.cpp
    foo.o: foo.h foo.cpp
    bar.o: bar.h bar.cpp
# 닫기
$ make
$ ./myapp
> Hello main
> hello foo
> hello bar
```

자세한 Makefile 문법은 ~~나도 잘 모르니까~~ 넘어가기로 하고 파일들의 의존 관계를 기술했다는 것만 이해하고 넘어가자. 어쨌든 이렇게 Makefile을 만들고 `make`라는 실행하면 빌드가 되는데 중요한 것은 알아서 변경한 부분만 빌드해준다는 것이다. `make`를 실행한 후 한번 더 `make`를 실행해보거나 소스 파일 하나만 수정한 후 `make`를 실행해서 나오는 출력을 보면 이해할 수 있다.   

그러나 이것도 프로젝트 규모가 커지면 Makefile을 관리하는 것도 점점 버거워진다. 일단 소스 파일이 많아지면 중간 결과물 (`*.o`)도 많아지고 파일들의 의존 관계를 코드가 바뀔때마다 수정해주는 것도 번거롭고 빌드 target이 여러 개이거나 프로젝트가 다단계(?)로 이루어진 경우 등 Makefile은 쉽게 방대해지고 관리하기가 어려워진다.   

그래서 우리에게 필요한 것은? **CMake**



## 1.2 CMake

이번엔 cmake로 같은 프로젝트를 빌드해보자. 위 예제에서 cmake를 위한 설정 파일인 `CMakeLists.txt`를 한 줄만 쓰면 빌드가 된다. make와 cmake의 가장 큰 차이는 중간 단계를 생략해도 된다는 것이다. make는 중간 단계에서 생기는 object 파일들을 일일이 지정하고 소스 사이의 의존 관계도 직접 지정해줘야 하지만 cmake는 중간단계는 알아서 만들어주고 소스코드를 분석하여 의존성도 알아서 찾는다. 그래서 소스 파일과 결과물만 지정하면 된다. cmake는 실행하면 여러 파일과 디렉토리가 자동 생성되므로 `build` 디렉토리를 따로 만들어 그 안에서 실행하자.

```bash
$ rm *.o *.out
$ gedit CMakeLists.txt
# CMakeLists.txt 작성
ADD_EXCUTABLE(myapp main.cpp foo.cpp bar.cpp)
# 닫기
$ mkdir build
$ cd build
$ cmake ..
$ make
$ ./myapp
> Hello main
> hello foo
> hello bar
```

한 줄짜리 `CMakeLists.txt`로 빌드했는데 많은 것들이 생겼다. 각각의 파일을 열어 어떻게 생겼는지 구경해보자. 간단한 프로젝트 인데도 상당히 복잡한 파일들이 생성된 것을 볼 수 있다. 프로젝트가 커져도 온갖 상황에 대처할 수 있게 설계하다보니 기본 설정이 다소 많은듯 하다.

```
CMakeCache.txt
cmake_install.cmake
CMakeFiles/
Makefile
```

`CMakeCache.txt`는 빌드 설정 변수들을 모아놓은 것인데 일반 에디터에서 열어서 수정해도 되지만 `cmake-gui` 툴을 이용하면 편하고 안전하게 수정할 수 있다. 설치는 `sudo apt install cmake-qt-gui`로 하면 된다. 주로 외부 라이브러리 경로를 자동으로 못 찾을 때 이 파일에서 수동으로 지정해준 후 다시 `cmake`를 하면 잘 된다. 사실 **나머지 파일/디렉토리는 사용자가 열어볼 필요가 없다.** 사용자는 일단 `CMakeLists.txt`를 잘 작성하는 것이 중요하다. 위 예시는 간단한 예제였고 실제로는 좀 더 자세한 설정이 필요하다.



### 2.1 CMakeLists.txt 주요 명령어

cmake의 주요 명령어를 하나씩 알아보면서 `CMakeLists.txt`를 작성해보자.



#### cmake 예약 변수

cmake에는 자동으로 지정된 변수들이 많은데 이들을 잘 알아야 원하는 빌드 설정을 하고 `CMakeLists.txt`도 쉽게 작성할 수 있다. 대부분 `CMAKE_`로 시작하며 `CMakeCache.txt`를 보면 어떤 변수들이 있는지 볼 수 있다. 그 중 중요한 몇 개만 여기에 설명한다.

- `CMAKE_CURRENT_SOURCE_DIR`: 소스 파일들을 찾을 디렉토리 경로로 `CMakeLists.txt`가 있는 경로가 기본 값으로 들어있다. 다른 경로를 설정할 때 이 변수를 기준으로 상대 경로를 만들면 편하다.
- `CMAKE_INSTALL_PREFIX`: `make install` 할때 빌드 결과물이 복사될 경로로 `/usr/local`이 기본 값이다.
- `CMAKE_PREFIX_PATH`: `find_package()` 등의 명령에서 외부 프로젝트를 검색할 경로다. 어떤 프로젝트에서 다른 프로젝트의 라이브러리를 사용하고자 할 때 이 변수의 경로에서 하위 디렉토리까지 전부 검색한다.
- `CMAKE_PROJECT_NAME `: `project()`라는 명령어에서 설정할 수 있는 프로젝트의 이름이다.
- `CMAKE_BUILD_TYPE`: 빌드 형상을 지정할 수 있는 변수인데 빌드 형상은 대표적으로 `Debug`와 `Release`가 있다.



#### CMAKE_MINIMUM_REQUIRED() 최소 cmake 버전 확인

외부 라이브러리(3rd party software)를 이용하는 경우 외부 라이브러리의 `*.cmake` 파일에서 빌드 설정을 가져오는 경우가 많은데 이때 서로 `CMakeLists.txt`를 작성할 당시의 cmake 버전이 어느정도 맞아야 작동할 수 있다. 버전이 너무 다르면 cmake의 변수명이나 문법이 조금씩 다를 수 있기 때문이다. 그래서 보통 **`CMakeLists.txt`의 맨 위**에 이 명령어를 쓴다. 아래는 cmake 버전 2.8 이상을 요구하는 명령어다. 현재 시스템에 설치된 cmake가 이보다 낮으면 에러가 발생하게 된다.

```cmake
# cmake_minimum_required(VERSION <x.y.z.w>)
cmake_minimum_required(VERSION 2.8)
```



#### PROJECT() 프로젝트 이름 지정

이 명령어를 통해 프로젝트의 이름을 지정하면 `CMAKE_PROJECT_NAME`라는 변수에 저장된다. 이후 프로젝트 이름과 같은 실행 파일이나 라이브러리를 만들 때 이 변수를 사용할 수 있다.

```
# project(<프로젝트 명>)
project("HelloCMake")
```



#### MESSAGE() 문자열 출력

cmake를 하는 과정에서 어디까지 진행되었는지 확인하거나 변수에 어떤 값이 들어있는지 출력해서 확인하고자 할 때 쓰인다. 변수 내용을 참조할 때는 `${VAR_NAME}` 형식으로 사용한다. 변수는 따옴표 안팎에서 동일하게 사용된다.

```cmake
# message(<메시지>)
message("project name: ${CMAKE_PROJECT_NAME}")
message("install dir: ${CMAKE_INSTALL_PREFIX}")
```



#### SET() 변수 정의

set() 명령어는 cmake에서 가장 많이 쓰이는 명령어로 변수를 정의한다. 값이 하나 일때는 하나만 쓰면 되고 값이 여러 개인 목록일 때는 띄어쓰기로 구분한다.

```cmake
# 변수 정의
# set(<변수명> <값>)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_BUILD_TYPE Release)
# 목록 변수 정의
# set(<목록 변수명> <항목> <항목> <항목> ...)
set(SOURCE_FILES main.cpp foo.cpp bar.cpp)
set(CMAKE_PREFIX_PATH /usr/lib /usr/local/lib)
message("set vars: ${CMAKE_BUILD_TYPE}, ${SOURCE_FILES}")
```

cmake를 실행해보면 `set vars: Release, main.cpp;foo.cpp;bar.cpp`라는 메시지를 볼 수 있다. 목록변수를 내부적으로는 `;`으로 구분해서 보관하고 있는 것이다. 만약 하나의 값에 공백(띄어쓰기)이 들어있다면 값을 따옴표("")로 묶어주면 된다.



#### ADD_EXECUTABLE() 실행 파일 target 추가

빌드 결과물로 생성될 실행 파일과 실행 파일을 빌드하는데 필요한 소스 파일을 지정한다. 굳이 헤더파일을 쓰지 않아도 코드를 분석해서 알아서 헤더파일을 찾아준다. 실행파일의 이름은 `myapp`으로 하고 필요한 소스 파일 목록은 `SOURCE_FILES`라는 변수로 저장했으므로 변수로 대체한다.

```cmake
# add_executable(<실행_파일명> <소스_파일> <소스_파일> ... )
# add_executable(myapp main.cpp foo.cpp bar.cpp)
add_executable(myapp ${SOURCE_FILES})
```

일단 여기까지 쓰고 다시 빌드를 해보자. 자세한 설정이 더 들어갔지만 현재까지 결과는 같다. 지금까지 쓴 `CMakeLists.txt`는 다음과 같다. 한 줄씩 다시 보면서 의미를 떠올려보자.

```cmake
cmake_minimum_required(VERSION 2.8)
project("HelloCMake")
message("project name: ${CMAKE_PROJECT_NAME}")
message("install dir: ${CMAKE_INSTALL_PREFIX}")

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_BUILD_TYPE Release)
set(SOURCE_FILES main.cpp foo.cpp bar.cpp)
set(CMAKE_PREFIX_PATH /usr/lib /usr/local/lib)
message("set vars: ${CMAKE_BUILD_TYPE}, ${SOURCE_FILES}")

add_executable(myapp ${SOURCE_FILES})
```



### 2.2 라이브러리 생성 및 활용

실제로 쓸만한 프로그램을 만들기 위해서는 보통 외부의 라이브러리도 많이 쓰고 직접 라이브러리를 만들기도 한다. 라이브러리를 만들고 활용하는 방법에 대해 알아보자. 



- eigen 이용한 라이브러리 mymatlib 만들기
- add_library()
- find_package()
- INCLUDE_DIRECTORIES
- LINK_DIRECTORIES
- LINK_LIBRARIES
- mymatlib 이용한 실행파일 만들기
- add_executable
- ADD_DEPENDENCIES()

















