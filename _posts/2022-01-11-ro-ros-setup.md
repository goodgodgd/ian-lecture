---
layout: post
title:  "[Ro] ROS 2 Setup"
date:   2022-01-11 09:00:13
categories: Robotics
---



## ROS 2 Setup

ROS 2 Foxy Fitzroy 버전을 설치하는 방법을 소개한다. 본 내용은 다음 링크들을 참조하여 작성했다.

<https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>

<https://cafe.naver.com/openrt/25288>



### 1. 지역 설정

Locale 설정이란 터미널에 보이는 문자열의 언어나 형식을 설정하는 것이다. 터미널에 표시되는 경고나 에러를 한글이나 영어로 표시할 수 있고 날짜 등의 표시 형식을 특정 지역의 형식으로 맞출 수 있다. ROS2를 위해서는 locale이 UTF-8 계열로 설정되어 있어야 한다. 설정은 시스템 재시작 후 제대로 적용된다.

```
# 현재 locale 확인
$ locale

$ sudo apt update && sudo apt install locales
# locale 파일 생성
$ sudo locale-gen en_US en_US.UTF-8
# locale 설정 (영어)
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
$ export LANG=en_US.UTF-8

# 시스템 재시작
# 변경된 locale 재확인
$ locale
```

<https://www.thomas-krenn.com/en/wiki/Configure_Locales_in_Ubuntu>

<https://antilibrary.org/2729>

<https://jellybeanz.medium.com/ubuntu-locale%EB%B3%80%EA%B2%BD-9fab2d85d9bf>



### 2. 소스 설정

ROS2 저장소를 시스템에 추가한다. 여기서 *저장소*란 apt 명령어를 통해 패키지를 다운로드 받을 수 있는 원격 서버를 의미한다. 데비안 계열 리눅스에서 저장소 주소들을 가지고 있는 파일들은 `/etc/apt` 디렉토리 아래 있다. `cat /etc/apt/sources.list`를 쳐보면 기본 우분투 저장소들을 볼 수 있다. 그 외 다른 저장소들은 주로 `/etc/apt/sources.list.d/` 아래에 있다. 여기서도 `/etc/apt/sources.list.d/ros2.list` 파일에 ROS2를 위한 저장소 주소를 저장한 후 `apt update`를 하면 ROS2 패키지들을 설치할 수 있다. 간단하게는 아래 명령어를 쓰면 된다.

```
echo "deb http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

그런데 유명 저장소의 경우 대부분 저장소 주소를 추가하기 전에 저장소의 패키지를 검증 할 수 있는 키(key)를 받게 한다. 유명 저장소는 원활한 패키지 전달을 위해 여러 곳에 미러(mirror) 서버가 분산되어있는데 누군가 악의적으로 패키지 관리자가 승인하지 않은 이상한 서버 주소를 알려주면 의도하지 않은 다른 패키지를 설치할 수도 있다. 그래서 유명 저장소는 대부분 키 서버를 두고 암호화된 키를 먼저 사용자가 받게 한다. 키가 있어야 정상적인 저장소의 패키지에 걸린 서명을 인증하여 패키지를 다운로드 할 수 있다.

```
$ sudo apt update && sudo apt install curl gnupg2 lsb-release

# 키 서버에서 패키지 키 다운로드
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# 저장소 추가 (시스템 아키텍쳐와 키 파일 지정)
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 패키지 목록 업데이트
$ sudo apt update
```

참고자료: <https://www.redhat.com/sysadmin/rpm-gpg-verify-packages>



### 3. ROS 2 설치

ROS 2 설치 자체는 명려어 한 줄로 끝난다. ROS의 대부분의 패키지를 설치하는 `ros-foxy-desktop`과 DDS 구현을 설치하면 된다.

```
sudo apt install ros-foxy-desktop ros-foxy-rmw-fastrtps* ros-foxy-rmw-cyclonedds*
```

ROS가 제대로 설치됐는지 확인하기 위해 노드 간의 메시지 통신을 테스트해본다. ROS 설치 패키지에 간단한 통신 확인용 패키지가 포함되어 있다.  

터미널1에서 먼저 `/opt/ros/foxy/setup.bash`를 실행하는데 이것은 터미널에서 ROS 명령어를 쓸수 있도록 설정하는 스크립트다. 그리고 메시지를 전송하는 `talker`를 실행한다.

```
$ source /opt/ros/foxy/setup.bash
$ ros2 run demo_nodes_cpp talker
[INFO]: Publishing: 'Hello World: 1'
[INFO]: Publishing: 'Hello World: 2'
[INFO]: Publishing: 'Hello World: 3'
[INFO]: Publishing: 'Hello World: 4'
[INFO]: Publishing: 'Hello World: 5'
[INFO]: Publishing: 'Hello World: 6'
...
```

터미널2에서도 `setup.bash` 실행 후 `listener`를 실행하면 `talker`에서 보내는 메시지를 잘 받아오는 것을 볼 수 있다.

```
$ source /opt/ros/foxy/setup.bash
$ ros2 run demo_nodes_py listener
[INFO]: I heard: [Hello World: 3]
[INFO]: I heard: [Hello World: 4]
[INFO]: I heard: [Hello World: 5]
[INFO]: I heard: [Hello World: 6]
...
```



### 4. 개발 툴 설치

아래 소프트웨어는 ROS2를 이용한 로봇 프로그래밍에 필수적인 소프트웨어 모음이다.

```
$ sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget

$ sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev \
  libcunit1-dev
```

아래 파이썬 패키지는 파이썬 패키지 개발 시 필요한 패키지이지만 시스템에 바로 설치할 필요는 없다. 나중에 가상환경 구축 후 설치하면 된다.

```
$ python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
```



### 5. 작업공간 만들기

ROS 자체는 `/opt/ros/foxy`에 설치했고 이곳에 이미 많은 패키지가 설치돼있지만 추가로 여러분이 ROS 패키지를 직접 만든다던가 다른데서 ROS 패키지를 소스로 받아올때는 새로운 **작업공간**(workspace)에서 관리하는 것이 낫다. 작업공간은 사용자가 임의의 폴더를 만들어 지정할 수 있으며 동시에 여러개의 작업공간을 만들어도 된다. 여기서 사용하는 `~/robot_ws`는 관습적인 저장소 경로일 뿐이다. 빌드를 하고 나면 build, install과 같은 폴더들이 생긴다. 특히 install/local_setup.sh 라는 스크립트가 자동 생성되는데 이 스크립트를 실행해야 이 작업공간 내부의 패키지들을 활용할 수 있다.

```
$ source /opt/ros/foxy/setup.bash
$ mkdir -p ~/robot_ws/src
$ cd ~/robot_ws/
$ colcon build --symlink-install
```



### 6. ~/.bashrc 설정

우분투에는 `$HOME` 폴더에 `.bashrc`라는 숨김 파일이 있다. 이 파일은 터미널이 실행될 때 자동으로 실행되어야 하는 명령어를 담은 스크립트다. 리눅스의 모든 명령어는 터미널을 통해 실행되기 때문에 시스템에 어떤 설정을 하고 싶다면 주로 이곳에 설정 명령어를 넣으면 된다.

```
# ROS와 workspace 활성화
source /opt/ros/foxy/setup.bash
source ~/robot_ws/install/local_setup.bash

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/vcstool-completion/vcs.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/robot_ws

export ROS_DOMAIN_ID=7
export ROS_NAMESPACE=robot1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export RMW_IMPLEMENTATION=rmw_connext_cpp
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

# export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}]: {message}'
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1

alias cw='cd ~/robot_ws'
alias cs='cd ~/robot_ws/src'
alias cb='cd ~/robot_ws && colcon build --symlink-install'
```



## 개발환경 설정

ROS를 설치했다면 이제는 ROS 프로그래밍을 할 수 있는 환경을 만들어보자. 여기서는 VSCode를 이용한 C++/Python 개발환경을 세팅할 것이다. 먼저 [VSCode에 대한 기본적인 설명](https://goodgodgd.github.io/ian-lecture/archivers/cpp-how-to-use-code)을 보고 오자.

### 1. VSCode 설치

설치파일 [다운로드](https://code.visualstudio.com/download) 후 아래 명령어를 실행하자.

```
$ cd Downloads
$ sudo dpkg -i code_xxx.deb
# VSCode 실행
$ code
```



### 2. 작업공간 열기

ROS의 작업공간과 VSCode의 작업공간은 좀 다른 개념이지만 여기서는 같은 디렉토리를 사용한다. `~/robot_ws`를 VSCode의 작업공간으로 열어보자.

- File - Add Folder to Workspace
- ~/robot_ws 선택



### 2. 확장(Extention) 설치

왼쪽의 Extensions 탭에서 아래 확장들을 설치하여 VSCode에 다양한 기능을 추가해보자.

| 이름                                       | 필수 | 설명                                                         |
| ------------------------------------------ | ---- | ------------------------------------------------------------ |
| C/C++                                      | O    | C/C++ InstalliSense, 디버깅 및 코드 검색                     |
| CMake                                      | O    | CMake 언어 지원                                              |
| CMake Tools                                | O    | CMake 언어 지원 및 다양한 툴                                 |
| Python                                     | O    | Linting, IntelliSense, 디버깅, 코드 서식 지정, <br>리팩토링, 단위 테스트 등 |
| ROS (Microsoft)                            | O    | ROS 개발 지원                                                |
| URDF (smilerobotics)                       |      | URDF/xacro 지원                                              |
| Colcon Tasks (deitry)                      |      | colcon 명령어를 위한 VSCode Task                             |
| XML Tools                                  |      | XML, XQuery, XPath 지원                                      |
| YAML                                       |      | YAML 지원                                                    |
| Makedown All in One                        |      | Markdown 지원                                                |
| Highlight Trailing White <br>Spaces (Yves) |      | 의미 없이 사용된 공백의 스페이스 문자 강조                   |
| Bracket Pair Colorizer                     |      | 괄호 열기/닫기를 짝을 맞추어 색상화시킴                      |
| Better Comments                            |      | 코멘트 하이라이팅 강화                                       |



### 4. VSCode 개발환경 설정

#### 4.1. Settings

'settings.json'은 VSCode의 사용자별 글로벌 환경 설정을 지정하는 파일이다. 이 파일에 기술된 설정들은 모든 작업 공간(workspace)에서 적용된다. 예를 들어, 미니맵 사용, 세로 제한 줄 표시, 탭 사이즈 등 이다. 

ROS와 관련된 설정은 3가지 정도로 `"ros.distro": "foxy"`와 같이 ROS 버전을 지정하고, `"colcon.provideTasks": true`와 같이 colcon이 지원되는 Task를 사용한다는 의미로 지정한다. 그리고 `"files.associations"`을 통해 확장자로 알 수 없는 *.repos, *.world, *.xacro 와 같이 ROS에서만 사용되는 확장자를 가진 파일들을 어떤 형식으로 표시해야 하는지 설정을 하게된다.

> Command Palette (Ctrl + Shift + P)
>
> Preferences: Open Settings (JSON)

```json
{
  "cmake.configureOnOpen": false,
  "editor.minimap.enabled": false,
  "editor.mouseWheelZoom": true,
  "editor.renderControlCharacters": true,
  // white vertical lines in editor
  "editor.rulers": [90, 110],
  "editor.tabSize": 2,
  "files.associations": {
    "*.repos": "yaml",
    "*.world": "xml",
    "*.xacro": "xml"
  },
  "files.insertFinalNewline": true,
  "files.trimTrailingWhitespace": true,
  "terminal.integrated.scrollback": 1000000,
  "workbench.iconTheme": "vscode-icons",
  "workbench.editor.pinnedTabSizing": "compact",
  "ros.distro": "foxy",
  "colcon.provideTasks": true,
  "files.autoSave": "afterDelay",
  "editor.detectIndentation": false,
}
```



#### 4.2. C/C++ Properties

C/C++ 관련 설정이다. 현재 지정한 작업 공간 (여기서는 ~/robot_ws)의 운영체제는 무엇인지, include 폴더의 경로는 어떻게 되는지, C/C++ 규칙은 어떤 표준을 기준을 사용할지의 여부, 컴파일의 경로, intelliSense 모드 등을 설정하게 된다. 여기서 설정하는 것이 실제 빌드에 적용된다기 보다는 코드 자동완성을 위한 설정에 가깝다.  

> Command Palette (Ctrl + Shift + P)
>
> C/C++: Edit Configurations (JSON)

```json
{
  "configurations":
  [
    {
      "name": "Linux",
      "includePath": [
        "${default}",
        "${workspaceFolder}/**",
        "/opt/ros/foxy/include/**"
      ],
      "defines": [],
      "compilerPath": "/usr/bin/g++",
      "cStandard": "c99",
      "cppStandard": "c++14",
      "intelliSenseMode": "linux-gcc-x64"
    }
  ],
  "version": 4
}
```



#### 4.3. Tasks (optional)

VSCode에서는 외부 프로그램을 Command Line Interface (CLI) 을 통해 연동하게 하는 기능이 있는데 이를 Task라고 한다. 단순한 텍스트 에디터 기능이 기본인 VSCode가 다양한 기능을 수행하고 쉽게 기능 확장을 사용할 수 있게 된 것도 이 Task 기능이 있었기 때문이다. 

아래의 내용은 ROS 2에서 빌드할 때 사용되는 colcon과 관려한 build, test, clean 작업을 Task로 만들었다. 이를 통해 VScode에서 `Ctrl + Shift + b`로 빌드할 수 있고 아래와 같이 기본 설정 이외의 Task도 실행할 수 있다.



> VSCode의 탐색기(explorer) 탭에서 `.vscode` 디렉토리 아래에 `tasks.json` 생성

```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "colcon: build",
      "type": "shell",
      // 개발 중일때는 Debug 테스트할 때는 Release
      "command": "colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Debug'",
      "problemMatcher": [],
      "group": {
        "kind": "build",
        "isDefault": true
      }
    },
    {
      "label": "colcon: test",
      "type": "shell",
      "command": "colcon test && colcon test-result"
    },
    {
      "label": "colcon: clean",
      "type": "shell",
      "command": "rm -rf build install log"
    }
  ]
}
```

- 실행: Ctrl + Shift + p → Tasks: Run Tasks → Task 선택

- 단축키 설정: Ctrl + Shift + p → Tasks: Run Tasks → 옆에 톱니바퀴 모양 선택 → 단축키 설정 (Ctrl + t)



#### 4.4. Launch (optional)

VSCode에서의 Launch는 'Run and Debug' (`Ctrl + Shift + d`)에서 사용되는 실행 명령어로 언어별, 디버거별로 설정이 가능하고 세부 옵션으로  Launch가 실행되기 전 즉 디버깅하기 전에 사용할 Task를 지정하거나 콘솔 기능을 설정할 수도 있다.

여기서는 ROS 2에서 많이 사용되는 Python과 C++ 언어에 맞추어 VSCode의 디버깅 툴을 지정하였다. C++의 경우 GDB, Python의 경우 debugpy으로 설정하였고 Python의 경우 별도 빌드 없이 디버깅하도록 하였고, C++의 경우에는 GDB를 실행하기 전에 colcon build 를 수행하도록 하였다.

```json
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "Debug-rclpy(debugpy)",
      "type": "python",
      "request": "launch",
      "program": "${file}",
      "console": "integratedTerminal"
    },
    {
      "name": "Debug-rclcpp(gbd)",
      "type": "cppdbg",
      "request": "launch",
      "program": "${workspaceFolder}/install/${input:package}/lib/${input:package}/${input:node}",
      "args": [],
      "preLaunchTask": "colcon: build",
      "stopAtEntry": true,
      "cwd": "${workspaceFolder}",
      "externalConsole": false,
      "MIMode": "gdb",
      "setupCommands": [
        {
          "description": "Enable pretty-printing for gdb",
          "text": "-enable-pretty-printing",
          "ignoreFailures": true
        }
      ]
    }
  ],
  "inputs": [
    {
      "id": "package",
      "type": "promptString",
      "description": "package name",
      "default": "topic_service_action_rclcpp_example"
    },
    {
      "id": "node",
      "type": "promptString",
      "description": "node name",
      "default": "argument"
    }
  ]
}
```









