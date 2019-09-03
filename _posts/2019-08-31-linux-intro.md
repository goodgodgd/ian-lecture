---
layout: post
title:  "Introduction to Linux"
date:   2010-08-31 09:00:13
categories: WIP
---



# 1. Unix/Linux 역사

## 1.1 유닉스(Unix) 역사

1960년대 중반 MIT, AT&T의 벨 연구소, General Electric 등은 새로운 컴퓨터 GE645와 MULTICS 라는 새로운 운영체제를 개발을 시작했다. 멀틱스(MULTICS) 프로젝트의 목적은 끊이지 않고 돌아갈 수 있는 다중 사용자, 다중 프로세서, 다중 프로세스 기반의 시스템을 만드는 것이었다. (강력한 보안기능은 덤) 하지만 시스템이 너무 복잡해져서 이 프로젝트는 결국 실패하게 된다.  

1969년 벨 연구소의 켄 톰슨(Ken Thompson)은 멀틱스 개발에 참여하고 있었는데 사무실에 안쓰는 PDP-7이라는 미니컴퓨터(그때 당시의 거대했던 컴퓨터에 비해..)를 쓰기 위해 어셈블리어로 간단한 운영체제를 직접 구현해보게된다. 이에 흥미를 가진 데니스 리치(Dennis Ritchie), Rudd Canaday 등과 함께 멀틱스의 주요 개념들을 가져오면서 단순화 시킨 운영체제인 유닉스(Unix)를 만들게된다.  

1970년대 초반 유닉스를 PDP의 최신 버전인 PDP-11/20에서 작동가능하게 업데이트를 하며 점점 기능을 추가해 나가며 사용자가 늘어나게 된다.  

1973년 데니스 리치가 여러 종류의 컴퓨터에서 사용할 수 있는 C언어를 개발하고 유닉스를 C언어로 다시 쓰게 된다. 기존엔 어셈블리어로 되어있어서 컴퓨터마다 모든 프로그램을 다시 작성해야 했지만 C언어로는 모든 컴퓨터에서 하나의 코드로 작동할 수 있게 되었다. (바로 이식 가능했던 건 아니고 PDP외 다른 컴퓨터에 이식된건 몇 년 후의 일이다.)

1970년대 말에는 여러 대학에서 사용되는데 UC Berkeley의 빌 조이(Bill Joy)와 척 핼리(Chuck Haley)라는 학생들이 유닉스 코드에 수정을 가해서 멀티 태스킹이라던지 네트워크 기능이 추가되었다. 이로부터 발전한 시스템이 BSD (Berkeley Software Distribution) 유닉스가 된다. AT&T에서 개발한 유닉스는 System III, IV, V 등으로 발전하여 업계 표준으로 자리잡지만 BSD도 꾸준히 개발되며 다른 유닉스 배포판에 큰 영향을 미치게 된다. 

1980년대에는 업계 표준으로 자리잡으며 다양한 회사에서 고유의 상업용 유닉스를 개발하며 경쟁하고 합쳐지고 사라지는 등 다양하게 발전하지만 인터페이스가 각각 달라 표준화에 대한 요구가 높아지고 1988년 IEEE에서 유닉스 시스템 표준 API 명세인 POSIX를 발표하게 된다.



## 1.2 리눅스(Linux) 역사

1980년대 리처드 스톨만(Richard Stallman)은 유닉스의 상업화에 반대하여 자유 소프트웨어 재단(Free Software Foundation)을 세우고 유닉스의 무료 버전을 만들기 위해 GNU (GNU is Not Unix) 프로젝트를 시작했다. gcc (C 컴파일러), emacs (에디터) 등 유용한 도구들을 개발했으나 결정적으로 하드웨어를 다루는 커널을 개발하는데 어려움을 겪고 있었다.

한편, 1991년 헬싱키 대학의 21살의 대학생 리누스 토발즈(Linus Tovalds)는 유닉스와 호환되는 커널을 개발했는데 이것이 리눅스의 시초다. 1991년 8월 25일 토발즈는 미닉스(minix) 사용자 게시판에 이런 글을 올렸다.

> Hello everybody out there using minix -
>
> I'm doing a (free) operating system (just a hobby, won't be big and professional like gnu) for 386(486) AT clones. This has been brewing since april, and is starting to get ready. I'd like any feedback on things people like/dislike in minix, as my OS resembles it somewhat (same physical layout of the file-system (due to practical reasons) among other things).
>
> I've currently ported [bash(1.08)](https://en.wikipedia.org/wiki/Bash_(Unix_shell)) and gcc(1.40), and things seem to work. This implies that I'll get something practical within a few months, and I'd like to know what features most people would want. Any suggestions are welcome, but I won't promise I'll implement them :-)
>
> Linus (torvalds@kruuna.helsinki.fi)
>
> PS. Yes - it's free of any minix code, and it has a multi-threaded fs. It is NOT portable (uses 386 task switching etc), and it probably never will support anything other than AT-harddisks, as that's all I have :-(.
>
> — Linus Torvalds

처음엔 자신이 사용하던 컴퓨터에서만 작동가능했다. 그리고 커널만으로는 할 수 있는게 별로 없기 때문에 토발즈는 GNU 소프트웨어가 필요했고 GNU는 커널이 필요했기 때문에 두 프로젝트는 자연스럽게 합쳐진다. 리눅스는 공식적으로 GNU/Linux라고 부르는게 맞다. GNU 소프트웨어들은 곧 리눅스 커널에서 완전히 작동하게 되었고 점점 지원가능한 하드웨어가 많아지고 그에 따라 사용자/개발자가 늘어나 오늘날까지 발전하게 된다.

**출처**

- <https://en.wikipedia.org/wiki/History_of_Unix>
- <https://en.wikipedia.org/wiki/History_of_Linux>
- <http://coffeenix.net/doc/misc/unix-history.html>
- <http://wiki.kldp.org/HOWTO/html/Secure-Programs-HOWTO/history.html>
- <https://rakuraku.tistory.com/107>
- <https://12bme.tistory.com/220>



# 2. 커널(kernel)과 쉘(shell)

운영체제(Operating System, OS)란 컴퓨터 하드웨어 자원을 관리하고 소프트웨어를 실행할 수 있는 환경을 말한다. 유닉스/리눅스에서 이런 역할을 하는 소프트웨어를 **커널(kernel)**이라고 한다. 엄밀히 말해 커널 자체가 유닉스/리눅스(=운영체제)라고 할 수 있으며 커널 외의 다른 소프트웨어들을 응용 프로그램이라고 볼 수 있다.  

아래 그림처럼 커널은 시스템의 핵심이다. 커널의 주요 기능은 다음과 같다.

- 프로세스 관리(Process management):여러 프로그램이 실행될 수 있도록 프로세스들을 CPU 스케줄링하여 동시에 수행되도록 한다. 
- 파일 관리(File management): 디스크와 같은 저장장치에 파일 시스템을 구성하여 파일을 관리한다.
- 메모리 관리(Memory management): 메인 메모리가 효과적으로 사용될 수 있도록 관리한다. 
- 통신 관리(Communication management): 네트워크를 통해 정보를 주고받을 수 있도록 관리한다. 
- 주변장치 관리(Device management): 모니터, 키보드, 마우스와 같은 장치를 사용할 수 있도록 관리한다.

![unix_structure](../assets/robotics-linux/unix_structure.png)

사용자가 리눅스를 다루는 방법은 크게 두 가지 방법이 있다. 하나는 쉘(shell)을 통해 CLI (Command Line Interface)로 직접 시스템을 조작하는 방법과 다른 하나는 프로그램을 통해 시스템을 조작하는 방법이다. 요즘은 GUI도 많이 쓰지만 그건 어디까지나 GUI 입력을 CLI 명령으로 바꿔서 실행해주는 환경일 뿐이다. 두 방법 모두 시스템 호출(System Call)을 통해 커널의 기능을 사용한다. 시스템 호출(System call)은 커널을 제어할 수 있는 프로그래밍 인터페이스다. 사용자는 프로그래밍을 할 때 시스템 호출을 통해 커널의 기능(e. g. 메모리 할당, 파일 읽기/쓰기 등)을 사용할 수 있다.  

사용자가 커널을 직접 사용하는건 불안정하고 비효율적이기 때문에 CLI로 명령을 입력할 때는 쉘(shell)을 쓰고 프로그램으로 명령을 줄 때는 라이브러리를 주로 사용한다. 쉘은 유닉스 계열 시스템을 다루는 가장 기본적인 방법이기 때문에 켄 톰슨이 유닉스를 처음만들때부터 있었으며 리누스 토발즈가 리눅스에 처음 포팅한 소프트웨어가 gcc와 본 쉘(Bourne Shell)이었다.  

쉘의 종류도 여러가지가 있고 쓰는 방법도 약간씩 다른데 리눅스에서 기본적으로 쓰는 쉘은 Bash(Bourne Again Shell)다. 





















