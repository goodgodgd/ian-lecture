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

한편, 1991년 헬싱키 대학의 21살의 대학생 리누스 토발즈(Linus Tovalds)는 유닉스와 호환되는 커널을 개발했는데 이것이 리눅스의 시초다. 처음엔 자신이 사용하던 컴퓨터에서만 작동가능했다. 하지만 커널만으로는 할 수 있는게 별로 없기 때문에 토발즈는 GNU 소프트웨어가 필요했고 GNU는 커널이 필요했기 때문에 두 프로젝트는 자연스럽게 합쳐진다. GNU 소프트웨어들은 곧 리눅스 커널에서 완전히 작동하게 되었고 점점 지원가능한 하드웨어가 많아지고 그에 따라 사용자/개발자가 늘어나 오늘날까지 발전하게 된다.









출처

- <https://en.wikipedia.org/wiki/History_of_Unix>
- <https://en.wikipedia.org/wiki/History_of_Linux>
- <http://coffeenix.net/doc/misc/unix-history.html>
- <http://wiki.kldp.org/HOWTO/html/Secure-Programs-HOWTO/history.html>
- <https://rakuraku.tistory.com/107>
- <https://12bme.tistory.com/220>