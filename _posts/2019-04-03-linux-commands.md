---
layout: post
title:  "Linux Commands"
date:   2019-04-03 12:00:01
categories: 2019-1-seminar
---



# 1. Basic Linux Utils

## 사전약속
- box 안에서 각 줄의 앞에 붙는 기호의 의미
```
# 주석
(현재 경로)$ 명령어 입력
> 명령어 결과 출력
```
- 터미널 열기 단축키: alt + ctrl + t
- 디렉토리와 폴더는 동의어이나 커맨드 라인에서는 디렉토리가 조금 더 정확한 용어이다.
- 리눅스에서는 디렉토리도 일종의 특수한 파일이기 때문에 일반적으로 '파일'이라 함은 보통의 파일과 디렉토리를 통칭한다.  


## cd (change directory)
: 터미널에서 현재 경로를 변경하는 명령어  
> $ cd [dirname]

아래와 같은 폴더 구조가 있을 때  

```bash
/home/ian  
/usr/local/bin/  
    /include/X11  

# 현재의 절대경로 확인
$ pwd
> /home/ian

# 절대경로 활용해 이동 (ian은 자신의 사용자명으로 대체)
~$ cd /usr/local
/usr/local$

# 사용자의 홈디렉토리로 이동
/usr/local$ cd ~
~$

# 다시 /usr/local로 이동
$ cd /usr/local
/usr/local$

# 상대경로: 하위 디렉토리인 bin으로 이동
/usr/local$ cd bin
/usr/local/bin$

# 상대경로: 다시 상위 디렉토리인 local로 이동
/usr/local/bin$ cd ..
/usr/local$

# 상대경로: 상위 디렉토리로 갔다가 그 아래 include/X11로 이동
/usr/local$ cd ../include/X11
/usr/include/X11$

# 상대경로: 상위 디렉토리로 두 번 이동
/usr/include/X11$ cd ../..
/usr$

# 사용자 홈 디렉토리를 상대경로를 이용하여 이동
/usr$ cd ../home/ian
~$
```

## ls (list)
: 현재 경로에 있는 파일을 확인하는 명령어  
> $ ls (option) (path)

```bash
$ cd ~
# 파일 목록 출력
~$ ls
> Desktop    Downloads         Music     Public  Templates  workplace
  Documents  examples.desktop  Pictures  snap    Videos

# 자세한 정보 출력
~$ ls -l
# 파일타입+권한 링크수 사용자 파일크기 수정시간 이름
> drwxr-xr-x 2 ian ian 4096  8월 14 15:48 Desktop
  drwxr-xr-x 2 ian ian 4096  8월 14 15:48 Documents
  drwxr-xr-x 3 ian ian 4096  8월 14 16:54 Downloads
  -rw-r--r-- 1 ian ian 8980  8월 14 15:29 examples.desktop
  drwxr-xr-x 2 ian ian 4096  8월 14 15:48 Music
  drwxr-xr-x 2 ian ian 4096  8월 14 15:48 Pictures
  drwxr-xr-x 2 ian ian 4096  8월 14 15:48 Public
  drwxr-xr-x 3 ian ian 4096  8월 14 17:02 snap
  drwxr-xr-x 2 ian ian 4096  8월 14 15:48 Templates
  drwxr-xr-x 2 ian ian 4096  8월 14 15:48 Videos
  drwxrwxr-x 3 ian ian 4096  8월 14 17:04 workplace

# 숨김 파일까지 출력
~$ ls -a
> .              examples.desktop  .PyCharmCE2018.2
  ..             .gconf            snap
  .bash_history  .gnupg            .sudo_as_admin_successful
  .bash_logout   .ICEauthority     Templates
  .bashrc        .java             Videos
  .cache         .local            workplace
  .config        .mozilla          .Xauthority
  .dbus          Music             .xinputrc
  Desktop        Pictures          .xsession-errors
  .dmrc          .presage          .xsession-errors.old
  Documents      .profile
  Downloads      Public

# 숨김 파일까지 자세히 출력
~$ ls -al

# 특정 경로 안의 내용 출력
~$ ls /usr
> bin  games  include  lib  local  locale  sbin  share  src
```

## mkdir (make directory)
: 디렉토리를 만드는 명령어  
> $ mkdir 폴더명

```bash
# workplace 라는 디렉토리 만들기
~$ mkdir workplace
~$ cd workplace
~/workplace$ cd ..

# 존재하는 폴더 다시 만들면 에러
~$ mkdir workplace
> mkdir: cannot create directory ‘workplace’: File exists

# 하위 폴더까지 폴더 경로 생성
~$ mkdir -p workplace/foo/bar

# -p 옵션을 줄 시 기존 폴더가 있어도 에러를 출력하지 않음
~$ mkdir -p workplace/foo/bar
~$ mkdir -p workplace
```

## rm (remove)
: 파일 삭제
> $ rm (-r) 파일명

```bash
# 임시 파일/디렉토리 생성
~$ cd workplace
~/workplace$ mkdir dirA dirB dirC dirD
~/workplace$ touch fileA fileB fileC fileD
~/workplace$ ls
> dirA  dirB  dirC  dirD  fileA  fileB  fileC  fileD

# 파일 삭제
~/workplace$ rm fileA
~/workplace$ ls
> dirA  dirB  dirC  dirD  fileB  fileC  fileD

# 패턴으로 파일 삭제
~/workplace$ rm file*
~/workplace$ ls
> dirA  dirB  dirC  dirD

# 디렉토리 삭제
~/workplace$ rm -r dirA
~/workplace$ ls
> dirB  dirC  dirD

# 패턴으로 디렉토리 삭제
~/workplace$ rm -r dir*
~/workplace$ ls
> 

# 임시 파일/디렉토리 다시 생성
~$ cd workplace
~/workplace$ mkdir dirA dirB dirC
~/workplace$ touch fileA fileB fileC

# 현재 디렉토리 아래 모든 파일/디렉토리 삭제
~/workplace$ rm -r *
```

## cp (copy)
: 파일 복사 
> $ cp 대상파일 목적지

```bash
~$ cd ~/workplace
~/workplace$ rm -r *
~/workplace$ mkdir apple
~/workplace$ touch banana

# 파일 복사
~/workplace$ cp banana grape
~/workplace$ ls
> apple  banana  grape

# 복사를 할 때 목적지가 새로운 파일명이면 파일을 만들고
# 목적지가 존재하는 디렉토리면 그 안에 원본과 같은 이름으로 복사된다.
# 목적지의 디렉토리를 지정해도 그 아래 파일명까지 쓰면 새로운 이름으로 복사할 수 있다.
~/workplace$ cp banana apple 
~/workplace$ cp banana apple/banana2
~/workplace$ ls apple
> banana  banana2

# 폴더를 복사할 때는 -r을 붙인다. 
~/workplace$ cp -r apple cherry
~/workplace$ cp -r cherry apple
~/workplace$ ls
> apple  banana  cherry  grape
~/workplace$ ls apple
> banana  banana2  cherry
```

## mv (move)
: 파일 이동
> $ cp 대상파일 목적지

```bash
~$ cd ~/workplace
~/workplace$ rm -r *
~/workplace$ mkdir apple
~/workplace$ touch banana
~/workplace$ ls
> apple  banana

# mv를 이용한 파일명 변경
~/workplace$ mv apple cherry
~/workplace$ mv banana grape
~/workplace$ ls
> cherry  grape

# mv를 이용한 위치 변경
# mv는 -r 옵션을 붙이지 않아도 폴더를 이동할 수 있다.
~/workplace$ mv grape cherry
~/workplace$ ls
> cherry
~/workplace$ ls cherry
> grape
```

## find
: 현재 디렉토리 아래 있는 모든 파일과 디렉토리를 검색
```bash
# 파일, 폴더 생성
~$ cd ~/workplace
~/workplace$ rm -r *
~/workplace$ mkdir -p apple/banana/cherry/foo
~/workplace$ mkdir -p grape/kiwi/foo
~/workplace$ touch apple/banana/cherry/foo/bar1
~/workplace$ touch grape/kiwi/foo/bar2
~/workplace$ ls
> apple  grape

# foo 파일 검색
~/workplace$ find -name foo
> ./grape/kiwi/foo
  ./apple/banana/cherry/foo

# bar로 시작하는 파일 검색
~/workplace$ find -name bar*
> ./grape/kiwi/foo/bar2
  ./apple/banana/cherry/foo/bar1

# 수정일자, 파일 크기 등으로 검색하는 옵션도 있으나 이름 검색이 가장 많이 쓰임
```

## echo, cat

```bash
# 새 파일 생성하여 내용 입력, 기존 파일있다면 덮어쓰기  
$ echo/cat "내용" > 파일명   
# 기존 파일 있다면 붙여쓰기, 없다면 새 파일 생성  
$ echo/cat "내용" >> 파일명   
```
터미널에 텍스트 출력, 파일 출력, 참고자료 <http://withcoding.com/109>

```bash
# 텍스트 출력
~$ cd ~/workplace
~/workplace$ echo hello world
> hello world

# 환경 변수 출력
~/workplace$ echo $PATH
> /home/ian/bin:/home/ian/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin

# 간단한 텍스트 파일 생성 >
~/workplace$ echo "hello sweetie!" > mango
# 파일 내용 출력
~/workplace$ cat mango
> hello sweetie!

# 파일에 내용 추가 >>
~/workplace$ echo "I like mango" >> mango
~/workplace$ cat mango
> hello sweetie!
  I like mango

# 간단한 텍스트 파일 생성
~/workplace$ cat > apple
apple is sweet!
(ctrl+D)
# 파일 내용 확인
~/workplace$ cat apple
> apple is sweet!

# 파일에 내용 추가
~/workplace$ cat >> apple
but I prefer banana
(ctrl+D)
# 파일 내용 확인
~/workplace$ cat apple
> apple is sweet!
  but I prefer banana
```

## chmod
: 파일 권한 설정, 참고자료 <https://ko.wikipedia.org/wiki/Chmod>
> $ chmod [permission] [file]

```bash
~$ cd ~/workplace
~/workplace$ rm -r *
# 파일 생성하고 권한 확인
~/workplace$ echo "hello apple" > apple
~/workplace$ ls -l
> -rw-rw-r-- 1 ian ian 12  8월 14 23:01 apple
# 사용자는 읽기/쓰기 권한 있음

# 사용자의 읽기/쓰기 권한 제거
~/workplace$ chmod u-rw apple
~/workplace$ ls -l
> ----rw-r-- 1 ian ian 12  8월 14 23:01 apple

# 읽기 시도 실패
~/workplace$ cat apple
> cat: apple: Permission denied
# 쓰기 시도 실패 
~/workplace$ echo "apple is not fruit" >> apple
> bash: apple: Permission denied

# 사용자의 읽기/쓰기 권한 복원
~/workplace$ chmod u+rw apple
~/workplace$ cat apple
> hello apple
~/workplace$ echo "apple is not fruit" >> apple
~/workplace$ cat apple
> hello apple
  apple is not fruit
```


# 2. Package Management

## 패키지 업데이트, 패키지 목록보기
```bash
# 저장소 패키지 목록 업데이트
$ sudo apt update

# 설치된 패키지 최신 버전 업그레이드
$ sudo apt upgrade

# 설치된 패키지 목록
$ sudo apt list --installed

# 업그레이드 가능한 패키지 목록
$ sudo apt list --upgradable
```

## 패키지 검색
```bash
# 패키지 저장소의 패키지 검색
$ sudo apt search [package name or pattern]
$ sudo apt search unity-tweak-*

# 설치된 패키지 검색
$ sudo list --installed [package name or pattern]
$ sudo list --installed gedit*
> Listing... Done
  gedit/xenial,now 3.18.3-0ubuntu4 amd64 [installed]
  gedit-common/xenial,xenial,now 3.18.3-0ubuntu4 all [installed]
```

## 패키지 설치 및 삭제
```bash
# 패키지 설치 (예시: unity-tweak-tool)
$ sudo apt install [package1] [package2]
$ sudo apt install unity-tweak-tool

# 패키지 삭제
$ sudo apt remove [package1] [package2]
$ sudo apt remove unity-tweak-tool

# 패키지 완전 삭제 (설정파일까지 삭제)
$ sudo apt purge [package1] [package2]

# 패키지 삭제 후에는 쓰이지 않게된 의존 패키지들을 삭제한다.
$ sudo apt autoremove
```

## 저장소 추가
```bash
# atom editor 설치 예시
$ sudo add-apt-repository ppa:webupd8team/atom
$ sudo apt update; sudo apt install atom
```

