---
layout: post
title:  "Linux Installation"
date:   2019-04-03 09:00:01
categories: 2019-1-seminar
---

## Linux Installation

1. VirtualBox 설치
    - 접속 https://www.virtualbox.org/wiki/Downloads
    - `Windows Hosts` 다운로드 받아서 설치

2. Create Ubuntu Image
    - 접속 http://releases.ubuntu.com/16.04/
    - `ubuntu-16.04.5-desktop-amd64.iso` 다운로드
    - VirtualBox 시작
    - Click "새로 만들기"
    - 머신 이름: *Ubuntu16*
    - 메모리 용량 설정: 2048MB
    - "yes-yes-yes"로 계속 진행
    - 디스크 사이즈: 15GB

3. Install Ubuntu
    - 시작 *Ubuntu16*
    - 언어선택: English
    - 이름은 가급적 짧게 짓기
    - 설치 후 머신 재시작

4. 한글 세팅
    - 링크 참조: [link1](http://androidtest.tistory.com/52) or [link2](https://m.blog.naver.com/PostView.nhn?blogId=opusk&logNo=220986268503&proxyReferer=https%3A%2F%2Fwww.google.co.kr%2F)

5. Qt 설치
    - 접속 https://download.qt.io/archive/qt/5.11/5.11.1/
    - 설치 파일 다운로드 후 `~/Downloads`에 있는 다운로드 더블클릭하여 실행 후 설치 진행

6. Additional settings
    - 외부 USB 연결
        - [여기](https://www.virtualbox.org/wiki/Downloads) 에서 다운로드 후 설치: "VirtualBox Extension Pack"
        - 가상 머신 시작
        - "장치" - "게스트 확장 CD 이미지 삽입" 후 설치
        - 가상 머신 재시작
        - "장치" - "USB" - 연결할 USB 선택
    - host PC와 공유 폴더 설정
        - 외부 USB 설정처럼 설치
        - VirtualBox에서 "설정" - "공유폴더" - "머신폴더" 아래에 공유할 폴더 선택 
        - 재시작
    - 시스템이 너무 느릴 때
        - VirtualBox 실행
        - 가상머신 우클릭 - "설정" - "시스템" - "프로세서" 에서 프로세서 개수 할당

