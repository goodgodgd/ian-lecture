---
layout: post
title: "[Cpp] Lecture Introduction"
date: 2021-08-19 09:00:03
categories: CppAlgorithm
---



## 1. 왜 (아직도) C++을 배워야 하나?

요즘같이 편리한 언어들이 넘쳐나는 세상에 왜 어렵고 불편한 C++을 배워야하나 하는 생각이 들 수 있다. 딥러닝을 주로 하는 연구자라면 파이썬만 알아도 큰 문제가 없을 수 있고 일반적인 사용자 서비스 개발자라면 자바, 자바스크립트, C#, Go 등 훨씬 생산성이 높은 언어(시간대비 구현 효율이 높은)를 주로 쓸 것이다. 반면 C++은 개발 생산성이 크게 떨어진다. 요즘 C/C++의 주 용도는 다른 언어에서 쓰이는 고성능 라이브러리를 만드는 것이고 보통의 개발자들은 그 라이브러리를 생산성 높은 다른 언어에서 사용하기 때문에 C++을 직접 쓸 일이 없다는 말도 한다. C/C++은 로우 레벨 언어의 성능 지향적인 측면이 있기 때문에 맞는 말이기도 하다. 모두가 python, javascript 등 스크립트 언어의 인터프리터나, unreal engine, OpenCV, tensorflow 같은 걸 만들 필요는 없다. 대다수의 개발자는 만들어진 라이브러리, 프레임워크, 인터프리터를 기반으로 새로운 어플리케이션을 만들 것이다.  

그럼 왜 아직도 C++을 배워야 하나?  

**프로그래밍에 대한 이해**

C++은 저수준부터 고수준 언어의 특성을 골고루 갖춘 언어다. 메모리를 직접 관리할 수도 있고 스마트 포인터를 이용해 자동으로 관리할 수도 있다. 함수만 써도 되지만 클래스를 쓰면 고차원적인 객체지향 프로그래밍이 가능하다. C++을 이해하는 것은 다른 모든 프로그래밍 언어를 더 잘 이해하는데 도움이 된다.  

**실시간 고성능 시스템 구현**

C++은 일단 고성능이다. 프로그래밍에 대한 충분한 지식이 있는 개발자가 C++언어로 최적화된 코드를 짜고 컴파일러에서 최적화 옵션을 잘 설정하여 빌드하면 다른 어떤 언어보다도 고성능의 프로그램을 구현할 수 있다. 그래서 전통적으로 실시간 반응이 필요한 로봇분야에서는 C/C++이 기본 언어로 사용되었다. 로봇에서는 다른 언어를 쓰고 싶어도 사용해야할 라이브러리가 C++로 되어 있어서 못 쓰는 경우도 많다.  

단, 초보 개발자가 컴파일러 최적화도 안하고 빌드하면 numpy, matlab보다 느리다. numpy, matlab은 전문가가 C언어로 코딩과 빌드를 최적화하여 만든 라이브러리를 사용하기 때문에 커다란 matrix의 단순 연산은 최적화된 C언어와 별 차이가 없다.

---

C++의 철학은 프로그램 내에서 일어나는 모든 일을 프로그래머가 제어한다는 것이다. 메모리 관리 책임도 프로그래머에 있고, 함수에 입력인자를 참조로 줄지, 포인터로 줄지, 값으로 전달할지 모두 프로그래머가 선택한다. 컴퓨터가 하는 모든 메모리 관리와 연산을 내가 제어하고 예측할 수 있다. 이로 인한 언어 자체의 장점도 있지만 단점도 있다. 

**C++의 장점**

1. 값 의미론: C++은 값(value)과 참조(reference)를 모두 지원하는데 값 의미론이 기본이고 객체를 전달할 때는 참조도 쓸 수 있다. 이 말은 객체의 소유권을 정확하게 제어할 수 있다는 것이다. 어떤 객체를 함수에 전달해서 함수 내부에서 그 객체를 수정할 때 함수 밖에서도 영향을 받을지 아닐지를 C++에서는 명확하게 알 수 있다.
2. const 정확성: 다른 언어에는 없는 기능인데 const 키워드를 통해 객체의 수정 가능성을 명시적으로 제어할 수 있다. 이를 통해 잘못된 코딩을 예방할 수 있고 보다 견고한 코드 베이스를 구축할 수 있다.
3. 하드 타입이기 때문에 타입마다 딱 필요한 만큼의 고정된 메모리를 사용한다. 파이썬의 경우 int 자체가 클래스 객체라서 숫자 `1`만 써도  24비트를 쓴다.
4. 이건 C++의 장점이라기보단 파이썬의 약점인데 멀티쓰레딩(multi-threading) 혹은 멀티프로세싱(multi-processing)을 제대로 활용할 수 있다. 파이썬에서 멀티쓰레딩을 하면 싱글쓰레드보다 느려지는 마법을 경험할 수 있다. 이게 바로 로봇분야에서 파이썬을 쓰기 어려운 가장 큰 이유다.

**C++의 단점**

1. 어렵다. 저수준부터 고수준언어의 여러가지 특성을 적극적으로 담다 보니 문법이나 STL의 사용법이 너무 다양해져서 그 문법을 다 아는 사람이 지구상에 존재할 수 없을 정도로 복잡하졌다. 저수준에서 최적화하면서 편리하게 코딩하고자하는 이율배반적인 욕심이 만들어낸 결과다.
2. 빌드 시간이 길다. 컴파일 시간이 아예 없는 인터프리터 언어를 쓰다가 C++을 쓰면 빌드하는데 걸리는 시간이 아깝게 느껴진다. 복잡한 라이브러리를 많이 include 할수록 컴파일 시간은 길어진다.
3. 헤더가 필요하다. 클래스를 정의하기 위해 헤더(.h)와 소스(.cpp) 두 가지 파일이 필요하고 둘 사이의 동기화를 수동으로 맞춰줘야 한다. 수많은 헤더들이 누적으로 소스 코드에 추가됨으로써 빌드 시간도 느려진다. C++20에서는 모듈이라는 개념이 생겨서 파이썬처럼 헤더 없이 바로 import 할수 있다고 하는데... 아직 정착되려면 시간이 필요한 것 같다.
4. 다른 라이브러리를 가져오기가 불편하다. 파이썬은 pip 한줄로 설치후 import 한줄로 사용 가능한데 C++은 운좋으면 시스템에 apt로 설치해서 쓸수도 있지만 직접 소스코드 받아와서 빌드해야하는 경우도 많다. apt로 설치가능한 라이브러리도 버전이 안맞아서 소스코드부터 빌드하게 된다. 여러곳에 흩어진 라이브러리를 프로젝트와 연결하는 것도 프로그래머의 몫이다.

정리하면 C++은 배워야 할것도 많고 타이핑도 더 많이 해야하지만 안전하고 견고한 프로그램을 만들수 있고 속도/메모리 최적화가 가능하다.



## 2. 강의 목표

본 강의에서는 C++의 심화과정과 이를 이용한 알고리즘 구현에 대해 배운다. 구체적인 내용은 다음과 같다.

1. 낯선 문법: 기본 문법 중에서도 헷갈리거나 익숙하지 않지만 알아두어야 할 내용 정리
2. STL container: vector, list, deque 등 STL에서 제공하는 자료구조
3. 값 의미론: lvalue, rvalue 등 값 전달과 관련된 개념
4. C++1x: C++11/14/17에 나온 Modern C++에서 추가된 문법, STL 정리
5. Big O 표현법
6. STL에 없는 자료구조
7. 알고리즘: 분할 정복, 동적 계획법 등



