---
layout: post
title:  "Module and Package"
date:   2019-03-21 09:00:13
categories: 2019-1-systprog
---


# Module and Package

## 1. 모듈과 패키지는 무엇?

지금까지 우리는 하나의 파일에서만 프로그램을 실행해왔지만 실제로 개발을 할 때는 다수의 파이썬 파일을 만들어 유기적으로 연결해야 할 뿐만 아니라 남이 만든 코드도 적극적으로 활용해야 한다. 어떤 파이썬 파일에서 `import` 키워드를 통해 외부 파이썬 파일의 함수, 변수, 클래스를 가져다 쓸 수 있다. 외부 코드는 기능별로 함수나 클래스 등을 묶어서 파일에 저장하는데 코드 규모가 작으면 하나의 파일에 담을 수도 있고 규모가 크면 여러 파일에 구현한 다음 하나의 폴더에 담을 수도 있다. 이때 각각의 `.py` 파일을 **모듈(Module)**이라 하고 여러 모듈을 묶은 폴더를 **패키지(Package)**라고 한다. 파이썬 오픈소스는 패키지 단위로 설치가 되기 때문에 패키지라는 용어를 자주 사용한다. 앞으로 배울 GUI나 영상처리도 모두 외부 패키지를 이용하여 구현하는 것이다.

## 2. 모듈 만들기

모듈이란 쉽게 말해 파이썬 파일(`.py`)이다. 어떤 파이썬 파일에 함수들을 구현하고 그 함수를 다른 파이썬 파일에서 가져다 써보자. 실습을 위해 `list_ops.py`라는 리스트 연산 모듈을 만들어보자.

```python
# list_ops.py
def add(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f + b)
    return out

def substitute(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f - b)
    return out

def multiply(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f * b)
    return out

def divide(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f / b)
    return out
```

그 옆에 `use_list_ops.py`라는 새로운 파일을 만들어 저 함수들을 사용해보자. `import`를 활용해 외부 모듈을 가져오는 방법은 다양하다. 

1) 모듈 이름 그대로 가져오는 방법

```python
foo = [1, 2, 3, 4, 5]
bar = [24, 52, 13, 27]

import list_ops

goo = list_ops.add(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))

goo = list_ops.multiply(foo, bar)
print("{} * {} = {}".format(foo, bar, goo))
```

2) 모듈 이름을 바꿔서 가져오는 방법

```python
import list_ops as lo

goo = lo.substitute(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
goo = lo.divide(bar, foo)
print("{} * {} = {}".format(bar, foo, goo))
```

3) 모듈에서 지정한 기능(함수)만 가져오는 방법

```python
from list_ops import add, substitute

goo = add(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
goo = substitute(bar, foo)
print("{} - {} = {}".format(bar, foo, goo))
```





## 3. 패키지 만들기

파일을 폴더 아래로 옮기고 폴더 아래 dict 연산 모듈을 추가해보자.

```python
# 두 개 dict add sub mul div
# set으로 교집합 찾아서 교집합만 연산
```





