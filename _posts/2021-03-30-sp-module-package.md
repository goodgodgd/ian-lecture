---
layout: post
title:  "[Python] Module and Package"
date:   2021-03-30 09:00:13
categories: SystemProgram
---



## 1. 모듈과 패키지는 무엇?

지금까지 우리는 하나의 파일에서만 프로그램을 실행해왔지만 실제로 개발을 할 때는 다수의 파이썬 파일을 만들어 유기적으로 연결해야 할 뿐만 아니라 남이 만든 코드도 적극적으로 활용해야 한다. 어떤 파이썬 파일에서 `import` 키워드를 통해 외부 파이썬 파일의 함수, 변수, 클래스를 가져다 쓸 수 있다. 외부 코드는 기능별로 함수나 클래스 등을 묶어서 파일에 저장하는데 코드 규모가 작으면 하나의 파일에 담을 수도 있고 규모가 크면 여러 파일에 구현한 다음 하나의 폴더에 담을 수도 있다. 이때 각각의 파이썬 파일을 **모듈(Module)**이라 하고 여러 모듈을 묶은 폴더를 **패키지(Package)**라고 한다. 파이썬 오픈소스는 패키지 단위로 설치가 되기 때문에 패키지라는 용어를 자주 사용한다. 앞으로 배울 GUI나 영상처리도 모두 외부 패키지를 이용하여 구현하는 것이다.



## 2. 모듈 만들기

모듈이란 쉽게 말해 파이썬 파일(`.py`)이다. 어떤 파이썬 파일에 함수와 변수를 만들고 그것들을 다른 파이썬 파일에서 가져다 써보자. 실습을 위해 프로젝트 폴더에 `package`라는 폴더를 만들고 그 아래 `list_ops.py`라는 리스트 연산 모듈을 만들어보자.

```python
# list_ops.py
def add(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f + b)
    return out

def subtract(foo, bar):
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

spam = [51, 23]
ham = [34, 67]
if __name__ == '__main__':
    eggs = add(spam, ham)
```

리스트 원소에 대한 사칙연산 함수 외에 `spam, ham, eggs`라는 변수를 만들었다. 함수에서 쓰인 `foo, bar`나 아래서 선언된 변수명들을 프로그래밍에서 [Metasyntactic variable](https://futurecreator.github.io/2018/06/05/metasyntactic-variables-foo-bar/) 이라고 한다. syntactic이라고 하지만 프로그래밍 언어의 문법과는 아무상관이 없고 그냥 예시에서 관습적으로 많이 쓰이는 변수명들을 말한다. `foo, bar`는 모든 프로그래밍 언어에서 일반적으로 사용되는 이름이고 `spam, ham, eggs`는 주로 파이썬에서 많이 쓰인다고 한다. 그냥 참고사항으로 알아두자.  

`list_ops.py` 옆에 `use_list_ops.py`라는 새로운 파일을 만들어 저 함수와 변수들을 사용해보자. `import`를 활용해 외부 모듈을 가져오는 방법은 다양하다. 

### 모듈 이름 그대로 가져오는 방법

```python
import list_ops

foo = [1, 2, 3, 4, 5]
bar = [24, 52, 13, 27]
goo = list_ops.add(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
# => [1, 2, 3, 4, 5] + [24, 52, 13, 27] = [25, 54, 16, 31]
print("list_ops.spam: {}".format(list_ops.spam))
# => list_ops.spam: [51, 23]
goo = list_ops.multiply(list_ops.spam, list_ops.ham)
print("{} * {} = {}".format(list_ops.spam, list_ops.ham, goo))
# => [51, 23] * [34, 67] = [1734, 1541]
```

가장 기본적인 방식이지만 가장 덜 쓰이는 방법이다. 모듈에서 어떤 객체를 가져올 때는 `모듈명.객체명`으로 가져오면 된다. 위 예제에서 더하기 함수와 곱하기 함수가 성공적으로 수행된 것을 확인할 수 있다. 여기서 봐야할 것은 `list_ops`에서 변수를 가져올 수 있다는 것이다. 얼핏 보면 당연한 것 같지만 C언어를 생각해보면 당연하지는 않다. `list_ops.spam`을 프린트 했을 때 `[51, 23]`이 나온다는 것은 `list_ops.py`라는 스크립트가 **실행**되었다는 것을 뜻하기 때문이다. `list_ops`를 import 만 했지만 내부적으로는 `list_ops.py`가 실행된 것이다. 그래서 모듈을 만들때 외부에서 사용할 변수가 아니라면 함수 밖에서 변수 선언이나 복잡한 연산을 하지 않는 것이 좋다.   

`list_ops.py`에 실행 가능한 코드를 넣되 외부에서 import 할 때는 실행되지 않게 하고 싶다면 함수에서 배웠듯이 `__name__` 변수를 써야한다. `list_ops.py`에서는 `eggs`라는 변수가 `if __name__ == '__main__':` 조건문 아래 있기 때문에 외부에서 import 할 때는 선언되지 않는다.

```python
try:
    print("list_ops.eggs: {}".format(list_ops.eggs))
except Exception as e:
    print(e)
    # => module 'list_ops' has no attribute 'eggs'
```

### 모듈 이름을 바꿔서 가져오는 방법

일반적으로 모듈 이름은 모듈의 기능을 이해하기 쉽게 몇 개의 단어를 이어서 길게 짓는것이 보통이다. 게다가 패키지에서 복잡한 폴더 구조 아래 있는 모듈을 가져올 때는 더욱 import 문이 길어진다. 하지만 모듈의 객체를 불러올 때마다 긴 모듈 이름을 모두 쓰는 것은 번거로우므로 짧은 별명을 지어 가져올 수 있다. `import A as B` 라고 하면 `A` 모듈을 `B`라는 이름으로 가져오겠다는 뜻이다. 아래 코드에서도 `list_ops` 대신 `lo`를 써서 모듈 객체를 불러옴을 볼 수 있다.

```python
import list_ops as lo

goo = lo.subtract(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
goo = lo.divide(bar, foo)
print("{} * {} = {}".format(bar, foo, goo))
```

### 모듈에서 지정한 객체만 가져오는 방법

아예 외부 모듈명을 쓰지 않고 바로 함수나 변수를 사용하고 싶을 때는 `from module import object`를 쓰면 된다. `from`을 써서 가져온 객체는 내부에서 정의한 것과 동일하게 사용가능하다.

```python
from list_ops import add, subtract, spam

goo = add(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
goo = subtract(bar, foo)
print("{} - {} = {}".format(bar, foo, goo))
print("spam = {}".format(spam))
```

## 3. 패키지 만들기

패키지는 여러 모듈이 폴더 단위로 모인 것으로 폴더 안에 폴더를 넣으면 계층적인 패키지를 만들 수도 있다. 여러 모듈을 모은 패키지를 만들기 위해 `package`란 폴더 아래 `dict_ops.py`를 다음과 같이 만들어보자. 함수를 만들 때 원래는 함수 이름을 `list_add`나 `dict_add`처럼 함수 이름에 기능이 모두 표현되는 것이 좋으나 앞에 붙은 접미사는 모듈명으로 대체 가능하기 때문에 함수 이름을 간단하게 지어도 된다.

```python
# dict_ops.py
def add(foo, bar):
    out = {}
    for key in foo:
        if key in bar:
            out[key] = foo[key] + bar[key]
    return out

def subtract(foo, bar):
    out = {}
    for key in foo:
        if key in bar:
            out[key] = foo[key] - bar[key]
    return out

def multiply(foo, bar):
    out = {}
    for key in foo:
        if key in bar:
            out[key] = foo[key] * bar[key]
    return out

def divide(foo, bar):
    out = {}
    for key in foo:
        if key in bar:
            out[key] = foo[key] / bar[key]
    return out
```

이제 프로젝트 폴더 바로 아래 새로운 파이썬 파일을 만들고 (`package` 폴더 옆에) 예제를 실행해보자. 여러 사람의 키와 몸무게를 입력하여 BMI를 계산하는 코드다. 첫 줄을 보면 `package` 라는 패키지 안의 `list_ops`라는 모듈을 `lo`라는 약칭으로 불러왔다. C언어에서 `#include`를 항상 코드 맨 위에서 하듯이 파이썬에서도 `import`는 그 모듈이 언제 쓰이던지 항상 맨 윗줄에 사용하는 것을 권장한다.

```python
import package.list_ops as lo
import package.dict_ops as do

weights = [65, 90, 42, 76]
heights = [1.65, 1.78, 1.59, 1.80]
heights_sq = lo.multiply(heights, heights)
bmi = lo.divide(weights, heights_sq)
print("BMI:", bmi)
# => BMI: [23.875114784205696, 28.40550435551067, 16.61326688026581, 23.456790123456788]
```

그런데 알고보니 키와 몸무게를 사람 순서대로 넣은 것이 아니라 임의로 섞여서 들어간 것이었다. 이를 정확히 처리하기 위해 리스트를 딕셔너리로 바꿔서 연산을 해보자. `w_names`는 `weights`에 해당하는 이름이고 `h_names`는 `heights`에 해당하는 이름이다. 이름의 순서와 구성이 다르다는 것을 알 수 있다. 같은 사람의 키와 몸무게를 이용해 BMI를 계산하기 위해 우선 `dict(zip(list1, list2))`로 두 개의 리스트를 딕셔너리로 만들고 `dict_ops`를 사용하였다.

```python
w_names = ["RM", "Suga", "Jin", "V"]
h_names = ["Jimin", "RM", "Suga", "Jin"]
weights = dict(zip(w_names, weights))
heights = dict(zip(h_names, heights))
print("dict weights:", weights)
print("dict heightss:", heights)
heights_sq = do.multiply(heights, heights)
bmi = do.divide(weights, heights_sq)
print("BMI:", bmi)
# => BMI: {'RM': 20.515086478979924, 'Suga': 35.59985760056959, 'Jin': 12.962962962962962}
```

`dict_ops`는 입력인자 `foo, bar`에 공통적으로 있는 key에 대해서만 연산을 하여 결과를 출력한다. 예시의 결과를 보면 키와 몸무게에서 이름이 겹치는 RM, Suga, Jin 세 명의 BMI만 나온 것을 확인할 수 있다.  

### 연습문제

1. 위에서 만든 `list_ops.py`와 `dict_ops.py` 함수들을 list comprehension, dict comprehension을 이용해 한줄로 구현한 `iter_ops`라는 패키지를 만들어 보시오.
2. `numeric`이란 패키지를 만들고 → 패키지에 `prime_number`라는 모듈을 만들고 → 모듈에 소수를 확인하는 `check_prime_number(numbers)` 함수를 구현하시오. 이 함수는 임의의 숫자 리스트를 입력으로 받아서 각 숫자가 소수인지 확인하여 bool의 리스트로 출력한다. (예시: [341, 12, 523, 59] => [False, False, True, True])

