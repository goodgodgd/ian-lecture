---
layout: post
title:  "Python Basics"
date:   2020-08-30 09:00:13
categories: 2020-2-robotics
---



# 파이썬(Python) 소개




## 파이썬(Python) 유래

 > 파이썬(Python)은 1990년 암스테르담의 귀도 반 로섬(Guido Van Rossum)이 개발한 인터프리터 언어이다. 귀도는 파이썬이라는 이름을 자신이 좋아하는 코미디 쇼인 "몬티 파이썬의 날아다니는 서커스(Monty Python’s Flying Circus)"에서 따왔다고 한다. ([영상](https://www.youtube.com/watch?v=C-M2hs3sXGo&index=1&list=PLutTviYZk9KhOPSfWfBMU4-VNRCs6S4Jr)) 파이썬의 사전적인 의미는 고대 신화에 나오는 파르나소스 산의 동굴에 살던 큰 뱀을 뜻하며, 아폴로 신이 델파이에서 파이썬을 퇴치했다는 이야기가 전해지고 있다. 대부분의 파이썬 책 표지와 아이콘이 뱀 모양으로 그려져 있는 이유가 여기에 있다. <https://wikidocs.net/4307#fn:interpret>



## Python 2 vs 3

현재 쓰이는 파이썬은 2000년에 발표된 Python2와 2008년 발표된 Python3가 있다. 둘은 기본적으로 코드 호환이 되지 않는다. 가장 큰 차이는 Python3에서부터 유니코드(unicode)를 기본 문자로 지원하여 다국어(한국어 등)를 코드에서 자연스럽게 쓸 수 있다. (한글 변수명도 만들 수 있다.) 2010년대 중반까지만 해도 Python2의 영향력이 컸지만 어대3이라 이제 대부분의 프로젝트가 Python3로 넘어왔다. Python2는 올해까지만 관리가 된다. <https://pythonclock.org/> 그래서 현재는 Python2를 배울 필요가 없고 (사실 큰 차이도 없지만) 앞으로 파이썬은 Python3 라고 봐도 무방하다.



## 파이썬 설치

로봇공학 수업에서는 리눅스(Linux)에서의 개발을 목표로 하지만 파이썬 문법을 배우는 단계에서는 일단 윈도우(Windows)에서 진행한다. 학생들에게 윈도우가 익숙하기도 하고 리눅스를 사용하기 위해서는 키트 배포와 설치에 시간이 걸리기 때문에 파이썬을 윈도우에서 먼저 배운다. 파이썬은 윈도우에서도 리눅스와 동일하게 동작하고 윈도우에서도 파이썬은 유용하기 때문에 학생들에게 도움이 될 것이다.  

파이썬 인터프리터와 개발환경인 PyCharm(파이참)을 다운로드 받아 설치하자.  

<https://www.python.org/downloads/release/python-379/>   

- "Windows x86-64 executable installer" 선택  

<https://www.jetbrains.com/ko-kr/pycharm/download/#section=windows>  

- "Community" 선택

C++이나 C#으로 개발할 때 Visual Studio를 쓰듯이 파이썬을 개발할 때는 파이썬에 최적화된 PyCharm이라는 IDE(통합개발환경)를 많이 쓴다. 물론 Visual Studio에서도 파이썬 개발이 가능하지만 PyCharm이 훨씬 설치에 부담이 적고 반응도 빠르다. 또한 리눅스에서도 설치하여 동일한 개발환경을 유지할 수 있다. 



## 파이썬의 특징

### 1. 문법이 간결하고 쉽다.

기존 프로그래밍 언어를 익숙하게 사용하는 사람이라면 몇일 안에 기본문법을 다 배울 수 있다. 파이썬은 단락을 구분하는 기호(`{}, end`)나 줄의 끝을 표현하는 기호(`;`)가 필요 없어서 간결하고 실수를 줄일 수 있다. 파이썬은 오직 들여쓰기만으로 단락을 구분한다.

```python
if a == 1:
    print(1)
for a in range(10):
    print(a)
```

### 2. 인간의 언어와 비슷하다.

기존 언어에서 기호로 표현했던 operator들을 단어로 대체하여 파이썬 문법을 모르는 사람도 문장을 읽듯이 해석이 가능하다. 파이썬을 '실행가능한 의사코드 (executable pseudo code)'라고 부르는 이유다.
```python
mylist = ['abc', 'bc', 'cde']
# if 'abc' is in mylist
if 'abc' in mylist:
    print("abc is in mylist")
# iterate values in mylist as v
for v in mylist:
    # if v starts with 'a' and length of v is > 1
    if v.startswith('a') and len(v) > 1:
        print(v)
```

### 3. 인터프리터 언어다.

C/C++ 은 실행파일을 생성하기 위해 빌드(build)를 해야한다. 빌드란 프로그래밍 언어를 기계어로 번역하는 컴파일(compile) 과정과 이를 실행파일로 만드는 과정(좁은 의미의 빌드)이 합쳐진 것이다. 이렇게 프로그램 전체를 컴파일 한 후 실행 가능한 언어를 **컴파일 언어**(compile language)라고 한다. C/C++, C#, Java 등이 이에 해당한다.  
반대로 **인터프리터 언어**란 인터프리터(해석기)에 의해 소스코드 한 줄 한 줄을 즉시 실행할 수 있는 언어다. 컴파일 언어는 독립적으로 실행 가능한 실행 파일을 만들지만 인터프리터 언어는 인터프리터가 설치되어야 실행 가능하다. 대신 인터프리터만 있다면 컴파일 과정 없이 바로 실행 가능하기 때문에 한 줄씩 짜면서 편리하게 결과를 확인할 수 있다. Python, MATLAB, JavaScript, ruby 등 대부분의 스크립트 언어가 이에 해당한다.   
인터프리터 언어는 대부분 플랫폼 독립적이다. 인터프리터가 설치될 수만 있다면 운영체제(Windows, Linux, MacOS)에 상관 없이 같은 코드로 동일한 동작을 할 수 있다. (C++도 Qt를 사용하면 플랫폼 독립적으로 사용가능하나 C++ 자체의 기능이 아닌 Qt 라이브러리의 도움이 필요하다.) 파이썬은 현재 Windows, Linux, MacOS 등 주로 PC용 운영체제를 지원하며 Android는 공식적으로 지원하고 있지 않다.  

### 4. 동적 타입을 지원한다.

파이썬은 변수에 타입을 지정하지 않는다. 데이터 타입이 있긴 있으나 각 변수는 언제든 자신의 타입을 바꿀 수 있다. 이것은 프로그램에 대단히 높은 자유도를 주지만 대신 프로그램의 오류를 찾기 힘들게도 한다. 정적 타입은 꼭 필요한 메모리만 사용하고 속도도 빠르지만 타입명을 쓰는게 귀찮고 타입에 의한 구현상 제약이 많다. 동적 타입은 메모리도 더 쓰고 속도도 느리지만 코드가 간결해지고 구현상 더 다양한 시도를 할 수 있다.  
(정적 타입 언어도 클래스를 잘 쓰면 동적 타입처럼 사용 가능은 하지만 더 많은 노오력이 필요하다. 어차피 파이썬 인터프리터도 C 언어 기반으로 만들어졌다.)

### 5. 오픈 소스(open-source)다.

(모든 오픈 소스가 무료는 아니지만) 파이썬은 **무료** ~~(공짜, free)~~다. 파이썬의 장점은 간결하고 쉬운 언어 자체의 특성도 있지만 오픈 소스의 특성상 사용자가 많아질수록 사용가능한 라이브러리(파이썬에서는 패키지 package라고 부른다.)가 풍부해지고 그럴수록 사용자가 더 많아지고 라이브러리는 더 풍부해지는 선순환이 정착되어 있다.  
게다가 이 과정을 극도로 편리하게 해주는 `PyPI`라는 저장소가 있다. 사용자 누구나 이곳에 자신이 만든 패키지를 업로드할 수 있고 남이 만든 패키도 `pip install [package name]` 이 한 줄의 명령어로 설치할 수 있다. 마치 안드로이드 앱을 플레이스토어에서 쉽게 받을 수 있는 것과 같다. 2019년 3월 2일 현재 170,382개의 프로젝트가 이곳에 올라와 있다. <https://pypi.org/>  
이러한 시스템이 파이썬에만 있는 것은 아니지만 (비슷한 예로 Ruby의 gem이 있다. <https://rubygems.org/>) 이런 쉬운 공유와 배포로 인해 사용자가 많아지고 발전이 가속되어 모든 것을 할 수 있는 만능 언어가 되었고 특히 Machine Learning이나 데이터 분석 같은 분야에서는 독보적인 위치를 가지고 있다.



# 기본 데이터 타입



## 1. 변수와 타입

파이썬 소개에서도 말했다시피 파이썬 변수는 동적 타입이다. 그러므로 변수에 값을 넣으면 알아서 타입이 그에 맞춰 변한다.

```python
a = "hello"
print("type:", type(a), "value:", a)
# > type: <class 'str'> value: hello

a = 1234
print("type:", type(a), "value:", a)
# > type: <class 'int'> value: 1234

a = 1.234
print("type:", type(a), "value:", a)
# > type: <class 'float'> value: 1.234

a = True	# False도 있는데 첫 문자는 대문자로 써야 한다.
print("type:", type(a), "value:", a)
# > type: <class 'bool'> value: True
```



## 2. 숫자 타입

C언어를 배울 때 우리는 다양한 정수형과 실수형 데이터 타입을 배웠다. `short`, `int`, `unsigned int`, `float`, `double` 등... 각기 데이터의 형식과 표현 범위가 다른 다양한 타입들이 있었다. 표현 범위는 데이터가 차지하는 메모리 용량에 의해 결정됐는데 `int`, `float`은 4 byte, `double`은 8 byte 로 표현할 수 있는 숫자범위를 나타냈다.   

하지만 파이썬에서는 숫자를 나타내는 built-in 타입이 세 가지 밖에 없다. `int` (정수), `float` (실수), `complex`(복소수). 물론 그렇기 때문에 메모리를 효과적으로 쓸 순 없다. 데이터의 범위는? 파이썬 [공식 문서](<https://docs.python.org/3/library/stdtypes.html#numeric-types-int-float-complex>)를 보면 놀랍게도 정수형 `int`는 범위가 무한하다. 그 이유는 `int` 타입의 메모리 크기를 지정하지 않고 필요에 따라 늘릴수 있게 만들었기 때문이다. 하지만 `float`은 `sys.float_info`를 통해 확인할 수 있는데 c언어의 `double`과 같은 범위를 가진다.



### 숫자 연산자

파이썬에서 숫자에 대한 기본 `operator`들은 다음과 같다. 여기서부터는 스크립트 파일을 만들어 실행해보자.

```python
import sys
a = 135.68
b = 15
print("float", type(a), sys.getsizeof(a))
print("integer", type(b), sys.getsizeof(b))

# 기본적인 사칙연산
print(a + b)
print(a - b)
print(a * b)
print(a / b)

# 거듭제곱
print(b ** 2)
# 나누기 후 나머지
print(a % b)
# 나누기 후 몫
print(a // b)
```

결과

```
float <class 'float'> 24
integer <class 'int'> 28
150.68
120.68
2035.2
9.045333333333334
225
0.6800000000000068
9.
```

파이썬에서도 C 언어처럼 bit-wise operator들이 있다. `(&, |, ~, ^, >>, <<)`  그러나 경험상 파이썬에서 비트연산을 쓸 일이 별로 없기 때문에 필요한 경우에 찾아서 쓰면 된다. <https://wikidocs.net/20704>

### 연습문제

1) 위 연산을 이용해서 13의 3승을 16진수로 표현하시오.



## 3. 조건문(if)과 비교 연산자

파이썬에서도 `>, <, <=, >=, ==, !=` 연산자를 통해 숫자의 크기를 비교할 수 있다. 비교 연산자를 배우는 김에 `if`문까지 배워보자. 파이썬에서 `if`는 다음과 같이 쓴다.

```python
if cond1:
    statement1
    statement2
elif cond2:
    statement3
    statement4
else:
    statement5
```

C언어의 if문과 비교해보면, 

1. 조건문(if) 아래의 코드 블럭에 괄호가 없고 오직 들여쓰기로만 블럭을 구분한다. 
2. 들여쓰기는 일반적으로 탭(Tab)이 아닌 공백 4칸을 사용한다. 탭은 에디터마다 길이가 다르므로 일정한 공백을 사용하는 것이 좋다. 탭과 공백을 섞어쓰면 에러가 난다. 하지만! PyCharm이나 VScode 같은 파이썬 전문 에디터에서는 탭을 알아서 공백 4칸으로 만들어준다.
3. 새로운 블럭은 항상 `:`(colon) 기호 아래 들여쓰기 후 시작한다. 
4. 첫 번째 조건문에는 `if`를 쓰고 두 번째 조건이 있을 시 ~~`else if`가 아닌~~ `elif`를 쓴다. 나머지 조건은 `else`로 처리한다.  

이러한 규칙에 따라 조건문을 작성해보자.

```python
print("if statements")
if 13 ** 3 > 50 **2:
    print("13**2 > 50**2")
elif 13 ** 3 != 2197:
    print("13**3 != 2197")
elif 13 ** 3 >= 30 **2:
    print("13**3 >= 30**2")
else:
    print("13**3 < 30**2")
```



### is, and, or, in 연산자

파이썬에는 `is, and, or, in` 처럼 단어로된 연산자들이 있다. 부호 연산자를 쓰는 것 보다 단어 연산자를 쓰면 코드가 인간의 언어에 가까워져서 자연스럽게 읽고 이해할 수 있다.  

`is`는 `==`와 헷갈리기 쉽지만 전혀 다른 연산자다. `==` 앞 뒤의 두 값(value)이 같음을 비교한다면 `is`는 앞 뒤의 두 객체가 동일한 객체인지를 비교한다. 동일한 객체라는 뜻은 메모리를 공유하는, C언어로 치면 포인터 값이 같은 변수라는 뜻이다. 상수와의 비교에서는 `==` 연산자와 비슷하게 기능한다.

```python
print("\nWhat is difference between `is` and `==`?")
print("little difference for built-in types(int, float, str)")
intvar = 12
print("intvar == 12:", intvar == 12)
print("intvar is 12:", intvar is 12)
boolvar = True
print("boolvar == True:", boolvar == True)
print("boolvar is True:", boolvar is True)

print("big difference for object types")
foo = [1, 2]
bar = [1, 2]
print("foo == bar:", foo == bar)
print("foo is bar:", foo is bar)
goo = bar
goo[0] = 0
print("bar, goo:", bar, goo)
print("foo == goo", foo == goo)
print("bar is goo", bar is goo)
```

결과

```
What is difference between `is` and `==`?
little difference for built-in types(int, float, str)
intvar == 12: True
intvar is 12: True
boolvar == True: True
boolvar is True: True
big difference for object types
foo == bar True
foo is bar False
bar, goo: [0, 2] [0, 2]
foo == goo False
bar is goo True
```

`and, or`는 각각 C언어의 `&&, ||`에 해당한다. 파이썬에는 비트 연산자인 `&, |`는 있으나 논리 연산자로는 `&&, ||`이 아닌 `and, or`를 사용한다.

```python
value = 10
if (value % 2 == 0) and (value < 12):
    print("value is even and less than 12")
if (value % 2 != 0) or (value < 0):
    print("value is odd or negative")
```

`in` 연산자는 어떤 객체가 문자열이나 리스트, 딕셔너리 등의 배열형 객체에 원소로서 포함되어 있는지를 확인하는 연산자인데 앞으로 배열형 객체를 배우면서 `in`의 용법에 대해서도 배울 것이다.



## 4. 문자열 타입

파이썬에선 문자열을 정말 편리하게 다룰수 있다. 파이썬의 많은 장점 중 하나이다.  문자열은 간단하게 `" "`이나 `' '`사이에 글자를 쓰면 된다.

```python
string1 = "Life is too short"
string2 = 'You need python'
print(type(string1), type(string1))
# 문자열 안에 따옴표(', ") 입력
print("Life 'is' too short")
print('You "need" python')
# 특수문자 입력
print("Life \"is\" too short,\nYou \'need\' python")
# 한글도 잘 나온다.
print("안녕? 파이썬")
```

결과

```
<class 'str'> <class 'str'>
Life 'is' too short
You "need" python
Life "is" too short,
You 'need' python
안녕? 파이썬
```

다음은 특수문자 기호이다.

| symbol | meaning          |
| ------ | ---------------- |
| \n     | line break       |
| \t     | tab              |
| \\\\   | back slash(\\)   |
| \\'    | quote(\')        |
| \\"    | double quote(\") |



## 5. 문자열(str) 사용법

파이썬의 문자열(str) 클래스는 매우 편리한 기능들이 많다. 프로그래밍에서 문자열은 자주 사용되기 때문에 다양항 활용법을 알아두면 유용하게 쓸 수 있다.

### 문자열 연산

문자열에 무슨 '연산'? 생소할 수 있지만 파이썬에는 문자열에 작동하는 `연산자(operator)`가 존재한다. 문자열 사이에 `+`를 쓰면 문자열을 이어붙일 수 있고 (concatenation) `*`를 쓰면 문자열을 반복할 수 있다. (repetition) 문자열 연산자를 통해 프로듀스 101의 "Pick Me" 가사를 어떻게 쉽게 써보자.

> 가사 원본  
>
> can you feel me 나를 느껴 봐요  
> can you touch me 나를 붙잡아 줘  
> can you hold me 나를 꼭 안아 줘  
> I want you pick me up  
> pick me pick me pick me up  
> pick me pick me pick me up  
> pick me pick me pick me pick me  
> pick me pick me pick me up  
> pick me pick me pick me up  
> pick me pick me pick me up  
> pick me pick me pick me pick me  
> I want you pick me up  

```python
print("can you feel me 나를 느껴 봐요 \ncan you touch me 나를 붙잡아 줘")
print("can you hold me 나를 꼭 안아 줘 \nI want you pick me up")
print(("pick me "*3 + "up ") * 2)
print("pick me "*4)
print(("pick me "*3 + "up ") * 3)
print("pick me "*4 + "I want you pick me up")
```



### 인덱싱과 슬라이싱 (Indexing and Slicing)

문자열을 배열로보고 특정 인덱스(index)의 문자를 출력하는 것을 인덱싱(indexing), 특정 인덱스 범위의 문자열을 추출하는 것을 슬라이싱(slicing)이라 한다. 슬라이싱은 `시작 인덱스:끝 인덱스`로 인덱스 범위를 지정하면 인덱스 범위 `시작 인덱스 <= i < 끝 인덱스` 범위의 문자열을 가져온다. 예를 들어 `string[2:5]`이면 string이란 변수에서 2~4번째 문자들을 가져오는 것이다. 시작 인덱스가 생략되어 있으면 처음부터 가져오는 것이고 끝 인덱스가 생략되어 있으면 끝까지 가져오는 것이다. 인덱스를 뒤에서부터 셀 경우 음수로 표현한다.

```python
text = "Life is too short, You need Python"
# Life is too short, You need Python
# 0         1         2         3 
# 0123456789012345678901234567890123
print("index t:", text[8], text[16], text[30], text[-4])
print("slice 'Life':", text[:4])
print("slice 'short':", text[12:17])
print("slice 'Python':", text[28:])
print("slice 'need'", text[23:-7])
```

### 연습문제

1) `text`를 슬라이싱과 문자열 연산을 통해 `"Python is too short, You need Life"` 으로 바꿔보시오.



### 문자열 포매팅(Formatting)

C언어에서 `printf("%03d", 10)` 하듯이 파이썬에서도 문자열에 데이터를 특정 형식으로 넣을 수 있다. Python 3.6 이상 버전에서는 세 가지 문자열 포매팅 방법이 존재한다.

#### a. 포맷 코드를 이용한 방법

C언어의 `printf` 와 비슷한 방식으로 포매팅을 한다. 이 방식이 가장 오래된 방식이지만 데이터 형식에 따라 포맷 코드를 지정해줘야 하는 불편함이 있다. C언어에서는 당연한 것이지만 Python에서는 동적 타입이기 때문에 변수를 특정 타입으로만 입력할 수 있다는 것은 잠재적 에러의 요인이 된다.  
포매팅 방법은 문자열에 포맷 코드를 넣고 문자열 옆에 `%`를 쓴 후 값이나 변수명을 쓰면 그 값이 포맷 코드로 들어간다. 포맷 코드는 C언어와 마찬가지로 문자열은 `%s`, 정수형은 `%d`, 실수형은 `%f`를 쓴다.

```python
_class = "warrior"
print("\n" + "="*30, "\nString formatting 1: %")
print("class: %s, HP: %d, DPS: %f" % (_class, 100, 1456.23))
```

결과

```
============================== 
String formatting 1: %
class: warrior, HP: 100, DPS: 1456.230000
```

포맷 코드를 이용한 방식은 문자열에 들어올 데이터 타입에 맞는 포맷 코드를 입력해줘야하기 때문에 까다롭고 번거롭다. 그래서 사실 포맷 코드는 Python 2에서 쓰던 방식이고 요즘은 잘 쓰이지 않는다.

#### b. format 함수를 이용한 방법

Python 3에는  `str`타입에 내장된 `format()` 함수가 있다. 문자열 내부에 `{}`(brace)가 있으면 이를 외부 데이터가 들어올 자리로 인식하고 `format()` 함수를 통해 데이터 값을 받는다. 이 방법의 장점은 어떤 데이터 타입이든 상관없이 brace에 자동으로 입력이 된다는 것이다. `{}` 안에는 `format(arg)` 함수의 인자가 따로 `print(arg)`되었을 때와 같은 문자열이 들어간다.

```python
print("\n" + "="*30, "\nString formatting 2: {}.format()")
print("class: {}, HP: {}, DPS: {}".format(_class, 100, 1456.23))
```

#### c. f문자열 포매팅

포맷 코드보다 `format()` 함수가 쓰기는 더 쉽지만 `{}` 칸이 많아지면 몇 번째 인자인지 헷갈릴 수 있다. (format 함수에 숫자나 별명으로 순서를 명시적으로 지정하는 방법도 있다.) Python 3.6에서부터는 f문자열 포매팅을 지원하여 더 직관적으로 문자열에 데이터를 삽입하는 방법이 있다. `f"{var}"` 처럼 문자열 앞에 f를 붙이고 brace `{}`사이에 변수명이나 값을 넣는 것이다.

```python
print("\n" + "="*30, "\nString formatting 3: 'f' formatting")
print(f"class: {_class}, HP: {100}, DPS: {1459.23}")
```



### 문자열 함수

파이썬은 built-in 타입이라도 클래스 객체로 구현이 되어있고 특히 `str` 클래스는 유용한 내장 함수들이 많다. 더 신기한 것은 문자열 변수를 만들지 않아도 문자열 자체에서 함수를 실행할 수 있다는 것이다. 특정 문자열의 포함 횟수를 세는 `count()` 함수를 통해 문자열 함수를 쓰는 방법을 알아보자.

```python
print("\n" + "="*30)
print("str class functions")
# 문자열 함수를 이용하는 두 가지 방법
# 1. 문자열 자체에서 사용, 2. 문자열 변수에서 사용
print("count substring 'pick me':")
print("pick me pick me pick me up".count('pick me'))
text = "pick me pick me pick me up"
print(text.count('pick me'))
```

문자열에는 이 외에도 다양한 함수들이 있다.

- `find, index`: 전체 문자열 내부에 입력 받은 문자열이 어디에 있는지 앞에서부터 찾아서 인덱스를 반환한다. `find`와 `index`의 차이는 입력 문자열을 못 찾았을 때의 처리 방법이다. `find`는 -1을 반환하고 `index`는 에러를 낸다. 어떤것을 쓰느냐에 따라 예외처리를 다르게 해줘야한다.

```python
print("\nfind substring")
text = "For the python, of the python, by the python"
# 문자열 위치 찾기 (find)
pyind = text.find("py")
print(f"'py' found at {pyind} in `{text}`")
pyind = text.find("py", pyind+1)
print(f"'py' found at {pyind} in `{text}`")
pyind = text.find("py", pyind+1)
print(f"'py' found at {pyind} in `{text}`")
pyind = text.find("ruby")
print(f"'ruby' found at {pyind} in `{text}`")

pyind = text.index("py")
print(f"'py' indexed at {pyind} in `{text}`")
pyind = text.index("py", pyind+1)
print(f"'py' indexed at {pyind} in `{text}`")
pyind = text.index("py", pyind+1)
print(f"'py' indexed at {pyind} in `{text}`")
try:
    pyind = text.index("ruby")
    print(f"'ruby' found at {pyind} in `{text}`")
except ValueError as ve:
    print("'ruby' not indexed, value error:", ve)
```

- `upper, lower`: 대소문자 변경
  문자열들을 쓰다보면 문자열끼리 비교할 경우가 자주 생긴다. 문자열도 `==`을 통해 비교하면 되는데 대소문자가 다르면 다른 문자로 인식한다. 대소문자를 무시하고 비교하고 싶은 경우엔 모두 대문자나 소문자로 통일한 후 비교하면 된다.

```python
print("\nuppper and lower case")
mixed = "PYthon"
small = "python"
print(f"compare {mixed} and {small}")
print(f"{mixed} == {small}:", mixed == small)
print(f"{mixed}.lower() == {small}:", mixed.lower() == small)
print(f"{mixed}.upper() == {small}.upper():", mixed.upper() == small.upper())
print(f"{mixed}.lower() is {small}.lower():", mixed.lower() is small.lower())
```

- `strip, lstrip, rstrip`: 공백이나 불필요한 시작/끝 문자 지우기
  문자열을 다루다 보면 문자열에 불필요한 기호나 공백들이 앞이나 끝에 붙어있을 때가 있다. 대표적으로 공백이나 `\n, ", :`등의 특수문자들이 있다. 이들을 제거해주는 함수가 `strip`이다. 입력인자를 넣지 않으면 자동으로 공백을 없애고 입력 문자열을 넣으면 그 문자열을 삭제한다. 문자열 앞쪽(왼쪽)만 지우고 싶을 때는 `lstrip`을 쓰고 문자열 뒤쪽만 지우고 싶을 때는 `rstrip`을 쓴다.

```python
print("\nstrip string")
wise_saying = ' "Walking on water and developing software ' \
              'from a specification are easy if both are frozen..." '
print(wise_saying)
wise_saying = wise_saying.strip()
print(wise_saying)
wise_saying = wise_saying.strip('\"')
print(wise_saying)
wise_saying = wise_saying.rstrip('.')
print(wise_saying)
```

- `replace`: 문자열 바꾸기
  말 그대로 문자열 안의 특정 문자열을 다른 문자열로 교체하는 함수다. 교체할 문자열을 먼저 쓰고 삽입할 문자열을 뒤에 쓰면 된다. `strip`은 문자열 양 끝에 있는 것만 지울수 있어서 중간에 있는 것을 지울 때는 `replace`를 쓴다.

```python
print("\nreplace string")
Lincoln_said = "for the people, by the people, of the people"
We_say = Lincoln_said.replace("people", "python")
Simply_say = We_say.replace("the ", "")
print("Lincoln said:", Lincoln_said)
print("We say:", We_say)
print("Simply say:", Simply_say)
```

- `split`: 문자열 나누기
  문자열을 특정 문자열을 기준으로 나눠서 문자열의 `list`로 출력하는 함수다.

```python
print("\nsplit string into list of substrings")
print("split by words:", We_say.split(" "))
print("split by phrase:", We_say.split(","))
```



### 연습문제

1) "대리암"의 가사를 문자열 연산을 통해 만들어 보세오.  

```python
marble = \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "그간 많은 stress 견뎌내며 비로소 대리암이 되었다네\n" \
    "모든 게 완벽했던 그 어느 날 난 너를 만나게 된 거야\n" \
    "모든 게 완벽했던 그 어느 날 난 너를 만나게 된 거야\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "나를 보고 웃기라도 하는 날엔 하루 종일 아무것도 할 수 없네\n" \
    "그 눈으로 날 똑바로 바라보면 나는 녹아버릴 거야\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "이것이 염산반응이다\n" \
    "이것이 염산반응이다\n" \
    "Hcl이다 CaCO3다\n" \
    "2Hcl + CaCO3 -> CaCl2 +CO2 + H2O다.\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "나는 대리암 염산과 반응하면 이산화탄소를 내며 녹는 대리암\n" \
    "나는 대리암 나는 대리암"
```

2) 가사에서 "대리암"이 몇 번 나오는지, 세 번째로 나오는 위치는 어디인지 찾아보세요.  
3) 가사에서 "대리암"을 "현무암"으로 바꿔보세오.  

