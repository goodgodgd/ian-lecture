---
layout: post
title:  "[Python] Number and String in Python"
date:   2019-03-01 09:00:13
categories: 2019-1-systprog
---


# 파이썬 기본 데이터 타입

## 1. 파이썬 변수와 타입

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

이러한 특성이 C언어에 익숙한 사람들에게는 *위험한* 코드로 보일 수 있으나 깊이 사용하다보면 변수명을 아끼는데 큰 도움이 된다. 변수명이 줄어들면 변수명이 짧아지고 코드가 단순해 보인다.

```python
import numpy as np	# 수학적 행렬을 다루는 패키지
import pandas as pd	# 엑셀같은 표를 다루는 패키지
# 똑같은 행렬을 같은 변수명으로 list, numpy array, dataframe 여러가지로 바꿔가면서 사용한다.
myarray = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
# [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
myarray = np.array(myarray)
# array([[1, 2, 3],
#       [4, 5, 6],
#       [7, 8, 9]])
myarray = pd.DataFrame(myarray)
#    0  1  2
# 0  1  2  3
# 1  4  5  6
# 2  7  8  9
```

## 2. 숫자 타입

C언어를 배울 때 우리는 다양한 정수형과 실수형 데이터 타입을 배웠다. `short`, `int`, `unsigned int`, `float`, `double` 등... 각기 데이터의 형식과 표현 범위가 다른 다양한 타입들이 있었다. 표현 범위는 데이터가 차지하는 메모리 용량에 의해 결정됐는데 `int`, `float`은 4 byte, `double`은 8 byte 로 표현할 수 있는 숫자범위를 나타냈다.   

하지만 파이썬에서는 숫자를 나타내는 built-in 타입이 세 가지 밖에 없다. `int` (정수), `float` (실수), `complex`(복소수). 물론 그렇기 때문에 메모리를 효과적으로 쓸 순 없다. 게다가 숫자 자체를 객체로 인식하기 때문에 숫자 표현외에 추가적인 메모리가 더 들어간다. 데이터의 범위는? 파이썬 공식 문서를 보자.   

> There are three distinct numeric types: integers, floating point numbers, and complex numbers. In addition, Booleans are a subtype of integers. **Integers have unlimited precision. Floating point numbers are usually implemented using double in C**; information about the precision and internal representation of floating point numbers for the machine on which your program is running is available in **sys.float_info**. <https://docs.python.org/3/library/stdtypes.html#numeric-types-int-float-complex>

놀랍게도 정수형 `int`는 범위가 없다. 그 이유는 `int` 타입의 메모리 크기를 지정하지 않고 필요에 따라 늘릴수 있게 만들었기 때문이다. 하지만 `float`은 `sys.float_info`를 통해 확인할 수 있는데 c언어의 `double`과 같은 특성이 나온다.
```
sys.float_info(max=1.7976931348623157e+308, max_exp=1024, max_10_exp=308, min=2.2250738585072014e-308, min_exp=-1021, min_10_exp=-307, dig=15, mant_dig=53, epsilon=2.220446049250313e-16, radix=2, rounds=1)
```

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

그런데 파이썬에는 비교 연산자가 하나 더 있다. 바로 `is`다. 파이썬에서 두 객체가 '같음'을 비교하는 연산자는 `is`와 `==` 두 가지가 있다. 둘은 비슷해 보이지만 전혀 다른 연산자다. `is`는 동일한 객체, 즉 같은 주소(메모리)를 공유하는 객체인지를 확인하는 것이고, `==`는 객체의 값(value)이 같은지를 확인하는 것이다. 다음 예시를 통해 확인해보자.

```python
print("\nWhat is difference between `is` and `==`?")
print("little difference for basic types(int, float, bool, str)")
print("12 == 12:", 12 == 12)
print("12 is 12:", 12 is 12)
print("12 == True:", 12 == True)
print("12 is True:", 12 is True)

print("big difference for other types")
foo = [1]
bar = [1]
print("foo == bar", foo == bar)
print("foo is bar", foo is bar)
```

결과
```
What is difference between `is` and `==`?
little difference for basic types(int, float, str)
12 == 12: True
12 is 12: True
12 == True: False
12 is True: False
big difference for other types
foo == bar True
foo is bar False
```

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

### 문자열 연산

문자열에 무슨 '연산'? 생소할 수 있지만 파이썬에는 문자열에 작동하는 `연산자(operator)`가 존재한다. 문자열 연산자를 통해 I.O.I의 "너무너너무너무" 가사를 어떻게 쉽게 써보자.

```
print("날 " + ("너무" * 3 + " ")*5  + "좋아하면 그때 말해줘")
print("내가 " + ("자꾸" * 3 + " ")*5 + "떠오르면 그때 불러줘")
```

`+`를 쓰면 문자열을 이어붙일 수 있고 (concatenation) `*`를 쓰면 문자열을 반복할 수 있다.

### 문자열 인덱싱과 슬라이싱

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

### 문자열 포매팅(Formatting)

C언어에서 `printf("%03d", 10)` 하듯이 파이썬에서도 문자열에 데이터를 특정 형식으로 넣을 수 있다. 현재 Python 3.6에서는 세 가지 문자열 포매팅 방법이 존재한다.

#### a. 포맷 코드를 이용한 방법
C언어의 `printf` 와 비슷한 방식으로 포매팅을 한다. 이 방식이 가장 오래된 방식이지만 데이터 형식에 따라 포맷 코드를 지정해줘야 하는 불편함이 있다. C언어에서는 당연한 것이지만 Python에서는 동적 타입이기 때문에 변수를 특정 타입으로만 입력할 수 있다는 것은 잠재적 에러의 요인이 된다.  
포매팅 방법은 문자열에 포맷 코드를 넣고 문자열 옆에 `%`를 쓴 후 값이나 변수명을 쓰면 그 값이 포맷 코드로 들어간다. 포맷 코드는 C언어와 마찬가지로 문자열은 `%s`, 정수형은 `%d`, 실수형은 `%f`를 쓴다.

```python
print("\n" + "="*30)
print("String formatting 1: %")
print("class: %s" % "warrior")
print("HP: %d" % 100)
print("DPS: %f" % 1456.23)
```

다음과 같이 여러 값을 한번에 넣을 수도 있고 패턴 자체를 변수에 넣고 쓸수도 있다. 문자열의 세부적인 포매팅 (길이, 자리수)도 C언어와 비슷하게 형식을 지정할 수 있다.
```python
_class = "warrior"
HP = 100
DPS = 1456.23
# 문자열은 10칸 사용, 정수는 5칸 사용, 실수는 10칸에 소수점은 3자리까지 사용
pattern = "class: %10s, HP: %5d, DPS: %10.3f"
char_intro = pattern % (_class, HP, DPS)
print(char_intro)
print(pattern % ("healer", 200, 834.79))
```

결과
```
==============================
String formatting 1: %
class: warrior
HP: 100
DPS: 1456.230000
class:    warrior, HP:   100, DPS:   1456.230
class:     healer, HP:   200, DPS:    834.790
```

#### b. format 함수를 이용한 방법

위와 같은 문자열을 `str`타입에 내장된 `format()` 함수로 구현해보자. 문자열 내부에 `{}`(brace)이 있으면 이를 외부 데이터가 들어올 자리로 인식하고 `format()` 함수를 통해 데이터 값을 받는다. 이 방법의 장점은 어떤 데이터 타입이든 상관없이 brace에 자동으로 입력이 된다는 것이다. `{}` 안에는 `format(arg)` 함수의 인자가 따로 `print(arg)`되었을 때와 같은 문자열이 나온다.

```python
print("\n" + "="*30)
print("String formatting 2: {}.format()")
print("class: {}".format("warrior"))
print("HP: {}".format(100))
print("DPS: {}".format(1456.23))
```

마찬가지로, 여러개의 값을 한번에 입력할 수도 있다. 여러개의 값을 넣을 때는 왼쪽 brace부터 순서대로 들어간다. 정해진 칸 안에 문자열을 왼쪽(<), 오른쪽(>), 가운데(^)로 정렬할 수 있다. 정렬 방법은 `:` 뒤에 정렬기호를 입력하면 된다.
```python
_class = "warrior"
HP = 100
DPS = 1456.23
# 모두 왼쪽 정렬, 문자열은 10칸, 정수는 5칸, 실수는 10칸에 소수점은 3자리까지 사용
pattern = "class: {:<10}, HP: {:<5}, DPS: {:<10.3f}"
char_intro = pattern.format(_class, HP, DPS)
print(char_intro)
print(pattern.format("healer", 200, 834.79))
```

교재 <https://wikidocs.net/13#format>를 보면 `format()` 함수의 인자가 들어가는 순서를 명시적으로 지정하는 방법도이 나와있다.  
순서 지정:  `"{0} {1}".format(1, 2)`  
별명 지정:  `"{first} {second}".format(first=1, second=2)`  
혼합 지정:  `"{} {second}".format(1, second=2)`  
이러한 코드는 결과를 더 명시적으로 예측할 수 있게 해준다. 다만 손이 더 가므로 본문의 예시와 같이 입력 순서를 이용해도 무방하다.

#### c. f문자열 포매팅

포맷 코드보다 `format()` 함수가 쓰기는 더 쉽지만 코드가 약간 더 옆으로 길어지는 단점도 있다. Python 3.6에서부터는 f문자열 포매팅을 지원하여 더 간결하게 구현할 수 있다. 방법은 `f'{var}'` 처럼 문자열 앞에 f를 붙이고 braket 사이에 변수명이나 값을 넣는 것이다. 대신 f문자열 포매팅을 이용할 경우 앞선 방법처럼 문자열 패턴을 변수에 넣고 반복 사용하기는 어렵다. 예시를 보자.

```python
print("\n" + "="*30)
_class = "warrior"
HP = 100
DPS = 1456.23
# 모두 왼쪽 정렬, 문자열은 10칸, 정수는 5칸, 실수는 10칸에 소수점은 3자리까지 사용
char_intro = f"class: {_class:<10}, HP: {HP:<5}, DPS: {DPS:<10.3f}"
print(char_intro)
char_intro = f"class: {'healer':<10}, HP: {200:<5}, DPS: {834.79:<10.3f}"
print(char_intro)
```
결과
```
==============================
class: warrior   , HP: 100  , DPS: 1456.230  
class: healer    , HP: 200  , DPS: 834.790   
```

### 문자열 함수

파이썬은 built-in 타입이라도 클래스 객체로 구현이 되어있고 특히 `str` 클래스는 유용한 내장 함수들이 많다. 더 신기한 것은 문자열 변수를 만들지 않아도 문자열 자체에서 함수를 실행할 수 있다는 것이다. 특정 문자열의 포함 횟수를 세는 `count()` 함수를 통해 문자열 함수를 쓰는 방법을 알아보자.
```python
print("\n" + "="*30)
print("str class functions")
# 문자열 함수를 이용하는 두 가지 방법
# 1. 문자열 자체에서 사용, 2. 문자열 변수에서 사용
print("count substring")
print("count '너무':", "날 너무너무너무".count('너무'))
text = "날 " + ("너무" * 3 + " ")*5 + "좋아하면 그때 말해줘"
print("count '너무':", text.count('너무'))
```

문자열에는 이 외에도 다양한 함수들이 있다. 무엇이 있는지 예시를 통해 익혀두고 필요할 때 찾아쓰도록 하자.
- `find, index`: 전체 문자열 내부에 입력 받은 문자열이 어디에 있는지 앞에서부터 찾아서 인덱스를 반환한다. `find`와 `index`의 차이는 입력 문자열을 못 찾았을 때의 처리 방법이다. `find`는 -1을 반환하고 `index`는 에러를 낸다. 문자열을 못 찾아도 큰 문제가 없을 때는 `find`를 쓰고 꼭 찾아야 하는 경우엔 `index`를 써서 이후에 오류가 생기지 않도록 하는게 좋다.

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

1) 자신이 좋아하는 후크송 가사를 문자열 연산을 통해 만들어 보세오.
2) 가사에서 반복되는 단어가 몇 번 나오는지, 첫 번째로 나오는 위치는 어디인지 찾아보세요.
3) 가사에서 반복되는 단어를 다른 단어로 바꾸세오.

