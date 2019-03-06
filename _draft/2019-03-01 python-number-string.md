---
layout: post
title:  "Number and String in Python"
date:   2019-03-01 09:00:13
categories: 2019-1-systprog
---

## 파이썬 변수와 타입

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

C언어를 배울 때 우리는 다양한 정수형과 실수형 데이터 타입을 배웠다. `short`, `int`, `unsigned int`, `float`, `double` 등... 각기 데이터의 형식과 표현 범위가 다른 다양한 타입들이 있었다. 표현 범위는 데이터가 차지하는 메모리 용량에 의해 결정됐는데 `int`, `float`은 4 byte, `double`은 8 byte 로 표현할 수 있는 숫자범위를 나타냈다.   

하지만 파이썬에서는 숫자를 나타내는 built-in 타입이 세 가지 밖에 없다. `int` (정수), `float` (실수), `complex`(복소수). 데이터의 범위는? 파이썬 공식 문서를 보자.   

> There are three distinct numeric types: integers, floating point numbers, and complex numbers. In addition, Booleans are a subtype of integers. **Integers have unlimited precision. Floating point numbers are usually implemented using double in C**; information about the precision and internal representation of floating point numbers for the machine on which your program is running is available in **sys.float_info**. <https://docs.python.org/3/library/stdtypes.html#numeric-types-int-float-complex>

놀랍게도 정수형 `int`는 범위가 없다. 그 이유는 `int` 타입의 메모리 크기를 지정하지 않고 필요에 따라 늘릴수 있게 만들었기 때문이다. 하지만 `double`은 `sys.float_info`를 통해 확인할 수 있는데 c언어의 double과 같은 특성이 나온다.
```
sys.float_info(max=1.7976931348623157e+308, max_exp=1024, max_10_exp=308, min=2.2250738585072014e-308, min_exp=-1021, min_10_exp=-307, dig=15, mant_dig=53, epsilon=2.220446049250313e-16, radix=2, rounds=1)
```

## 숫자 연산

파이썬에서 숫자에 대해 연산 가능한 `operator`들은 다음과 같다.

```
a = 135.68
b = 15

# 기본적인 사칙연산
a + b
a - b
a * b
a / b

# 거듭제곱
b ** 2
# 나누기 후 나머지
a % b
# 나누기 후 몫
a // b
```

### 연습문제


## 문자열 연산

