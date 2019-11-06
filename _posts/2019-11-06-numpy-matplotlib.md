---
layout: post
title:  "Numpy and Matplotlib"
date:   2010-11-06 09:00:13
categories: WIP
---



# Numpy

`numpy`는 배열 객체를 만들고 배열 연산을 할 수 있는 패키지다. `numpy`를 쓰면 `MATLAB`의 행렬 연산과 비슷한 기능을 한다. `numpy`의 다양한 기능과 세부적인 용법은 책 한권 분량이기 때문에 여기서는 기초적인 내용만 다룬다. 이후 배울 영상처리에서도 영상을 `numpy`의 배열로 처리하게 되므로 잘 알아두어야 한다. `numpy`에 관한 내용은 점프투파이썬이 아닌 [이곳](http://taewan.kim/post/numpy_cheat_sheet/)을 참고해서 만들었다. 이곳에 더 자세한 내용이 있으니 들어가서 공부해보길 바란다.

## 2.1 Array vs Matrix

MATLAB과 numpy의 가장 큰 차이는 기본 데이터 형식이 MATLAB은 **행렬(matrix)**이고 numpy는 **배열(array)**이라는 것이다. 그럼 행렬과 배열은 무엇이 다른가? 똑같이 `A=[1 2; 3 4]`라는 배열과 행렬이 있을 때 `A*A`를 하면 MATLAB에서는 `[7 10; 15 22]`가 나오고 numpy에서는 `[1 4; 9 16]`이 나온다. MATLAB은 행렬의 곱셈을 한 것이고 numpy는 배열의 같은 위치의 원소끼리 곱셈을 한 것이다. 행렬은 수학적인 matrix를 의미하는 것이고 배열은 프로그래밍에서 같은 종류의 데이터 여러 개를 모아놓은 것이다. 물론 MATLAB에서도 배열 연산이 가능하고 numpy에서도 행렬 연산이 가능하지만 기본 데이터 형식이 다르다는 것을 알아두어야 한다.

## 2.2 Array Creation

배열을 생성하는 가장 기본적인 방법은 리스트를 이용하는 것이다. 다중 리스트를 이용하면 다차원 배열도 만들 수 있다. `np.array()` 함수에 list를 넣으면 되는데 `dtype`이란 입력인자로 데이터 타입도 정할 수 있다.

```python
import numpy as np
array1d = [1, 2, 3, 4]
array2d = [[1, 2], [3, 4]]
array3d = [[[1, 2], [3, 4]], [[5, 6], [7, 8]]]
print("array1d", np.array(array1d, dtype=int))
print("array2d\n", np.array(array2d, dtype=float))
print("array3d\n", np.array(array3d))
```

> array1d [1 2 3 4]
> array2d [[1. 2.] [3. 4.]]
> array3d [[[1 2]  [3 4]] [[5 6]  [7 8]]]

`np.zeros(), np.ones(), np.identity(), np.eye()`는 0이나 1로 채워진 배열을 원하는 크기로 만드는 함수다. 일정 간격의 숫자를 만들때는 `np.linspace()`나 `np.arange()`를 쓰는데 `np.linspace()`는 숫자 개수를 기준으로, `np.arange()`는 간격을 기준으로 배열을 만든다. `np.permutation(n)`은 `[0, n)` 범위의 정수를 랜덤하게 섞은 배열을 생성해준다. 

```python
print("ones((2, 4))\n", np.ones((2, 4)))
print("zeros((3, 2))\n", np.zeros((3, 2)))
print("identity(3)\n", np.identity(3))
print("eye(3)\n", np.eye(3))
print("linspace(5, 10, 11):", np.linspace(5, 10, 11))
print("arange(5, 10, 0.5):", np.arange(5, 10, 0.5))
print("permutation(10):\n", np.random.permutation(10))
```

> ones((2, 4))
>  [[1. 1. 1. 1.]
>  [1. 1. 1. 1.]]
> zeros((3, 2))
>  [[0. 0.]
>  [0. 0.]
>  [0. 0.]]
> identity(3)
>  [[1. 0. 0.]
>  [0. 1. 0.]
>  [0. 0. 1.]]
> eye(3)
>  [[1. 0. 0.]
>  [0. 1. 0.]
>  [0. 0. 1.]]
> linspace(5, 10, 11): [ 5.   5.5  6.   6.5  7.   7.5  8.   8.5  9.   9.5 10. ]
> arange(5, 10, 0.5): [5.  5.5 6.  6.5 7.  7.5 8.  8.5 9.  9.5]
> permutation(10): [7 3 9 0 5 8 4 6 1 2]

`np.linspace()`와 `np.arange()`로 비슷한 배열을 만들었는데 `np.linspace()`는 범위가 `[5, 10]`이고 `np.arange`는 `[5, 10)`이라서 마지막 10이 안 들어간다는 것에 유의하자.  

난수로 배열을 생성하는 것도 가능하다. `np.random` 아래의 함수들을 사용한다.

- np.random.rand(d0, d1, ..., dn): 입력한 크기의 난수배열 생성. 값은 [0, 1) 사이의 값을 uniform sampling 한다.
- np.random.randn(d0, d1, ..., dn): 입력한 크기의 난수배열 생성. 값은 평균 0, 표준편차 1의 정규분포로부터 표본추출한다.
- np.random.randint(low, high, size): 입력한 크기의 정수 난수배열 생성. `[low, high)` 사이의 정수를 랜덤생성하여 `size`의 크기의 배열을 만든다.

```python
np.set_printoptions(precision=5)
print("uniform over [0, 1)\n", np.random.rand(3, 4))
print("normal by N(0, 1)\n", np.random.randn(3, 4))
print("random int over [0, 5)\n", np.random.randint(0, 5, size=(2, 3)))
```

> uniform over [0, 1)
>  [[0.91393 0.85865 0.61936 0.58212]
>  [0.28647 0.51623 0.22979 0.98041]
>  [0.59227 0.97546 0.12851 0.91155]]
> normal by N(0, 1)
>  [[ 0.10321  0.93799  2.14548  0.25712]
>  [-1.15454 -0.76393 -0.14389  0.51151]
>  [ 0.4485   0.22908  0.04046  0.33101]]
> random int over [0, 5)
>  [[3 0 0]
>  [2 2 4]]

## 2.3 Array Shape

배열에서 `np.ndarray.shape` 변수는 각 차원의 크기 정보를 튜플로 가지고 있다. 전체 차원수는  `np.ndarray.ndim` 으로 확인할 수 있다.

```python
foo = np.ones((3, 4, 2))
print("foo.shape:", foo.shape)
print("foo.ndim:", foo.ndim)
```

> foo.shape: (3, 4, 2)
> foo.ndim: 3

배열을 다루다 보면 shape을 바꾸고 싶을 때가 있다. 1차원 벡터를 2차원 배열로 바꾼다던지 3차원 배열을 1차원 벡터로 늘여서 표현한다던지 등의 경우가 있다. 이럴때는 `np.ndarray.reshape()`이라는 함수를 쓰면 된다. 아래 예시에서 어떻게 이런 결과가 나오는지 헷갈릴 수 있지만 데이터를 한 줄로 쭉 펴놓고 배열 모양에 순서대로 하나씩 넣는다고 생각하면 이해가 된다.

```python
foo = np.arange(0, 6)
print("foo", foo)
print("foo (2,3)\n", foo.reshape(2, 3))
foo3d = foo.reshape(2, 3, 1)
print("foo (2,3,1)\n", foo3d)
print("foo (3,2)\n", foo3d.reshape(3, 2))
print("foo (3,2)\n", foo3d.reshape(2, 3))
```

> foo [0 1 2 3 4 5]
> foo (2,3)
>  [[0 1 2]
>  [3 4 5]]
> foo3d (2,3,1)
>  [[[0]  [1]  [2]]
>  [[3]  [4]  [5]]]
> foo3d (3,2)
>  [[0 1]
>  [2 3]
>  [4 5]]
> foo (3,2)
>  [[0 1]
>  [2 3]
>  [4 5]]



## 2.4 Array Indexing, Slicing

다차원 `numpy` 배열에서 특정 데이터 값이나 일부 배열을 가져오는 방법은 리스트와 비슷하다. 다차원 배열을 인덱싱, 슬라이싱 할 때는 하나의 `[d0, d1, ...]`안에 모든 인덱스를 넣는다. 기존에 MATLAB을 써본 사람은 배열에서 인덱스를 정할 때 n번째 인덱스가 가로축인지 세로축인지 깊이축인지 헷갈릴 수 있다. 그럴때는 다차원 리스트 안에 들어있는 값에 접근할 때를 생각해보면 된다. 가장 바깥쪽 리스트가 `d0`에 해당하고 점점 안으로 들어갈 수록 `d1, d2, ...`가 되는 것이다. 가로축, 세로축을 기준으로 생각하면 헷갈리니 개념을 잘 잡아야 한다. 다음 예제를 보면서 익혀보자.

```python
data_list = [[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]]
print("data_list = ", data_list)
print("data_list[0] =", data_list[0])
print("data_list[0][1] =", data_list[0][1])
print("data_list[0][1][2] =", data_list[0][1][2])
data_np = np.array(data_list)
print("data_np =\n", data_np)
print("data_np[0] =\n", data_np[0])
print("data_np[0, 1] =", data_np[0, 1])
print("data_np[0, 1, 2] =", data_np[0, 1, 2])
```

> data_list =  [[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]]
> data_list[0] = [[1, 2, 3], [4, 5, 6]]
> data_list\[0][1] = [4, 5, 6]
> data_list\[0]\[1][2] = 6
> data_np =
>  [[[ 1  2  3]
>   [ 4  5  6]]
>  [[ 7  8  9]
>   [10 11 12]]]
> data_np[0] =
>  [[1 2 3]
>  [4 5 6]]
> data_np[0, 1] = [4 5 6]
> data_np[0, 1, 2] = 6

인덱싱 하는 순서는 다차원 리스트와 같고 인덱스를 여러 괄호에 따로 쓰느나 한 괄호에 쓰느냐의 차이만 있다. 슬라이싱에서는 더 큰 차이가 있는데 다차원 리스트는 다차원 슬라이싱을 할 수 없지만 `numpy` 배열에서는 가능하다. 

```python
print("\ndata: shape={}\n{}".format(data.shape,  data))
print("1) data[0, :, :]: shape={}\n{}".format(data[0, :, :].shape, data[0, :, :]))
print("2) data[:, :, 1]: shape={}\n{}".format(data[:, :, 1].shape, data[:, :, 1]))
print("3) data[0, :, 1:]: shape={}\n{}".format(data[0, :, 1:].shape, data[0, :, 1:]))
print("4) data[0, 1, :]: shape={}\n{}".format(data[0, 1, :].shape, data[0, 1, :]))
print("5) data[0, 1:, :]: shape={}\n{}".format(data[0, 1:, :].shape, data[0, 1:, :]))
print("6) data[:1, 1:, :]: shape={}\n{}".format(data[:1, 1:, :].shape, data[:1, 1:, :]))
```

> 1) array[0, :, :]: shape=(2, 3)
> [[1 2 3]
>  [4 5 6]]
> 2) array[:, :, 1]: shape=(2, 2)
> [[ 2  5]
>  [ 8 11]]
> 3) array[0, :, 1:]: shape=(2, 2)
> [[2 3]
>  [5 6]]
> 4) array[0, 1, :]: shape=(3,)
> [4 5 6]
> 5) array[0, 1:, :]: shape=(1, 3)
> [[4 5 6]]
> 6) array[:1, 1:, :]: shape=(1, 1, 3)
> [[[4 5 6]]]

- 1)은 첫번째 차원은 인덱싱을 하고 나머지에선 전체 슬라이싱을 하여 결과가 2차원 배열로 나와야 하고 `data_np`에서 위쪽 배열이 출력된다. 
- 2)에서는 반대로 세번째 차원을 인덱싱하고 나머지에서 전체 슬라이싱을 했는데  `data_np`의 가장 안쪽 리스트의 가운데 숫자들이 나옴을 볼 수 있다. 
- 3)에서는 마지막 차원을 슬라이싱하여 결과가 2x2로 나온다.
- 4)에서는 두 차원을 인덱싱하여 1차원 배열이 나왔고 `data_list[0][1]`과 같은거라서 `[3, 4, 9]`가 나온다. 인덱싱을 한 첫번째 두 개의 차원은 사라지고 슬라이싱한 마지막 차원만 배열로 나타난다.
- 5)에서 나오는 숫자들은 4)와 같지만 `d1`에서 인덱싱이 아닌 슬라이싱을 했기 때문에 길이 1인 차원이 하나 더 생겼다. 리스트에서 인덱싱하면 값이 나오고 슬라이싱하면 리스트가 나온다는 것을 상기하자. 슬라이싱을 하면 크기와 관계없이 그 차원은 배열로 남고 인덱싱을 하면 그 차원은 값만 남기고 사라진다.
- 6)은 모든 차원을 슬라이싱 했기 때문에 3차원 배열이 된다.

## 2.5 Array Operations

배열 사이의 연산은 단순히 원소 사이의 연산으로 치환된다. 아래 예시에서 operator와 그에 해당하는 함수도 봐두자.

```python
print("\nmatrix operations")
foo = np.array([[9, 3, 2], [1, 3, 9], [1, 6, 8]])
bar = np.array([[1, 4, 2], [3, 3, 4], [2, 1, 3]])
print("foo\n", foo)
print("bar\n", bar)
print("foo + bar\n", foo + bar, "\n", np.add(foo, bar))
print("foo - bar\n", foo - bar, "\n", np.subtract(foo, bar))
print("foo * bar\n", foo * bar, "\n", np.multiply(foo, bar))
print("foo / bar\n", foo / bar, "\n", np.divide(foo, bar))
print("foo ** bar\n", foo ** bar, "\n", np.power(foo, bar))
print("foo // bar\n", foo // bar, "\n", np.floor_divide(foo, bar))
print("foo % bar\n", foo % bar, "\n", np.remainder(foo, bar))
print("foo x bar\n", foo @ bar, "\n", np.dot(foo, bar))
print("foo.T (transpose)\n", foo.T)
```

`np.dot` 함수는 배열이 아닌 dot product로 행렬의 곱하기를 계산하는 함수다. 배열에 `.T`를 붙이면 transpose 된 배열이 나온다. 배열에서 특정 조건을 만족하는 숫자들을 뽑아내고 싶을 때는 `True, False (TF)` 배열을 이용한 인덱싱도 가능하다.

```python
print("\ncomparison operators")
print("foo > bar\n", foo > bar)
print("greater\n", np.greater(foo, bar))
print("foo[foo > bar]:", foo[foo > bar])
print("foo > bar\n", foo <= bar)
print("less_equal\n", np.less_equal(foo, bar))
print("foo[foo > bar]:", foo[foo <= bar])
print("foo[foo >= 5]:", foo[foo >= 5])
print("foo[bar < 3]:", foo[bar < 3])
print("foo[foo % 2 == 0]:", foo[foo % 2 == 0])
```

> foo > bar
>  [[ True False False]
>  [False False  True]
>  [False  True  True]]
> foo[foo > bar]: [9 9 6 8]
> foo > bar
>  [[False  True  True]
>  [ True  True False]
>  [ True False False]]
> foo[foo > bar]: [3 2 1 3 1]
> foo[foo >= 5]: [9 9 6 8]
> foo[bar < 3]: [9 2 1 6]
> foo[foo % 2 == 0]: [2 6 8]

`foo > bar`를 프린트해보면 각각의 원소에 대해서 비교 연산을 한 TF 배열이 나온다. 이 TF 배열을 인덱스처럼 넣으면 `True`에 해당하는 원소들만 1차원 배열로 나오게 된다. 

## 2.6 Math Functions

`numpy`는 `sin, cos, tan, exp, log, sqrt` 등의 기본적인 수학함수들도 제공한다. 배열에 적용하면 배열의 모든 원소에 대해 각각 계산한 결과가 나온다.

```python
print("\nbasic math functions")
np.set_printoptions(precision=4, suppress=True)
foo = np.random.rand(5)
print("foo", foo)
print("np.sin(foo):", np.sin(foo))
print("np.cos(foo):", np.cos(foo))
print("sin^2 + cos^2 = 1:", np.sin(foo)**2 + np.cos(foo)**2)
print("np.exp(foo):", np.exp(foo))
print("np.log(foo):", np.log(foo))
print("np.log(exp(foo))==foo", np.log(np.exp(foo)))
print("np.sqrt(foo):", np.sqrt(foo))
print("np.sqrt(foo)^2==foo:", np.sqrt(foo)*np.sqrt(foo))
```

그 외에 필요하다고 생각되는 함수는 대부분 다 제공한다. `mean, max, min, std, sum`등의 통계적인 함수도 있는데 이들 함수는 집계 함수(aggregate function)이라고 해서 여러 숫자에 대한 통계를 내기 때문에 어떤 축을 기준으로 평균 등을 계산하느냐에 따라서 결과가 달라진다. `axis=None`이면 모든 숫자에 대해 계산하고 `axis=0`이면 첫 번째 축의 값에 대해 계산후 첫 번째 차원을 없앤다. `axis=1`이면 두 번째 축의 값에 대해 계산후 두 번째 차원을 없앤다. 다음 예시를 보면서 이해해보자.

```python
print("\naggregate functions")
rgen = np.random.RandomState(2)
foo = rgen.rand(2, 4)
print("data:", foo)
print("1) mean over all:", np.mean(foo))
print("2) mean over axis 0:", np.mean(foo, axis=0))
print("3) mean over axis 1:", np.mean(foo, axis=1))
```

> data: [[0.43599 0.02593 0.54966 0.43532]
>  [0.42037 0.33033 0.20465 0.61927]]
> 1) mean over all: 0.3776910284933438
> 2) mean over axis 0: [0.42818 0.17813 0.37716 0.5273 ]
> 3) mean over axis 1: [0.36173 0.39366]

1)은 foo의 모든 숫자에 대해 평균을 계산한 것이다. 2)는 세로줄의 평균을 구해서 4개의 숫자가 나왔다. 3)은 가로줄의 평균을 구해서 2개의 결과가 나왔다. 이러한 차원의 방향을 생각하며 아래 예시들도 이해해보자.

```python
print("sum", np.sum(foo, axis=0))
print("min", np.min(foo, axis=1))
print("max", np.max(foo, axis=0))
print("std", np.std(foo, axis=1))
```

> sum [0.85636 0.35626 0.75431 1.05459]
> min [0.02593 0.20465]
> max [0.43599 0.33033 0.54966 0.61927]
> std [0.19938 0.15112]

## 2.7 Iterate by for

`numpy` 배열에 대해서도 for문을 돌 수 있는데 단순히 첫 번째 차원(d0)에 대해 순서대로 인덱싱한 결과가 나온다고 보면 된다. 다음 예시에서 인덱싱한 결과와 배열을 for문에 바로 넣은 결과가 같은 것을 볼 수 있다.

```python
for i in range(len(foo)):
    print("row", i, foo[i])
for row in foo:
    print("row", row)
for row in foo:
    for value in row:
        print("v:", value)
```


## 연습문제

`np.mean`과 똑같이 작동하는 `find_mean(array, axis=None)` 함수를 구현하세요. 배열에 대해 for loop을 돌면서 주어진 axis에 대해 indexing 하여 평균으로 합칠 값들을 추출하고 평균을 계산하세요.



# Matplotlib



## 함수형 vs 객체형

https://riptutorial.com/ko/matplotlib/topic/881/matplotlib-시작하기

https://matplotlib.org/3.1.1/tutorials/index.html





---

## TODO

- 11.5: 가상환경 및 pycharm 설정, 프로젝트 공지
- 11.6: navigation base
- 11.12: ros name and roslaunch
- 11.13: 2차원 좌표계 변환
- 11.19: 
- 11.20: 2차원 변환 numpy 코딩
- 11.26: rviz, rqt - 좌표계 변환 결과 보여주기, config 저장 불러오기, image/LDS 보여주기
- 11.27: 3차원 좌표계 변환
- 12.3: 
- 12.4: 3차원 변환 코딩

