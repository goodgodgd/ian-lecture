---
layout: post
title:  "[Python] Use Basic Packages"
date:   2020-04-02 09:00:13
categories: 2020-1-systprog
---



# 강력한 파이썬의 패키지들

지금까지 배운 편리한 파이썬 문법을 기반으로 수많은 사용자들이 자신이 개발한 패키지를 pypi에 공유하여 파이썬 생태계는 지속적으로 확장되고 있다. 파이썬 패키지는 간단한 유틸리티들부터 웹 개발, 데이터분석, 인공지능 등 거의 없는 것이 없다. 무엇인가를 개발하기 전에 항상 활용할 수 있는 기존 패키지를 찾아보는 것이 좋다. 이번 장에서는 파이썬 기본 패키지인 `sys, os, glob` 등과 따로 설치해야 하는 외부 패키지인 `numpy`의 사용법에 대해 알아볼 것이다. 각각의 패키지들이 가지고 있는 기능을 모두 설명하려면 몇 일이 걸릴지 모르니 가장 많이 쓰이는 기능만 골라서 공부해보자.

# 1. Utilities

## 1.1 sys.path

`sys.path`는 파이썬이 패키지를 검색하는 경로들이다. 파이썬에서 `import package_name`을 할 때 `package_name`을 찾는 디렉토리 위치가 지정되어 있다. 일단 한번 프린트해보자.

```python
import sys
print(sys.path)
```

현재 프로젝트 경로와 파이썬과 관련된 여러 경로들이 나올 것이다. 만약 내가 특정 폴더의 파이썬 패키지나 모듈을 가져다 쓰고 싶다면 그곳의 경로를 `sys.path`에 추가하면 된다. 이전 시간에 만든 `package`라는 폴더를 추가하여 `list_ops.py`를 직접 import 해보자.

```python
try:
    import list_ops as lo
    print(lo.spam)
except ModuleNotFoundError as e:
    print(e)

import sys
new_path = 'D:/Work/ian-lecture/scripts/package'
if new_path not in sys.path:
    sys.path.append(new_path)
print(sys.path)
import list_ops as lo
print(lo.spam)
```

경로를 추가하지 않고 `import package.list_ops` 가 아니라 바로 `import list_ops`를 실행하면 에러가 난다는 것을 확인할 수 있다. 추가할 경로를 `new_path`에 만들었는데 주의할 점은 윈도우에서 부모폴더와 자식폴더는 `//` 또는 `\`로 구분해야한다. 경로를 탐색기에서 복사해서 붙이면 그냥 `/` 하나만 있는데 둘 중 하나로 수정해줘야 한다. 그리고 무턱대고 `sys.path`에 경로를 추가하면 같은 경로를 여러번 추가할 수도 있으므로 기존 경로 중에 `new_path`가 없다는 것을 확인하고 입력하는 것이 좋다. 경로를 추가한 후에 다시 `import list_ops`를 하니 import가 되고 `lo.spam` 값도 잘 나오는 것을 확인할 수 있다.

## 1.2 파일이나 폴더 생성/삭제

파이썬에서는 내장 패키지를 통해 파일이나 폴더를 생성/삭제 할 수 있는 함수를 제공한다.

- os.mkdir(dirpath): `dirpath` 경로에 폴더를 만든다. 이미 존재하는 폴더를 만들라고 하면 에러를 발생시킨다.
- shutil.rmtree(dirpath, ignore_errors): `dirpath` 경로의 폴더를 지운다. 없는 폴더를 지우라고하면 에러를 발생시킨다. `ignore_errors=True`이면 폴더가 없어도 에러를 무시한다.
- os.remove(filepath): `filepath` 경로의 파일을 지운다. 없는 파일을 지우라고하면 에러를 발생시킨다.

다음 예제에서 `create`이 1 (또는 True)일 때는 파일과 폴더를 생성하고 0 (또는 False)일 때는 삭제한다. 먼저 1로 실행하여 생성된 파일과 폴더를 확인하고 0으로 실행하여 삭제된 결과를 확인해보자. 이미 똑같은 생성/삭제 명령을 반복하면 에러가 나는 것을 볼 수 있다.

```python
create = 1
import os
import shutil
tempdir = "D:/tempdir"
tempfile1 = "D:/tempdir/tempfile1.txt"
tempfile2 = "D:/tempdir/tempfile2.txt"

if create == 1:
    os.mkdir(tempdir)
    try:
        os.mkdir(tempdir)
    except FileExistsError as e:
        print(e)

    with open(tempfile1, "w") as f:
        f.write("blurblurblur1")
    with open(tempfile2, "w") as f:
        f.write("blurblurblur2")
    print("two files created")

elif create == 0:
    os.remove(tempfile1)
    try:
        os.remove(tempfile1)
    except FileNotFoundError as e:
        print(e)

    shutil.rmtree(tempdir)
    try:
        shutil.rmtree(tempdir)
    except FileNotFoundError as e:
        print(e)
    shutil.rmtree(tempdir, ignore_errors=True)
    print("rmtree non-existing dir but no error")
```

## 1.3 파일  폴더 존재 확인

위 예제에서처럼 파일이나 폴더가 있는지를 모르고 파일 생성/삭제를 하게되면 에러가 나기 쉽다. 파일이나 폴더가 이미 있는지 확인하는 함수가 필요한데 `os.path.isdir()`과 `os.path.isfile()`을 이용하면 된다.

```python
import os
import shutil
tempdir = "D:/tempdir"
tempfile1 = "D:/tempdir/tempfile1.txt"
os.mkdir(tempdir)

if os.path.isfile(tempfile1):
    print(f"{tempfile1} exists. remove it now")
    os.remove(tempfile1)
else:
    print(f"{tempfile1} does NOT exists")

if os.path.isdir(tempdir):
    print(f"{tempdir} exists. remove it now")
    shutil.rmtree(tempdir)
else:
    print(f"{tempdir} does NOT exists")
```

## 1.4 경로명 만들기

폴더경로와 파일명을 합칠 때는 둘 사이에 `//`나 `\`를 넣어주면 된다. 문자열 연산으로 해도 되지만 합치는 방식이 운영체제 별로 조금씩 다르기 때문에 `os.path.join()` 함수를 쓰면 좋다. 반대로 전체 경로에서 마지막 파일이나 폴더명을 분리할 때는 `os.path.basename()`을 쓰고 그것이 속한 경로명만 추출하고 싶을 때는 `os.path.dirname()`을 사용한다.

```python
import os
curfile = __file__
curfile_path = os.path.abspath(curfile)
print("current file:", curfile)
print("current file absolute path:", curfile_path)
filename = os.path.basename(curfile_path)
print("current file name:", filename)
pathname = os.path.dirname(curfile_path)
print("current dir path:", pathname)
print("grand parent dir path:", os.path.dirname(pathname))
newfile = os.path.join(pathname, "newfile.txt")
print("new file name:", newfile)
with open(newfile, "w") as f:
    f.write("new file created beside python file")
newpath = os.path.join(pathname, "new", "path", "name")
print([newpath])
```

## 1.5 파일목록 출력

다수의 파일이나 폴더들을 자동으로 관리해야 한다고 했을 때 일단 어떤 파일들이 있는지 목록을 만들수 있어야 한다. 파이썬에는 파일 목록을 보는 여러 방법이 있지만 그 중 `os.listdir()`과 `glob.glob()`이 쓰기 쉬운 편이다. 간단한 예제를 실행해보자.

```python
import os
import glob
curfile = os.path.abspath(curfile)
curdirpath = os.path.dirname(curfile)

files = os.listdir(curdirpath)
print("file list:", files)

search_pattern = os.path.join(curdirpath, "*")
print("search pattern:", search_pattern)
filelist = glob.glob(search_pattern)
print("file list:", filelist)

search_pattern = os.path.join(curdirpath, "*.py")
print("search pattern:", search_pattern)
pyfilelist = glob.glob(search_pattern)
print("python file list:", pyfilelist)
```

`os.listdir()`은 입력된 폴더 내의 모든 객체들의 이름만 리스트로 출력하고 `glob.glob()`은 입력된 패턴과 일치하는 모든 객체를 경로명과 함께 출력한다. `glob`에서는 Regular expression으로 검색 패턴을 받아들이는데 그냥 `*`이 모든 문자열을 대체할 수 있다는 것만 알아도 유용하게 쓸 수 있다. 주의할 점은 파일 목록에 파일명만 나오는 것이 아니라 입력한 경로 아래 폴더명도 나오기 때문에 이들을 구분할 필요가 있다. 이때도 `os.path.isdir()` 과 `os.path.isfile()`이 유용하게 쓰인다.

```python
filelist = [os.path.basename(path) for path in filelist if os.path.isfile(path)]
dirlist = [os.path.basename(path) for path in filelist if os.path.isdir(path)]
print("file list in current dir:", filelist)
print("subdir list in current dir:", dirlist)
```


## HW2

다음 [주소](<https://drive.google.com/file/d/1x_DII1VEvpdYHx8GQjAEgWsG2fsqO8L4/view?usp=sharing>)에서 압축파일을 받아 압축을 푸세요. `lyrics`라는 폴더 아래 봄 노래 세 곡의 가사가 텍스트 파일로 저장되어 있습니다. 다음 기능을 하는 파이썬 스크립트를 만들어보세요.

1. `lyrics` 옆에 `lyrics_winter`라는 폴더를 만들기, 단 폴더가 있는지를 확인해서 없을때만 만들기
2. `lyrics` 내부의 파일 목록을 만들고 한 파일씩 가사 데이터를 읽기
3. 읽어온 가사에서 `봄`은 `겨울`로 `벚꽃`은 `눈`으로 바꾸기
4. 수정된 가사를 `lyrics_winter` 폴더에 같은 파일명으로 저장하기 e.g. `lyrics/봄봄봄.txt` → `lyrics_winter/봄봄봄.txt`

다음 스크립트에 위 기능을 채워넣어 보세요.

```python
import os
import glob
def chagne_lyrics_and_save(srcdir, dstdir, change_terms):
    # create dstdir if it does not exist (os.path.isdir)
    # create text file list (os.mkdir)
    # for loop over file list
        # open and read src text file (with open as fr; fr.read)
        # replace srcterms to dstterms: for src, dst in change_terms.items()
        # create new file name located at dstdir (replace)
        # open and write new file (with open as fw, fw.write)

if __name__ == "__main__":
    # file url: http://naver.me/54MAmyO1
    srcdir = "D:/path/to/lyrics"
    dstdir = "D:/path/to/lyrics_winter"
    change_terms = {"봄": "겨울", "벚꽃": "눈"}
    chagne_lyrics_and_save(srcdir, dstdir, change_terms)
```



# 2. numpy

`numpy`는 배열 객체를 만들고 배열 연산을 할 수 있는 패키지다. `numpy`를 쓰면 `MATLAB`의 행렬 연산과 비슷한 기능을 한다. `numpy`의 다양한 기능과 세부적인 용법은 책 한권 분량이기 때문에 여기서는 기초적인 내용만 다룬다. 이후 배울 영상처리에서도 영상을 `numpy`의 배열로 처리하게 되므로 잘 알아두어야 한다. `numpy`에 관한 내용은 점프투파이썬이 아닌 [이곳](http://taewan.kim/post/numpy_cheat_sheet/)을 참고해서 만들었다. 이곳에 더 자세한 내용이 있으니 들어가서 공부해보길 바란다.

## 2.1 Array vs Matrix

MATLAB과 numpy의 가장 큰 차이는 기본 데이터 형식이 MATLAB은 **행렬(matrix)**이고 numpy는 **배열(array)**이라는 것이다. 그럼 행렬과 배열은 무엇이 다른가? 똑같이 `A=[1 2; 3 4]`라는 배열과 행렬이 있을 때 `A*A`를 하면 MATLAB에서는 `[7 10; 15 22]`가 나오고 numpy에서는 `[1 4; 9 16]`이 나온다. MATLAB은 행렬의 곱셈을 한 것이고 numpy는 배열의 같은 위치의 원소끼리 곱셈을 한 것이다. 행렬은 수학적인 matrix를 의미하는 것이고 배열은 프로그래밍에서 데이터 여러 개를 모아놓은 것이다. 물론 MATLAB에서도 배열 연산이 가능하고 numpy에서도 행렬 연산이 가능하지만 기본 데이터 형식이 다르다는 것을 알아두어야 한다.

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

`np.zeros(), np.ones(), np.identity(), np.eye()`는 0이나 1로 채워진 배열을 원하는 크기로 만드는 함수다. 일정 간격의 숫자를 만들때는 `np.linspace()`나 `np.arange()`를 쓰는데 `np.linspace()`는 숫자 개수를 기준으로, `np.arange()`는 간격을 기준으로 배열을 만든다. `np.permutation(n)`은 `[0, n)` 범위의 정수를 랜덤하게 섞은 배열을 생성해준다. 

```python
print("ones\n", np.ones((2, 4)))
print("zeros\n", np.zeros((3, 2)))
print("identity\n", np.identity(3))
print("identity\n", np.eye(3))
print("linear space:", np.linspace(5, 10, 11))
print("arange:", np.arange(5, 10, 0.5))
print("permutation:\n", np.random.permutation(10))
```

`np.linspace()`와 `np.arange()`로 비슷한 배열을 만들었는데 `np.linspace()`는 범위가 `[5, 10]`이고 `np.arange`는 `[5, 10)`이라서 마지막 10이 안 들어간다는 것에 유의하자.  

난수로 배열을 생성하는 것도 가능하다. `np.random` 아래의 함수들을 사용한다.

- np.random.rand(d0, d1, ..., dn): 입력한 크기의 난수배열 생성. 값은 [0, 1) 사이의 값을 uniform sampling 한다.
- np.random.randn(d0, d1, ..., dn): 입력한 크기의 난수배열 생성. 값은 평균 0, 표준편차 1의 정규분포로부터 표본추출한다.
- np.random.randint(low, high, size): 입력한 크기의 정수 난수배열 생성. `[low, high)` 사이의 정수를 랜덤생성하여 `size`의 크기의 배열을 만든다.

```python
print("uniform over [0, 1)\n", np.random.rand(3, 4))
print("normal by N(0, 1)\n", np.random.randn(3, 4))
print("random int over [0, 5)\n", np.random.randint(0, 5, size=(2, 3)))
```

## 2.3 Array Shape

배열에서 `np.ndarray.shape` 변수는 각 차원의 크기 정보를 튜플로 가지고 있다. 전체 차원수는  `np.ndarray.ndim` 으로 확인할 수 있다.

```python
foo = np.ones((3, 4, 2))
print(foo.shape)
print(foo.ndim)
```

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

## 2.4 Array Indexing, Slicing

다차원 `numpy` 배열에서 특정 데이터 값이나 일부 배열을 가져오는 방법은 리스트와 비슷하다. 다차원 배열을 인덱싱, 슬라이싱 할 때는 하나의 `[d0, d1, ...]`안에 모든 인덱스를 넣는다. 기존에 MATLAB을 써본 사람은 배열에서 인덱스를 정할 때 n번째 인덱스가 가로축인지 세로축인지 깊이축인지 헷갈릴 수 있다. 그럴때는 다차원 리스트 안에 들어있는 값에 접근할 때를 생각해보면 된다. 가장 바깥쪽 리스트가 `d0`에 해당하고 점점 안으로 들어갈 수록 `d1, d2, ...`가 되는 것이다. 가로축, 세로축을 기준으로 생각하면 헷갈리니 개념을 잘 잡아야 한다. 다음 예제를 보면서 익혀보자.

```python
data_list = [[[5, 6, 2], [3, 4, 9]], [[1, 7, 2], [3, 8, 0]]]
data = np.array(data_list)
print("data\n", data)
print("data_list[0] =", data_list[0])
print("data_list[0][1] =", data_list[0][1])
print("data_list[0][1][2] =", data_list[0][1][2])
print("data[0] =\n", data[0])
print("data[0, 1] =", data[0, 1])
print("data[0, 1, 2] =", data[0, 1, 2])
```

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
print("\ncompare operations")
print("foo > bar\n", foo > bar, "\n", np.greater(foo, bar))
print("foo[foo > bar]:", foo[foo > bar])
print("foo <= bar\n", foo <= bar, "\n", np.less_equal(foo, bar))
print("foo[foo <= bar]:", foo[foo <= bar])
print("foo[foo >= 5]:", foo[foo >= 5])
print("foo[bar < 3]:", foo[bar < 3])
print("foo[foo % 2 == 0]:", foo[foo % 2 == 0])
```

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
foo = np.random.rand(2, 4)
print("data", foo)
print("1) mean over all", np.mean(foo))
print("2) mean over axis 0", np.mean(foo, axis=0))
print("3) mean over axis 1", np.mean(foo, axis=1))
```

1)은 foo의 모든 숫자에 대해 평균을 계산한 것이다. 2)는 세로줄의 평균을 구해서 4개의 숫자가 나왔다. 3)은 가로줄의 평균을 구해서 2개의 결과가 나왔다. 이러한 차원의 방향을 생각하며 아래 예시들도 이해해보자.

```python
print("sum", np.sum(foo, axis=0))
print("min", np.min(foo, axis=1))
print("max", np.max(foo, axis=0))
print("std", np.std(foo, axis=1))
```

## 2.7 Merging Arrays

여러개의 배열이 있는데 이들을 하나의 배열로 합치고 싶을 땐 어떻게 해야할까? `np.stack`이나 `np.concatenate` 함수를 쓰면 된다. 둘 다 배열을 합치는 함수인데 차이는 `np.stack`는 새로운 차원을 추가하며 합친다는 것이고 `np.concatenate`는 기존 배열이 갖고 있는 차원수를 유지하며 합친다는 것이다. 무슨 말인지 예시를 보며 이해해보자.

```python
print("\nmerging arrays")
foo = np.array([[1, 2, 3], [4, 5, 6]])
bar = np.array([[11, 12, 13], [14, 15, 16]])
print("stack axis=0\n", np.stack([foo, bar], axis=0))
print("stack axis=1\n", np.stack([foo, bar], axis=1))
stack1 = np.array([ [[1, 2, 3], [11, 12, 13]], 
                    [[4, 5, 6], [14, 15, 16]] ])
print("stack axis=1 equivalent\n", stack1)
print("stack axis=2\n", np.stack([foo, bar], axis=2))
print("concat axis=0\n", np.concatenate([foo, bar], axis=0))
print("concat axis=1\n", np.concatenate([foo, bar], axis=1))
```

결과를 보면 `np.stack`의 결과는 모두 차원이 늘어나서 3차원이고 `np.concatenate`의 결과를 차원이 유지가되서 2차원인 것을 볼 수 있다. `np.stack`의 원리는 지정한 차원에서의 원소끼리 `[]`로 한번더 묶어주는 것이다. `stack1`을 만드는 과정을 보면 하위 배열들을 배열로 한번 더 묶는 것이다. 반면에 `np.concatenate`은 두 배열을 배열의 원소로서 묶는게 아니라 하나의 배열로 합친다. 

## 2.8 Iterate by for

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

1. 임의의 3x4 배열 두 개를 만들고 `np.concatenate`로 합친 배열을 만드세요.
2. `np.mean`과 똑같이 작동하는 `find_mean(array, axis=None)` 함수를 구현하세요. 배열에 대해 for loop을 돌면서 주어진 axis에 대해 indexing 하여 평균으로 합칠 값들을 추출하고 평균을 계산하세요.

