---
layout: post
title:  "[Python] Function"
date:   2021-03-19 09:00:13
categories: 2021-1-systprog
---


# 함수

함수란 주어진 입력에 대해서 정해진 동작을 하고 필요시 결과값을 반환하는 코드 묶음이다. 함수는 입력과 출력이 있을수도 있고 없을수도 있다. 함수를 사용하는 이유는 크게 두 가지다. 

1. 반복적으로 사용되는 기능(코드)을 재사용하기 위해서다. 함수가 없다면 어떤 기능이 필요할 때마다 같은 코드를 다시 써야하고 그러면 전체 코드가 불필요하게 길어진다. 또한 함수로 만들어 놓으면 다른 프로젝트에서도 해당 기능만 가져다 쓸 수 있다.
2. 그러나 보통은 한번만 쓰이는 코드라도 함수로 묶는 경우가 많다. 그 이유는 코드를 기능별로 묶기 위해서다. 함수를 기능단위로 묶으면 코드를 계층적으로 구조화할 수 있어 코드를 읽기도 편하고 특정 기능을 하는 코드를 찾고 수정하는 일도 편해진다. 코드를 기능별로 잘 나누고 함수 이름을 기능에 맞게 지으면 코드에 주석을 쓸필요가 거의 없어진다.



## 1. 함수 정의

C언어에서 함수의 프로토 타입을 먼저 적는 것을 함수 선언, 함수 내용을 구현하는 코드를 함수 정의라고 부르는데 파이썬에서는 이를 나누지 않고 바로 정의만하면 된다. 기본적인 함수의 구조는 다음과 같다. 파이썬은 다중 입력과 다중 출력을 지원하고 입력이나 출력이 없을 수도 있다. 

```python
def function_name(arg1, arg2):
    statement1
    statement2
    return out1, out2

def function_no_output(arg1, arg2):
    statement1
    statement2

def function_no_input_output():
    statement1
    statement2
```

가장 간단한 예로 아래와 같은 더하기 함수를 만들 수 있다.

```python
def add(n1, n2):
    return n1+n2

print(add(1, 2))
```

좀 더 복잡한 리스트 평균을 내는 함수를 다음과 같이 정의할 수 있다. `average_list`는 이름 그대로 리스트의 평균을 구하는 함수인데 시작 인덱스(`start`)와 끝 인덱스(`end`)를 지정할 수 있고 중간에 빠져야할 인덱스(`skip`)를 지정할 수 있다. 

```python
def average_list(data, start, end, skip, verbose):
    if end is None:
        avg_data = data[start:]
    else:
        avg_data = data[start:end]

    sum = 0
    for ind, num in enumerate(avg_data):
        if ind not in skip:
            sum += num
    dlen = len(avg_data) - len(skip)
    average = sum / dlen
    if verbose:
        print(f"average {start}~{end} with skipping {skip} = {average}")
    return average
```



## 2. Keyword Arguments

파이썬은 동적 타입이기 때문에 입력인자에 어떤 값을 넣어도 일단 실행은 된다. 그렇기 때문에 함수에 입력인자가 많을 때는 순서에 맞지 않는 입력인자를 넣는 실수를 범할 가능성이 크다. 위에서 정의한 `average_list`는 다음과 같이 사용할 수 있다.

```python
data = [1, 2, 3, 4, 5, 6, 7, 8, 9]
avg = average_list(data, 2, 7, [4], True)
```

이렇게 입력인자가 다수인 경우에 어떤 인자를 몇 번째에 넣어야 할지 헷갈릴 수 있다. 혹은 다른 사람이 코드를 읽을 때 `average_list(data, 2, 7, [4])` 만 보면 의미를 이해하기 어렵다.  
파이썬에서는 명시적인 코드를 선호하기 때문에 다음과 같이 입력 값이 어느 입력인자로 들어가야 하는지 명시적으로 보여줄 수 있다. 이렇게 입력하는 인자를 **keyword argument (키워드 인자)**라 한다. 반면 기존 방식대로 순서에 의해 할당되는 입력 인자를 **positional argument (위치 인자)**라 한다.

```python
print("use keyword arguments")
avg = average_list(data, 2, 7, skip=[4], verbose=True)
avg = average_list(data, 2, end=7, skip=[4], verbose=True)
avg = average_list(data, start=2, end=7, skip=[4], verbose=True)
avg = average_list(data=data, start=2, end=7, skip=[4], verbose=True)
```

키워드 인자를 쓸 때 유의할 점은 위치 인자와 섞어 쓸 경우 **키워드 인자는 반드시 위치 인자 뒤**에 나와야 한다는 것이다. 따라서 아래 코드는 에러가 난다.

```python
avg = average_list(data, start=2, 7, [4])
```

조금 더 생각해보면 어느 인자로 어떤 값이 들어가는지 지정해주니 굳이 순서를 지킬 필요도 없다. **키워드 인자 끼리는 순서를 섞어도 된다.** 이때도 위치 인자를 먼저 쓰고 키워드 인자를 써야한다.

```python
print("mix keyword arguments")
avg = average_list(data, start=2, end=7, skip=[4], verbose=True)
avg = average_list(data, end=7, start=2, skip=[4], verbose=True)
avg = average_list(data, skip=[4], start=2, end=7, verbose=True)
```



## 3. 인자 기본값 지정

`average_list`는 네 개의 입력 인자로 리스트의 평균 계산 과정을 자세히 조절할 수 있지만 보통 많이 쓰는 기능은 단순히 리스트 전체에 대해서 평균을 구하는 것일 것이다. 이때도 `average_list(data, 0, None, [])`과 같이 나머지 입력인자를 다 써주는 것이 번거로울 수 있다.  `data`로 들어오는 필수 인자를 제외하고 나머지 인자에 기본값을 지정해주면 입력 인자를 적게 넣어도 작동한다. 기본값을 주는 방법은 C언어와 동일하게 함수 선언에서 입력인자에 기본 값을 할당하면 된다. (`argument=default_value` )  `average_list`를 기본 값을 이용해 사용하게 해주는 `average_list_with_default`를 다음과 같이 정의하였다.

```python
def average_list_with_default(data, start=0, end=None, skip=None, verbose=False):
    if skip is None:
        skip = []
    return average_list(data, start, end, skip, verbose)

print("function default arguments")
avg = average_list_with_default(data)
print("average_list_with_default(data) =>", avg)
avg = average_list_with_default(data, 3)
print("average_list_with_default(data, 3) =>", avg)
avg = average_list_with_default(data, end=5)
print("average_list_with_default(data, end=5) =>", avg)
avg = average_list_with_default(data, skip=[3, 4])
print("average_list_with_default(data, skip=[3, 4]) =>", avg)
```

- 첫 번째 호출에서는 `data`를 제외한 나머지 인자를 기본값으로 사용한 것이다.
- 두 번째 호출은 `start`까지만 값을 입력한 것이다. 
- 세 번째 호출은 키워드 인자를 사용하여 `start`는 기본값을 쓰고 `end`만 입력한 것이다.
- 네 번째 호출은 키워드 인자를 사용하여 `start, end`는 기본값을 쓰고 `skip`만 입력한 것이다.

다음은 그래프를 그리는 파이썬 패키지인 `matplotlib`에서 boxplot을 그리는 함수의 선언부다. 입력인자가 굉장히 많지만 대부분 `None`이라는 기본 값이 들어가 있어 필수 입력인자는 아니라는 것을 알 수 있다. 실제로 `x`라는 입력인자에 데이터만 넣으면 기본 boxplot이 그려지고 나머지는 boxplot의 세부적인 모양에 관한 옵션이다.

```python
matplotlib.pyplot.boxplot(x, notch=None, sym=None, vert=None, whis=None, positions=None,
                          widths=None, patch_artist=None, bootstrap=None, 
                          usermedians=None, conf_intervals=None, meanline=None, 
                          showmeans=None, showcaps=None, showbox=None, showfliers=None, 
                          boxprops=None, labels=None, flierprops=None, medianprops=None, 
                          meanprops=None, capprops=None, whiskerprops=None, 
                          manage_xticks=True, autorange=False, zorder=None, *, data=None)
```



### 연습문제

두 개의 숫자 리스트를 입력인자로 받아 원소별로 합산한 리스트를 출력하는 함수를 구현하시오. 함수는 다음과 같은 입력인자를 가져야 한다. 함수를 구현 후 이를 다양하게 사용해보시오.

- `list1, list2`: 두 개의 리스트 입력인자는 필수 입력인자로 받음, 기본값이 없는 필수 인자
- `shortlen`: True이면 두 리스트 중 짧은 쪽에 출력 길이를 맞춤, False이면 두 리스트 중 긴 쪽에 출력 길이를 맞춤 (e.g. [2, 3, 4] + [1, 2, 3, 4, 5] = [3, 5, 7, 4, 5]), 기본값은 True
- `start`: 지정하면 두 리스트에서 `start` 인덱스부터 더함, 기본값은 0
- `verbose`: True이면 계산 결과를 함수 내부에서 프린트함, 기본값은 False



## 4. 변경 가능한 입력인자 개수

### 4.1 *args

코딩을 하다보면 함수에 들어갈 입력인자의 개수를 미리 알 수 없거나 너무 많아서 함수 선언에 다 쓰기 힘든 경우도 있다. C++ 같은 경우 모든 가능한 경우의 입력인자 개수만큼 함수 선언을 따로 해줘야 하지만 파이썬에서는 `*args`라는 입력인자 하나로 여러개의 입력인자를 받을 수 있다. `args`라는 이름은 관습적으로 많이 쓰이는 것이고 `*input`처럼 다른 이름으로 바꿀수 있고 앞에 `*`만 붙이면 된다. `*args`는 반드시 마지막 입력인자로 들어가야하며 `*args` 앞에는 일반적인 고정형 입력인자가 들어갈 수 있다.

```python
subject_scores = {"cpp": [57, 36, 80],
                  "java": [46, 88, 72],
                  "ruby": [85, 23, 34]}

def average_multi_subjects(scores, *args):
    averages = {}
    print("[average_multi_subjects] args:", args)
    print("[average_multi_subjects] *args:", *args)
    for subject in args:
        avg = average_list_with_default(scores[subject], verbose=False)
        print(f"average over {subject} scores: {avg:.1f}")
        averages[subject] = avg
    return averages

result = average_multi_subjects(subject_scores, "cpp")
result = average_multi_subjects(subject_scores, "cpp", "java", "ruby")
```

결과

```
[average_multi_subjects] args: ('cpp',)
[average_multi_subjects] *args: cpp
average over cpp scores: 57.7
[average_multi_subjects] args: ('cpp', 'java', 'ruby')
[average_multi_subjects] *args: cpp java ruby
average over cpp scores: 57.7
average over java scores: 68.7
average over ruby scores: 47.3
```

결과를 보면 `args`는 여러 개의 인자를 튜플로 받아오고 `*args`는 튜플로 묶인 여러 데이터를 각각의 데이터로 풀어준다는 것을 볼 수 있다.  

아래 코드는 `*args`를 응용한 또 다른 예시다.

```python
def average_variable_arguments(data, multiple, *args):
    # do some process ...
    data = [d*multiple for d in data]
    avg = average_list_with_default(data, *args)
    return avg

result = average_variable_arguments(data, 10, 1, 7)
print("average_variable_arguments(data, 1, 7) =>", result)
result = average_variable_arguments(data, 10, 1, 7, [3], False)
print("average_variable_arguments(data, 1, 7, [3], True) =>", result)
result = average_list_with_default(data, 1, 7)
print("average_list_with_default(data, 1, 7) =>", result)
result = average_list_with_default(data, 1, 7, [3], False)
print("average_list_with_default(data, 1, 7, [3], True) =>", result)
```

결과

```
average_variable_arguments(data, 1, 7) => 45.0
average_variable_arguments(data, 1, 7, [3], True) => 44.0
average_list_with_default(data, 1, 7) => 4.5
average_list_with_default(data, 1, 7, [3], True) => 4.4
```

위 함수에서는 입력인자로 받은 `*args`를 그대로 `average_list_with_default()`의 입력인자로 전달했다. `average_list_with_default()`는 두 번째 인자부터는 기본값이 있기 때문에 값을 넣어도 되고 안 넣어도 된다. `args`는 튜플이고 튜플의 길이는 `average_variable_arguments()` 입력에 따라 달라질 수 있지만 `average_list_with_default()`는 0~4개의 인자를 (순서에 맞춰) 자유롭게 받을 수 있다. 두 함수에서  입력인자를 입력하는 방식은 비슷하다. 그럼 이걸 왜 쓰는 걸까?  

`average_variable_arguments()`에는 `average_list_with_default()`에는 없는 값을 `multiple` 배하는 데이터 처리 기능이 있다. 어떤 데이터처리를 하고 나서 `average_list_with_default()`를 호출하고 싶은데 호출하려는 함수의 입력인자가 너무 많아서 그것을 `average_variable_arguments()`에 똑같은 입력인자로 추가하는 것이 번거로울 수 있다. 내부 호출 함수의 입력인자 순서를 잘 알고 있다면 `*args`를 통해 한번에 전달하는 것이 편리할 수 있다. 하지만 함수의 입력인자가 명시적이지 않아 코드의 가독성은 떨어진다.



### 4.2 **kwargs

그런데 `*args`를 이용한 방식은 비슷한 데이터를 여러개 넣을 때는 좋지만 다른 종류의 입력인자를 여러개 넣어야 할 때는 헷갈리기 십상이다. 이런 경우 `keyword argument`를 쓸 수 있다. 함수 입력인자에 `**kwargs`를 넣으면 함수 실행시 `key=value` 형태로 받은 입력들을 `dict`로 받을 수 있게 해준다. 이것도 마찬가지로 마지막 입력인자로 들어가야 하는데 `*args`와 함께 쓸 때는 `*args`를 먼저 쓰고 마지막에 `**kwargs`를 써야 한다.

```python
def average_keyworded_args(data, multiple, **kwargs):
    print("[average_subjects_varargs] kwargs:", kwargs)
    # do some process ...
    data = [d*multiple for d in data]
    avg = average_list_with_default(data, **kwargs)
    return avg

result = average_keyworded_args(data, 10, start=1, skip=[2, 3])
print("average_keyworded_args(data, start=1, skip=[2, 3]) =>", result)
result = average_keyworded_args(data, 10, start=1, end=7)
print("average_keyworded_args(data, start=1, end=7) =>", result)
```

결과

```
[average_subjects_varargs] kwargs: {'start': 1, 'skip': [2, 3]}
average_keyworded_args(data, start=1, skip=[2, 3]) => 58.333333333333336
[average_subjects_varargs] kwargs: {'start': 1, 'end': 7}
average_keyworded_args(data, start=1, end=7) => 45.0
```

함수 내부에서 `**kwargs`를 출력해보면 dictionary 타입이라는 것을 알 수 있다. `average_keyworded_args()`의 입력인자로 들어온 `**kwargs`를 그대로 호출함수에 전달해줄 수 있다. 첫 번째 호출에서 `average_list_with_default(data, **kwargs)`는 `average_list_with_default(data, start=1, skip=[2, 3])` 하는 것과 같다. `*args`를 이용해 `average_list_with_default()`의 입력인자를 순서대로 넣어주는 것보다는 위와 같이 키워드를 이용해 입력하는 것이 코드를 읽기에 훨씬 좋다.



### 연습문제

위 예시에서 `average_multi_subjects()` 함수와 `average_keyworded_args()`를 조합해 입력한 과목들에 `multiple` 배한 점수의 평균을 `start, end, skip, verbose` 등의 입력인자를 반영하여 계산하는 함수를 구현하시오.

```python
subject_scores = {"cpp": [57, 36, 80, 53, 23],
                  "java": [46, 88, 72, 15, 54],
                  "ruby": [85, 23, 34, 91, 42]}
def average_subjects_kwargs(scores, multiple, *args, **kwargs):
	pass

result = average_subjects_kwargs(subject_scores, 10, "cpp", "java", start=1, end=4, skip=[2])
# => result = {'cpp': 580.0, 'java': 800.0}
```



## 5. 변수 범위(Scope)

변수 범위란 코드에서 변수에 접근 가능한 범위를 말한다. C언어에서는 `{}`로 묶인 범위 안에서 선언된 변수는 `{}` 안에서만 접근 가능하다. 파이썬에서는 `{}` 대신 들여쓰기로 코드 블럭을 구분하지만 블록 안에서 선언한 변수를 블록 밖에서도 쓸 수 있다. 하지만 이 경우 블록이 실행되지 않으면 변수도 생성되지 않기 때문에 에러가 날 가능성도 있다.

```python
print("\nvariable scope")
if True:
    var_created = "created"
print("variable created inside block:", var_created)
if False:
    var_not_created = "not created"
try:
    print("variable NOT created inside block:", var_not_created)
except NameError as ne:
    print(ne)
```

함수 내부에서 선언한 변수는 지역 변수(local variable)로 인식되어 함수 외부에서 쓸 수 없다. 반대로 함수 외부에서 선언된 변수는 전역 변수(global variable)로 인식되어 함수 내부에서도 쓸 수 있다. 하지만 경우에 따라 에러가 날 수도 있으므로 주의해야 한다. 

**함수 내부에서 어떤 위치건 `myvar=value`처럼 변수 이름에 값을 넣는 코드가 있으면 그 변수는 지역 변수로 인식**한다. 어떤 변수(`myvar`)를 지역 변수로 인식하는 함수가 있고 그 외부에도 `myvar`이라는 전역 변수가 있을 때 지역 변수를 생성하기 전에 전역 변수의 값을 읽으려하면 에러가 난다. 아래 예시를 통해 확인해보자.

```python
global_var = 10
def add_ten_local():
    local_var = global_var + 10
    print("add_ten_local:", local_var)

def add_ten_global():
    try:
        global_var = global_var + 10
        print("add_ten_global:", global_var)
    except NameError as ne:
        print(ne)

def add_ten_global_two_steps():
    try:
        local_var = global_var + 10
        global_var = local_var
        print("add_ten_global_two_steps:", global_var)
    except NameError as ne:
        print(ne)

def add_ten_global_use_global():
    global global_var
    global_var = global_var + 10
    print("add_ten_global_use_global:", global_var)

add_ten_local()
add_ten_global()
add_ten_global_two_steps()
add_ten_global_use_global()
print("global_var=", global_var)
```

함수 내부에서 전역 변수의 값을 바꿔주기 위해서는 반드시 `add_ten_global_use_global()` 처럼 `global`이란 키워드를 이용해 함수 내부에서 읽고 쓸수 있는 변수로 지정해줘야한다.



## 6. 바람직한 파이썬 코딩 스타일

파이썬 코딩을 할 때 보통은 지금까지 해온 것 처럼 함수밖에 바로 코드를 쓰진 않는다. 함수 밖에 생성된 변수들은 모두 전역 변수(global variable)로 인식되어 함수 내부 변수들과 섞여서 버그를 유발할 가능성이 높다. **따라서 모든 코드는 함수 안에 있어야한다.** 스크립트에서 실행될 메인 함수를 먼저 만들고 함수 밖에는 메인 함수를 호출하는 코드만 넣는다. 메인 함수 안에서는 코드를 기능별로 묶어서 세부 함수로 나누어 세부 함수들을 호출한다. 다음 예시를 따라해 보자.

```python
def main():
    data = [1, 2, 3, 4, 5, 6, 7, 8, 9]
    result = average_keyworded_args(data, 10, start=1, skip=[2, 3])
    print("average_keyworded_args(data, start=1, skip=[2, 3]) =>", result)
    result = average_keyworded_args(data, 10, start=1, end=7)
    print("average_keyworded_args(data, start=1, end=7) =>", result)

def average_keyworded_args(data, multiple, **kwargs):
    data = [d*multiple for d in data]
    avg = average_list_with_default(data, **kwargs)
    return avg

def average_list_with_default(data, start=0, end=None, skip=None, verbose=False):
    if skip is None:
        skip = []
    return average_list(data, start, end, skip, verbose)

def average_list(data, start, end, skip, verbose):
    if end is None:
        avg_data = data[start:]
    else:
        avg_data = data[start:end]

    sum = 0
    for ind, num in enumerate(avg_data):
        if ind not in skip:
            sum += num
    dlen = len(avg_data) - len(skip)
    average = sum / dlen
    if verbose:
        print(f"average {start}~{end} with skipping {skip} = {average}")
    return average

if __name__ == '__main__':
    main()

```

맨 아래 `if __name__ == '__main__':` 이라는 조건문이 보이는데 이것은 이 스크립트 파일이 직접 실행이 된 것인지를 확인하는 것이다. 나중에 나오겠지만 여러 파일이 연계되서 실행될 때 이 스크립트가 다른 스크립트에서 사용되는 스크립트인지 주로 실행되는 스크립트인지를 확인하는 것이다. 주로 실행되는 스크립트인지를 확인하고 `main()` 함수를 실행한다. `main()` 함수는 다른 함수들을 호출한다. **호출하는 함수는 호출 받는 함수 위에 있어야** 위에서부터 아래로 코드를 쉽게 읽을 수 있다. **신문처럼 자연스럽게 읽을 수 있는 코드가 좋은 코드다.**   

강의에서는 문법을 익히는데 필요한 코드만 보여주기 위해 함수 밖에 바로 코드를 쓰는 예시를 보여줄 수도 있지만 앞으로 숙제, 프로젝트 등을 할 때는 반드시 위와 같은 스타일로 스크립트를 작성해야 한다.

