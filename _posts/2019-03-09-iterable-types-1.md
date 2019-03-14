---
layout: post
title:  "Python Data Structure (1): List"
date:   2019-03-09 09:00:13
categories: 2019-1-systprog
---


# 파이썬 데이터 구조

파이썬에는 네 가지 데이터 구조가 존재한다. `List, Dictionary, Tuple, Set` 이다. 이 네 가지 기본 자료구조는 파이썬이 가진 편리함의 원천이다. 파이썬에서 가장 많이 쓰이는 패키지인 `numpy`와 `pandas`도 기본 자료구조로부터 시작하게 되어있으므로 기본 자료 구조를 익숙하게 쓸 수 있어야 고급 패키지들도 잘 활용할 수 있다. 네 가지 자료형은 List > Dictionary >> Tuple > Set 순서로 많이 쓰인다. 특히 List와 Dictionary는 많이 쓰이기도 하고 정말 쉽고 유용하기 때문에 잘 알아두도록 하자.

## 1. List

`List`는 말 그대로 여러개의 데이터를 목록(list)처럼 담아둘 수 있는 자료형이다. 그런데 파이썬 자체가 동적 타입이기 때문에 타입에 상관없이 아무 자료나 담을 수 있다. 리스트 안에 리스트를 담을 수도 있고 이 후에 배울 Dictionary 나 클래스 객체 등 어떤 데이터든 담을 수 있다. 

```python
empty_list1 = []
empty_list2 = list()
basic_list = ["Hello", 1234, 1.234, True]
depth2_list = ["Hello", 1234, [1.234, True]]
depth3_list = [["Hello"], [1234, [1.234, True]]]
```

`empty_list1`, `empty_list2`는 비어있는 리스트를 만드는 두 가지 방법이다. ~~파이썬의 철학은 어디에...~~  `basic_list`은 네 가지 기본 자료형으로 만든 리스트이고 `depth2_list`, `depth3_list`은 리스트 안에 리스트를 담은 것이다.

### 1.1 리스트 인덱싱과 슬라이싱

리스트를 다루는 방법은 앞서 문자열을 다루는 방법과 비슷하다. (사실은 문자열이 리스트처럼 구현된 것이지만) 리스트에서 특정 원소를 출력하는 인덱싱(Indexing)은 `[]`(braket operator)를 쓴다. 문자열처럼 뒤에서부터 음수로 인덱싱도 가능하다. 리스트가 여러겹으로 겹쳐있다면 원하는 원소가 나올때까지 단계별로 인덱스를 붙이면 된다. 마지막 줄처럼 인덱스가 리스트 범위를 넘어가면 에러가 발생하므로 주의해야 한다.  

```python
print("\nIndexing: 1.234에 접근하기")
print("basic indexing:", basic_list[2])
print("negative indexing:", basic_list[-2])
print("first indexing:", depth2_list[2])
print("second indexing:", depth2_list[2][0])
print("first indexing:", depth3_list[1])
print("second indexing:", depth3_list[1][1])
print("third indexing:", depth3_list[1][1][0])
try:
    print(depth2_list[5])
except IndexError as ie:
    print("IndexError:", ie)
```

결과

```
Indexing: 1.234에 접근하기
basic indexing: 1.234
negative indexing: 1.234
first indexing: [1.234, True]
second indexing: 1.234
first indexing: [1234, [1.234, True]]
second indexing: [1.234, True]
third indexing: 1.234
IndexError: list index out of range
```

인덱싱이 리스트의 원소를 출력한다면 슬라이싱(Slicing)은 리스트에서 일부 원소들을 묶어 **새로운 리스트로 출력**한다. 마찬가지로 음수를 이용해 뒤에서부터 슬라이싱 범위를 지정할 수도 있다. 슬라이싱의 경우 슬라이싱 범위가 인덱스 범위를 넘어가더라도 슬라이싱 범위 안의 데이터가 있으면 그만큼만 출력하고 없으면 빈 리스트(`[]`)를 출력한다.

```python
print("\nSlicing")
print("[start:end]", basic_list[1:3])
print("[start:]", basic_list[2:])
print("[:end]", basic_list[:2])
print("[start:negative_end]", basic_list[1:-1])
print("[:negative_end]", basic_list[:-2])
print("[negative_start:negative_end]", basic_list[-4:-2])
print("[nested list1]", depth2_list[1:3])
print("[nested list2]", depth3_list[0:2])
print("[partially overlap]", basic_list[2:10])
print("[out of range]", basic_list[5:10])
```

결과

```
Slicing
[start:end] [1234, 1.234]
[start:] [1.234, True]
[:end] ['Hello', 1234]
[start:negative_end] [1234, 1.234]
[:negative_end] ['Hello', 1234]
[negative_start:negative_end] ['Hello', 1234]
[nested list1] [1234, [1.234, True]]
[nested list2] [['Hello'], [1234, [1.234, True]]]
[partially overlap] [1.234, True]
[out of range] []
```

마지막 실행 결과를 보면 `basic_list[2:10]`은 2번부터 9번까지의 데이터를 출력해야 하는데 `basic_list`가 3번까지 밖에 없으므로 2, 3번 원소만 출력한다. `basic_list[5:10]`는 슬라이싱 범위 전체가 `basic_list`의 범위를 벗어나 있으므로 빈 리스트(`[]`)를 출력한다.  

#### 연습문제

1)  `depth2_list`에서 `[1.234]`를 출력해 보세요. (주의: 1.234가 아닌 [1.234] 입니다.)

### 1.2 리스트 연산

리스트 역시 `+`를 통해 합칠 수 있고 `*`를 통해 반복할 수 있다.

```python
mammal = ["dog", "cat", "human"]
reptile = ["snake", "lizard", "frog"]
bird = ["eagle", "sparrow", "chicken"]
animal = mammal + reptile + bird
print("\nlist concatenation")
print("animal:", animal)
# => animal: ['dog', 'cat', 'human', 'snake', 'lizard', 'frog', 'eagle', 'sparrow', 'chicken']
```

예를 들어 여러 사람의 여러 점수를 여러 회차에 걸쳐 기록해야 한다고 할 때 표(table)의 세로축은 회차가 되고 가로축에는 사람의 이름과 점수의 종류가 나타나야 한다. 즉 테이블의 윗 줄에는 다음과 같은 목록이 있어야 한다.

```
['나연', '나연', '나연', '정연', '정연', '정연', '지효', '지효', '지효']
['vocl', 'dance', 'rap', 'vocl', 'dance', 'rap', 'vocl', 'dance', 'rap']
```

이러한 리스트는 다음 코드로 만들수 있다. `members[0]`이 아닌 `members[0:1]`을 쓰는 이유는 문자열 `"나연"`이 아닌 리스트 형태의 `["나연"]`을 반복하기 위해서다.

```python
members = ["나연", "정연", "지효"]
tests = ["vocl", "dance", "rap"]
first_row = members[0:1]*3 + members[1:2]*3 + members[2:3]*3
second_row = tests*3
print("\nlist repetition")
print(first_row)
print(second_row)
```

### 1.3 리스트 관련 함수

- `len()`:  파이썬 내장 함수로 여러 원소를 담고 있는 대부분의 객체들은 이 함수로 길이(갯수)를 잴 수 있다. 문자열(str)의 길이, `list, dict, tuple, set` 등의 자료구조의 원소 수를 읽을 수 있다.

```python
string = "Hello"
print("\nlen of {}:".format(string), len(string))
# => len of Hello: 5
mylist = [1, 2, 3, 4]
print("len of {}:".format(mylist), len(mylist))
# => len of [1, 2, 3, 4]: 4
```

- `del`: 객체를 삭제하는 함수로 자료 구조에서 특정 원소를 삭제할 때 쓰인다. `del()`이 아님에 유의하자.

```python
mylist = [1, 2, 3, 4, 5]
del mylist[2]
print("\nafter deleting [2]:", mylist)
# => after deleting [2]: [1, 2, 4, 5]
del mylist[2:]
print("after deleting [2:]:", mylist)
# => after deleting [2:]: [1, 2]
```

- 원소 변경: 인덱싱이나 슬라이싱으로 잡은 리스트 범위에 `=` operator로 원소들을 수정할 수 있다. 참고로 문자열은 `=`를 이용한 문자 수정이 안된다. 대신 `replace()`나 슬라이싱을 이용하자.

```python
mylist = [1, 2, 3, 4, 5]
mylist[0] = "Life"
print("\nchange element by indexing:", mylist)
# => change element by indexing: ['Life', 2, 3, 4, 5]
mylist[1:4] = ["is", "too", "short"]
print("\nchange elements by slicing:", mylist)
# => change elements by slicing: ['Life', 'is', 'too', 'short', 5]
```

- `join()`: 리스트의 내부 문자열 원소들을 하나의 문자열로 연결해준다. 연결할 때 각 문자열 사이에 `" "` 사이에 들어있는 문자열을 끼워 넣어준다.

```python
print("\njoin strings")
path = ["/home", "ian", "work", "ian-lecture"]
print("joined path:", "/".join(path))
# => joined path: /home/ian/work/ian-lecture
time = ["13", "20", "30"]
print("joined time:", ":".join(time))
# => joined time: 13:20:30
```

- `in`: `in`은 함수가 아니라 operator다. 리스트에 특정 원소가 들어있는지 확인할 때 쓴다.

```python
print("\n'in' operator")
twice = ["나연", "정연", "모모", "사나", "지효", "미나", "다현", "채영", "쯔위"]
if "채영" in twice:
    print("채영은 트와이스 입니다.")
if "채령" not in twice:
    print("채령은 트와이스가 아닙니다.")
```

### 1.4 리스트 내장 함수

리스트를 쓰다보면 (크기순, 알파벳순) 정렬을 한다던지 (`sort`), 원소를 추가한다던지 (`append`), 중간에 삽입한다던지 (`insert`), 특정 원소의 인덱스(위치)를 반환한다던지(`index`) 등의 기능이 필요하다. 함수명을 굳이 외울 필요없이 해당기능을 영어로 써보면 그 함수가 이미 있다. 다음 예시를 보며 함수명들을 익혀보자. 리스트 함수들을 쓸 때 주의할 점은 대부분의 함수들이 `in-place` 함수라는 것이다. 함수로 수정한 리스트 결과가 리턴으로 나오지 않고 함수를 실행한 변수 자체를 수정한다는 것이다. 하지만 대부분의 외부 패키지들은 데이터를 수정하는 함수들이 원본은 그대로 둔채 수정된 결과를 리턴하므로 잘 구분해서 써야한다.

```python
print("\nlist functions")
tottenham = ['Kane', 'Moura', 'Lloris', 'Sissoko', 'Alli', 'Rose']
print("Tottenham vs Southampton 2019-03-10 starting line up: \n", tottenham)
# sort(): 원소 알파벳순, 크기순 정렬, in-place 함수기 때문에 아무것도 리턴하지 않는다.
print("sort() is in-place function:", tottenham.sort())
print("sort by name:", tottenham)
# remove(): 입력한 원소를 삭제
tottenham.remove('Moura')
# insert(): 원하는 위치에 원소 삽입
tottenham.insert(1, 'Son')
print("At 72, Moura out Son in:", tottenham)
# pop(): 입력이 없으면 마지막 원소를 삭제하고 pop(index)는 index의 원소를 삭제한다.
print("At 82, pop Alli:", tottenham.pop(0))
print("At 82, del Rose:", del tottenham[-2])
# append(): 원소를 마지막에 추가한다. 두 리스트의 원소들을 합칠 때는 +나 extend()를 쓴다.
tottenham.append('Davies')
tottenham.append('Llorente')
print("At 82, Davies and Llorente in:", tottenham)
# reverse(): 순서를 거꾸로 뒤집는다.
tottenham.reverse()
print("reverse order", tottenham)
```

결과
```
list functions
Tottenham vs Southampton 2019-03-10 starting line up: 
 ['Kane', 'Moura', 'Lloris', 'Sissoko', 'Alli', 'Rose']
sort() is in-place function: None
sort by name: ['Alli', 'Kane', 'Lloris', 'Moura', 'Rose', 'Sissoko']
At 72, Moura out Son in: ['Alli', 'Son', 'Kane', 'Lloris', 'Rose', 'Sissoko']
At 82, pop Alli: Alli
At 82, del Rose
At 82, Davies and Llorente in: ['Son', 'Kane', 'Lloris', 'Sissoko', 'Davies', 'Llorente']
reverse order ['Llorente', 'Davies', 'Sissoko', 'Lloris', 'Kane', 'Son']
```

#### 연습문제

1) `wdgirls_debut`에서 멤버를 삭제, 추가하여 `wdgirls_final`을 만들어 보세요.

```python
wdgirls_debut = ["선예", "예은", "소희", "현아", "선미"]
# => wdgirls_final = ["예은", "선미", "유빈", "혜림"]
```

### 1.5 반복문(for)과 리스트

`for`문과 리스트는 뗄레야 뗄수 없는 사이기에 리스트와 함께 배우는 것이 좋다. `for`문은 다음과 같이 쓴다.
```python
for e in somthing_iteratable:
    statement1
    statement2
```

`if`와 마찬가지로 `for`로 시작하는 줄은 `:`로 끝나고, `for`에 의해 반복되는 블럭은 들여쓰기로 구분한다. `in`을 기준으로 앞에는 원소 변수(`e`)를 쓰고 뒤에는 반복가능한(iterable) 객체를 쓴다. 여기서 iterable 하다는 것은 정확히 말하면 `iter` 객체로 변환할 수 있는 객체고 `iter` 객체는 `next()`함수를 통해 원소를 하나씩 꺼낼 수 있다. 이것이 반복문의 내부 원리다.
```python
itzy = ["예지", "리아", "류진", "채령", "유나"]
print("convert to iter object")
itzy = iter(itzy)
try:
    print("next1:", next(itzy))
    print("next2:", next(itzy))
    print("next3:", next(itzy))
    print("next4:", next(itzy))
    print("next5:", next(itzy))
    print("next6:", next(itzy))
except StopIteration:
    print("error: iterator finished")
```

여기서 배우는 `list, dict, tuple, set` 모두 여러 원소를 담고 있는 iterable 객체이기 때문에 `for`문을 통해서 리스트의 원소들을 하나씩 처리할 수 있다.
```python
print("square numbers")
numbers = [1, 2, 3, 4, 5]
for n in numbers:
    print("square of {}  is {}".format(n, n*n))
```

`for`를 사용하는 다양한 기법들은 다음에 더 공부해보기로 하자.