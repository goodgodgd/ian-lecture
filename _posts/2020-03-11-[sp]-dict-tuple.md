---
layout: post
title:  "[Python] Data Structure 2: Dictionary, Tuple, Set"
date:   2019-03-09 09:00:13
categories: 2019-1-systprog
---

## 2. Dictionary

데이터를 다루다 보면 단순히 자료를 나열하는 리스트로는 충분하지 않을 때가 있다. 리스트는 같은 속성의 자료를 여러개 가지고 있을 때는 유용하지만 다른 속성의 자료를 리스트에 넣으면 코드를 읽기가 힘들어진다. 

```python
good_for_list = ["pooh", "tigger", "piglet", "rabbit"]
pooh = ["bear", 5, 50]
tigger = ["tiger", 4, 40]
print("list based data management")
print("pooh's species is", pooh[0])
print("pooh's weight is", pooh[2])
print("tigers's age is", tigger[1])
```

첫 줄의 `good_for_list`는 리스트로 저장하기 좋은 자료다. 모두가 이름을 나타내고 있다. 하지만 그 아래를 보라. 자료를 이렇게 저장하면 순서를 똑같이 맞추지 못 할 경우 잘못된 결과를 나타내기 쉽고 코드만 봐서는 왜 [0]이 종이고 [1]이 나이인지 이해하기 어렵다. 이러한 대응관계를 명시적으로 보여줄 수 있는 자료 구조가 딕셔너리(`Dictionary`)다. 사전에서 어떤 단어를 찾으면 그 단어에 대산 설명이 나오듯이 `Dictionary`는 키(Key)를 입력하면 그에 해당하는 값(Value)를 출력해주는 자료 구조다. 리스트는 `[]`(bracket)으로 만들었지만 딕셔너리는 `{}`(brace)로 만든다. `:`(colon)을 사이에 두고 앞에 Key 값을 쓰고 뒤에 Value 값을 쓴다. Key:Value 쌍은 쉼표(,)로 구분한다. 값을 꺼낼때는 리스트, 딕셔너리 모두 `[]`(bracket)를 쓰고 리스트는 인덱스 숫자를 쓰지만 딕셔너리는 키 값을 입력한다.

```python
mydict = {Key1:Value1, Key2:Value2, ...}
print("call value from key1", mydict[Key1])
```

곰돌이 푸우 예제를 이해하기 쉽게 다시 써보자. 타이핑하는데 손은 더 들지만 코드를 명시적으로 이해할 수 있다. 특히 여럿이 같이 작업할 때는 남들이 편하게 이해할 수 있게 쓰는 것이 중요하다.

```python
pooh = {"species": "bear", "age": 5, "weight": 50}
tigger = {"species": "tiger", "age": 4, "weight": 40}
print("\ndict based data management")
print("pooh's species is", pooh["species"])
print("pooh's weight is", pooh["weight"])
print("tigers's age is", tigger["age"])
```

딕셔러너리를 만들 때 `Value`에는 어떤 객체가 들어가도 상관없다. 리스트가 들어갈 수도, 또 다른 딕셔너리가 들어가도 된다. 하지만 `Key`는 숫자나 문자열 (혹은 그 변수)을 써야한다. 딕셔너리는 `Key`로 들어온 값을 `hash()` 함수를 통해 hash를 생성하고 이를 기록해 두었다가 특정 키를 찾을 때 활용한다. `Key` 값을 직접 비교하지 않고 hash로 변환해서 검색하기 때문에 검색 속도가 빠르다. 다음 예시를 실행해보자.

```python
print("\nhash example")
print("hash of 1:", hash(1))
print("hash of python:", hash("python"))
try:
    print("hash of []:", hash([]))
except TypeError as te:
    print("[TypeError]", te)
```

이제 딕셔너리의 활용법에 대해 더 알아보자.

### 2.1  기본 사용법

딕셔너리의 기본 특징은 **순서가 없다**는 것이다.(Unordered) 보통은 입력한 순서가 그대로 유지되는 것 처럼 보이지만 값을 넣고 빼고 하다보면 `print()` 했을 때 순서가 뒤바뀔 때도 있다. 사실 딕셔너리는 오직 `Key`로만 `Value`에 접근할 수 있으니 순서는 상관없다. 다음 예제를 통해 기본 사용법을 익혀보도록 하자.

```python
print("\nbasic usage")
pooh = {"species": "bear", "age": 5, "weight": 50}
# 데이터 읽기: 특정 `key`에 연결된 `Value`를 읽기 위해서는 `[Key]`하면 된다.
print("pooh's age is", pooh["age"])
# 데이터 수정: 특정 `key`에 연결된 `Value`를 수정할 때는 그냥 값을 키에 넣으면 된다.
pooh["age"] = 10
print("pooh's age is", pooh["age"])
# 데이터 추가: 이미 만들어진 딕셔너리에 `Key:Value` 쌍을 추가하는 방법 역시 그냥 값을 키에 넣으면 된다.
pooh["height"] = 1.2
print("pooh:", pooh)
# 데이터 삭제: 리스트처럼 `del`을 이용한다.
del pooh["weight"]
print("pooh:", pooh)
```
결과
```
basic usage
pooh's age is 5
pooh's age is 10
pooh: {'species': 'bear', 'age': 10, 'weight': 50, 'height': 1.2}
pooh: {'species': 'bear', 'age': 10, 'height': 1.2}
```

만약 없는 키를 입력하면 `KeyError`가 발생한다. 해당 키가 있는지 확신이 없을 때는 `get()` 함수를 쓰거나 `in` 연산자를 통해 키의 존재유무를 확인 후 쓸 수도 있다. `get()`을 쓰면 키가 없을 경우 `None`을 출력하는데 `None`은 말 그대로 아무것도 아니라는 뜻으로 True, False 처럼 파이썬에 지정된 키워드다.

```python
pooh = {"species": "bear", "age": 5, "weight": 50}
try:
    print("\ntry non-existing key")
    print("pooh's color?", pooh["color"])
except KeyError as ke:
    print("[KeyError]", ke)

print("pooh's color?", pooh.get("color"))
if "color" in pooh:
    print("pooh's color is", pooh["color"])
else:
    print("pooh has no color")
```
결과
```
try non-existing key
[KeyError] 'color'
pooh's color? None
pooh has no color
```

### 2.2 관련 함수

딕셔너리를 쓰다보면 Key만 혹은 Value만 따로 쓰고 싶을 때가 있다. 예를 들어 딕셔너리를 `"이름":"점수"` 형태로 만들었는데 명단만 필요하거나 점수만 필요한 경우가 있다. 혹은 이들을 `Key:Value` 관계가 아닌 동등만 데이터 쌍(tuple)으로 받고 싶을 수도 있다. 다음 예제에서 그 답을 찾아보자.
```python
print("\ndict functions")
scores = {"pooh": 80, "tigger": 70, "piglet": 90, "rabbit": 85}
print("names:", scores.keys())
print("scores:", scores.values())
print("items:", scores.items())
```
결과
```
dict functions
names: dict_keys(['pooh', 'tigger', 'piglet', 'rabbit'])
scores: dict_values([80, 70, 90, 85])
items: dict_items([('pooh', 80), ('tigger', 70), ('piglet', 90), ('rabbit', 85)])
```

각 결과는 리스트처럼 보이지만 `dict_keys`, `dict_values`, `dict_items` 라는 다른 데이터 형식이다. 이들은 언제든 `list()`로 감싸면 쉽게 리스트로 변환할 수 있다.
```python
print("names:", list(scores.keys()))
print("scores:", list(scores.values()))
print("items:", list(scores.items()))
```
결과
```
names: ['pooh', 'tigger', 'piglet', 'rabbit']
scores: [80, 70, 90, 85]
items: [('pooh', 80), ('tigger', 70), ('piglet', 90), ('rabbit', 85)]
```

딕셔너리 자체로는 `for`문을 통해 반복할 수 없고 `Key`가 필요한지, `Value`가 필요한지, 둘 다 필요한지에 따라 맞는 함수를 통해 반복가능한(iterable) 자료형으로 바꾸어 쓰면 된다.
```python
print("\niterate over keys")
for name in scores.keys():
    print("name:", name)
for score in scores.values():
    print("score:", score)
for name, score in scores.items():
    print("name:score:", name, ":", score)
```

## 3. Tuple

튜플(Tuple)을 한 마디로 말하면 수정불가능한 리스트다. 리스트는 `[]`, 딕셔너리는 `{}`로 만들었다면 튜플은 `()`(parenthesis)로 만든다. 혹은 `()`를 생략해도 된다. 튜플은 생성만 할 뿐 원소를 수정하거나 삭제할 수 없다. 생성된 튜플에서는 값을 읽을 수만 있는데 리스트와 동일하게 인덱싱과 슬라이싱을 통해 읽을 수 있다.

```python
print("\nHow to use tuple")
empty_tuple1 = ()
empty_tuple2 = tuple()
basic_tuple1 = ("Hello", 1234, 1.234, True)
basic_tuple2 = "Hello", 1234, 1.234, True
depth2_tuple = ("Hello", 1234, (1.234, True))
print("read tuple", basic_tuple1[0])
print("read tuple", basic_tuple2[1])
print("read tuple", basic_tuple[:3])
```

튜플은 여러 값을 하나에 담았다가 다시 여러 변수에 나눠줄 수 있어서 경우에 따라 유용하게 쓸 수 있다.

```python
print("\ndistribute values")
pooh = "pooh", "bear", 5, 50
name, species, age, weight = pooh
print("tupled pooh info:", name, species, age, weight)
```

## 4. Set

집합(set) 자료 구조는 중복을 허용하지 않는다. 어떤 리스트나 튜플에 중복된 자료가 있을 때 이를 집합으로 변환하면 중복을 없애고 유일한 자료만 남길 수 있다. 예를 들어 주차장 입구에서 차종을 인식하고 들어오는 순서대로 차종을 기록한다고 해보자. 나중에 어떤 차종이 있었는지 종류만 알고 싶다면 `set()`을 써보자.
```python
print("\nHow to use set")
entrance_order = ["k3", "aventador", "k3", "a6", "cayenne", "a6"]
car_set = set(entrance_order)
print("car set:", car_set)
# => car set: {'a6', 'aventador', 'k3', 'cayenne'}
```

결과를 보면 중복이 사라졌을 뿐만 아니라 순서도 바뀐것을 알수 있다. 집합도 딕셔너리와 마찬가지로 순서가 없다. 어떤 데이터가 있는지 없는지를 확인하기 위한 자료 구조기 때문에 순서는 중요하지 않다. 집합은 자주 쓰이진 않지만 두 집합의 교집합, 합집합, 차집합을 구할 때 유용하게 쓰인다.
```python
print("\nset example: companies")
large_comps = {"samsung", "hyundai", "lg", "sk"}
motor_comps = {"hyundai", "kia", "gm"}
print("intersection set by &:", large_comps & motor_comps)
print("intersection set by function:", large_comps.intersection(motor_comps))
print("union set by |:", large_comps | motor_comps)
print("union set by function:", large_comps.union(motor_comps))
print("difference set by -:", large_comps - motor_comps)
print("difference set by function:", large_comps.difference(motor_comps))
```

결과

```
set example: companies
intersection set by &: {'hyundai'}
intersection set by function: {'hyundai'}
union set by |: {'lg', 'kia', 'hyundai', 'samsung', 'sk', 'gm'}
union set by function: {'lg', 'kia', 'hyundai', 'samsung', 'sk', 'gm'}
difference set by |: {'lg', 'sk', 'samsung'}
difference set by function: {'lg', 'sk', 'samsung'}
```

