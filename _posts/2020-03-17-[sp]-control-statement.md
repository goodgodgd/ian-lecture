---
layout: post
title:  "[Python] Control statements: if, for and while"
date:   2020-03-17 09:00:13
categories: 2020-1-systprog
---



## 1. Basic Usages

이전 장에서도 배웠듯이 파이썬에도 `if, for, while`이 있다. 특히 파이썬은 자료구조형과 밀접하게 연계되어 자료구조에 들어있는 자료들에 대해 쉽게 반복문을 실행할 수 있다. 제어문의 기본 용법과 편리하게 쓸 수 있는 다양한 문법들을 배워보자. `if, for, while`의 기본 용법은 다음과 같다.

```
if condition1:
	statements_when_condition1_is_true
elif condition2:
	statements_when_condition2_is_true
else:
	statements_when_no_condition_is_true

for elem in something_iterable:
	statements_to_process_elem

while condition:
	statements_while_condition_is_true
```

마블과 DC의 영웅들~~아니고 돈벌이 캐릭터~~로 `if, for, while`의 기본 용법을 배워보자. 

```python
marvel_heroes = ["iron man", "thor", "hulk", "spider man", "black widow", "capt. america", "capt. marvel"]
dc_heroes = ["batman", "superman", "aquaman", "wonder woman", "harley quinn"]
all_heros = marvel_heroes + dc_heroes

myhero = "batman"
if myhero in marvel_heroes:
    print(f"{myhero}: Avengers are super cool!")
elif myhero in dc_heroes:
    print(f"{myhero}: We save martha...")
else:
    print(f"{myhero}: We are villains!!")

print("list of marvel heroes")
for hero in marvel_heroes:
    print(f"    {hero}")

print("list of dc heroes")
for i in range(len(dc_heroes)):
    print(f"{i})  {dc_heroes[i]}")

i = 0
print("DC's male heroes ends with 'man'")
while i < len(dc_heroes) and dc_heroes[i].endswith('man'):
    print(f"    {dc_heroes[i]}")
    i += 1
```



## 2. 반복문 제어: `continue, break`

반복문을 돌리다 보면 조건에 따라 중간에 처리를 건너뛰고 싶거나 그만두고 반복문을 종료하고 싶을 때가 있다. `continue`는 반복문에서 `continue` 이후의 과정을 건너뛰고 다음 loop으로 넘어가는 것이고, `break`는 반복문 자체를 끝낸다.

```python
print("list of marvel heroes")
for hero in marvel_heroes:
    if hero.startswith("spider"):
        print("    Peter Parker by Tobey Maguire was not a kid: \"With great power comes great responsibility.\"")
        continue
    if hero.startswith("capt"):
        print("    one captain is enough. let me stop here")
        break
    print(f"    {hero} is cool")

name = None
print("Press 'q' to quit")
while name != 'q':
    print("type dc hero's name")
    name = input()
    if name is 'q':
        break
    if name not in dc_heroes:
        print(f"{name} is not dc hero")
        continue
    index = dc_heroes.index(name)
    print(f"{name}'s index =", index)
```



## 3. for 응용: `enumerate, zip`

파이썬에서는 `while`보다 `for`문의 활용법이 다양하다. 대표적으로 `enumerate, zip`이 있다. `enumerate`는 `for`문에서 리스트를 반복할 시 원소 뿐만 아니라 인덱스도 받을 수 있는 반복 객체를 만들어 준다. `zip`은 두 개의 리스트를 묶어서 각 리스트의 원소가 하나씩 합쳐진 튜플의 반복 객체를 만들어준다. 이들을 리스트로 변환해보면 무엇을 주는지 명확히 볼 수 있다.
```python
print("\nwhat does enumerate return?", enumerate(marvel_heroes))
print(list(enumerate(marvel_heroes)))
print("what does zip return?", zip(marvel_heroes, dc_heroes))
print(list(zip(marvel_heroes, dc_heroes)))
```

```python
print("\nwhat does enumerate return?", enumerate(marvel_heroes), list(enumerate(marvel_heroes)))
print("print only first 3 heroes with index")
for index, name in enumerate(marvel_heroes):
    if index >= 3:
        break
    print("marvel hero:", index, name)

print("\nwhat does zip return?", zip(marvel_heroes, dc_heroes), list(zip(marvel_heroes, dc_heroes)))
print("print pairs of marvel and dc heroes")
for mv, dc in zip(marvel_heroes, dc_heroes):
    print("{} vs {}".format(mv, dc))

print("\nprint only 3 pairs of marvel and dc heroes")
for ind, (mv, dc) in enumerate(zip(marvel_heroes, dc_heroes)):
    if ind >= 3:
        break
    if ind % 2 == 1:
        print("{0} vs {1}: {0} win!".format(mv, dc))
    else:
        print("{0} vs {1}: {1} win!".format(mv, dc))
```



## 4. List Comprehension

리스트를 다루다 보면 다른 리스트에 기반해서 새로운 리스트를 만들거나 기존 리스트에서 필요한 것만 걸러서 새로운 리스트를 만들고 싶을 때가 있다. 일단 위에서 배운 내용을 토대로 이를 구현해보자.
```python
print("\ncreate a new list from the existing list")
super_heroes = []
for hero in marvel_heroes:
    super_heroes.append(hero + "_super")
print("super heroes", super_heroes)

print("\nextract a subset of the existing list")
man_heroes = []
for hero in marvel_heroes:
    if hero.endswith("man"):
        man_heroes.append(hero)
print("man heroes:", man_heroes)
```

파이썬에는 리스트를 선언하는 `[]` 안에 `for`문을 넣어 저 두 개의 `for`문을 한 줄로 처리하는 방법이 있는데 이를 `list comprehesion`이라고 한다.
```python
# [process item for item in list]
print("\ncreate a new list by list comprehension")
super_heroes = [hero + "_super" for hero in marvel_heroes]
print("super heroes", super_heroes)

print("\nextract a subset by conditioned list comprehension")
man_heroes = [hero for hero in marvel_heroes if hero.endswith("man")]
print("man heroes:", man_heroes)

print("\nsquare of integers")
int_square = [i ** 2 for i in range(10)]
print(int_square)
```

`list comprehension`은 앞에 간단한 처리과정을 넣어서 다양하게 쓰일 수 있고 다중 `for`문이 들어갈 수도 있다. 이러한 기능을 `list`만 쓸 수 있는 건 아니고 `dictionary`와 `set`에도 적용될 수 있다.

```python
print("\ndictionary comprehension")
abilities = ["suit", "Mjölnir", "physical power", "spider web"]
heroes = {name: power for name, power in zip(marvel_heroes, abilities)}
print("hero's ability", heroes)
```



## HW1. 파이썬 자료구조 연습

다음 코드의 주석들을 구현하는 코드를 작성하시오.

```python
heroes = ["iron man", "thor", "spider man", "hulk", "captain america", "hawkeye"]
print("original heroes:", heroes)
# 1. 히어로들을 알파벳 역순으로 정렬하시오.

print("1. sorted heroes:", heroes)

# 2. 아래 new_heroes에서 첫번째와 마지막 히어로를 제거하고 captain marvel과 black widow을 추가하시오.
new_heroes = [hero for hero in heroes]

print("2. change heroes:", new_heroes)

# 3. for문을 이용하여 hereos 중 'man'으로 끝나는 히어로 이름에서 'man'을 'woman'으로 바꾼 heroines를 만드시오.
heroines = []

print("3. transgender heroines:", heroines)

# 4. list comprehension을 이용하여 heroines 중 'woman'이 들어간 히어로 이름을 'human'으로 바꾼 neutralized_heroes를 만드시오.

print("4. neutralized heroes:", neutralized_heroes)

# 5. heroes에서 enumerate를 이용하여 홀수번째 히어로만 뽑아낸 새 리스트 odd_heroes를 만드시오.

print("5.2 odd heroes", odd_heroes)

# 6. for문에 zip 함수를 써서 hero 중 앞의 4명 이름과 배우 이름을 딕셔너리로 묶은 hero_actors를 만들어 보시오.
actor_names = ["햄식이", "톰 홀랜드", "로다주", "마크 러펄로"]

print("6. hero actors", hero_actors)
print("6. thor was played by", hero_actors["thor"])
print("6. spider man was played by", hero_actors["spider man"])

# 7. 2의 new_heroes를 연기한 배우들의 이름을 6의 hero_actors를 이용해 출력하시오.
# 이름이 있으면 6번과 같이 이름을 출력하고 없으면 "?? has no name"을 출력하시오.

```


