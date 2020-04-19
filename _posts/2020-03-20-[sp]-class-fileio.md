---
layout: post
title:  "[Python] Class and File IO"
date:   2020-03-20 09:00:13
categories: 2019-1-systprog
---



# Python Class

프로그래밍을 하다 보면 어떤 변수를 함수 내부에서 한 번 쓰고 버려도 되는 게 있고 어떤 변수는 지속적으로 사용되거나 업데이트 된다. 함수가 끝나도 어떤 변수가 사라지지 않고 유지가 되게 하려면 **클래스(class)**를 써야 한다. 객체지향프로그래밍(OOP, Object Oriented Programming) 언어에서 클래스는 데이터(=변수)와 그 데이터를 처리하는 메소드(=함수)를 합쳐 **객체(instance)**를 만든다.  

클래스는 프로그래밍에서 데이터를 객체화(instantiate)하고 비슷한 데이터와 함수의 조합을 다양하게 재사용할 수 있도록 돕는다. 일반적으로 객체지향적인 방식으로 클래스를 사용하면 코드의 가독성이 좋아지고 코드의 수정/변경이 쉬워진다. 여기서는 객체지향 프로그래밍까지는 다루지 않고 파이썬의 클래스 문법만 살펴본다.

파이썬에서 클래스는 `class`라는 키워드로 시작하며 함수와 마찬가지로 들여쓰기로 클래스 정의의 범위를 지정한다. 간단한 예시를 통해 클래스 정의 방법을 알아보자. 예를 들어 어떤 게임에서 강아지 캐릭터가 있다면 이름과 위치에 대한 *데이터*가 있을거고 짖거나 이동하는 *동작*이 있을 것이다. 이를 객체로 구현하면 다음과 같다.

```python
class Dog:
    def __init__(self, name):
        self.name = name	# declare member variables
        self.position = 0
    def bark(self):
        print(f"{self.name}: Wal! Wal!")
    def move(self, distance):
        self.position += distance
        print(f"{self.name} is at {self.position}")

puppy = Dog("dangdang")
puppy.bark()	# => dangdang: Wal! Wal!
puppy.move(10)	# => dangdang is at 10
print("current position:", puppy.position)	# => current position: 10
```



### 생성자 함수 `__init__`

파이썬은 특정 기능을 특정 이름에 지정해 놓은 경우가 많은데 클래스 안에 `__init__`은 생성자 역할을 하여 객체가 생성될 때 자동으로 실행된다. 입력인자를 받을 수 있으나 출력(return)을 할 순 없다. 주로 멤버 변수를 선언하고 초기화 하는데 사용한다. `__init__`외에 다른 함수에서도 멤버 변수를 추가할 순 있으나 파이썬에서는 가급적  `__init__`에서 모든 멤버 변수를 미리 선언할 것을 권장한다.

### Self는 무엇?

클래스의 모든 함수에 공통적인 입력 인자가 보이는데 `self`라는 변수다. static 함수를 제외하고 모든 클래스 멤버 함수에 첫 번째 인자로 `self`를 넣어야 한다. 이것은 C++ 클래스의 `this`와 비슷하다고 보면 된다. **`self`는 객체 자신을 의미하는 것이며 객체의 함수나 변수는 `self`를 통해서만 접근할 수 있다.** `move`라는 함수에서 `position` 변수에 접근하기 위해 `self.position`이라고 썼다. 함수 안에서 다른 함수를 부를 때도 `self`를 쓴다.  

`self`라는 입력 인자는 함수를 호출할 때 직접 객체를 입력하는 것이 아니고 호출한 객체가 자동으로 들어간다. 예를 들면 `puppy.bark()`를 호출했을 때 첫 번째 인자인 `self`에 `puppy`가 들어가고 `puppy.move(10)`를 호출했을 때는 `self, distance`에 `puppy, 10`이 각각 들어간다.  

### 사용법

`self`가 들어갈 뿐 함수를 쓰는 방법은 일반적인 함수를 정의하는 것과 크게 다르지 않다. 클래스를 사용하는 방법은 다른 언어와 비슷하다. 객체를 생성할 때는 `()`를 써야하며 생성자의 입력인자가 있다면 입력 값을 넣어준다. 생성된 객체 뒤에 `.`을 통해 내부 함수를 실행하거나 내부 변수에 접근할 수 있다. 파이썬은 포인터가 없으므로 (사실 모두 포인터이므로) `->`는 쓰지 않는다.

### 클래스와 객체

클래스와 객체(instance)의 관계는 붕어빵에 비유할 수 있다. 붕어빵을 찍어낼 수 있는 빵틀은 클래스라 할 수 있고 이를 통해 생성된 실체인 붕어빵은 객체에 해당한다. 위 예시에서 `Dog`는 클래스고 `puppy`는 객체이다. 클래스는 형식이고 객체는 구현이다.

## 상속과 다형성

OOP를 하는데 있어서 핵심적인 문법이 상속(inheritance)과 다형성(polymorphism)이다. **상속**이란 클래스 사이에 부모와 자식 혹은 상위와 하위 관계가 있어서 자식 클래스는 부모 클래스의 함수와 변수를 물려받는다. **다형성**이란 형태는 같은데 다른 일을 하는 함수가 여러개 있을 수 있다는 것이다. 상속을 통해 부모 클래스에서 받은 메소드들을 오버라이딩(overriding)을 통해 바꿔쓸 수 있다는 것이다. **OOP는 상속과 다형성을 통해 코드를 절약하고 기능을 한 곳으로 응집시켜서 버그를 줄이고 변화에 강인한 코드를 만든다.** 예제를 통해 알아보자.

```python
print("===== object oriented programming")
class Animal:
    def __init__(self, name):
        self.name = name
    def introduce(self):
        print("my name is", self.name)
    def sound(self):
        print("...")
class Cow(Animal):
    def __init__(self, name):
        super().__init__(name)
    def sound(self):
        print("ummer~~~")
class Cat(Animal):
    def __init__(self, name):
        super().__init__(name)
    def sound(self):
        print("nyaong~~")

cow = Cow("cow1")
cow.introduce()
cow.sound()
animals = [Animal("ani"), Cow("cow2"), Cat("cat")]
for ani in animals:
    ani.introduce()
    ani.sound()
```

- `Animal`은 부모클래스고 `Cat, Cow`는 자식클래스다. 상속을 받을 때는 `class 자식클래스(부모클래스)` 형식으로 선언한다. 
- 자식의 생성자에서 `super().__init__(name)`는 부모클래스의 생성자를 호출한다는 것이다. 이를통해 굳이 자식에서 `self.name=name` 라는 똑같은 코드를 쓰지 않아도 부모클래스의 생성자를 실행해 같은 기능을 수행할 수 있다.  
- `Cat, Cow`는 `Animal`로부터 `introduce()`라는 메소드를 물려받아 그대로 사용하기 때문에 자식클래스에 `introduce()`를 다시 정의하지 않아도 `introduce()`를 쓸 수 있다.
- `Cat, Cow`는 `Animal`로부터 `sound()`라는 메소드를 물려받아 이를 오버라이딩(overriding)하여 함수의 기능을 수정하였다. 셋 다 `sound()`라는 똑같은 함수를 실행해도 다른 기능을 한다.
- for loop을 돌때 함수의 형식이 같기 때문에 어떤 클래스의 객체든 상관없이 `introduce(), sound()`가 실행된다.



### 연습문제

위의 `Animal, Cow, Cat` 클래스 예시처럼 각자에게 친숙하면서도 개성적인 객체를 클래스로 구현하고 이를 사용해보시오. 상속구조를 가진 계층화된 클래스를 구현하고 함수 오버라이딩을 활용하여 같은 함수로 다른 기능을 하는 객체들을 구현하시오.

# 파일 입출력

파이썬의 파일 입출력은 매우 간편하다. 간단한 예시를 통해 파일 입출력을 경험해보자.

```python
fout = open("testfile.txt", "w")
fout.write("I think Microsoft named .Net so it wouldn’t show up in a Unix directory listing.")
fout.close()
print("file was written")

fin = open("testfile.txt", "r")
contents = fin.read()
fin.close()
print(contents)
```

파일을 열고(open) 쓰고(write) 닫고(close), 파일을 열고(open) 읽고(read) 닫고(close) 이게 거의 전부다. 맨 아래 `print`를 통해 출력되는 문장은 파일 내용을 읽어서 출력한 것이다.

## 1. 파일 열기

프로그래밍에서 파일을 연다는 것은 파일을 읽거나 쓸수 있는 객체를 생성한다는 것이다. 파이썬에서는 `open()`이라는 함수로 파일 객체를 만들수 있다. `open()`의 프로토타입은 다음과 같다.

```python
open(file, mode='r', buffering=-1, encoding=None, errors=None, newline=None, closefd=True, opener=None)
```

- file: 파일 이름을 입력한다. 파일의 경로명과 확장자까지 모두 들어가야 하며 절대경로와 상대경로 모두 지원한다.
- mode: 파일을 다룰 모드를 지정한다.
  - `r`: 읽기 모드로서 `file`이 존재해야만 한다. 존재하지 않을 시 `FileNotFoundError`가 난다.
  - `w`: 쓰기 모드로서 파일이 없으면 새 파일을 만들고 있으면 기존 내용을 지우고 빈 파일을 만든다.
  - `a`: 추가 모드로서 기존 내용을 보존하고 기존 내용 끝에 새 내용을 추가한다.
- encoding: 문자열은 결국 바이트 코드로 저장이 되는데 문자열을 바이트 코드로 변환하는 과정을 인코딩, 반대 과정을 디코딩이라 한다. 인코딩 방식이 매우 다양한데 방식을 지정하지 않으면 운영체제 기본 값이 들어간다. 윈도우에서는 `cp949`라는 인코딩이 기본인데 이는 윈도우에만 주로 쓰는 것이고 가장 보편적인 인코딩은 `utf-8`이다. 플랫폼 독립적인 코드를 만들기 위해서는 인코딩에도 신경을 써야한다.

더 자세한 내용은 `open()`에 대한 [파이썬 공식문서](https://docs.python.org/3/library/functions.html#open)를 참고하자.  

만약 없는 파일을 열려고 하면 어떻게 될까? 쓰기 모드는 새 파일을 만들지만 읽기 모드에서는 에러가 난다. 실제 개발을 할때는 아래 예시처럼 예외처리를 하기보다는 파일이 있는지를 내장 패키지 함수를 통해 확인 후 열게 된다.

```python
try:
    f = open("nofile.txt", "r")
except FileNotFoundError as fe:
    print(fe)
```



파일을 열 때 위 예시처럼 `fp = open(filename, mode)` 이렇게 열어도 되지만 이 경우 반드시 `fp.close()`를 실행해야 한다. 코딩을 하다보면 이걸 깜빡하기 쉬운데 이렇게 열고 닫아야 하는 코드가 있을 경우 파이썬에서는 **context manager**라는 기능을 쓴다. `with`라는 키워드로 객체를 생성하고 그 아래 들여쓰기 블록에서 생성된 객체를 사용하고 그 블록을 빠져나가면 자연스럽게 객체가 파괴(close)된다. 아래 예시를 보면 쉽게 이해할 수 있다. 열린 객체를 `fr`로 받고 아래 블록에서 객체를 사용하고 블록을 나가면 자연스럽게 객체가 닫힌다.

```python
with open("testfile.txt", "r") as fr:
    data = fr.read()
    print("check closed under 'with':", fr.closed)
    print(data)

print("check closed outside 'with':", fr.closed)
```



## 2. 파일 쓰기

파이썬 파일 입출력 객체는 단순하게 `write()`라는 함수에 문자열을 입력하면 파일에 문자열이 출력된다. 파이썬이 워낙 문자열을 잘 다루다 보니 굳이 숫자를 직접 출력하는 함수는 만들지 않은 듯 하다. 문자열의 리스트를 갖고 있다면 `writelines()`를 쓸 수도 있다. 로이킴의 `봄봄봄` 가사를 파일에 써보자. 여러 줄을 하나의 문자열로 한번에 넣을 수도 있지만 문자열의 리스트로 먼저 가사를 쓴 후 파일에 출력해보자. 하는 김에 줄 번호도 같이 출력해보자.

```python
springx3 = ["봄 봄 봄 봄이 왔네요",
            "우리가 처음 만났던 그때의 향기 그대로",
            "그대가 앉아 있었던 그 벤치 옆에 나무도 아직도 남아있네요",
            "살아가다 보면 잊혀질 거라 했지만",
            "그 말을 하면 안될거란걸 알고 있었소",
            "그대여 너를 처음 본 순간 나는 바로 알았지",
            "그대여 나와 함께 해주오 이 봄이 가기 전에"]

print("\nwrite lyrics into file")
with open("springx3.txt", "w") as f:
    for i, line in enumerate(springx3):
        f.write(f"{i:2}:" + line + "\n")
```



## 3. 파일 읽기

파이썬 파일 입출력 객체는 세 가지 읽기 함수를 제공한다.

1. read(size=-1): `size`에 해당하는 byte만큼 파일을 읽어 하나의 `str` 객체로 리턴한다. 기본 값 -1로 두면 파일 끝까지 읽는다.
2. readline(): 파일을 한 줄씩 `str` 객체로 읽는다. 다 읽으면 `None`을 출력한다.
3. readlines(): 전체 파일을 한 줄씩 `str` 객체로 읽어서 `list of str`을 출력한다.

위 세 가지 함수를 이용해 `봄봄봄` 가사를 파일로부터 읽어와 출력하는 함수를 만들어보자.

1) read()
```python
print("use read")
with open("springx3.txt", "r") as f:
    lyrics = f.read()
    print(lyrics)
```

2) readline()
```python
print("use readline")
with open("springx3.txt", "r") as f:
    lyrics = []
    line = f.readline()
    while line:
        line = line.rstrip("\n")
        lyrics.append(line)
        line = f.readline()
print("\n".join(lyrics))
```

3) readlines()
```python
print("use readlines")
with open("springx3.txt", "r") as f:
    lyrics = f.readlines()
    lyrics = [line.rstrip("\n") for line in lyrics]
    print("\n".join(lyrics))
```

### 연습문제

다음 코드에서 `matrix_str`은 2x2 행렬을 나타내는 문자열이다. 이를 파일로 출력 후 다시 파일을 읽어서 2중 리스트에 저장한 후 행렬의 determinant를 구하시오. 파일을 읽고 쓸때 `with` 키워드를 이용하시오.

```python
matrix_str = "4 2\n5 3"
# matrix.txt 에 저장

# 파일 읽어서 matrix에 2중 리스트로 저장

print("read matrix:", matrix)
# => read matrix: [[4, 2], [5, 3]]
# determinant 계산
```



