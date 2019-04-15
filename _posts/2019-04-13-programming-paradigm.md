---
layout: post
title:  "[Python] Programing Paradigm and Class"
date:   2019-04-13 09:00:13
categories: 2019-1-systprog
---



## 프로그래밍 패러다임

클래스와 객체지향 프로그래밍을 맛보기 전에 이것이 무엇인지 왜 쓰는지 알기 위해서는 객체지향 외에 다른 프로그래밍 패러다임 또한 알아야 서로의 장단점을 알 수 있다. 프로그래밍 패러다임은 프로그램을 작성하는 철학이나 접근법을 의미하며 대부분의 언어는 멀티 패러다임을 지원하지만 더 적합한 특정 패러다임이 있다. 예를 들어 C언어는 절차적 프로그래밍에, C#과 Java는 객체지향적 프로그래밍에 특화 되어있고, 파이썬은 두 가지를 섞어 쓰는 경우가 많다. 프로그래밍 패러다임은 기준에 따라 다양하게 분류할 수 있는데 여기서는 세 가지만 언급한다.

1. **절차적 프로그래밍 (Procedural Programming)**: 여러분이 늘 하고 있는 '그것'이다. 파이썬 스크립트처럼 위에서 아래로 차례대로 (절차적으로) 실행되는 프로그램이다. 연산을 하는 알고리즘을 프로그래머가 구체적으로 기술해야 한다. 프로그램의 복잡도가 높아지면 함수나 외부 모듈, 패키지들을 만들어 구조화시킬 수 있다. 문법적으로 보면 변수와 함수로 이루어진 프로그램이다. 이러한 프로그래밍은 하드웨어 제어처럼 시간에 따라 직선적인 흐름을 갖는 프로그램에는 적합하다.
2. **객체지향적 프로그래밍 (Object-Oriented Programming, OOP)**: 절차적 프로그래밍처럼 순서대로 실행되지만 **객체(instance)**가 중심이 된다. 절차적 프로그래밍은 다수의 이벤트들이 동시 다발적으로 발생할 수 있는 GUI, 웹서버, 게임 등에는 적합하지 않다. 너무 많은 변수와 함수를 만들어야 하고 변수와 함수들이 복잡하게 얽혀 있어서 코드가 길어지고 이해할 수 없게 된다. 객체지향적 프로그래밍은 관련있는 데이터와 메소드(method, 객체지향적 프로그램에서 함수를 지칭하는 단어)를 묶어서 객체를 만든다. 객체 밖에서는 객체 내부 데이터에 접근할 수 없고 오직 그 객체의 메소드를 통해서만 접근하거나 연산이 가능하다. 내부적인 구현의 복잡성은 객체 안으로 숨기고 객체의 사용자는 객체에게 할 일을 명령하기만 하면 된다. 문법적으로는 클래스 (class) 위주로 작성하는 프로그램이다.
3. **함수형 프로그래밍 (Functional Programming)**: 최근 Hot한 패러다임이다. 절차적 프로그래밍의 복잡성을 줄이기 위해 객체지향이 나왔지만 이 또한 프로그램의 규모가 커질수록 너무 많은 객체들이 서로 관련되어 함수 하나, 변수 하나를 수정한 여파가 다수의 객체로 퍼져나가 많은 버그를 유발한다. 함수형 프로그래밍은 말 그대로 *함수*가 다시 중심이 된다. 얼핏 보면  절차적 프로그래밍으로 돌아간 것 같지만 함수형 프로그래밍은 지켜야 할 엄격한 룰들이 있다. 그 중 가장 중요한 원칙은 *함수에는 반드시 입력이 있어야 하며 출력은 반드시 입력에만 의존해야 한다*는 것이다. 객체의 메소드는 대부분 입력과 함께 객체 내부 데이터도 사용하므로 저 원칙을 위반한다. 함수에 필요한 모든 데이터가 입력 인자로 들어와야 하므로 입출력 관계가 보다 명확하게 보인다. 전체 프로그램을 작은 함수들의 조합으로 만들고 변수라는 개념을 없애고 데이터를 상수로만 쓰기 때문에 프로그램의 복잡성을 줄일 수 있다. 하지만 이미 절차적 혹은 객체지향적 프로그래밍을 배운 다음에 이러한 패러다임을 머릿속에 주입한다는 것이 매우 어렵기 때문에 아직은 인기가 없다.

함수형 프로그래밍은 뇌를 포맷하고 새로 배워야 하는 전혀 다른 개념이므로 논외로 하고 여기서는 절차적 프로그래밍과 객체지향적 프로그래밍의 장단점을 알아보고 어떤 경우에 어떤 패러다임을 써야 하는지 (단순하게 말하면 클래스를 써야 할지 말아야 할지) 알아보자.  

참고: 
- 프로그래밍 패러다임과 파이썬: <https://tech.peoplefund.co.kr/2018/11/28/programming-paradigm-and-python-kor.html>
- 프로그래밍 패러다임: <http://melodyonlight.com/?p=374>
- 함수형 프로그래밍이란 무엇인가?: <https://medium.com/@jooyunghan/%ED%95%A8%EC%88%98%ED%98%95-%ED%94%84%EB%A1%9C%EA%B7%B8%EB%9E%98%EB%B0%8D%EC%9D%B4%EB%9E%80-%EB%AC%B4%EC%97%87%EC%9D%B8%EA%B0%80-fab4e960d263>


## 클래스 문법

일단 파이썬에서 클래스를 쓰는 방법에 대해 알아보자. 클래스는 `class`라는 키워드로 시작하며 함수와 마찬가지로 들여쓰기로 클래스 정의의 범위를 지정한다. 간단한 예시를 통해 클래스 정의 방법을 알아보자.

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

클래스의 모든 함수에 공통적인 입력인자가 보이는데 `self`라는 변수다. static 함수를 제외하고 모든 클래스 멤버 함수에 첫 번째 인자로 `self`를 넣어야 한다. 이것은 C++ 클래스의 `this`와 비슷하다고 보면 된다. `self`는 객체 자신을 의미하는 것이며 객체의 함수나 변수는 `self`를 통해서만 접근할 수 있다. `move`라는 함수에서 `position` 변수에 접근하기 위해 `self.position`이라고 썼다. 함수 안에서 다른 함수를 부를 때도 `self`를 쓴다. `self`라는 입력인자는 함수를 호출할 때 직접 객체를 입력하는 것이 아니고 호출한 객체가 자동으로 들어간다. 예를 들면 `puppy.bark()`를 호출했을 때 첫 번째 인자인 `self`에 `puppy`가 들어가고 `puppy.move(10)`를 호출했을 때는 `self, distance`에 `puppy, 10`이 각각 들어간다.  

### 사용법

`self`가 들어갈 뿐 함수를 쓰는 방법은 기본 함수를 정의하는 것과 크게 다르지 않다. 클래스를 사용하는 방법은 다른 언어와 비슷하다. 객체를 생성할 때는 `()`를 써야하며 생성자의 입력인자가 있다면 입력 값을 넣어준다. 생성된 객체 뒤에 `.`을 통해 내부 함수를 실행하거나 내부 변수에 접근할 수 있다. 파이썬은 포인터가 없으므로 (사실 모두 포인터이므로) `->`는 쓰지 않는다.

### 클래스와 객체

클래스와 객체(instance)의 관계는 붕어빵에 비유할 수 있다. 붕어빵을 찍어낼 수 있는 빵틀은 클래스라 할 수 있고 이를 통해 생성된 실체인 붕어빵은 객체에 해당한다. 위 예시에서 `Dog`는 클래스고 `puppy`는 객체이다. 클래스는 형식이고 객체는 구현이다.

## 상속과 다형성

OOP를 하는데 있어서 핵심적인 문법이 상속(inheritance)과 다형성(polymorphism)이다. **상속**이란 클래스 사이에 부모와 자식 혹은 상위와 하위 관계가 있어서 자식 클래스는 부모 클래스의 함수와 변수를 물려받는다. **다형성**이란 형태는 같은데 다른 일을 하는 함수가 여러개 있을 수 있다는 것이다. 상속을 통해 부모클래스에서 받은 메소드들을 오버라이딩(overriding)을 통해 바꿔쓸 수 있다는 것이다. **OOP는 상속과 다형성을 통해 코드를 절약하고 기능을 한 곳으로 응집시켜서 버그를 줄이고 변화에 강인한 코드를 만든다.** 예제를 통해 알아보자.

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
- 자식의 생성자에서 `super().__init__(name)`는 부모클래스의 생성자를 호출한다는 것이다. 이를통해 굳이 자식에서 `self.name=name` 라는 똑같은 코드를 쓰지 않아도 부모클래스의 생성자를 통해 같은 기능을 수행할 수 있다.  
- `Cat, Cow`는 `Animal`로부터 `introduce()`라는 메소드를 물려받아 그대로 사용하기 때문에 자식클래스에 `introduce()`를 다시 정의하지 않아도 `introduce()`를 쓸 수 있다.
- `Cat, Cow`는 `Animal`로부터 `sound()`라는 메소드를 물려받아이를 오버라이딩하여 함수의 기능을 수정하였다. 셋 다 `sound()`라는 똑같은 함수를 실행해도 다른 기능을 한다.
- for loop을 돌때 함수의 형식이 같기 때문에 어떤 클래스의 객체든 상관없이 `introduce(), sound()`가 실행된다.



## 객체지향적 프로그래밍 (OOP)

클래스를 사용하면 절차적 프로그래밍에 비해 코드를 효과적으로 구조화할 수 있고 상속과 다형성을 활용하면 절차적 프로그래밍에서는 상상하지 못 했던 신박한 프로그램들을 만들수 있다. 위 예시의 코드를 절차적으로 다시 쓴다면 어떻게 될까?

```python
print("===== procedural programming")
animal_name = "ani"
cow_name = "cow"
cat_name = "cat"
def introduce(name):
    print("my name is", name)
def animal_sound():
    print("...")
def cow_sound():
    print("ummer~~~")
def cat_sound():
    print("nyaong~~")

introduce(animal_name)
animal_sound()
introduce(cow_name)
cow_sound()
introduce(cat_name)
cat_sound()
```

얼핏 봐서는 이쪽이 더 친숙(?)하고 더 단순해 보인다. 사실 이 정도까지는 절차적 프로그래밍도 별문제가 없고 코드양도 OOP가 더 길다. OOP를 이용해서 코드가 짧아지는 경우를 보자. 사용자로부터 두 개의 입력을 받는데 첫 번째는 동물의 종류를 받고 두 번째는 동작(operation)을 받는다. 입력을 받고 해당 동물의 해당 함수를 실행한다. 동물의 종류는 `animal, cow, cat`이 있고 동작은 `intro, sound` 두 가지가 있다.

```python
ani_type = input()
operation = input()
if ani_type == "animal":
    if operation == "intro":
        introduce(animal_name)
    elif operation == "sound":
        animal_sound()
elif ani_type == "cow":
    if operation == "intro":
        introduce(cow_name)
    elif operation == "sound":
        cow_sound()
elif ani_type == "cat":
    if operation == "intro":
        introduce(cow_name)
    elif operation == "sound":
        cat_sound()
```

세 가지 동물과 두 가지 동작을 조합하여 6개의 if 문이 필요하다. 이번에는 객체를 이용하여 같은 동작을 해보자.

```python
ani_type = input()
operation = input()
if ani_type == "animal":
    ani = Animal("ani")
elif ani_type == "cow":
    ani = Cow("cow")
elif ani_type == "cat":
    ani = Cat("cat")
if operation == "intro":
    ani.introduce()
elif operation == "sound":
    ani.sound()
```

첫 번째 `if~elif`에서는 객체를 먼저 선택하고 다음 `if~elif`에서는 동작을 선택하여 절차적 프로그램에서 이중 조건문을 단순 조건문으로 바꾸고 조건문의 수도 5개로 줄어들었다. 6개에서 5개로 줄어든게 큰 차이일까?  

### 객체지향의 장점

객체지향의 진가는 소프트웨어를 변경할 때 나타난다. 여러분들이 사용하고 있는 모든 소프트웨어는 시간이 지나면 둘 중 하나를 선택해야한다. 변하거나 사라지거나. 소프트웨어에 요구되는 기능과 사용환경이 모두 빠르게 변하는 시대이기 때문에 **배포중인 소프트웨어는 항상 변하고 있다.** 좋은 프로그램 설계란 어떤 예상치 못한 변경사항이 들어오더라도 쉽고 빠르게 변경할 수 있는 설계이다. 설계 변경의 대부분은 기능의 추가이다. 위 예시에서 동물의 종류가 3종이 아니라 10종이고 할 수 있는 동작이 5개로 늘어난다면 어떻게 될까? 절차적 프로그래밍은 if문이 50개로 늘어나겠지만 OOP에서는 15개가 될 뿐이다. 이것은 극단적인 예시가 아니다. 여러분이 하는 게임에 몇 가지 챔피언이 있고 그들의 동작이 몇 가지씩 있는지 생각해보라.

이를 체감하기 위해 위 코드에 딱 두 가지 변경사항만 넣어보자. 각 동물에 `weight`라는 속성과 `is_fat()`이라는 함수를 추가해보자. 먼저 절차적으로 구현해보자.

```python
print("===== procedural programming")
animal_name = "ani"
animal_weight = 20
cow_name = "cow"
cow_weight = 100
cat_name = "cat"
cat_weight = 5

def introduce(name):
    print("my name is", name)
def animal_sound():
    print("...")
def animal_is_fat(weight):
    print("is fat as an animal?", weight > 50)
def cow_sound():
    print("ummer~~~")
def cow_is_fat(weight):
    print("is fat as a cow?", weight > 120)
def cat_sound():
    print("nyaong~~")
def cat_is_fat(weight):
    print("is fat as a cat?", weight > 3)

ani_type = input()
operation = input()
if ani_type == "animal":
    if operation == "intro":
        introduce(animal_name)
    elif operation == "sound":
        animal_sound()
    elif operation == "isfat":
        animal_is_fat(animal_weight)
elif ani_type == "cow":
    if operation == "intro":
        introduce(cow_name)
    elif operation == "sound":
        cow_sound()
    elif operation == "isfat":
        cow_is_fat(cow_weight)
elif ani_type == "cat":
    if operation == "intro":
        introduce(cat_name)
    elif operation == "sound":
        cat_sound()
    elif operation == "isfat":
        cat_is_fat(cow_weight)
```

코드가 상당히 길어진 것을 체감할 수 있다. 3개의 `weight` 변수와 3개의 `is_fat()`함수, 이를 처리하기 위한 3개의 `elif`가 늘어났다. 게다가 복붙을 하다보니 잘 보이지 않는 작은 버그도 있다. 이번에는 OOP로 구현해보자.

```python
print("===== object oriented programming")
class Animal:
    def __init__(self, name, weight):
        self.name = name
        self.weight = weight
    def introduce(self):
        print("my name is", self.name)
    def sound(self):
        print("...")
    def is_fat(self):
        print("is fat as an animal?", self.weight > 50)
class Cow(Animal):
    def __init__(self, name, weight):
        super().__init__(name, weight)
    def sound(self):
        print("ummer~~~")
    def is_fat(self):
        print("is fat as a cow?", self.weight > 120)
class Cat(Animal):
    def __init__(self, name, weight):
        super().__init__(name, weight)
    def sound(self):
        print("nyaong~~")
    def is_fat(self):
        print("is fat as a cat?", self.weight > 3)

ani_type = input()
operation = input()
if ani_type == "animal":
    ani = Animal("ani", 20)
elif ani_type == "cow":
    ani = Cow("cow", 100)
elif ani_type == "cat":
    ani = Cat("cat", 5)
if operation == "intro":
    ani.introduce()
elif operation == "sound":
    ani.sound()
elif operation == "isfat":
    ani.is_fat()
```

드디어 OOP의 코드가 절차적 프로그래밍 보다 상대적으로 짧아졌다. 두 프로그램 모두 3개의 `is_fat()` 함수를 정의해주는 것은 같지만 OOP에서는 1개의 변수와 1개의 `elif`만 늘어났다. 위 코드에서 볼 수 있는 객체지향의 장점은 다음과 같다.

- **캡슐화**: 클래스에서는 데이터와 이를 다루는 함수가 묶여있다. 이를 캡슐화라 한다. 캡슐화가 되면 어떤 변수에 영향을 주는 함수를 찾기가 쉽다. 반면에 다수의 변수와 함수만으로 이루어진 프로그램은 어떤 변수를 어느 함수에서 읽거나 수정하는지 알기위해서는 전체 코드를 봐야한다. 절차적 프로그래밍은 복사와 붙여넣기를 빈번하게 하기 때문에 `cat_is_fat(cow_weight)`와 같은 실수가 자주 일어나는데 봐야할 곳이 많아서 디버깅하기가 어렵다.
- **코드 재사용성**: 만약 위 코드에서 고양이와 관련된 코드만 다른 곳에서 다시 쓰고 싶다면 어떻게 해야할까? 절차적 프로그래밍에서는 `import module_name.variable_or_function ` 처럼 고양이와 관련된 여러개의 변수와 함수를 가져와야 한다. 반대로 OOP에서는 `import module_name.Cat`만 하면 고양와 관련된 모든 데이터와 동작을 가져올 수 있다.
- **변경에 대한 유연성/강인성**: 위 예시처럼 기능이 추가되거나 수정될 경우 변경점을 찾기가 쉽고 여러 클래스에 대해 공통적인 변경은 부모클래스를 통해 쉽게 바꿀 수 있다. 수정을 하다보면 어떤 동물에 대해서는 수정을 다 하고 어떤 동물에 대해서는 하다 마는 경우도 있는데 그래도 당장은 에러가 나지 않고 버그로 남아있을 수 있다. 하지만 OOP에서 수정사항에 따라 부모클래스를 바꾸면 자식클래스도 따라서 바뀌어야 하므로 그런 일이 적다.
- **통일된 네이밍**: 일단 클래스가 없으면 함수와 변수명이 길어진다. 그리고 지금까지는 `동물이름_동작()` 형식으로 함수를 만들었지만 언제 다른 개발자가 와서 `동작_동물이름()` 형식으로 새로운 함수를 추가할지 모른다. 같은 부모 클래스를 상속하면 언제나 일정한 네이밍을 유지할 수 있다.

### 클래스와 상속을 써야 할 때

지금까지 객체지향 프로그래밍에 대한 찬양을 늘어놓았다. 그렇다면 OOP가 항상 옳은 것일까? 그렇지는 않다. OOP는 비슷한 명령을 다르게 수행하는 다양한 객체 종류(클래스)가 있을 때 혹은 데이터와 동작이 긴밀한 관계가 있을 때 적합하다. 단순히 A에서 B를 더해 C를 만들고 C에 D를 빼서 E를 만들고... 식의 직선적인 흐름의 프로그램에는 절차적 프로그래밍이 낫다. 그래서 파이썬 패키지들을 보면 객체 없이 직접 함수를 제공하는 경우가 많다. 전체적으로 보면 하나의 프로그램 안에서도 두 가지 패러다임을 적절히 섞어서 짜는 것이 좋다.   

그럼 파이썬에서는 언제 클래스를 써야할까? 일단은 먼저 절차적으로 프로그램을 짜본다. 짜다보면 두 개 이상의 함수에서 같은 전역변수를 공유하고 싶어질 때가 있다. 물론 입출력인자로 써도 되긴하지만 공통적으로 쓰는 데이터나 파라미터가 있다면 전역변수가 편할 것이다. **전역변수가 필요하다면 클래스를 만들자.** 전역변수는 해당 모듈 전체에서 접근할 수 있으므로 전역변수로 인한 에러는 디버깅하기 쉽지 않다. 전역변수와 함수를 클래스로 묶어서 변수에 접근할 수 있는 함수를 제한하는 것이 좋다. 위의 절차적 프로그램 예시에서도 `~name`이나 `~weight`같은 전역변수들이 있고 특정 변수를 특정 함수에서만 사용하므로 클래스로 바꿔주는 것이 좋다.  

그럼 상속은 언제 써야할까? 방금 얘기한대로 클래스를 만들다보니 공통점들이 있고 `Animal`이라는 상위개념으로 `Cow, Cat`을 묶을 수 있는 상황이다. 이런 경우 클래스들을 상속관계로 만들면 코드도 줄일 수 있고 수정사항이 생겼을 때 각각의 클래스를 따로 수정하는 것보다 공통의 부모클래스를 수정하는 것이 효율적이다.