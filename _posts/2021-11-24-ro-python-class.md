---
layout: post
title:  "Final Project and Python Class"
date:   2020-10-29 09:00:13
categories: 2020-2-robotics
---



# Python Class

프로그래밍을 하다 보면 어떤 변수를 함수 내부에서 한 번 쓰고 버려도 되는 게 있고 어떤 변수는 지속적으로 사용되거나 업데이트 된다. 로봇 프로그래밍을 한다면 센서의 한 프레임(frame) 동안만 쓰는 변수가 있고 여러 프레임에 걸쳐서 사용되는 변수도 있다. 함수가 끝나도 어떤 변수가 사라지지 않고 유지가 되게 하려면 **클래스(class)**를 써야 한다. 객체지향프로그래밍(OOP, Object Oriented Programming) 언어에서 클래스는 데이터(=변수)와 그 데이터를 처리하는 메소드(=함수)를 합쳐 **객체(instance)**를 만든다.  

클래스는 프로그래밍에서 다루는 데이터를 객체화(instantiate) 시키고 비슷한 데이터와 함수의 조합을 다양하게 재사용할 수 있도록 돕는다. 일반적으로 객체지향적인 방식으로 클래스를 사용하면 코드의 가독성이 좋아지고 코드의 수정/변경이 쉬워진다. 여기서는 객체지향 프로그래밍까지는 다루지 않고 파이썬의 클래스 문법만 살펴본다.

파이썬에서 클래스는 `class`라는 키워드로 시작하며 함수와 마찬가지로 들여쓰기로 클래스 정의의 범위를 지정한다. 간단한 예시를 통해 클래스 정의 방법을 알아보자.

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
- 자식의 생성자에서 `super().__init__(name)`는 부모클래스의 생성자를 호출한다는 것이다. 이를통해 굳이 자식에서 `self.name=name` 라는 똑같은 코드를 쓰지 않아도 부모클래스의 생성자를 통해 같은 기능을 수행할 수 있다.  
- `Cat, Cow`는 `Animal`로부터 `introduce()`라는 메소드를 물려받아 그대로 사용하기 때문에 자식클래스에 `introduce()`를 다시 정의하지 않아도 `introduce()`를 쓸 수 있다.
- `Cat, Cow`는 `Animal`로부터 `sound()`라는 메소드를 물려받아이를 오버라이딩(overriding)하여 함수의 기능을 수정하였다. 셋 다 `sound()`라는 똑같은 함수를 실행해도 다른 기능을 한다.
- for loop을 돌때 함수의 형식이 같기 때문에 어떤 클래스의 객체든 상관없이 `introduce(), sound()`가 실행된다.



# Navigation Base

자율주행 프로젝트를 수행하는데 어디서부터 시작해야 할지 막막한 학생들을 위하여 기본 틀을 제공하고자 한다. 자율주행 패키지를 구현하기 위해서는 매 프레임마다 세 가지 일을 반복해야 한다.

- LDS 토픽을 입력으로 받기
- LDS 입력을 바탕으로 주변 상황을 파악하고 로봇의 이동 속도 결정 (알고리즘)
- 속도를 토픽으로 발행하여 로봇 제어

여기서는 중간의 알고리즘을 제외하고 입출력 부분만 구현한 예시를 보여준다. 



## Create Package

다음 명령어를 통해 패키지와 노드 스크립트를 생성한다.

```
$ cd ~/catkin_ws/src
$ catkin create pkg self_drive --catkin-deps rospy std_msgs sensor_msgs geometry_msgs
$ cd self_drive/src
$ touch self_drive.py
$ chmod a+x self_drive.py
```



## Write Node Script

`package.xml`이나 `CMakeLists.txt`는 기존 강의자료를 참조하여 정리한다. 프로젝트에서는 `package.xml` 좀 더 상세히 작성해야 한다. 노드 스크립트는 다음과 같이 작성할 수 있다.

> 첫 줄의 shebang을 자신의 가상환경에 맞게 수정하자.

```python
#!/path/to/virtual/env/bin/python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class SelfDrive:
    def __init__(self, publisher):
        self.publisher = publisher
        self.count = 30

    def lds_callback(self, scan):
        # scan 분석 후 속도 결정
        # ...
        print("scan[0]:", scan.ranges[0])
        turtle_vel = Twist()
         # 전진 속도 및 회전 속도 지정
        if self.count < 100:
            turtle_vel.linear.x = 0.1
            self.count += 1
        else:
            turtle_vel.linear.x = 0.0
        turtle_vel.angular.z = 0.0
         # 속도 출력
        self.publisher.publish(turtle_vel)

def main():
    rospy.init_node('self_drive')
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    driver = SelfDrive(publisher)
    subscriber = rospy.Subscriber('scan', LaserScan,
                                  lambda scan: driver.lds_callback(scan))
    rospy.spin()

if __name__ == "__main__":
    main()
```

위 예시에서는 `driver` 객체의 `lds_callback` 함수를 서브스크라이버의 콜백 함수로 전달하기 위해 `lambda`(람다) 함수를 사용하였다. 람다 함수는 임시 함수를 만들어내는 문법이다. 다음 예시를 보자.

```python
# lambda <input args>: <function implementation>
sum = lambda a, b: a+b
print("lambda sum:", sum(1, 2))
```

self_drive 노드에서는 서브스크라이버에 `SelfDrive` 클래스의 `lds_callback`이라는 메소드를 콜백(callback) 함수로 넘기고 싶은데 그냥 `SelfDrive.lds_callback`을 넘기면 객체가 없어서 콜백 함수가 실행되지 않는다. `lds_callback`는 객체에서 실행되어야 하는 함수인데 콜백 함수 등록에는 객체가 못 들어가므로 람다 함수를 콜백 함수로 등록하고 람다 함수 내부에서 객체를 통해 `lds_callback` 함수를 불러왔다.  

콜백 함수를 클래스 메소드가 아닌 일반 함수로 구현했다면 굳이 람다 함수를 쓸 필요는 없다. 하지만 `publisher`와 `count` 변수가 콜백 함수에서 지속적으로 사용되어야 하므로 이를 클래스로 만들었다. 그리고 클래스 메소드를 콜백 함수로 등록하기 위해 람다 함수를 이용하였다.  





