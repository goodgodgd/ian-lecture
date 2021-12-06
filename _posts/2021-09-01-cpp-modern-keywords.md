---
layout: post
title:  "[Cpp] Modern keywords"
date:   2021-09-01 09:00:13
categories: CppAlgorithm
---





# Modern Keywords

이번 강의에서는 C++11/14에서 추가된 키워드와 간단한 문법들을 모아서 정리한다. auto, 스마트 포인터, 람다 등의 굵직한 기능을 다루기 전에 중괄호 생성자, nullptr, constexpr, using, delete, override, noexcept 등의 자잘한(?) 키워드들을 정리하고 넘어가자.



## 1. 중괄호 생성자

C++11부터 중괄호({}, brace)를 이용한 새로운 객체 초기화 문법이 생겼다. 기존의 소괄호()를 이용한 초기화와 비슷하면서도 약간 다르다. 객체를 초기화 하는 방법은 다음 네 가지 방법이 있다.

```cpp
int x1(0);
int x2 = 0;
int y1{0};
int y2 = {0};
```



<https://modoocode.com/286>





## 2. constexpr

`constexpr`은 `const`와 의미적으로는 비슷하지만 용법은 크게 다르다. C++ 1x에 익숙치 않은 사람들에게 가장 낯설게 느껴지는 키워드다. constexpr도 const처럼 객체에 사용될 때와 함수에 사용될 때 다른 의미를 가진다. 하나씩 알아보자.

> 참고자료: (Modern Effective C++) 항목 15: 가능하면 항상 constexpr을 사용하라

### 2.1. 객체 선언

C++ 11 이전에 변하지 않는 상수를 사용하는 방법은 const 뿐이었다. 하지만 상수도 세분화하면 두 가지 종류가 있다. 컴파일 시점에 결정되는 상수와 실행 시점에 결정되는 상수다. 아래 코드에서 `foo`는 컴파일 시점에 값을 알 수 있으며 프로그램 실행중에 변하지 않을 상수일 것이다. 이런 경우에는 const보다 constexpr을 쓰는게 낫다. 반면 `baz`의 값은 앞서 `bar` 변수에 대한 연산에 의존하므로 컴파일 시점에 알 수 없고 실행 시점에만 초기화 될 수 있다.

```c++
constexpr int foo = 1;
int bar;
...
const int baz = bar;
```

유의할 점은 constexpr은 컴파일 시점에 알 수 있는 값으로만 초기화를 해야한다는 것이다. 일반 변수로 초기화 시 컴파일 에러가 난다.  

const 보다 constexpr을 쓰는게 나은 이유는 뭘까? 프로그래밍에서 constexpr의 유용한 점은 리터럴 타입(Literal type)이 들어가야 하는 곳에 constexpr 변수를 넣을 수 있다는 것이다. 리터럴 타입이란 `int a = 1;  char foo[] = "hello"`에서 1이나 "hello"처럼 코드 안에 들어간 데이터를 말한다. 컴파일 시점에 값을 알 수 있고 값이 변하지 않는다. 더 정확히 말하면 **constexpr은 정수 상수 표현식(integral constant expression)이 요구되는 문맥에서 사용할 수 있다.** 배열의 크기, 템플릿 정수 인수, enum 값 등에 10, 20 같은 리터럴 타입 말고 constexpr을 쓸 수 있다는 것이다. 다음 예시를 살펴보자.

```cpp
#include <array>
enum { EnumSize = 10 };

int main() {
    int sz;
    sz = rand() * 2;
    // constexpr int cxp_size1 = sz;	// sz 값은 컴파일 시점에 알수 없으므로 에러!
    constexpr int cxp_size2 = 10;		// 리터럴로 초기화 OK
    char arr1[cxp_size2];				// constexpr을 정수 상수 표현식으로 활용
    std::array<int, cxp_size2> arr2;	// constexpr을 정수 상수 표현식으로 활용

    const int cst_size1 = sz;			// const는 실행 시점의 sz 값으로 초기화
    const int cst_size2 = 10;			// 리터럴로 const 초기화
    // std::array<int, cst_size1> arr3;	// cst_size1 값을 컴파일 시점에 알 수 없으므로 에러!
    std::array<int, cst_size2> arr4;	// constexpr은 아니지만 컴파일 시점에 값을 알 수 있으므로 OK
    std::array<int, EnumSize> arr5;		// enum 또한 컴파일 시점에 값을 알 수 있으므로 OK
}
```

상수(정수, 실수)가 필요한 경우 네 가지 해결 방법이 있다.

1. 하드 코딩 (하책) : 같은 의미를 갖는 숫자이 여럿 있을 경우 일괄적으로 수정하기 어렵다. **가급적 사용하지 말것**
2. #define (중책) : 이름이 있고 수정도 한번에 할 수 있다. 하지만 컴파일시 같은 값이 여러곳에서 중복으로 복사되어 컴파일 된 코드 크기가 커질 수 있다. 에러가 났을 때 define 이름이 아니라 define 값으로  에러문이 나와서 디버깅이 어렵다.
3. constexpr (상책) : 값에 대한 메모리가 한번만 할당되고 디버깅이 쉽다.
4. enum (상책) : 3과 같은데 정수만 쓸 수 있다.



### 2.2. 함수 반환 타입

constexpr 함수는 반환 타입에 constexpr이 붙어있는 함수다. 이런 함수에 상수를 입력하면 컴파일 시점에 리턴값을 미리 계산해 둘 수 있다. (런타임에 계산 비용이 들지 않는다.) 상수가 아닌 일반 변수를 넣더라도 작동하지만 컴파일 시점에 미리 값을 계산할 순 없다. 상수 계산용으로도, 일반 함수로도 쓸 수 있기 때문에 항목 15의 제목이 *"가능하면 항상 constexpr을 사용하라"*인 것이다. 다음은 정수형 지수함수를 구현한 예시다.

```c++
#include <iostream>

constexpr int pow(int base, int exp) {
    auto result = 1;
    for (int i=0; i<exp; ++i)
        result *= base;
    return result;
}

int main() {
    constexpr int result1 = pow(3, 4);
    std::cout << "compile-time result: " << result1 << std::endl;
    const int foo = rand() % 5;
    const int bar = rand() % 5;
    const int result2 = pow(foo, bar);
    std::cout << "runtime result: " << foo << ", " << bar << ", " << result2 << std::endl;
}
```

constexpr 함수의 결과를 또 다른 constexpr 함수에 입력하면 두 번째 함수까지 컴파일 시점에 계산할 수 있다. 사용자 정의 클래스를 constexpr 객체로 사용하기 위해서는 생성자 함수에 constexpr을 붙이면 된다. 다음은 사용자 정의 클래스에서 constexpr을 이용한 최적화 예시다.

```c++
#include <iostream>

class Point {
public:
    constexpr Point() noexcept : x{}, y{} {}
    constexpr Point(double x_, double y_) noexcept : x(x_), y(y_) {}
    constexpr double xValue() const noexcept { return x; }
    constexpr double yValue() const noexcept { return y; }
    constexpr void setX(double x_) noexcept { x = x_; }
    constexpr void setY(double y_) noexcept { y = y_; }
private:
    double x, y;
};

constexpr Point midpoint(const Point& p1, const Point& p2) noexcept {
    return { (p1.xValue() + p2.xValue()) / 2., (p1.yValue() + p2.yValue()) / 2. };
}

constexpr Point reflect(const Point& p) noexcept {
    Point result;
    result.setX(-p.xValue());
    result.setY(-p.yValue());
    return result;
}

int main()
{
    constexpr Point p1(9.4, 27.7);
    constexpr Point p2(28.8, 5.3);
    constexpr auto mid = midpoint(p1, p2);
    constexpr auto reflected = reflect(mid);
    std::cout << "mid: " << mid.xValue() << ", " << mid.yValue() << std::endl;
    std::cout << "reflected: " << reflected.xValue() << ", " << reflected.yValue() << std::endl;
}
```

> mid: 19.1, 16.5
> reflected: -19.1, -16.5

- `main()` 함수의 첫 두 줄은 constexpr 생성자를 호출하여 constexpr 객체를 생성할 수 있다. (생성자의 결과물이 객체기 때문에)
- `midpoint` 함수 호출에서는 네 단계로 컴파일 시점의 값이 전달된다.
  1. `constexpr Point` 객체를 입력 받아서 컴파일 시점에 입력 값을 알 수 있다. 
  2. 내부 값을 불러올 때도 constexpr 함수인 `xValue(), yValue()` 함수를 사용하여 컴파일 시점에 값을 전달할 수 있다. 
  3. 결과를 리턴할 때도 `midpoint`가 constexpr 함수이므로 컴파일 시점에 값을 정할 수 있다.
  4. 결과를 받는 변수도 constexpr 이므로 최종적으로 변수의 값이 컴파일 시점에 정해진다.

- `reflect` 함수 호출에서는 `midpoint` 함수에 비해 `setX(), setY()` 과정만 추가되었다. 그런데 이 함수들도 constexpr 이므로 컴파일 시점에 입력 값이 정해지면 컴파일 시점에 값을 변경해버릴 수 있다. 그리고 나서 constexpr 객체를 리턴하는 과정은 동일하다. 객체를 변경하지 못하게 하는 const와는 전혀 다른 의미니 유의하자.

이처럼 constexpr을 적용가능한 모든 곳에 적용하면 컴파일 시점 상수가 연계되어 더 깊은 곳까지 컴파일 시점에 정해질 수 있고 이는 실행 속도가 빨라진다는 것이다. (대신 컴파일 시간이 길어질 수 있다.)  

대신 constexpr을 어중간하게 쓰면 여기저기서 에러가 난다. `Point` 클래스에서 constexpr 키워드를 하나씩 빼보면서 어떤 에러가 나는지 각자 확인해보자. 어떤 클래스를 constexpr 객체로 쓸거라면 모든 멤버 함수에 constexpr을 붙여야 한다.



## 3. noexcept

constexpr 예시에서 모든 함수에 `noexcept`라는 키워드가 붙은걸 볼 수 있다. '예외 없음'이라는 뜻 같은데 정확히 어떤 뜻인지 알아고보고 왜 저렇게 모든 함수에 덕지덕지 붙는지도 알아보자.

> 참고자료: (Modern Effective C++) 항목 14: 예외를 방출하지 않을 함수는 noexcept로 선언하라

함수에 붙은 `noexcept`라는 키워드는 해당 함수에서 예외를 방출하지 않는다는 것을 프로그래머가 보장하는 것이다. 그런데 만약 noexcept가 있어도 함수에서 예외가 발생하면 프로그램이 종료되고 없어도 프로그램이 종료된다. 그럼 무슨 차이일까?  

일반적으로, 예외 명세가 위반되면 (예외가 발생하면) 호출 스택이 그 함수를 호출한 곳까지 풀리며(unwind) 객체들을 생성 반대순서로 파괴한 후 프로그램이 종료된다. 하지만 noexcept 키워드가 있으면 예외가 없을거라고 가정하고 컴파일러에서 최적화를 하기 때문에 이러한 종료과정을 지킬수도 있고 안 지킬수도 있다. 프로그램에서 지켜야 하는 제약이 줄어들기 때문에 컴파일러에서는 더 높은 수준의 최적화를 할 수 있게 된다. vector의 push_back이나 swap 같은 함수에서도 noexcept가 있으면 복사 생성자보다 효율이 좋은 이동 생성자를 선택하여 성능 향상을 꾀할수 있다. 즉 **noexcept 함수는 일반 함수보다 최적화의 여지가 크다.** 

C++11 이후에서는 메모리 해제 연산자(operator delete or operator delete[])나 소멸자에서 예외가 발생하면 안되기 때문에 두 가지 함수는 암묵적으로 noexcept 함수가 된다.

noexcept는 constexpr처럼 대부분의 함수에 적용하는 건 아니고 예외가 발생하지 않도록 유지가 가능한 함수에만 적용해야 하며 함수 내부에서 호출하는 함수들도 그러한 함수여야 한다.



