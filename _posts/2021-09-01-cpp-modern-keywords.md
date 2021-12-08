---
layout: post
title:  "[Cpp] Modern keywords"
date:   2021-09-01 09:00:13
categories: CppAlgorithm
---



# Modern Keywords

이번 강의에서는 C++11/14에서 추가된 키워드와 간단한 문법들을 모아서 정리한다. auto, 스마트 포인터, 람다 등의 굵직한 기능을 다루기 전에 중괄호 생성자, nullptr, constexpr, using, delete, override, noexcept 등의 자잘한(?) 키워드들을 정리하고 넘어가자.



## 1. Brace Initialization

> 참고자료: (Modern Effective C++) 항목 7: 객체 생성 시 괄호()와 중괄호{}를 구분하라

C++11부터 중괄호({}, brace)를 이용한 새로운 객체 초기화 문법이 생겼는데 이를 균일 초기화(uniform initialization) 혹은 중괄호 초기화(brace initialization)라 한다. 기존의 소괄호()를 이용한 초기화와 비슷하면서도 약간 다르다. 다양한 객체 초기화 방법을 테스트해 보았다.

```cpp
#include <iostream>
struct Foo {
    int n;
    Foo() { std::cout << "default constructor\n"; }
    Foo(int n_) : n(n_) { std::cout << "general constructor " << n << "\n"; }
};
int main() {
    Foo x1(1);
    Foo x2 = 2;
    Foo x3 = Foo(3);
    Foo *x4 = new Foo(4);
    Foo y1{1};
    Foo y2 = {2};
    Foo y3 = Foo{3};
    Foo *y4 = new Foo{4};
}
```

> general constructor 1  
> general constructor 2  
> general constructor 3  
> general constructor 4  
> general constructor 1  
> general constructor 2  
> general constructor 3  
> general constructor 4  

xn은 기존 초기화 방식이고 yn은 중괄호 초기화 방식이다. 이런 일반적인 객체 생성의 경우 결과적으로 모두 다 같은 결과를 낳는다. 하지만 새로운 문법이 생긴데는 이유가 있다.  

### 1.1. 중괄호 초기화 장점



#### A. 모든 문맥에서 활용

중괄호 초기화는 초기화가 필요한 모든 문맥에서 사용가능하다. 

```cpp
std::vector<int> v{1, 2, 3};	// 배열 초기값 지정
// 멤버 변수 초기화
class Widget {
    int x{0};	// OK
    int y=0;	// OK
    int z(0);	// ERROR
}
```



#### B. 좁히기 변환 금지

C++에서는 double 타입 데이터도 int 타입의 변수로 할당이 가능하다. 데이터의 손실이 일어날 수 있는 변환이지만 컴파일러는 개의치 않고 변환을 실행해준다. 이렇게 데이터 손실이 일어날 수 있는 변환을 좁히기 변환(narrowing conversion)이라 한다. float이나 unsigned int에서 int로의 변환도 똑같은 4 byte지만 표현 범위가 다르므로 좁히기 변환에 해당한다.  

좁히기 변환은 잠재적인 버그의 원인이 될 수 있기 때문에 중괄호 초기화에서는 이를 원칙적으로 금지한다... 고 되어있지만 gcc로 테스트한 결과 아래 모든 좁히기 변환에서 **Warning만 나옴**을 확인했다.

```cpp
double d = 3.14;
float f = 3.14f;
unsigned int ui = 314;
unsigned char uc = 3;
// narrowing conversions
int i1{d};
int i2{f};
int i3{ui};
char c1{ui};
char c2{uc};
```

> ...:19:12: warning: narrowing conversion of ‘d’ from ‘double’ to ‘int’ [-Wnarrowing]  
> [build]    19 |     int i1{d};  
> ...:20:12: warning: narrowing conversion of ‘f’ from ‘float’ to ‘int’ [-Wnarrowing]  
> [build]    20 |     int i2{f};  
> ...:21:12: warning: narrowing conversion of ‘ui’ from ‘unsigned int’ to ‘int’ [-Wnarrowing]  
> [build]    21 |     int i3{ui};  
> ...:23:13: warning: narrowing conversion of ‘ui’ from ‘unsigned int’ to ‘char’ [-Wnarrowing]  
> [build]    23 |     char c1{ui};  
> ...:24:13: warning: narrowing conversion of ‘uc’ from ‘unsigned char’ to ‘char’ [-Wnarrowing]  
> [build]    24 |     char c2{uc};  



#### C. Most vexing parse

**가장 성가신 구문 해석(most vexing parse)**이란 "선언으로 해석할 수 있는 것은 항상 선언으로 해석해야 한다"라는 C++의 규칙으로부터 비롯된 부작용이다.

```cpp
#include <iostream>
struct Foo {
    Foo() { std::cout << "default constructor\n"; }
    void do_something() {}
};
int main()
{
    std::cout << "parenthesis constructor\n";
    Foo f1();
    // f1.do_something();	// ERROR
    Foo f2 = Foo();
    Foo *f3 = new Foo();
    std::cout << "brace constructor\n";
    Foo b1{};
    Foo b2 = Foo{};
    Foo *b3 = new Foo{};
}
```

> parenthesis constructor  
> default constructor  
> default constructor  
> brace constructor  
> default constructor  
> default constructor  
> default constructor  

여기서 문제가 되는 줄은 f1을 생성하는 구문이다. 얼핏 보기엔 Foo 타입의 객체 f1을 기본 생성자로 생성하는 것처럼 보이지만 그게 아니다. C++ 컴파일러는 이것을 Foo 타입을 리턴하는 f1이라는 함수 선언으로 본다. 그렇게 보면 또 그렇게도 보인다. 앞서 말했다시피 C++은 선언으로 해석할 수 있는 것은 선언으로 해석하므로 f1 라인은 객체 생성이 아니라 함수 **선언**으로 해석한다. 그래서 그 아래서 멤버 함수 실행도 되지 않는다.  

결과를 보면 객체 생성을 소괄호()로 3개, 중괄호()로 3개씩 한 것 같지만 소괄호에서는 생성 메시지가 2개 밖에 나오지 않는다. 다른 언어에서는 이런 문제가 없는데 C/C++에만 있는 문제로 헷갈리기 쉬우니 주의해야 한다. 하지만 중괄호를 쓰면 이러한 문제도 해결이 된다. `Foo b1{};`은 아무 문제없이 기본 생성자로 객체를 생성한다.



### 1.2. 중괄호 초기화 주의사항

중괄호 초기치를 auto나 템플릿을 통해 형식을 자동 연역한다면 이것은 `std::initializer_list<T>`라는 템플릿 타입이 된다.

```cpp
auto x = {1, 2, 3};
std::initializer_list<int> y = {1, 2, 3};	// 윗줄과 동일
```

initializer_list를 클래스 생성자의 입력 인자로도 쓸 수 있는데 이럴 경우에 중괄호 생성자를 각별히 주의해서 써야한다. 일반 생성자와 initializer_list를 받는 생성자가 중복 적재(overloading)된 경우 컴파일러는 initializer_list 생성자를 **강하게** 선호한다. initializer_list 생성자가 가능한 상황에서는 우선적으로 선택되기 때문에 의도하지 않은 생성이 일어날 수 있다.  

```cpp
#include <iostream>
struct Widget
{
    Widget() { std::cout << "default constructor\n"; }
    Widget(bool b_, double f_) : b(b_), f(f_)
    { std::cout << "general constructor: " << b << ", " << f << "\n"; }
    Widget(bool b_, std::string s_) : b(b_), f(0.)
    { std::cout << "general string constructor: " << b << ", " << s_ << "\n"; }
    Widget(std::initializer_list<double> list) : b(*list.begin()), f(*(list.begin() + 1))
    { std::cout << "initializer_list constructor: " << b << ", " << f << "\n"; }
    bool b;
    double f;
};
int main()
{
    Widget p1(true, 1.5);
    Widget p2(1.0, 1.5);
    Widget p3(true, "hello");
    Widget b1{true, 1.5};
    Widget b2{1.0, 1.5};
    Widget b3{true, "hello"};
}

```

> general constructor: 1, 1.5  
> general constructor: 1, 1.5  
> general string constructor: 1, hello  
> initializer_list constructor: 1, 1.5  
> initializer_list constructor: 1, 1.5  
> general string constructor: 1, hello  

위 예시에서 배울 수 있는 것은 다음과 같다.

- b1의 경우 첫 번째 생성자가 적합해 보이지만 중괄호 생성 구문에서 initializer_list를 인자로 받는 생성자를 우선 선택하므로 세 번째 생성자가 실행되었다.
- b1에서 `true`는 `double 1.0` 으로 변환되었다가 다시 `bool true`로 변환되었을 것이다.
- 소괄호()로 객체를 생성하는 경우 절대 initializer_list 생성자를 호출할 수 없다. 따라서 initializer_list 생성자 호출이 우려되는 경우 소괄호()로 객체를 생성하면 된다.
- b3에서 `"hello"`는 double로 변환할 수 없으므로 어쩔수 없이(?) 두 번째 생성자가 호출된다.
- initializer_list를 입력 받는 생성자가 있을 경우 소괄호()와 중괄호{} 생성 방식에 따라 다른 생성자가 호출될 수 있다.

마지막 교훈의 대표적인 예시는 vector다. vector에는 initializer_list 생성자가 있어서 비슷한 생성 구문도 다르게 해석될 수 있다.

```cpp
std::vector foo(10, 20);	// 원소 10이 20개 들어있는 배열 초기화
std::vector foo{10, 20};	// 10, 20 두 개의 원소를 가진 배열 초기화
```



위 내용을 종합하면 객체를 생성할 때 소괄호 초기화나 중괄호 초기화 하나만 쓸 수는 없고 필요에 따라 두 가지를 적절히 섞어써야 한다는 것이다. 이때 두 가지 사항을 유의해야 한다.

1. **initializer_list를 받는 생성자는 다른 일반 생성자와 중복 적재 되지 않게 작성한다.** vector의 예시는 신기한 코드가 아니고 인터페이스의 오류로 간주된다. initializer_list를 받는 생성자를 작성할 때는 가급적 기존의 일반 생성자와 중복 적재되지 않도록 작성한다. 특히 기존에 일반 생성자를 사용하던 코드에 initializer_list를 받는 생성자를 추가하면 새로운 생성자가 객체 생성을 독차지 할 수 있다. 이러한 강력한 중복 적재 후보는 꼭 필요한 경우에만 추가해야 한다.
2. **객체를 생성할 때 소괄호와 중괄호를 세심하게 선택한다.** 두 가지를 다 쓴다고 해서 아무 원칙없이 혼용하면 코드의 일관성이 떨어진다. 두 가지 생성자를 선택하는 원칙이 있어야 한다. 대부분의 개발자는 둘 중 하나를 선택한다.
   1. 중괄호를 우선하고 일부 경우에만 소괄호 사용: 중괄호 초기화의 세 가지 장점으로 인해 중괄호 선호. 하지만 initializer_list를 받는 생성자가 있을 경우 소괄호를 적절히 활용한다.
   2. 소괄호를 우선하고 일부 경우에만 중괄호 사용: C++ 전통과의 일관성, 의도하지 않은 initializer_list 생성자 호출 때문에 소괄호 선호. 하지만 중괄호로만 가능한 경우(구체적인 값으로 컨테이너를 초기화 하는 등)에는 중괄호를 적절히 사용한다.



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



## 4. override

> 참고자료: (Modern Effective C++) 항목 12: 재정의 함수들을 override로 선언하라

> "C++에서 객체 지향 프로그래밍의 세계는 클래스, 상속, 가상 함수(virutal function)를 중심으로 돌아간다. 이 세계의 아주 근본적인 개념 중 하나는, 파생 클래스(derived class)의 가상 함수 구현이 기반 클래스(base class)의 해당 가상 함수 구현을 재정의(override) 한다는 것이다. 그런데 가상 함수의 재정의 가 얼마나 쉽게 잘못될 수 있는지를 안다면 실망할 것이다. ..."

기반 클래스의 가상 함수는 파생 클래스에서 재정의(overrride) 될 수 있다. 이것으로부터 객체 지향의 온갖 패턴들이 가능해진다. 하지만 조금만 실수해도 파생 클래스의 함수가 의도하지 않게 재정의가 아닌 새로운 함수 정의가 될 수 있다. 아래는 재정의에 실패한 사례들이다.

```cpp
class Base
{
public:
    virtual void mf1() const {}
    virtual void mf2(int x) {}
    virtual void mf3() & {}
    void mf4() const {}
};
class Derived : public Base
{
public:
    virtual void mf1() {}
    virtual void mf2(unsigned int x) {}
    virtual void mf3() && {}
    void mf4() const {}
};
int main() {
    Base *p = new Derived;
}
```

- mf1: Base에서는 const지만 Derived에서는 아니다.
- mf2: 입력 인자의 타입이 다르다.
- mf3: 참조 한정사의 종류가 다르다.
- mf4: Base에서 virutal로 선언되지 않았다.

모두 다 재정의에는 실패했지만 컴파일러에서는 경고 하나 나오지 않는다. 컴파일러로서는 사용자가 의도적으로 재정의를 피했는지 알 수 없기 때문이다. 기반 클래스의 가상 함수를 재정의하기 위해서는 파생 클래스에서 함수의 서명을 완전히 똑같이 만들어줘야 한다.  

그래서 재정의 선언은 실수하기 쉬우므로 주의깊게 해야 한다. 파생 클래스가 하나가 아닌 여러개라면 처음엔 잘 썼다고 할지라도 수정하는 과정에서 재정의 관계가 깨질 수 있다. C++11에서는 파생 클래스 함수가 기반 클래스 가상 함수를 재정의 한다는 것을 명시적으로 표현할 수 있다. 바로 파생 클래스 함수를 **override** 식별자를 넣어 선언하는 것이다.

```cpp
class Derived : public Base
{
public:
    virtual void mf1() override {}
    virtual void mf2(unsigned int x) override {}
    virtual void mf3() && override {}
    void mf4() const override {}
};
```

override로 선언한 함수는 반드시 기반 클래스의 함수를 재정의 해야하며 그렇지 못 할 경우 에러가 난다. 예시에서는 네 개의 함수 모두 에러가 난다. 제대로 정의하기 위해서는 다음과 같이 작성해야 한다.

```cpp
class Base
{
public:
    virtual void mf1() const {}
    virtual void mf2(int x) {}
    virtual void mf3() & {}
    virtual void mf4() const {}
};
class Derived : public Base
{
public:
    virtual void mf1() const override {}
    virtual void mf2(int x) override {}
    virtual void mf3() & override {}
    void mf4() const override {} // 파생 클래스에서는 virtual이 필수는 아니다.
};
int main() {
    Base *p = new Derived;
}
```

두 클래스의 함수 서명을 똑같이 맞추는 것 뿐만 아니라 기반 클래스의 mf4를 가상 함수로 선언하는 것도 코드가 제대로 컴파일되는데 필요한 일이다.

파생 클래스이 모든 재정의 함수를 override로 선언한다면 사용자에게 재정의 함수를 명시적으로 표시할 수 있고 컴파일러에게도 함수 서명이 같은지 확인하게 해준다. 또한 기반 클래스이 함수 서명을 바꿀때도 컴파일을 통해 그로 인한 피해가 어느 정도인지 파악할 수 있어서 수정할만한 가치가 있는지 판단하는데 도움이 된다.



**TODO**

- enum
- enum class
- nullptr
- using
- delete

