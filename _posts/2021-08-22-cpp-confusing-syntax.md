---
layout: post
title:  "[draft] Confusing Syntax"
date:   2020-08-22 09:00:13
categories: 2021-1-detector
---



## 0. Intorduction

이번 강의에서는 C++에서 자주 봤지만 헷갈리는 키워드나 자주 사용되지만 교육과정에서는 소외되기 쉬운 문법을 다룬다. 다시 말해 C++ 기초과정을 배우고 난 학생들이 여전히 잘 모르는 기초적인 내용을 다룬다.



## 1. const

`const` 키워드는 C++의 가장 큰 장점 중 하나다. 어떤 객체를 변경 불가능하게 만들고 컴파일러가 이 제약을 지켜준다. 어떤 값이 불변이어어야 한다는 의도를 컴파일러 및 다른 프로그래머에게 분명히 알려준다. const 객체는 코드 실행 중 객체 생성 시점에 값이 정해지는 반면 `#define`은 컴파일 시점에서 값이 고정된다. `const` 키워드는 의도하지 않은 방식으로 변수가 수정되는 오류를 줄여줄 뿐만 아니라 만들어진 코드를 읽거나 사용하는 사람에게 객체의 용도를 알려준다. 그래서 `const`를 쓸 수 있는 곳, 써도 에러가 안 나는 곳에는 `const`를 붙여주는 것이 코드 안정성에 도움이 된다. 다만 const는 변수 쓰이는 위치에 따라 의미가 달라지기 때문에 여기서 상황별 의미를 정리하고자 한다.

> 참고자료: (Effective C++) 항목 3 낌새만 보이면 const를 들이대 보자!

### 1.1. 객체 선언

아래 두 줄 모두 상수 `int` 객체가 1이라는 고정값을 가지도록 생성한다. const가 타입의 앞에 와도 뒤에 와도 결과는 같다.

```c++
const int a = 1;
int const a = 1;
```

### 1.2. 포인터 선언

const를 사용하기에 따라 포인터는 네 가지 종류가 있다. 반복자(iterator)도 포인터와 사용법이 비슷하니 비교해보자. 

```cpp
char greeting[] = "Hello";
std::vector<int> vec{1, 2, 3};
// 1. 비상수 포인터, 비상수 데이터 (주소와 값 모두 수정 가능)
char *p = greeting;
std::vector<int>::iterator iter = vec.begin();
// 2. 비상수 포인터, 상수 데이터 (주소 수정 가능, 값 수정 불가)
const char *p = greeting;
const std::vector<int>::iterator iter = vec.begin();
// 3. 상수 포인터, 비상수 데이터 (주소 수정 불가, 값 수정 가능)
char * const p = greeting;
std::vector<int>::const_iterator iter = vec.begin();
// 4. 상수 포인터, 상수 데이터 (주소와 값 모두 수정 불가)
const char * const p = greeting;
const std::vector<int>::const_iterator iter = vec.begin();
```

타입이 네 가지나 되서 헷갈리는데 이를 쉽게 기억하는 방법은 다음과 같다.

- const가 * 앞에 오면 포인터가 상수다
- const가 * 뒤에 오면 값이 상수다

### 1.3. 함수 반환 타입

함수 반환 타입을 상수로 정해 주면 에러를 줄이는데 도움이 된다.

```cpp
class Rational { ... };
const Rational operator*(const Rational& lhs, const Rational& rhs);
// const 없을 시 잘못된 사용예시
```

예를 들면 위와 같은 유리수 클래스에 대한 `*` operator 반환 값을 const로 지정하지 않으면 다음과 같은 어이없는 코드가 컴파일 될 수 있다. operator의 출력이 임시 객체라고 할지라도 `Rational` 타입이므로 값을 대입할 수 있다. 반면 `const Rational`로 출력하면 값을 수정할 수 없기 때문에 컴파일러 에러가 난다.

```cpp
Rational a, b, c;
(a*b) = c;
if ((a*b) = c)
    ...
```



### 1.4. 상수 멤버 함수

const의 진정한 묘미는 상수 멤버 함수에서 볼 수 있다. 멤버 함수 선언문 뒤에 붙는 const는 "해당 멤버 함수가 상수 객체에 대해 호출될 함수다"라는 뜻이다. 사용자에게는 이 함수가 객체를 변경할 수 있는지를 알려준다. 컴파일러에게는 이 함수가 상수 객체에서 사용할 수 있는지를 알려준다. 

> 상수 멤버 함수를 써야 하는 이유?  
>
> 참고자료: (Effective C++) 항목 20 '값에 의한 전달'보다는 '상수객체 참조자에 의한 전달' 방식을 택하는 것이 대개 낫다.
>
> 컴파일러에서 코드를 최적화 할 때 일부 기본 타입은 CPU 레지스터에 넣어서 주기억장치(RAM)에서 동작하는 것보다 빠르게 동작할 수 있다. 함수에 클래스 객체 인자를 전달할 때 '값에 의한 전달'을 하면 레지스터를 활용할 기회가 없다. 하지만 '객체 참조자 전달'을 하면 내부적으로 포인터만 보내는 것이기 때문에 레지스터를 활용한 최적화를 할 수 있다.  
>
> 이때 '상수객체 참조자'를 쓰는 이유는 대부분의 함수에서 입력인자를 읽기 전용으로 쓰이기 때문이다. 상수객체 참조자를 인자로 받은 함수는 그 객체에서 하다못해 `get_xxx()`와 같은 데이터 읽기 함수라도 쓸텐데 그 함수가 상수 멤버 함수가 아니라면 쓸 수 없다. 그래서 객체를 변경하지 않는 함수는 가급적 상수 함수로 선언하는 것이 좋다.

보통의 경우 상수 멤버 함수 하나만 만들어도 상수 객체와 일반 객체에서 모두 쓸 수 있다.

```c++
#include <iostream>
using std::cout;
using std::endl;

// 개인적으로 { 를 선언문 옆에 쓰는 것을 극혐하지만 강의자료에서는 라인 수를 줄이기 위해 옆에 쓴다.
// 실제 코딩에서는 { 는 선언문 아래 쓰는 것을 권장한다.
class TextBlock {
public:
    TextBlock(const std::string& text_) : text(text_) {}
    const char& operator[](std::size_t position) const {
        cout << "<const operator[]> ";
        return text[position];
    }
private:
    std::string text;
};

int main() {
    TextBlock tb("Hello");
    cout << "normal object: " << tb[0] << endl;
    const TextBlock ctb("Hello");
    cout << "const object: " << ctb[0] << endl;
}
```

> normal object: <const operator[]> H
> const object: <const operator[]> H

하지만 위와 같은 경우 `tb[0] = 'J'`와 같은 코드는 컴파일 에러가 난다. 출력 타입이 `const char&`이기 때문에 여기에 값을 대입할 수 없다. 그렇다고 출력 타입에서 const를 빼버리면 operator[] 자체에서 에러가 난다. `char&` 타입을 리턴하면 내부 멤버 변수의 포인터를 전달하는 것이기 때문에 상수 객체의 멤버 변수를 외부에서 수정가능해진다. 이러한 가능성을 원천적으로 차단하기 위해 상수 함수에서는 비상수 참조나 포인터의 리턴을 금지하고 있다.

그렇다고 `tb.set(0, 'J')`  같은 함수를 만드는건 직관적이지 않다. 여기서 우리의 요구사항을 정리해보면 다음과 같다.

1. 상수 객체에 대해서는 [] 연산자를 이용해 값을 읽기만 한다.
2. 일반 객체에 대해서는 [] 연산자를 이용해 값을 읽고 쓰기를 모두 한다.

이를 만족시키기 위해서는 상수 함수와 비상수 함수를 모두 구현해야 한다.

```cpp
class TextBlock {
public:
    TextBlock(const std::string& text_) : text(text_) {}
    const char& operator[](std::size_t position) const {
        cout << "<const operator[]> ";
        return text[position];
    }
    char& operator[](std::size_t position) {
        cout << "<normal operator[]> ";
        return text[position];
    }
private:
    std::string text;
};

int main() {
    TextBlock tb("Hello");
    tb[0] = 'J';
    cout << endl;
    cout << "normal object: " << tb[0] << endl;

    const TextBlock ctb("Hello");
    cout << "const object: " << ctb[0] << endl;
}
```

> <normal operator[]> 
> normal object: <normal operator[]> J
> const object: <const operator[]> H

일반 객체에서는 비상수 함수를 호출하고 상수 객체에서는 상수 함수를 호출하는 것을 확인할 수 있다. 그러므로 `tb[0] = 'J';`도 작동한다.

### 1.5. 코드 중복 회피

요구조건을 다 만족했지만 여전히 찜찜함이 남는다. 코드 중복 때문이다. 상수 함수나 비상수 함수나 내부에서 하는 일은 똑같은데 리턴 타입 때문에 함수를 두 번 쓰는 것은 [나쁜 냄새](https://ko.wikipedia.org/wiki/%EC%BD%94%EB%93%9C_%EC%8A%A4%EB%A9%9C)가 난다. 지금은 한 줄 짜리 함수지만 제대로 구현하려면 예외처리까지 포함해야 할 것이다. 

```c++
    const char& operator[](std::size_t position) const {
        if (position >= text.size() || position < 0)
            throw std::out_of_range("TextBlock.operator[]: index out of range");
        return text[position];
    }
```

혹은 더 복잡한 함수를 비상수 함수와 상수 함수 두 가지로 구현해야 할 경우도 있을 것이다. 코드 중복을 없애는 좋은 방법은 비상수 함수에서 상수 함수를 불러내는 것이다. 까다로운 캐스팅 두 번만 거치면 가능해진다.

```c++
class TextBlock {
public:
    TextBlock(const std::string& text_) : text(text_) {}
    const char& operator[](std::size_t position) const {
        cout << "<const operator[]> ";
        return text[position];
    }
    char& operator[](std::size_t position) {
        cout << "<normal operator[]> ";
        return const_cast<char&>(
            static_cast<const TextBlock&>(*this)[position]
        );
    }
private:
    std::string text;
};
```

> <normal operator[]> <const operator[]> 
> normal object: <normal operator[]> <const operator[]> J
> const object: <const operator[]> H

일반 객체에서 [] 연산자를 쓸 때 비상수 함수를 거쳐 상수 함수를 호출한다는 것을 확인할 수 있다. 그럼 저 비상수 함수에서는 뭘 하는 걸까?

1. `static_cast<const TextBlock&>(*this)[position]`: 현재 `this`는 일반 객체이므로 이를 상수 객체로 변환한다. 상수 객체로 변환 후 [] 연산자를 써야 상수 함수를 쓸 수 있기 때문이다.
2. `const_cast<char&>(...)`: 1에서 상수 함수를 통해 받아온 데이터 타입은 `const char&`인데 비상수 함수에서는 `char&` 타입으로 반환해야 하므로 `const`를 제거할 수 있는 `const_cast<char&`>` 함수를 사용하여 일반 참조로 타입을 바꿔서 리턴한다.



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

const 보다 constexpr을 쓰는게 나은 이유는 뭘까? 컴파일러의 설계에 따라 다를 수 있지만 일단 상수가 지역변수라도 컴파일 시점에 미리 선언하며 고정된 주소를 가지게 하면 변수에 메모리를 할당하는 시간을 아낄 수 있다. 혹은 변수 자체를 건너뛰고 `foo`라는 변수 대신 1이라는 숫자를 바로 대입할 수 있을 것이다. (검색을 해도 정확한 효과는 찾을 수 없었으나 이러한 효과가 예상된다.)  **constexpr은 컴파일 시점에 값이 결정되는 상수에 대해 추가적인 최적화 기회를 제공한다.**

유의할 점은 constexpr은 컴파일 시점에 알 수 있는 값으로만 초기화를 해야한다는 것이다. 일반 변수로 초기화 시 컴파일 에러가 난다.  

프로그래밍에서 constexpr의 유용한 점은 리터럴 타입(Literal type)이 들어가야 하는 곳에 constexpr 변수를 넣을 수 있다는 것이다. 리터럴 타입이란 `int a = 1;  char foo[] = "hello"`에서 1이나 "hello"처럼 코드 안에 들어간 데이터를 말한다. 컴파일 시점에 값을 알 수 있고 값이 변하지 않는다. 더 정확히 말하면 **constexpr은 정수 상수 표현식(integral constant expression)이 요구되는 문맥에서 사용할 수 있다.** 배열의 크기, 템플릿 정수 인수, enum 값 등에 10, 20 같은 리터럴 타입 말고 constexpr을 쓸 수 있다는 것이다. 다음 예시를 살펴보자.

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

정수 상수가 필요한 경우 네 가지 해결 방법이 있다.

1. 하드 코딩 (하책) : 같은 의미를 갖는 숫자이 여럿 있을 경우 일괄적으로 수정하기 어렵다. **가급적 사용하지 말것**
2. #define (중책) : 이름이 있고 수정도 한번에 할 수 있다. 하지만 컴파일시 같은 값이 여러곳에서 중복으로 복사되어 컴파일 된 코드 크기가 커질 수 있다. 에러가 났을 때 define 이름이 아니라 define 값으로  에러문이 나와서 디버깅이 어렵다.
3. constexpr (상책) : 값에 대한 메모리가 한번만 할당되고 디버깅이 쉽다.
4. enum (상책) : 3과 같음



### 2.2. 함수 반환 타입

어떤 함수의 반환 타입을 constexpr로 설정하면 





Effective C++와  Modern Effective C++참조



## 3. namespace and using

**namespace**는 C언어에는 없는 C++만의 독자적인 기능이다. '이름 공간'이라는 뜻 그대로 다른 영역에서 만들어진 변수, 함수, 클래스 등의 이름들이 중복되지 않도록 구분해주는 기능을 한다. 예를 들어 아이돌 가수 '진영'이라고만 하면 이게 B1A4인지 갓세븐인지 알 수 없고 '종현'이라고 하면 샤이니인지 씨엔블루인지, '지민'라고 하면 AOA인지 BTS인지 알 수 없다. ~~( 현재의 인지도에 따라 더 유명한 사람이 전역변수가 되기도 하지만...)~~ 이러한 동명이인을 확실히 구분하려면 앞에 그룹명을 붙여줘야 하듯이 프로그래밍에서도 이렇게 이름 앞에서 이름의 공간을 구분하는 것이 namespace다.  

프로그램에서 이름이 겹치는 것은 온갖 에러의 원인이 될 수 있으므로 어느정도 규모있는 프로젝트라면 namespace를 지정해야 한다. 그래서 STL이나 OpenCV와 같은 라이브러리들이 `std, cv` 같은 namepsace를 사용한다.  

### 3.1. namespace 선언

내부 namesapce 까지

### 3.2. namespace 활용



### 3.3. using 선언문 (using declaration)



### 3.3. using 지시문 (using directive)





## 4. enum class

Effective C++ #define 쓰지마라

