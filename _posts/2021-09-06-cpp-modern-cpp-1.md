---
layout: post
title:  "[Cpp] C++1x (1) (auto)"
date:   2021-09-06 09:00:13
categories: CppAlgorithm
---



## 1. auto

`auto` 키워드는 아마 Modern C++을 대표하는 키워드 일 것이다. 그 전까지는 프로그래머가 모든 형식을 타이핑해서 입력해줘야 했지만 이제는 auto라는 간단한 키워드를 통해 컴파일러에서 타입을 자동으로 지정하게 할 수 있다. 그럼 이제 "C++이 변수의 타입을 마음대로 할 수 있는 동적 타입 언어가 된것인가?" 하면 그건 아니다. C++은 여전히 정적 타입을 사용하며 auto 키워드에 들어갈 타입은 컴파일시 결정되며 런타임에서 변하지 않는다. 런타임에 같은 변수의 타입이 변할 수 있는 파이썬과는 근본적으로 다르다. auto는 기존의 명시적 타입 지정을 좋아하던 프로그래머에게는 호불호가 갈리기도 하지만 대체로 auto를 쓰는 것이 코드를 깔끔하게 만들뿐만 아니라 더 효율적으로 만들기 때문에 적극적인 사용이 권장된다.  

예전에는 타입이 너무 길어지면 `typedef std::vector<Point>::const_iterator vPointCIter;` 이런식으로 축약된 별명 타입을 만들어 사용하곤 했다. 하지만 이제는 auto를 쓰면 그만이다. 여기서는 auto에 의한 형식 연역 규칙을 먼저 살펴보고 auto의 장점에 대해서 정리한다.



### 1.1. template, auto 형식 연역 규칙

> 참고자료: (Modern Effective C++) 항목 1. 템플릿 형식 연역 규칙을 숙지하라
>
> ​               (Modern Effective C++) 항목 2. auto의 형식 연역 규칙을 숙지하라
>
> ​               (Modern Effective C++) 항목 4. 연역된 형식을 파악하는 방법을 알아두라

**형식 연역**이란 컴파일러가 template이나 auto로 선언한 변수의 타입(형식)을 코드에서 유도해내는 것을 의미한다. C++이 가지고 있는 `const, &` 등에 의해 다양한 형식 연역이 발생하는데 여기서는 형식이 연역되는 규칙을 정리한다. template, auto 모두 타입을 컴파일러에서 자동으로 지정한다는 공통점이 있으며 형식 연역 규칙도 거의 비슷하므로 함께 공부하는 것이 좋다.  

일반적인 template과 auto의 사용예시는 다음과 같다.

```cpp
template<typename T>
void f(ParamType param);	// ParamType 예시: T, T&, const T, const T& 등
// 어떤 표현식으로 f를 호출
f(expr);
// 어떤 표현식으로 변수 초기화
TypeSpecifier v = expr;		// TypeSpecifier 예시: auto, auto&, const auto, const auto& 등
```

template과 auto를 비교했을 때 `auto`는 `T`에 해당하고 `TypeSpecifier`는 `ParamType`과 유사하다는 것을 느낄 수 있다. `expr`의 타입에 따라서 `T, ParamType, TypeSpecifier`가 달라진다. 구체적인 예시를 하나 살펴보자.

```cpp
#include <iostream>
#include <boost/type_index.hpp>
using std::cout;
using boost::typeindex::type_id_with_cvr;

template<typename T>
void f(T& param) {			// ParamType: T&
    cout << "template type: " << type_id_with_cvr<decltype(param)>().pretty_name() << std::endl;
}

int main() {
    const int expr = 10;	// expr: const int
    f(expr);
    auto& foo = expr;		// TypeSpecifier: auto&
    cout << "auto type: " << type_id_with_cvr<decltype(foo)>().pretty_name() << std::endl;
}
```

> template type: int const&
> auto type: int const&

변수의 타입을 예쁘게 출력하기 위해 boost 라이브러리의 `type_id_with_cvr` 클래스를 활용했다. 우분투에서 boost 라이브러리는 간단히 한줄로 설치할 수 있다.

> sudo apt install libboost-dev

`decltype()`은 () 안에 들어간 변수의 타입을 추출해주는 키워드로 `int a = 1; decltype(a) b = 2;`와 같이 사용가능하다. decltype의 결과가 타입이기 때문에 `type_id_with_cvr` 함수의 템플릿 인수로 들어갈 수 있다.

소스 코드를 보면 `expr` 변수는 `const int` 타입으로 되어있는데 이것이 `T&`와 `auto&` 타입으로 들어간다. 입력 타입에 의해 상수성(constness)도 붙고 형식지정자에 의해 참조성(reference-ness)도 붙기 때문에 결과적으로 template과 auto 모두 `const int&` 타입으로 연역된다. 여기서 `T`와 `auto`는 `const int`가 된 것이다. 전체적인 형식 연역 규칙을 알기 위해서 세 가지 경우로 분류하여 설명한다.  



**Case 1: ParamType이 포인터 형식이나 참조 형식이지만 보편 참조는 아님 (T&)**

ParamType이 포인터 형식이나 참조 형식이지만 보편 참조는 아닌 경우 형식 연역은 다음과 같이 진행된다.

1. 만일 expr이 참조 형식이면 참조 부분을 무시한다.
2. expr의 형식을 ParamType에 대해 패턴 매칭(pattern matching) 방식으로 대응시켜 T의 형식을 결정한다.

위에서 살펴본 예시가 바로 이 경우다. 위 예시를 다시 살펴보면 ParamType과 TypeSpecifier가 모두 참조 형식이기 때문에 `expr`이 참조 형식이건 아니건 상관이 없다. 이후 패턴 매칭을 시켜보면 다음과 같다.

```cpp
T& <- const int
auto& <- const int
```

참조 연산자는 대응되는 것이 없고 남은 `T`와 `auto`가 `const int`가 될 수 밖에 없다. `T`와 `auto`가 정해지고 나서 거기에 참조 연산자를 붙이면 최종 타입 `const int&`가 나온다.



**Case 2: ParamType이 보편 참조임 (T&&)**

아직 보편 참조(&&), 오른값, 왼값에 대한 개념을 배우지 않아서 이해하기 어렵겠지만 일단 정리하고 이후 관련 개념을 공부한 뒤 다시 여기를 보는 것이 좋겠다.

1. 만일 expr이 왼값이면, T와 ParamType 둘 다 왼값 참조로 연역된다. 
2. 만일 expr이 오른값이면, Case 1처럼 연역된다.

```cpp
#include <iostream>
#include <boost/type_index.hpp>
using std::cout;
using boost::typeindex::type_id_with_cvr;

template<typename T>
void f(T&& param) {			// ParamType: T&
    cout << "template type: " << type_id_with_cvr<decltype(param)>().pretty_name() << std::endl;
}

int main() {
    int x = 27;
    const int cx = x;
    const int& rx = x;
    f(x);	// x는 왼값, T는 int&, param도 int&
    f(cx);	// cx는 왼값, T는 const int&, param도 const int&
    f(rx);	// rx는 왼값, T는 const int&, param도 const int&
    f(27);	// 27은 오른값, T는 int, param은 int&&

    auto&& v = x;
    auto&& cv = cx;
    auto&& rv = rx;
    auto&& lt = 27;
    cout << "auto type x : " << type_id_with_cvr<decltype(v)>().pretty_name() << std::endl;
    cout << "auto type cx: " << type_id_with_cvr<decltype(cv)>().pretty_name() << std::endl;
    cout << "auto type rx: " << type_id_with_cvr<decltype(rv)>().pretty_name() << std::endl;
    cout << "auto type lt: " << type_id_with_cvr<decltype(lt)>().pretty_name() << std::endl;
}
```

> template type: int&
> template type: int const&
> template type: int const&
> template type: int&&
> auto type x : int&
> auto type cx: int const&
> auto type rx: int const&
> auto type lt: int&&



**Case 3: ParamType이 포인터도 아니고 참조도 아님 (T)**

ParamType이 포인터도 아니고 참조도 아니라면 인수가 함수에 값으로 전달되는 것이다. 따라서 param은 주어진 인수의 복사본, 즉 새로운 객체가 된다. 이로인해 다음과 같은 규칙이 적용된다.

1. 만일 expr의 형식이 참조이면, 참조 부분은 무시된다.
2. 만일 expr이 const이면 const 역시 무시한다.

이 경우에 대한 예시는 Case 2의 예시에서 보편 참조 연산자(`&&`)를 제거하기만 하면 된다. 결과는 다음과 같다. 모두 참조와 상수성이 제거된 타입이 된다는 것을 볼 수 있다.

>template type: int
>template type: int
>template type: int
>template type: int
>auto type x : int
>auto type cx: int
>auto type rx: int
>auto type lt: int



**auto에만 적용되는 규칙**

C++11에서부터는 균일 초기화(uniform initialization)를 지원하여 `()` 뿐만 아니라 `{}`도 객체 생성자를 호출하는데 쓰일 수 있다. 아래 네 가지 구문은 모두 동일한 결과를 낸다.

```cpp
int x1 = 27;
int x2(27);
int x3 = {27};
int x4{27};
```

하지만 타입을 auto로 지정하면 `x3`의 타입이 달라진다.

```cpp
#include <iostream>
#include <boost/type_index.hpp>
using std::cout;
using boost::typeindex::type_id_with_cvr;

int main() {
    auto x1 = 27;
    auto x2(27);
    auto x3 = {27};
    auto x4{27};
    cout << "auto type x1: " << type_id_with_cvr<decltype(x1)>().pretty_name() << std::endl;
    cout << "auto type x2: " << type_id_with_cvr<decltype(x2)>().pretty_name() << std::endl;
    cout << "auto type x3: " << type_id_with_cvr<decltype(x3)>().pretty_name() << std::endl;
    cout << "auto type x4: " << type_id_with_cvr<decltype(x4)>().pretty_name() << std::endl;
}
```

> auto type x1: int
> auto type x2: int
> auto type x3: std::initializer_list\<int\>
> auto type x4: int

auto로 선언된 변수의 초기치가 중괄호 쌍으로 감싸인 형태면, 연역된 형식은 `std::initializer_list<T>`가 된다. 반면 템플릿 함수에 동일한 중괄호 초기치를 입력하면 형식 연역에 실패하여 컴파일이 거부된다.

```cpp
#include <iostream>
template<typename T>
void f(T param) {}

int main() {
    auto x = {11, 23 , 9};  // -> std::initializer_list<int>
    // f({11, 23, 9});      // 컴파일 에러
}
```

균일 초기화를 사용하는 경우 auto를 사용하는 것에 주의를 기울여야 한다.

또한 함수의 입출력 타입을 auto로 지정한 경우에는 형식 연역이 템플릿의 형식 연역을 따르므로 중괄호 초기치를 입출력 할 수 없다. 람다 함수에서도 마찬가지다. 아래 두 함수 모두 컴파일 되지 않는다.

```cpp
// 출력 타입 auto
auto createList() { return {1, 2, 3}; }
// 입력 타입 auto
std::vector<int> v;
auto reset = [&v](const auto& new_value) { v = new_value; }
reset({1, 2, 3});
```



### 1.2. auto를 써야하는 이유

> 참고자료: (Modern Effective C++) 항목 5. 명시적 형식 선언보다는 auto를 선호하라

auto는 단순히 기다란 타입을 손으로 치는 타이핑을 줄이는 효과만 있는게 아니다. 타입이 복잡한 경우 프로그래머보다 auto가 더 정확하고 효율적인 타입을 지정해줄 수 있다. 그래서 일부 경우를 제외하고는 auto를 보편적으로 사용하는 것을 현대적인 C++에서는 권하고 있다. auto의 장점에 대해 하나씩 알아보자.



**A. 초기화 강제**

auto는 초기화 데이터에 의해 형식을 연역하므로 반드시 초기화가 필요하다. 프로그램에서 초기화를 하지 않아서 생기는 버그를 막을 수 있다.

```cpp
int x1;		// 초기화되지 않을 수 있음
auto x2;	// 오류!
auto x3 = 0;	// x3는 int
```



**B. 어려운 형식 연역**

다음 코드를 보면 템플릿 함수에서 컨테이너를 쓸 때 적절한 타입을 적는게 쉽지 않다는 것을 볼 수 있다. 

```cpp
#include <iostream>
#include <vector>
#include <deque>

template<typename It>	// It: 임의의 iterator 타입
typename std::iterator_traits<It>::value_type sum(It begin, It end) {
    // iterator가 가리키는 값의 타입
    typename std::iterator_traits<It>::value_type res = *begin++;
    for(; begin!=end; ++begin)
        res += *begin;
    return res;
}

int main() {
    std::vector<int> v = {1, 2, 3};
    std::deque<int> d = {1, 2, 3};
    auto v_sum = sum(v.begin(), v.end());
    auto d_sum = sum(d.begin(), d.end());
    std::cout << "sum of container: " << v_sum << ", " << d_sum << std::endl;
    // -> sum of container: 6, 6
}
```

임의의 Iterator로부터 그것이 가리키는 타입을 추출해내기 위해서는 `std::iterator_traits<It>::value_type`라는 복잡한 타입 지정이 필요하다. 그 앞에 붙는 `typename`은 C++에서 템플릿 타입 내부에 정의된 타입을 함수나 변수로 보는 경향이 있기 때문에 `value_type`이 함수나 변수가 아닌 타입이라는 것을 알려주는 역할을 한다. 템플릿 함수에서 템플릿의 내부 타입에 대해서는 반드시 `typename`을 붙여줘야 타입으로서 작동할 수 있다... 라는 복잡한 이론이 있다.  

프로그래머는 단순히 `*begin`에 들어있는 값을 받아줄 타입이 필요할 뿐인데 이렇게 복잡한 내용을 이해해야 모든 컨테이너의 iterator에서 작동하는 `sum`이라는 함수를 만들수 있다. 하지만 이제 저 타입은 그냥 auto로 자동추론하고 리턴 타입도 auto로 쓰면 그만이다.

```cpp
template<typename It>
auto sum(It begin, It end) {	// 훨씬 간단하다.
    auto res = *begin++;
    for(; begin!=end; ++begin)
        res += *begin;
    return res;
}
```



**C. 정확하고 효율적인 형식 연역**

auto를 사용하면 상황에 맞는 타입을 더 정확하게 지정할 수 있고 그렇게 지정했을때 프로그램의 효율성이 올라가는 경우가 많다.  

람다 함수(lambda function)는 아직 배우지 않았지만 일단 auto의 효과만 느껴보자. 람다 함수는 간단히 말해 함수 내부에서 선언하는 함수 객체다. 객체이므로 변수에 담아 다른 함수로 전달할 수도 있다. 람다 함수 **객체**(클로저, closer)의 타입은 람다 함수의 입출력 형식을 포함하므로 대개 긴 표현식을 가진다. 이를 auto에 담으면 구체적인 타입이 무엇인지 별로 신경쓸 필요가 없지만 만약 명시적 타입을 지정해서 객체를 생성하고 싶다면 다음과 같이 쓸 수 있다.

```cpp
std::function<bool(const std::unique_ptr<Widget>&, const std::unique_ptr<Widget>&)> 
    uptr_widget_less =
    [](const std::unique_ptr<Widget>& p1, const std::unique_ptr<Widget>& p1)
	{ return *p1 < *p2; }
```

복잡해 보이지만 사실 두 `Widget` 객체에 대해 비교연산 하는것 뿐이다. 그래서 람다 함수를 만들때는 보통 auto로 선언하고 C++14부터는 입력인자도 auto로 선언 가능하다. 아래 함수는 짧아졌을 뿐만 아니라 포인터처럼 사용가능한 모든 타입의 값을 비교할 수 있는 범용적인 함수가 됐다.

```cpp
auto ptr_less = [](const auto& p1, const auto& p2) { return *p1 < *p2; }
```

auto의 장점이 이것으로 끝나는 것이 아니다. 클로저는 사실 원래 `std::function` 타입의 객체가 아닌데 이를 `std::function` 타입으로 캐스팅하면서 메모리도 더 많이 사용하게 되고 인라인를 제한하는 특성 때문에 `std::function`으로 선언된 객체를 호출하는 것이 auto로 선언된 객체보다 거의 항상 느리다.  

두 번째 예시는 흔히 쓰는 명시적 타입이 초기화 데이터의 실제 타입과 다른 경우다. 다음은 흔히 볼 수 있는 예시다.

```cpp
std::vector<int> v;
unsigned int sz = v.size();
```

`v.size()`의 공식적인 반환 형식은 `std::vector<int>::size_type` 인데 이 타입의 범위는 시스템마다 달라질 수 있다. `unsigned int`와는 표현 범위가 달라질 수 있다는 것이다. 여기에 auto를 쓰면 캐스팅을 거치지 않고 자연스럽게 원래의 형식을 쓸 수 있는데 명시적인 타입을 지정한 탓에 잠재적인 오류 가능성을 갖게 됐다.  

세 번째 예시는 명시적 타입이 의도하지 않은 비효율성을 불러오는 경우다.  

```cpp
std::unordered_map<std::string, int> m;
for(const std::pair<std::string, int>& p : m) { ... }
```

`std::unordered_map<std::string, int>`은 파이썬의 dictionary와 비슷하다고 보면 되는데 key-value의 타입이 정해진것 뿐이다. for문에서는 key-value를 한 쌍씩 복사하지 않고 참조로 가져오는 것을 의도했을 것이다. 하지만 `std::unordered_map`에서 key는 상수이므로 가져오는 `std::pair`의 형식은 `std::pair<std::string, int>`이 아니고 `std::pair<const std::string, int>`이다.  

컴파일러에서는 어떻게든 코드에 지정된 형식을 만들어주기 위해서 `std::pair<const std::string, int>`로 나온 pair를 복사한 임시 객체를 생성하고 그 임시 객체를 가리키는 참조 p를 만든다. 이 코드의 의도는 p를 m의 각 원소를 가리키도록 만드는 것이었겠지만 실제로는 임시 객체를 가리키며 임시 객체는 루프 끝에서 파괴된다. 이 코드는 불필요한 복사를 수행하여 효율을 떨어트릴 뿐만 아니라 의도와 다른 객체를 참조하기 때문에 버그의 가능성이 있다. 이러한 의도하지 않은 형식 불일치 역시 auto로 날려 버릴 수 있다.

```cpp
for(const auto& p : m) { ... }
```



**D. 리팩터링 자동화**

예를 들어 어떤 함수가 `unsigned int`를 리턴하고 있었는데 나중에 음수도 출력할 수 있게 `int`로 변경하게 되었다. 이 경우 리턴을 받는 변수의 타입도 수정해야 하고, 그 변수가 들어가는 다른 변수의 타입이나 함수의 입력 타입도 수정을 해야하는 번거로움이 있다. 이 경우 변수나 함수 입력인자에 auto를 썼다면 컴파일러에서 자동으로 연관된 모든 타입을 `int`로 갱신해줄 것이다.



### 1.3. auto에 대한 우려

C++을 주로 사용하던 사람들에게 auto는 코드의 가독성을 떨어뜨리는 것으로 보이기 쉽다. 예전에는 코드를 흘깃 보기만 해도 타입을 알 수 있었는데 auto는 사람이 타입을 추론하면서 읽어야한다. 나도 개인적으로 그러한 생각을 가지고 있었으나 아예 동적 타입인 파이썬을 쓰다보니 타입을 쓰는게 귀찮고 타입을 굳이 추론하지 않아도 코드를 잘 짤 수 있게 되었다. 이제는 C++에서도 비슷한 느낌으로 코딩을 할 수 있게 돼서 다행이라고 생각한다. 모든 변수를 auto로 선언할 수 있는건 아니고 그렇게 해서는 안되는 경우도 있지만 (e.g. `{}` 초기화) 대부분의 경우에 auto는 코드를 더 정확하고 효율적으로 작동하게 해준다. 







