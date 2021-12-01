---
layout: post
title:  "[Cpp] Lambda expression"
date:   2021-11-02 09:00:13
categories: CppAlgorithm
---

# Lambda expression

## 1. Functor

코딩을 하다보면 전체적인 과정은 비슷한데 특정 단계에서만 다르게 동작하는 코드를 작성하는 경우가 많다. 예를 들어, 어떤 딥러닝 모델 클래스를 만든다고 생각해보자.

```cpp
class CnnModel {
    OutputType forward(InputType input) {
        input = preprocess(input);
        feature = backbone(input);
        output = head(feat);
        return output;
    }
    InputType preprocess(InputType input);
    FeatType backbone(InputType input);
    OutputType head(FeatType feature);
    FeatType feature;
    OutputType output;
}
```

CNN 모델 대부분이 입력 이미지를 받아서 전처리를 하고 일정한 backbone network를 통과해서 만든 feature를 'head'에 입력하면 출력이 나온다. 여기서 head에 따라 classifier, object detector, semantic segmentation 등 모델의 기능이 달라진다. 그럼 여기서 head 함수만 다양하게 변화시키고 싶다면 어떻게 해야할까? 여기서 제일 중요한 점은 head를 제외한 나머지 함수에 대해 코드 중복이 일어나서는 안 된다는 것이다. **아무도 이런 짓은 안 했으면 좋겠다.**

```cpp
class ClassifierModel {
    OutputType forward(InputType input) {
        input = preprocess(input);		// 공통 함수
        feature = backbone(input);		// 공통 함수
        output = head(feat);
        return output;
    }
    ...
}
class DetectorModel {					// ClassifierModel를 복사한 후 head만 수정
    OutputType forward(InputType input) {
        input = preprocess(input);
        feature = backbone(input);
        output = head(feat);
        return output;
    }
    ...
}
```

좀 더 나은 방법은 상속을 이용하는 것이다. 전체적인 진행과정을 인터페이스(부모 클래스)에서 정해놓고 그 중 일부 과정만 자식 클래스에서 오버라이딩하여 다른 기능을 할 있게 만든다. 이러한 방식을 디자인 패턴 중 하나인 **템플릿 메소드 패턴(template method pattern)**이라고 한다. 디자인 패턴이라고 부르기도 민망할 정도로 객체 지향의 기본이다.

```cpp
class CnnBaseModel {
    OutputType forward(InputType input) {
        input = preprocess(input);
        feature = backbone(input);
        output = head(feat);
        return output;
    }
    InputType preprocess(InputType input){}	// base에서 구현
    FeatType backbone(InputType input){}	// base에서 구현
    virtual OutputType head(FeatType feature) = 0;	// 구현 안함
    ...
}
class ClassifierModel {
    virtual OutputType head(FeatType feature) {}	// overriding
}
class DetectorModel {
    virtual OutputType head(FeatType feature) {}	// overriding
}
```

코등 중복을 피할 수 있긴 있는데 원래 보통 객체 지향에서 부모 클래스는 인터페이스(함수 선언 형식)를 지정하고 구체적인 구현은 자식 클래스에서 주로 하는데 이건 부모 클래스에서 대부분 구현하고 자식 클래스에서는 메소드 하나만 구현한다. 뭔가 좀 어색하다.  

이럴때 사용할 수 있는 다른 디자인 패턴으로 **전략 패턴(strategy pattern)**이 있다. 이것은 변하는 부분(head)을 클래스 내부에 두지 말고 외부에서 주입하는 것이다.

```cpp
class CnnModel {
    HeadBase *head;
    CnnModel(HeadBase *head_) : head(head_) {} 
    OutputType forward(InputType input) {
        input = preprocess(input);
        feature = backbone(input);
        output = head->forward(feat);
        return output;
    }
    ...
}
class HeadBase {
    OutputType forward(FeatType feature) {}
}
class ClassifierHead : public HeadBase {}	// 같은 메소드 오버라이딩
class DetectionHead : public HeadBase {}	// 같은 메소드 오버라이딩
```

CnnModel에는 HeadBase\*타입의 head가 있고 이것은 HeadBase를 상속한 클래스 객체의 포인터를 받을 수 있다. CnnModel 생성자에 ClassifierHead 타입을 넣으면 classifier가 될 수 있고, DetectionHead 타입을 넣으면 detector가 될 수 있다.  

그런데 전략 패턴을 쓰니 클래스 선언이 너무 많아진다. 만약 주입해야 하는 기능이 그렇게 복잡하지 않다면 간단한 함수 객체(functor)를 따로 만들어 써도 된다. 함수 객체 대신 [함수 포인터](https://boycoding.tistory.com/233)를 써도 되지만 개인적으로 함수 포인터가 C++에서 가장 난해한 코드라고 생각하기 때문에 추천하지 않는다.

```cpp
template<typename HeadType>
class CnnModel {
    HeadType head;
    CnnModel(HeadType head_) : head(head_) {} 
    OutputType forward(InputType input) {
        input = preprocess(input);
        feature = backbone(input);
        output = head(feat);
        return output;
    }
    ...
}
class ClassifierHead {
    OutputType operator()(FeatType feature) {}
}
class DetectionHead {
    OutputType operator()(FeatType feature) {}
}
int main()
{
    classifier = CnnModel(ClassifierHead());
    detector = CnnModel(DetectorHead());
}
```

하지만 이것 역시 클래스 선언이 많다. Head 클래스 마다 생성자와 소멸자 하나씩만 추가해도 줄이 크게 늘어난다. Head 구현이 복잡하다면 클래스를 따로 선언할만한 가치가 있겠지만 단 몇 줄의 코드라면 클래스를 선언하는게 좀 번거로운 느낌이 든다. 이럴때가 바로 **람다 표현식(lambda expression)**을 써야 할 때다.

```cpp
template<typename HeadType>
class CnnModel {
    HeadType head;
    CnnModel(HeadType head_) : head(head_) {} 
    OutputType forward(InputType input) {
        input = preprocess(input);
        feature = backbone(input);
        output = head(feat);
        return output;
    }
    ...
}
int main()
{
    auto clsf_head = [](FeaType feature){ /*body*/ };
    auto detc_head = [](FeaType feature){ /*body*/ };
    classifier = CnnModel(clsf_head);
    detector = CnnModel(detc_head);
}
```

람다 표현식은 클래스를 선언하지 않고 함수 내부에서 새로운 함수 객체를 만들수 있다. 클래스를 선언하지 않기 때문에 코드 라인이 크게 줄어든다. 보통 함수 객체가 사용되기 직전에 선언하기 때문에 (너무 길지 않다면) 가독성 측면에서도 더 낫다. (클래스 선언을 보러 왔다 갔다 할 필요가 없다.) 간편함이 람다 표현식의 가장 큰 장점이다. 하지만 이 간편함이 실행 코드의 크기나 속도를 개선하는 것은 아니다. 컴파일러는 람다 표현식을 익명의 클래스로 변환하여 함수 객체를 생성한다. 즉 컴퓨터가 보기에는 앞서 함수 객체(클래스)를 이용한 구현과 큰 차이가 없다는 것이다.  

람다 표현식의 가장 큰 특징은 호출 가능한 함수로 기능하는데 일반 객체처럼 변수에 담거나 함수의 입출력 인자로 사용할 수 있다는 것이다. 이러한 것을 [일급 객체(first class citizen)](https://issuemine.tistory.com/7)라 한다. 파이썬의 함수 또한 일반 객체처럼 다룰 수 있기 때문에 일급 객체다. 그래서 상황에 따라 일부분만 달라지는 알고리즘이 있을 때 그 달라지는 부분을 람다 함수로 주입하면 코드 복잡도를 크게 줄일 수 있다.

람다 함수가 가장 흔하게 사용되는 경우는 std::sort 를 쓸 때 객체 사이의 대소관계를 주입할 때다. std::sort는 객체들을 대소관계에 따라 정렬하는데 정렬하는 객체에 비교 연산자가 없거나, 아래 예시처럼 연산자로 정의되지 않은 특별한 기준으로 정렬하고 싶을때 람다 함수를 이용하여 비교 함수를 주입할 수 있다.

```cpp
auto abs_comp = [](int a, int b)(return std::abs(a) > std::abs(b));
std::vector<int> list = {3, 2, 1, 4, 5};
std::sort(list.begin(), list.end(), abs_comp)
```

앞서 배웠던 unique_ptr, shared_ptr의 커스텀 삭제자를 만들때나 간단한 콜백 함수를 만들 때 등 람다 표현식은 다양한 곳에서 유용하게 쓰일 수 있다.  

람다 표현식을 설명하기 위해 이와 관련된 두 가지 개념을 이해해야 한다.

- 클로져(closure): 람다 표현식에 의해 만들어진 실행시점 객체이다.
- 클로져 클래스: 클로저를 만드는데 쓰인 클래스를 말한다. 각각으 람다에 대해 컴파일러는 고유한 클로저 클래스를 만들어 낸다.



## 2. Lambda syntax

람다 표현식은 크게 4개 부위로 나눌 수 있다.

```cpp
[ captures ] ( params ) lambda-specifiers { body }
```

<https://en.cppreference.com/w/cpp/language/lambda>

1. captures: 캡쳐 절, 람다 함수가 정의되는 곳에서 접근 가능한 변수들 중에서 람다 함수 내부에서 사용할 변수의 범위를 지정한다. 값으로 가져올 수도 있고 참조로 가져올 수도 있다.
2. params (optional): 일반적인 함수의 입력 인자 정의와 같다. 입력인자가 없으면 생략해도 된다.
3. lambda-specifiers (optional): 람다 표현식의 다양한 속성을 지정한다.
   1. specifiers: mutable, constexpr
   2. exception: noexcept
   3. attr: [attribute specifier](https://en.cppreference.com/w/cpp/language/attributes)
   4. trailing-return-type: `-> ReturType` 형식으로 반환 타입을 명시적으로 지정한다.
4. body: 함수에서 실제 행하는 작업을 정의한다.



### 2.1. Capture Clause

#### 캡쳐 방법

대괄호[]로 이루어진 캡쳐 절은 람다 표현식의 시작을 알린다. 람다 표현식에서 사용할 외부 변수를 지정한다. 사용법은 다음과 같다. 캡쳐 절에서 &는 address operator가 아니라 참조 캡쳐를 한다는 뜻이다.

- [] : 비어있는 경우 외부 변수를 사용하지 않는다.
- [foo, bar] : 외부 변수 foo, bar를 **값**으로 캡쳐한다. (capture by value)
- [&foo, &bar] : 외부 변수 foo, bar를 **참조**로 캡쳐한다. (capture by reference)
- [foo, &bar], [&foo, bar] : 값 캡쳐와 참조 캡쳐를 섞어 쓸 수 있다.
- [=] : 값에 의한 기본 캡쳐 모드(capture-default mode by value), 현재 스코프(scope)에서 접근 가능한 모든 변수를 **값**으로 캡쳐한다.
- [&] : 참조에 의한 기본 캡쳐 모드(capture-default mode by reference), 현재 스코프(scope)에서 접근 가능한 모든 변수를 **참조**로 캡쳐한다.
- [=, &foo] : foo만 참조 캡쳐하고 나머지는 값 캡쳐를 한다.
- [&, foo] : foo만 값 캡쳐하고 나머지는 참조 캡쳐를 한다.

유의해야 할 케이스는 다음과 같다.

```cpp
struct S
{
    int bar;
    void f(int foo) {
		[foo, foo]{};	// WARNING, 하나의 변수를 중복 캡쳐할 순 없다.
		[=, foo]{};		// WARNING, foo가 이미 =에 의해 값 캡쳐됐으므로 중복 캡쳐다.
		[&, &foo]{}; 	// WARNING, foo가 이미 &에 의해 참조 캡쳐됐으므로 중복 캡쳐다.
        [=, this]{cout << bar;};	// WARNING, this 이미 =에 의해 값 캡쳐됐으므로 중복 캡쳐다.
        							// until C++20: Error: this when = is the default
                    				// since C++20: OK, same as [=]
        [=, *this]{cout << bar;};	// OK
        							// until C++17: Error: invalid syntax
                    				// since c++17: OK: captures the enclosing S by copy
        [&, this]{cout << bar;};	// OK
        [&, *this]{cout << bar;};	// OK
        
        [=] { cout << bar; }; 		// OK
        [&] { cout << bar; }; 		// OK
        [this] { cout << bar; }; 	// OK
        [*this] { cout << bar; }; 	// OK
        [=] { cout << this->bar; }; 	// OK
        [&] { cout << this->bar; }; 	// OK
        [this] { cout << this->bar; }; 	// OK
        [*this] { cout << this->bar; }; // OK
        
        [] { cout << bar; }; 		// ERROR
        [] { cout << this->bar; }; 	// ERROR
        [=] { foo = 10; }();		// ERROR, 캡쳐된 변수는 기본적으로 상수
		[=] { bar = 1; }();			// OK, this는 const S*가 아닌, S* const로 캡쳐
        [&] { foo = 10; }();		// OK, 참조 변수는 수정 가능
    }
}
```

람다 함수 캡쳐 절에서 this 포인터가 캡쳐되면 클래스의 non-static 멤버 변수들을 쓸 수 있다.

참고한 [MS의 문서](<https://docs.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170>)에서는 첫 4 가지 케이스가 에러가 날거라 했는데 일단 gcc에서는 빌드가 된다. 하지만 중복 캡쳐에 대한 WARNING이 뜨기 때문에 저런 코드는 쓰지 않는게 좋다. 4, 5번째 케이스는 [C++ reference](https://en.cppreference.com/w/cpp/language/lambda)에서도 에러가 나야한다고 나와있는데 gcc에서는 에러가 나지 않는다. :worried: 

gcc 9.3 에서 여러가지 케이스를 검사해본 결과 멤버 변수는 this가 어떻게든 캡쳐 범위에 들어가기만 하면 람다 함수 내부에서 사용할 수 있었다. 하지만 this 역참조(*this)를 값으로 넣으면 람다 함수로 들어가면서 객체가 복사되기 때문에 복사가 필요한 경우에만 주의해서 사용해야 한다.

**캡쳐된 변수들은 모두 기본적으로 상수이므로 수정할 수 없다.**



#### 초기화 캡쳐 (C++14)

> 참고자료: (Modern Effective C++) 항목 32. 객체를 클로저 안으로 이동하려면 초기화 갈무리를 사용하라

C++14부터 캡쳐 절에서 어떤 표현식을 새로운 변수명으로 받아올 수 있게됐다. 다음 예시를 보자.

```cpp
#include <iostream>
#include <memory>
struct Widget
{
    void print() { std::cout << "hello lambda\n"; }
};
int main()
{
    auto tpw = std::make_unique<Widget>();
    auto f1 = [pw = std::move(tpw)]{ pw->print(); };
    f1();
    auto f2 = [tpw]{ tpw->print(); };	// ERROR
    auto f3 = [&tpw]{ tpw->print(); };	// OK

}
```

> hello lambda

f1의 캡쳐 절에서 move 함수를 실행하고 그 결과를 pw라는 변수로 받아서 캡쳐를 했다. f2처럼 tpw을 바로 캡쳐하게 되면 값에 의한 복사가 일어나는데 unique_ptr은 복사할 수 없으므로 에러가 난다. 따라서 unique_ptr처럼 이동 전용 타입이거나 성능을 위해 이동을 통해 캡쳐하고 싶다면 초기화 캡쳐를 활용해야 한다. 물론 f3처럼 참조로 캡쳐하면 굳이 move가 필요 없을 수 있다.  

초기화 캡쳐는 마치 클래스의 생성자에서 멤버 변수를 초기화하는 것과 비슷하다. 실제로 초기화 캡쳐 변수(pw)는 클로저 클래스의 멤버 변수가 된다.

초기화 캡쳐를 통해 여러가지 표현식으로 초기값을 만들 수 있지만 대부분의 경우 람다 함수 위에서 새로운 변수를 만들어도 된다. 하지만 move의 경우 딱 목적 변수에 직접 할당을 해야하기 때문에 이런 문법이 생겨났다. 



### 2.2. Lambda specifiers

Lambda specifier는 입력 인자(parameter list)와 본문(body) 사이에서 람다 표현식의 특성을 지정해주는 키워드다. 여러가지가 있지만 여기서는 그 중 자주 사용될만한 몇 가지만 알아보자.

- mutable: 값 캡쳐된 변수들은 기본적으로 상수(const)인데 mutable을 붙이면 값을 수정하거나 객체의 비상수 멤버 함수를 사용할 수 있다. 하지만 값 캡쳐이기 때문에 람다 표현식 내부에서의 수정이 외부에 영향을 미치진 않는다.
- constexpr: 일반 함수의 리턴 타입에 constexpr을 붙이는 것과 같다. 캡쳐 변수나 입력인자가 상수표현식일 때, 출력도 상수표현식이 된다.
- noexcept: 일반 함수에 noexcept 붙이는 것과 같다. 예외 없음을 명시적으로 지정하여 컴파일러 최적화를 돕는다.



## 3. Effective Lambda

### 3.1. 기본 캡쳐 모드를 피하라

> 참고자료: (Modern Effective C++) 항목 31. 기본 갈무리 모드를 피하라

기본 캡쳐 모드는 값을 받는것과 참조를 받는것 두 가지가 있다. ([=], [&]) 둘 다 현재 위치에서 받을 수 있는 모든 변수를 람다 표현식 내부로 가져오는 것이기 때문에 코드를 쓰는 사람이나 읽는 사람이 어떤 변수를 어떻게 가져오는 것인지 파악하기 어렵다. 그럼으로인해 많은 버그의 가능성을 가지게 된다.

#### 기본 참조 캡쳐

참조 캡쳐를 할 경우 주의해야 할 점은 참조하는 대상이 사라질수도 있다는 것이다. 이것은 기본 참조 캡쳐 뿐만 아니라 명시적으로 변수를 지정한 캡쳐에서도 마찬가지다. 

```cpp
#include <iostream>
#include <vector>
struct Foo {
    decltype(auto) create_lambda() {
        std::vector<int> bar(10, 1);
        auto f = [&]{ std::cout << bar[5] << "\n"; };
        f();
        return f;
    }
};

int main() {
    Foo foo;
    auto f = foo.create_lambda();
    f();
}
```

위 예시에서 함수 내부에서 생성된 `bar`라는 변수는 지역 변수이기 때문에 create_lambda() 함수가 끝나면 사라진다. 그래서 람다 표현식 내부에서 참조로 들어갈 수는 있지만 이렇게 만들어진 람다 표현식은 create_lambda() 함수 밖에서는 못 쓰게 된다.

... 라고 했는데 gcc로 release 모드에서 잘 돌아가는 것을 확인함 :disappointed_relieved: debug 모드에서는 segmentation fault 발생

[&] 대신 [&bar]라고 써도 같은 내용이긴 한데 프로그래머가 코드를 쓰면서 좀 더 bar라는 변수에 신경을 쓰게 되어 이러한 버그를 알아챌 가능성이 높아진다. 그러므로 기본 참조 캡쳐는 가급적 피하는 것이 좋다.

#### 기본 값 참조 캡쳐

[=]처럼 값으로 모든 변수를 받으면 어디서 실행하든 상관없는 자기완결적 함수인것 같지만 this처럼 암묵적으로 들어가는 것도 있기 때문에 기본 값 참조 캡쳐도 위험할 수 있다.

```cpp
#include <iostream>
#include <vector>

struct Foo {
    int bar;
    decltype(auto) create_lambda() {
        bar = 10;
        auto f = [=]
        { std::cout << bar << "\n"; };
        f();
        return f;
    }
};

int main() {
    Foo *foo = new Foo;
    delete foo;
    auto f = foo->create_lambda();
    f();
}
```

이것 또한 foo라는 객체가 파괴되면 그 내부 변수도 사라지기 때문에 이후 람다 표현식을 사용할 때 문제가 될 수 있다.

... 라고 했는데 이것도 gcc로 release 모드에서 잘 돌아가는 것을 확인함 :disappointed_relieved: debug 모드에서는 segmentation fault 발생

