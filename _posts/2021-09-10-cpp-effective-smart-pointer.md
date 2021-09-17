---
layout: post
title:  "[Cpp] Effective C++1x (3): Smart Pointers"
date:   2020-09-11 09:00:13
categories: CppAlgorithm
---



# Smart Pointers

C++은 C언어로부터 유래됐고 C언어는 포인터를 이용해 메모리를 수동으로 관리한다. 포인터에 적절한 메모리를 할당하는 것(메모리 효율성), 해제하는 것(메모리 누수 방지), 포인터를 할당된 메모리 범위 내에서 사용하는 것과 메모리가 해제됐는지 확인하는 것(메모리 에러 및 버그 관리)가 모두 프로그래머의 책임이다. 포인터는 수많은 버그를 양산하는 원인이지만 메모리 관리를 위한 비용이 전혀 없기 때문에 프로그래머가 전지(全知)한 상태에서 모든 경우의 수를 고려해 *적절히* 짠다면 가비지 컬렉터(garbage collector)를 사용하는 언어보다 빠르고 효율적으로 동작할 수 있다. 하지만 여러 사람이 협업하는 대규모 프로젝트에서 메모리를 완벽하게 관리하는 것은 어려운 일이다.  

다음은 원시 포인터를 사용하기 힘든 이유들이다.  

1. 선언만 봐서는 하나의 객체인지 배열인지 구분할 수 없다. (`int []` 타입을 `int*` 타입으로 치환 가능)
2. 선언만 봐서는 누가 포인터의 피지칭 객체를 **소유**하고 있는지 알 수 없고 따라서 누가 객체를 파괴해야 하는지 알 수 없다.
3. 객체를 사용자가 직접 파괴해야 한다는 것을 안다고 해도 어떻게 파괴해야 하는지 알 수 없다. `delete`를 사용해야 할 수도 있고 다른 파괴 매커니즘을 이용해야 할 수도 있다.
4. `delete`를 이용해 파괴해야 한다고 해도, 1번 때문에 `delete`로 단일 객체를 파괴해야 하는지 `delete []`로 배열을 파괴해야 하는지 알 수 없다.
5. 포인터가 피지칭 객체를 소유하고 있으며 그것을 파괴하는 구체적인 방법을 알아냈다고 하더라도, 코드의 모든 경로에서 파괴가 **정확히 한번** 일어남을 보장하기 어렵다. (이를테면 예외 때문에) 파괴가 일어나지 않는 경로가 하나라도 있으면 자원 누수가 발생하며, 파괴를 여러 번 수행하는 것은 미정의 결과를 낳는다.
6. 대체로, 포인터가 피지칭 객체를 잃었는지 알아낼 방법이 없다. 포인터가 가리키는 주소에 유효한 객체가 존재하는지 확인할 수 없다.

그래서 C++11에서는 메모리를 자동으로 관리하여 메모리 누수 및 잘못된 접근 가능성을 크게 줄인 스마트 포인터(smart pointer)라는 템플릿 클래스가 생겼다. (기존의 포인터를 *원시 포인터(raw pointer)*라고 부르기로 한다.) 스마트 포인터는 내부적으로 원시 포인터와 그에 연결된 메모리를 관리하고 있으며 스마트 포인터 객체가 파괴될 때 혹은 더 이상 사용되지 않을 때 자동으로 메모리를 해제해준다. 스마트 포인터는 원시 포인터에 비해 성능이 2~3배 차이가 나지만 요즘 PC 성능에서는 큰 비용이 아니고 사용의 안전성과 편의성을 보장하므로 생산성 향상에 큰 도움이 된다. 이제는 원시 포인터는 프로그램의 원리를 배우는 정도로만 사용하고 실제 코딩에서는 스마트 포인터나 이후에 배울 컨테이너(container)를 사용하는 것이 적극 권장된다.  

스마트 포인터는 네 가지가 있는데 그 중 `auto_ptr`은 제대로 된 C++11이라 할 수 없으므로 제외하고 나머지 세 가지 스마트 포인터에 대해 알아본다.

1. unique_ptr
2. shared_ptr
3. weak_ptr



## 1. unique_ptr

> 참고자료: (Modern Effective C++) 항목 18. 소유권 독점 자원 관리에는 std::unique_ptr를 사용하라

스마트 포인터를 사용할 때 가장 먼저 고려해야 할 것은 unique_ptr이다. unique_ptr은 원시 포인터와 거의 같은 크기고 사용법도 거의 비슷하게 사용할 수 있다. unique_ptr은 shared_ptr에 비해 작고 빠르다.  

unique_ptr은 항상 자신이 가리키는 객체를 독점적으로 소유한다. 그래서 unique_ptr은 복사가 허용되지 않는 이동 전용 형식(move-only type)이다. 널(null)이 아닌 unique_ptr은 소멸시 자신이 가리키는 객체를 파괴한다. unique_ptr이 어떻게 앞서 말한 원시 포인터의 위험성을 제거하는지 살펴보자.

1. 선언만 봐서는 하나의 객체인지 배열인지 구분할 수 없다. :arrow_right: unique_ptr은 단일 객체와 객체 배열을 구분해서 선언한다. 정수 타입인 경우 `std::unique_ptr<int>, std::unique_ptr<int[]>` 두 가지로 명확히 구분한다.
2. 선언만 봐서는 누가 포인터의 피지칭 객체를 **소유**하고 있는지 알 수 없다. :arrow_right: unique_ptr은 객체를 독점 소유하므로 어떤 유효한 객체를 가리키는 unique_ptr 객체가 있다면 그것이 객체를 소유하고 있는 것이다.
3. 객체를 사용자가 직접 파괴해야 한다는 것을 안다고 해도 어떻게 파괴해야 하는지 알 수 없다. :arrow_right: 기본적으로 delete로 파괴하고 다른 방법이 필요한 경우에는 deleter 객체를 지정할 수 있다.
4. `delete`를 이용해 파괴해야 한다고 해도, 1번 때문에 `delete`로 단일 객체를 파괴해야 하는지 `delete []`로 배열을 파괴해야 하는지 알 수 없다. :arrow_right: 1과 동일한 대답​
5. 코드의 모든 경로에서 파괴가 **정확히 한번** 일어남을 보장하기 어렵다. :arrow_right: unique_ptr이 가리키는 객체는 힙(heap)에 있어서 예외 발생시 파괴되지 않을 수 있지만 unique_ptr 자체는 스택(stack)에 선언되므로 예외 발생시 스택 풀기(stack unwinding)과정에서 unique_ptr의 소멸자가 호출된다. [참고](https://modoocode.com/230)
6. 포인터가 피지칭 객체를 잃었는지 알아낼 방법이 없다. :arrow_right: unique_ptr 이 가리키는 객체가 해제되면 반드시 내부 원시 포인터가 `nullptr`을 가지게 되므로 `nullptr`과의 비교를 통해 확인할 수 있다.



### 1.1. 생성

> 참고자료: (Modern Effective C++) 항목 21. new를 직접 사용하는 것보다 std::make_unique와 std::make_shared를 선호하라



#### 생성 방식

unique_ptr을 생성하는 방식은 unique_ptr의 생성자를 사용하는 방법과 `make_unique` 함수를 쓰는 방법이 있다.  

```cpp
#include <iostream>
#include <memory>

int main() {
    // unique_ptr 생성자 사용, 객체 생성자 인수 없음
    std::unique_ptr<int> up1(new int);
    // make_unique 사용, 객체 생성자 인수 없음
    std::unique_ptr<int> up2 = std::make_unique<int>();
    // unique_ptr 생성자 사용, 객체 생성자 인수 입력
    std::unique_ptr<int> up3(new int(3));
    // make_unique 사용, 객체 생성자 인수 입력
    std::unique_ptr<int> up4 = std::make_unique<int>(5);
    // make_unique 사용, auto 타입 활용 -> 추천!!
    auto up5 = std::make_unique<int>(5);
}
```

unique_ptr의 생성자를 사용할 때는 `new` 연산자를 이용해 객체를 직접 생성해서 그 포인터를 넘겨준다. 이보다는 `make_unique` 함수를 쓰는 것이 권장되는데 그 이유는 다음과 같다.

1. 생성자를 사용하면 타입을 두 번 써야 한다. `up1`을 만들기 위해 `int` 타입을 두 번 쓰지만 `up5`에서는 auto를 이용하므로 `int`를 한 번만 쓴다. `new` 연산자를 이용해 객체를 직접 생성하는 경우 auto를 쓸 수 없다.
2. 생성자에서는 원시 포인터를 받으므로 이런 코드도 가능하다: `int* iptr=new int; unique_ptr<int> up(iptr);` 이 경우 포인터의 소유권이 분산되므로 unique_ptr의 목적대로 동작하지 않을 수 있다.

참고로 shared_ptr도 마찬가지로 `make_shared` 함수를 쓰는 것이 좋은데 그 이유는 위에서 말한 이유 외에 두 가지가 더 있다. 그에 대해서는 shared_ptr과 함께 설명하기로 한다.



#### Custom Deleter 사용

unique_ptr에서는 객체 파괴시 기본적으로 내부의 원시 포인터에 `delete` 연산자를 적용하고 `nullptr`을 대입하지만 추가적인 처리가 필요한 경우 이를 처리할 수 있는 커스텀 삭제자(custom deleter)를 지정할 수 있다. 커스텀 삭제자로는 functor, lambda function 처럼 호출 가능한(callable) 객체를 사용할 수 있다.  

다음은 커스텀 삭제자를 지정한 예시다. 예시에서 커스텀 삭제자로 `Deleter`라는 클래스를 구현하였고 이 클래스에는 `operator()`가 정의되어 있으므로 객체를 함수처럼 호출 가능하다(functor). 입력 인자로는 삭제하고자 하는 객체의 원시 포인터를 받아야 한다. 삭제자 클래스는 unique_ptr의 템플릿 인수로 넣으면 내부에서 자동으로 삭제자 객체를 생성해준다. 혹은 명시적으로 삭제자를 생성하여 입력할 수도 있다.

```cpp
#include <iostream>
#include <memory>

struct Object {
    Object(int v_) : v(v_) { std::cout << "Object constructor\n"; }
    ~Object() { std::cout << "Object destroyer\n"; }
    int v;
};

struct Deleter {
    Deleter() { std::cout << "create Deleter\n"; }
    void operator()(Object *o)
    {
        std::cout << "Deleter operator()\n";
        delete o;
    }
};

int main() {
    {
        std::cout << "===== implicit creation of deleter\n";
        std::unique_ptr<Object, Deleter> up1(new Object(1));
    }
    {
        std::cout << "===== explicit creation of deleter\n";
        Deleter deleter;
        std::unique_ptr<Object, Deleter> up2(new Object(1), deleter);
    }
}
```

> ===== implicit creation of deleter
> Object constructor
> create Deleter
> Deleter operator()
> Object destroyer
> ===== explicit creation of deleter
> create Deleter
> Object constructor
> Deleter operator()
> Object destroyer



### 1.2. 사용법

unique_ptr의 사용법을 다음 예제를 통해 알아보자.

```cpp
#include <iostream>
#include <memory>
#include <cassert>

class MyString {
    std::string str;

public:
    MyString(std::string str_) : str(str_) { std::cout << "[create] " << str << std::endl; }
    ~MyString() { std::cout << "[delete] " << str << std::endl; }
    void print() const { std::cout << "[print] " << str << std::endl; }
    void set(std::string newstr) {
        str = newstr;
        std::cout << "[set] " << str << std::endl;
    }
};

void print_mystring(const std::unique_ptr<MyString> mystr) {
    std::cout << "[print_mystring]";
    /* 내부 함수 호출: 일반 포인터와 사용법 동일: ->, * operator */
    mystr->print();
    (*mystr).print();
}

int main() {
    /* 생성 */
    auto foo = std::make_unique<MyString>("foo");
    auto bar = std::make_unique<MyString>("bar");

    /* (중요!) std::move를 이용한 객체 이동 */
    print_mystring(std::move(foo));
    /* unique_ptr은 복사 불가! */
    // print_mystring(foo);

    /* nullptr 확인 */
    // foo은 함수 내부로 이동했으므로 객체 소유권을 잃고 nullptr만 가지고 있음
    // nullptr과의 비교 연산 지원: ==, !=
    if (foo != nullptr)
        foo->print(); // 실행되면 Segmentation fault 에러 발생
    if (foo == nullptr)
        std::cout << "foo is nullptr\n";

    /* get(): 내부 원시 포인터 접근 */
    MyString *rpstr = bar.get();
    rpstr->set("bar2");
    bar->print();
    // WARNING!! 원시 포인터를 삭제하면 bar 소멸시 Segmentation fault 에러 발생
    // delete rpstr;

    /* 참조를 이용한 자원 공유는 허용된다. */
    MyString &refshare = *bar;
    refshare.print();

    /* release(): 자원 해제(delete) */
    bar.release();
    assert(bar == nullptr);
    /* reset(T *ptr = nullptr): 자원 해제 및 새로운 자원 할당 */
    bar.reset(new MyString("bar3"));
    bar->print();
    bar.reset();
    assert(bar == nullptr);
}
```



#### 생성·이동·복사

객체 생성에 대해서는 앞서 설명했듯이 `make_unique` 함수를 쓰는게 낫다. unique_ptr 객체를 다른 함수에 전달할 때는 (혹은 다른 unique_ptr 객체에 할당할 때는) 반드시 `move` 함수를 통해 소유권을 넘겨줘야 한다. 소유권을 넘겨준다는 것은 내부 포인터를 다른 객체에 넘겨주고 자신은 nullptr이 되는 것이다.  

아래 코드 마지막 줄처럼 객체 자체를 넘겨주면 입력 인자 전달 과정에서 객체 복사를 시도하는데 unique_ptr은 복사 생성자를 [삭제 선언](http://progtrend.blogspot.com/2017/03/deleted-functions.html) 했으므로 복사는 컴파일되지 않는다. `print_mystring`의 입력인자를 참조 타입으로 하면 마지막 줄도 가능하긴 하다.

```cpp
    auto foo = std::make_unique<MyString>("foo");
    auto bar = std::make_unique<MyString>("bar");
    print_mystring(std::move(foo));
    // print_mystring(foo);
```



#### nullptr 확인

unique_ptr은 소유권을 넘겨주고 나면 nullptr만 남게 되므로 이후에는 사용해서는 안된다. unique_ptr이 객체를 소유하고 있는지 확신할 수 없다면 사용하기 전에 확인할 필요가 있다. unique_ptr은 nullptr과 비교연산자(==, !=)를 지원하여 `get()`함수로 원시 포인터를 꺼내지 않고 바로 nullptr과 비교할 수 있다.


```cpp
    if (foo != nullptr)
        foo->print();
    if (foo == nullptr)
        std::cout << "foo is nullptr\n";
```

>  nullptr은 C++11에서 도입된 널 포인터 리터럴(null pointer literal)이다. 오로지 빈 포인터를 확인하기 위한 값이며 C++11 이전에 포인터에 0이나 NULL을 넣어서 빈 포인터를 확인하는 것보다 가독성이 좋고 0이라는 int 타입과 헷갈리지 않게 된다. [참고](https://blockdmask.tistory.com/501)



#### 자원 공유

unique_ptr은 원래 자원을 독점 소유하도록 만들어졌지만 약간의 구멍(?)은 남겨뒀다. 내부 포인터를 외부에서 접근한다거나 내부 객체에 대한 참조 변수를 만들 수 있다. 다만 내부 객체의 수명이 다했을 때 발생하는 오류는 프로그래머의 책임이니 주의해서 써야한다.

```cpp
    MyString *rpstr = bar.get();
    rpstr->set("bar2");
    bar->print();
    // WARNING!! 원시 포인터를 삭제하면 bar 소멸시 Segmentation fault 에러 발생
    // delete rpstr;
    MyString &refshare = *bar;
    refshare.print();
```



#### 자원 해제

스마트 포인터기 때문에 자원 해제를 명시적으로 해줄 필요는 자주 없지만 그런 기능을 지원하고 있다. 또한 내부 자원을 해제하고 새로운 자원을 받기도 한다.

```cpp
    bar.release();
    assert(bar == nullptr);
    /* reset(T *ptr = nullptr): 자원 해제 및 새로운 자원 할당 */
    bar.reset(new MyString("bar3"));
    bar->print();
    bar.reset();
    assert(bar == nullptr);
```



### 1.3. Factory 함수 예시

unique_ptr이 전형적으로 사용되는 예시는 팩터리(factory) 함수의 반환 형식으로 쓰이는 것이다. 다음과 같은 함수 계통 구조가 있다고 하자.

- Investment (base)
  1. Stock
  2. Bond
  3. RealEstate

이런 계통 구조에 대한 팩터리 함수는 흔히 힙에 객체를 생성하고 그 포인터를 돌려준다. 그 객체를 사용하지 않게 됐을때 삭제하는 것은 호출자의 책임이므로 unique_ptr은 이런 용법에 적합하다.






