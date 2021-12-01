---
layout: post
title:  "[Cpp] Move Semantics"
date:   2021-11-02 09:00:13
categories: CppAlgorithm
---



# Move Semantics



## 1. Introduction

C++11부터 *이동*이란 개념이 생겼다. 기존에는 할당 연산자(=)를 통해 *복사*만 할 수 있었다면 이제는 값비싼 복사 대신 저렴한 이동 연산을 통해 성능을 최적화 할 여지가 많아졌다. 하지만 이동 연산을 위해 C++의 문법적인 깊이가 더욱 깊어져버렸다. 이동 의미론(move semantics)를 깊이 있게 파고드는 건 추후 각자의 몫으로 남기고 여기서는 이동 의미론을 활용하는 전형적인 케이스에 대해 주로 배울 것이다.  

C++은 프로그래머가 최적화 기법들을 마음껏 적용할 수 있는 언어다. 대표적인 기법이 인라인 함수를 통해 함수 호출을 줄이는 것, 참조를 이용해 복사를 줄이는 것 등이 있다. 메모리 복사는 최적화에서 가장 중요한 이슈 중 하나다. **이동**이란 복사 없이 어떤 객체에 새로운 데이터를 주입시키는 작업이다. A라는 객체의 값을 B로 옮기는데 복사가 일어나지 않는거다. 어떻게 이런게 C++11부터 갑자기 가능해졌을까? 답을 알면 약간 실망할지도 모른다. 전형적이고 간단한 예시를 하나 들어보자.   

 

```cpp
#include <iostream>
#include <cstring>

struct Vector
{
    int *data;
    int n;
    Vector(int _n) : n(_n), data(nullptr)
    {
        std::cout << "[Vector] basic constructor: n = " << n << "\n";
        data = new int[n];
    }
    Vector(const Vector &o) : n(o.n), data(nullptr)
    {
        std::cout << "[Vector] copy constructor: n = " << n << "\n";
        data = new int[n];
        memcpy(data, o.data, n * 4);
    }
    ~Vector() { if(data) delete[] data; }
};

// you can process foo's data without copy or move
void process(Vector &bar)
{
    bar.data[1] = 3;
    std::cout << "[process] Vector, bar[1] = " << bar.data[1] << "\n";
}

struct ComplexData
{
    Vector ints;
    // you have to copy Vector to a member variable
    ComplexData(const Vector &other) : ints(other)
    {
        std::cout << "[ComplexData] create complex by copy\n";
    }
};

int main()
{
    Vector foo(10);
    process(foo);
    ComplexData baz(foo);
    return 0;
}
```

> [Vector] basic constructor: n = 10  
> [process] Vector, bar[1] = 3  
> [Vector] copy constructor: n = 10  
> [ComplexData] create complex by copy  

여기까지는 무난하다. 다수의 메모리를 사용하는 `Vector`라는 클래스가 있다. 일반 생성자와 복사 생성자를 구현했다. main 함수에서는 Vector 객체 foo를 생성해서 두 가지로 사용했다.

1. process() 함수 입력: 참조로 입력했기 때문에 복사가 일어나지 않는다. 상수가 아니기 때문에 내부에서 값도 수정가능하다. main()의 foo를 의도적으로 수정하는 함수거나 아니면 main()에서 foo가 더 이상 쓰이지 않아서 foo의 값 변화를 신경쓰지 않는 함수다.
2. ComplexData 생성자 입력: 이번에는 ComplexData 내부의 vec라는 멤버 변수를 초기화 하는데 사용됐기 때문에 복사를 피할수 없다. 복사 생성자가 실행된다.

복사를 피하고 싶은 경우 대부분 참조 변수로 전달하면 해결이 되지만 새로 만들거나 이미 만들어진 객체의 내용을 채울때는 복사를 해야만 한다. 하지만 이 경우에도 *이동* 연산을 이용하면 복사를 피할 수 있다. 이동은 어떻게 구현하고 어떻게 사용될까?

```cpp
#include <iostream>
#include <cstring>

struct Vector
{
    int *data;
    int n;
    Vector(int _n) : n(_n), data(nullptr)
    {
        std::cout << "[Vector] basic constructor: n = " << n << "\n";
        data = new int[n];
    }
    Vector(const Vector &o) : n(o.n), data(nullptr)
    {
        std::cout << "[Vector] copy constructor: n = " << n << "\n";
        data = new int[n];
        memcpy(data, o.data, n * 4);
    }
    Vector(Vector &&o) : n(o.n), data(nullptr)
    {
        std::cout << "[Vector] move constructor: n = " << n << "\n";
        data = o.data;
        o.data = nullptr;
        o.n = 0;
    }
    ~Vector()
    {
        if (data)
            delete[] data;
    }
};

struct ComplexData
{
    Vector vec;
    ComplexData(const Vector &other) : vec(other)
    {
        std::cout << "[ComplexData] create complex by copy\n";
    }
    ComplexData(Vector &&other) : vec(std::move(other))
    {
        std::cout << "[ComplexData] create complex by move\n";
    }
};

int main()
{
    Vector foo(10);
    std::cout << "---baz(foo);\n";
    ComplexData baz(foo);
    std::cout << "---bar(Vector(7));\n";
    ComplexData bar(Vector(7));
    std::cout << "---qux(std::move(foo));\n";
    ComplexData qux(std::move(foo));
    std::cout << "foo.n = " << foo.n << ", foo.data = " << foo.data << "\n";
    return 0;
}
```

> [Vector] basic constructor: n = 10  
> ---baz(foo);  
> [Vector] copy constructor: n = 10  
> [ComplexData] create complex by copy  
> ---bar(Vector(7));  
> [Vector] basic constructor: n = 7  
> [Vector] move constructor: n = 7  
> [ComplexData] create complex by move  
> ---qux(std::move(foo));  
> [Vector] move constructor: n = 10  
> [ComplexData] create complex by move  
> foo.n = 0, foo.data = 0  



새로운 예시에서는 Vector에 **이동 생성자**(`Vector(Vector &&o)`)가 추가되었다. 이동 생성자는 다른 객체의 데이터를 *훔쳐온다*. o가 가지고 있던 포인터를 복사해서 가져가고 o.data에는 nullptr을 넣어서 못 쓰게 만든다. 별게 없다. 포인터를 훔쳐가는 것 뿐이다. 그럼으로써 o.data에 할당된 메모리를 모두 복사하는 대신 포인터의 복사로 비용을 크게 줄였다. 이 말은 곧 int 등의 primitive(=built-in) type에는 별 효과가 없다는 뜻이다. 힙에 할당해야하는 큰 메모리가 있을 때 메모리 복사 대신 포인터 복사로 대체하는 것이다. 이동 생성자를 보면 `o.data = nullptr`을 제외하면 컴파일러가 자동 생성하는 복사 생성자와 비슷하다. 그렇다면 왜 원래 객체를 굳이 못 쓰게 만들까? 포인터를 복사하기 때문에 원래 객체의 포인터를 놔두면 객체의 소유권을 두 개의 포인터에서 가지게 된다. 그러면 앞서 말한 원시 포인터의 여러 단점들이 드러나기 때문에 원래 객체는 소유권을 해제하는게 안전하다.  

ComplexData 클래스에도 변화가 생겼다. 이동 연산을 통해 멤버 변수 vec를 초기화 하기 위해 `ComplexData(Vector &&other)`가 추가되었다. 참조 연산자가 두 개 붙은 && 연산자는 오른값(rvalue) 참조이며 거칠게 말해 임시 객체에 대한 참조를 의미한다.

`bar(Vector(7));`는 생성자 입력인자 자리에서 직접 생성한 Integer 객체가 들어간다. 이 객체는 임시 객체인 리터럴(literal)에 해당하며 따라서 이동 연산을 사용하는 두 번째 생성자가 호출된다. `std::move(other)`에 의해 other도 오른값 참조가 되어 Integer의 이동 생성자가 실행이 된다. 임시 객체의 데이터를 복사하지 않고 포인터만 가져가고 이후 임시 객체는 파괴될 것이기 때문에 버려져도 상관이 없다.

`qux(std::move(foo));`에서 foo는 임시 객체가 아니고 이미 존재하는 왼값(lvalue) 객체다. 하지만 std::move 함수를 통해 오른값으로 캐스팅되어 여기서도 이동 연산을 하는 두 번째 생성자가 호출된다. 이후로는 bar의 경우처럼 이동 연산이 일어난다. 이후에 다시 foo의 값을 확인해보면 nullptr이 된 것을 볼 수 있다.

아직 오른값, 왼값, std::move 등에 대해 배우지 않았기 때문에 이해하기 어렵겠지만 여기서 중요한 점은 이동 연산자(여기서는 이동 생성자)를 정의하여 필요에 따라 복사를 시킬수도 있고 이동을 시킬수도 있다는 것이다. 새로운 객체를 만들면서 원래 객체도 유지해야 한다면 복사를 해야하고 원래 객체를 유지할 필요가 없다면 이동이라는 저렴한 연산을 선택하는 것이 낫다. 그럼 이제 세부적인 내용을 알아보자.



## 2. Value Categories (Lvalue, Rvalue)

이동 연산이 생기면서 값들의 종류가 세분화됐다. 역사적으로는 단순히 할당 연산자(=)의 왼쪽에 있는 값을 왼값(lvalue), 오른쪽에 있는 값을 오른값(rvalue)라고 불렀다. 하지만 이동 연산과 복사 연산을 해야하는 상황을 컴파일러가 인식하기 위해서는 그보다는 미묘한 구분이 필요하다. 왼값, 오른값은 위치와는 약한 상관관계를 가지지만 그것이 전부는 아니며 그냥 새로운 개념의 고유명사로 인식하는 것이 낫다. 이러한 표현식의 분류를 값 범주(value category)라고 하며 실제로는 더 다양한 분류가 있다. 이들을 구분하는 주요 기준 중 하나가 식별자(identifier)가 있냐는 것인데, 식별자란 쉽게 말해 변수, 함수처럼 이름이 있어서 코드 내에서 언제든 불러낼 수 있는 객체를 말한다.  

C++에서는 모든 값을 lvalue 또는 rvalue로 분류할 수 있다.  

- lvalue: 일반적으로 lvalue는 식별자가 있어서 단일 표현식 이후에도 사라지지 않고 지속되는 객체다. 변수를 선언하면 변수를 사용하는 어떤 라인이 끝나도 그 변수는 계속 사용가능하다. lvalue의 특징은 주소값을 얻을 수 있다는 것이다. 
- rvalue: rvalue는 표현식이 종료된 이후에는 사라지는 임시적인 값이다. `int a=1`에서 1과 같은 리터럴이나, 함수의 반환값 등에 해당한다. lvalue가 아닌 모든 값은 rvalue이다. rvalue의 특징은 주소를 얻을 수 없다는 것이다.

그럼 `int a=1; int b=a;`라는 코드의 둘째줄에서 `a`는 lvalue 일까, rvalue 일까? `a`는 주소를 얻을 수 있는 lvalue이지만 오른쪽에 있기 때문에 rvalue로 사용한다. 컴파일러에서는 lvalue-to-rvalue conversion을 통해 `a`의 값을 가져온다. 그래서 *형식상으로 lvalue지만 의미적으로 rvalue이다.* 라는 괴상한 말도 할 수 있다. 반대로 `int &&c = 1;`에서 `c`의 타입은 rvalue 참조지만 주소를 얻을 수 있으므로 왼값이다.

아래 그림은 상황별로 세분화한 분류다. 하지만 lvalue와 rvalue가 타입이나 위치에 의해서만 정해지는 것이 아니라 상황별로 달라질 수 있다 정도만 이해하고 lvalue, rvalue라는 용어만 사용해도 이동 의미론 설명에 큰 지장이 없으므로 세부적인 분류는 중간중간 기회가 될 때 설명한다.

![value_category](../assets/cppalg/value_categories.png)



## 3. rvalue reference and std::move()

> 참고자료
>
> - (Modern Effective C++) 항목 23. std::move와 std::forward를 숙지하라
> - (Modern Effective C++) 항목 24. 보편 참조와 오른값 참조를 구분하라
> - (Modern Effective C++) 항목 25. 오른값 참조에는 std::move를, 보편 참조에는 std::forward를 사용하라

오른값 참조(rvalue reference)는 `Type&&` 형식으로 사용하며 말 그대로 오른값에 대한 참조 형식이다. 오른값이란 어떤 표현식이 끝나면 사라질 임시 객체라고 했는데 오른값 참조를 이 임시 객체에 묶으면(bind) 오른값 참조변수의 수명만큼 임시 객체의 수명도 늘어난다. 오른값을 위한 참조이기 때문에 오른값에만 묶일수 있고 왼값에는 사용하지 못한다. 다음은 할당 가능한 관계를 정리한 표다. gcc로 직접 테스트 해보았다. 오류가 나는 줄은 주석처리를 하였다.

```c++
int main()
{
    /* prepare different rhs types */
    int rhs_lval = 1;
    int &rhs_lref = rhs_lval;
    int &&rhs_rref = 1;
    const int &rhs_clref = rhs_lval;

    /* assign to lvalue */
    int lval2lval = rhs_lval;
    int lval2lref = rhs_lref;
    int lval2rval = 1;
    int lval2rref = rhs_rref;
    int lval2clref = rhs_clref;

    /* assign to lvalue reference */
    int &lref2lval = rhs_lval;
    int &lref2lref = rhs_lref;
    // int &lref2rval = 1;
    int &lref2rref = rhs_rref;
    // int &lref2clref = rhs_clref;

    /* assign to constant lvalue reference */
    const int &clref2lval = rhs_lval;
    const int &clref2lref = rhs_lref;
    const int &clref2rval = 1;
    const int &clref2rref = rhs_rref;
    const int &clref2clref = rhs_clref;

    /* assign to rvalue reference */
    // int &&rref2lval = rhs_lval;
    // int &&rref2lref = rhs_lref;
    int &&rref2rval = 1;
    // int &&rref2rref = rhs_rref;
    // int &&rref2clref = rhs_clref;

    return 0;
}
```

`lhs`는 할당 연산자의 왼쪽에 오는 표현식이고 `rhs`는 오른쪽에 오는 표현식이다. 결과를 표로 요약하면 다음과 같다.

| lhs ＼ rhs       | Lvalue | Lvalue ref | Rvalue | Rvalue ref | const Lvalue ref |
| ---------------- | ------ | ---------- | ------ | ---------- | ---------------- |
| Lvalue           | O      | O          | O      | O          | O                |
| Lvalue ref       | O      | O          | X      | O          | X                |
| const Lvalue ref | O      | O          | O      | O          | O                |
| Rvalue ref       | X      | X          | O      | X          | X                |

결과를 보면 오른값 참조에는 '진짜' 오른값만 들어갈 수 있고, 왼값 참조에는 왼값만 들어갈 수 있는데, 오른값 참조 자체는 왼값으로 취급된다는 것을 알 수 있다. 그냥 왼값이나 상수 왼값 참조에는 오른값 왼값 모두 들어갈 수 있다.  

참고로, 상수 오른값 참조(const rvalue reference, `const Type&&`)는 문법적으로 가능하긴 하지만 거의 쓰이진 않는다. 오른값은이동을 위한 것인데 상수는 이동할 수 없기 때문이다. 이동하면 일반적으로 원래 객체는 파괴되는데 상수기 때문에 파괴할 수 없다. 따라서 이동할 수 없다는 결론이 나온다.  

우리가 오른값 왼값의 특성에 대해 알아보는 것은 이동 연산자(이동 생성자, 이동 할당 연산자)가 어떤 조건에서 실행될 수 있는지 알아보기 위해서다. 이동 연산자는 오른값 참조를 입력 타입으로 받는데 (`Vector(Vector &&o)`) 그럼 받을 수 있는 값이 오른값 밖에 없게된다. 오른값은 그때그때 만드는 임시 객체인데, 우리가 보통 함수 입력인자로 넣는 것은 기존에 어떤 처리과정을 거쳐서 만들어진 왼값 객체다. (물론 왼값 객체를 이동 연산자에 넣으면 기존 객체가 파괴되기 때문에 주의해서 써야한다.) 왼값 객체를 오른값처럼 이동시키려면 어떻게 해야할까? 이런 상황을 위해서 **std::move()**라는 함수가 있다.

move라는 이름만 보면 마치 이 함수 자체에서 객체를 *이동*시켜줄 것 같지만 실제로는 **입력된 파라미터를 오른값으로 캐스팅** 하는 역할만 한다. 실행 시점에서는 아무것도 하지 않는다. C++14에서 move 함수를 다음과 같이 구현할 수 있다.

```cpp
template<typename T>
decltype(auto) move(T&& param)
{
    using ReturnType = std::remove_reference_t<T>&&;
    return static_cast<ReturnType>(param);
}
```

템플릿 함수의 인자는 `T&&`이다. 오른값 참조처럼 생겼지만 템플릿 타입의 경우 **전달 참조(forwarding reference)**라 부른다. 앞서 [template, auto 형식 연역 규칙](https://goodgodgd.github.io/ian-lecture/archivers/cpp-effective-auto#1-template-auto-%ED%98%95%EC%8B%9D-%EC%97%B0%EC%97%AD-%EA%B7%9C%EC%B9%99)을 배울때 자세히 언급하지 않고 지나갔지만 Case 2에서 전달 참조의 경우 들어오는 입력인자가 왼값(참조)면 왼값 참조로 연역되고 오른값(참조)면 오른값 참조로 연역된다.  

함수 내부에서 하는 일은 캐스팅 밖에 없는데 타입이 길어서 `using`을 사용했다. `remove_reference_t` 함수는 `T` 자체에 붙은 참조를 제거한다. 이후 다시 오른값 참조(&&)를 붙이므로 `ReturnType`은 확정적으로 오른값 참조가 된다. 템플릿 연역 규칙에 따라 들어오는 입력 인자가 왼값인 경우 `T` 자체가 왼값 *참조*가 되므로 (`T=SomeType&`) 이런 경우에 대비하여 참조를 제거하고 오른값 참조(&&)를 붙여야 모든 경우에도 확실히 오른값 참조를 만들 수 있다.

리턴 타입은 `decltype(auto)`인데 이것은 반환 값의 타입을 decltype의 형식 연역 규칙에 따라 정한다는 것이다. 즉 `decltype(auto) rtval = static_cast<ReturnType>(param);`을 했을때 연역되는 형식으로 리턴한다는 것이다. 그럼 여기서 반환 타입은 `ReturnType` 그대로 오른값 참조 형식으로 리턴된다.  

### 3.1. 언제 '이동'이 일어날까?

move를 이용한 이동 연산을 확인하기 위해 `Vector` 클래스를 확장하여 이동과 복사 연산들을 모두 구현하였다. 

```cpp
#include <iostream>
#include <cstring>

#define RULE_OF_FIVE

struct Vector
{
    int *data;
    int n;
    Vector() = default;	// 기본 생성자
    Vector(int _n) : n(_n), data(nullptr)	// 일반 생성자
    {
        std::cout << "[Vector] general constructor: n = " << n << "\n";
        data = new int[n];
    }
    ~Vector()	// 소멸자
    {
        if (data)
            delete[] data;
    }
    Vector(const Vector &o) : n(o.n), data(nullptr)	// 복사 생성자
    {
        std::cout << "[Vector] copy constructor: n = " << n << "\n";
        data = new int[n];
        memcpy(data, o.data, n * sizeof(int));
    }
    Vector &operator=(const Vector &o)	// 복사 할당 연산자
    {
        std::cout << "[Vector] copy assignment: n = " << o.n << "\n";
        if (this == &o)
            return *this;
        if (this->n != o.n)
        {
            delete[] data;
            this->data = new int[o.n];
        }
        this->n = o.n;
        memcpy(this->data, o.data, o.n * sizeof(int));
        return *this;
    }
#ifdef RULE_OF_FIVE
    Vector(Vector &&o) noexcept : n(o.n), data(nullptr)	// 이동 생성자, 이동 중에 예외 발생 금지
    {
        std::cout << "[Vector] move constructor: n = " << this->n << "\n";
        this->data = o.data;
        o.data = nullptr;
        o.n = 0;
    }
    Vector &operator=(Vector &&o) noexcept	// 이동 할당 연산자, 이동 중에 예외 발생 금지
    {
        std::cout << "[Vector] move assignment: n = " << o.n << "\n";
        if (this->data)
            delete[] data;
        this->data = o.data;
        this->n = o.n;
        o.data = nullptr;
        o.n = 0;
        return *this;
    }
#endif
};
```

main 함수에서는 어떤 경우에 이동 연산이 되는지 확인해보았다.

```cpp
int main()
{
    Vector val(1);
    Vector &lref = val;
    const Vector &clref = val;
    Vector v1 = val;              // copy
    Vector v2 = lref;             // copy
    Vector v3 = clref;            // copy
    Vector v4 = std::move(val);   // move
    val = Vector(1);              // move
    Vector v5 = std::move(lref);  // move
    Vector foo(2);
    val = foo;                    // copy
    Vector v6 = std::move(clref); // copy
    return 0;
}
```

> [Vector] general constructor: n = 1  
> [Vector] copy constructor: n = 1  
> [Vector] copy constructor: n = 1  
> [Vector] copy constructor: n = 1  
> [Vector] move constructor: n = 1  
> [Vector] general constructor: n = 1  
> [Vector] move assignment: n = 1  
> [Vector] move constructor: n = 1  
> [Vector] general constructor: n = 2  
> [Vector] copy assignment: n = 2  
> [Vector] copy constructor: n = 2  

- `v1~v3`는 기존 객체에서 복사가 되는게 당연하다.
- `v4`에서는 move 함수를 이용하여 왼값 객체를 오른값으로 변환하여 이동 생성을 실행할 수 있었다.
- 이동을 하고 나면 원래의 `val`의 자원이 해제되므로 다시 새로운 객체(`Vector(1)`)를 할당했는데 이 객체가 오른값(임시 객체)이므로 이동 할당 연산이 실행된다.
- `v5`에서는 왼값 참조도 move 함수를 이용하여 변환하여 이동 생성을 실행할 수 있었다.
- 다시 이동을 했으므로 새로운 값을 넣어준다. 이번에는 객체 변수를 만들어 왼값으로 넣었더니 복사 할당 연산자가 실행됐다.
- `v6`에서는 move 함수를 썼지만 move에서 const를 없애진 못한다. 여기서 들어오는 값이 앞서 언급한 상수 오른값 참조다. 이것도 상수이므로 이동을 못하고 복사를 한다.



### 3.2. 함수의 반환 값은 어떻게 줘야할까?

이동 연산은 기존 객체를 파괴하기 때문에 신중하게 써야하지만 함수 내부에서 지역 변수를 리턴할 때는 std::move로 이동을 시키는게 좋지 않을까? 결론적으로 함수 리턴에 std::move를 쓰는 것은 대부분의 경우에 좋은 생각이 아니다. 대부분의 경우 리턴 값 최적화(RVO, Return Value Optimization)를 이용하는 것이 낫다. 케이스 별로 나눠서 살펴보자. 함수 내부에서 리턴되는 변수와 함수 외부에서 결과를 받는 변수의 성격에 따라 다를수 있다.

#### 3.2.1. 지역 변수 :arrow_right: 지역 변수 생성

함수 내부에서 지역 변수를 만들고 이 변수를 리턴하여 함수 밖에서 새로운 변수를 만든다면 컴파일러에서는 자동으로 RVO를 적용하여 복사 생략(copy elision)할 수 있다. 아래 예시에서 함수 내부에서 만들어진 `v`가 바로 함수 밖의 `v1`이 된다.

```cpp
Vector f1() {
    Vector v(2);
    return v;
}
int main() {
    Vector v1 = f1();
}
```

> [Vector] general constructor: n = 2  

RVO를 적용하려면, 즉 컴파일러가 값 전달 방식으로 반환하는 함수의 어떤 지역 객체의 복사/이동을 제거할 수 있으려면

1. 그 지역 객체의 형식이 함수의 반환 형식과 같아야 한다
2. 그 지역 객체가 바로 함수의 반환 값이어야 한다.

실행 결과를 봐도 함수 내부에서 일반 생성자가 실행될 뿐 아무런 복사가 일어나지 않는다.  

여기서 함수 `f`의 리턴문에 std::move를 적용하면 반환하는 형식이 `Vector&&`가 되므로 1번 조건을 만족하지 않는다. 이 경우, 이동 생성자가 있으면 이동 생성을 하고, 없으면 복사 생성을 하게 된다. 불필요한 이동 또는 복사가 일어나게 되는것이다. 프로그래머가 컴파일러의 최적화를 오히려 방해한 것이다.

```cpp
Vector f2() {
    Vector v(2);
    return std::move(v);
}
int main() {
    Vector v1 = f2();
}
```

> [Vector] general constructor: n = 2  
> [Vector] move constructor: n = 2  



#### 3.2.2. 지역 변수 :arrow_right: 지역 변수 할당

함수의 리턴 값을 이미 생성된 지역 변수에 할당하는 것은 정말로 std::move를 통해 이동을 시켜줘야 하는것은 아닐까?  그것도 아니다. 컴파일러에서는 RVO를 적용하지 못하는 경우에 리턴 값을 자동으로 오른값으로 취급한다.

```cpp
Vector f1()
{
    Vector v(1);
    return v;
}
Vector f2()
{
    Vector v(2);
    return std::move(v);
}
int main()
{
    Vector v1(3);
    std::cout << "return by value\n";
    v1 = f1();
    std::cout << "return by right value reference\n";
    v1 = f2();
}
```

> [Vector] general constructor: n = 3  
> return by value  
> [Vector] general constructor: n = 1  
> [Vector] move assignment: n = 1  
> return by right value reference  
> [Vector] general constructor: n = 2  
> [Vector] move constructor: n = 2  
> [Vector] move assignment: n = 2  

`v1=f1();`에서는 리턴된 값을 오른값으로 취급하여 이동 할당 연산자가 있으면 이동을 하고 아니면 복사 할당을 한다. Modern Effective C++ 책에서는 이것이 마치 리턴문에 std::move를 적용한 것과 같다고 한다. 그런데 실제로 `f2`처럼 std::move를 적용해보면 이동 할당(move assignment)를 하기 전에 이동 생성자(move constructor)가 하나 더 실행된다. 그 이유는 위 main 함수와 같은 작업을 하는 아래의 main 함수를 보면 알 수 있다.  

```cpp
int main()
{
    Vector v1(3);
    std::cout << "return by value\n";
    Vector v2 = f1();
    v1 = std::move(v2);
    std::cout << "return by right value reference\n";
    Vector v3 = f2();
    v1 = std::move(v3);
}
```

함수는 리턴할 값을 담는 임시 객체를 생성 후 이를 목적 객체에 할당하는데 `v2=f1()`에서 `v2`가 만들어지는 과정은 RVO에 의해 생략된다. 하지만 `v3=f2()`에서는 `f2`가 오른값 참조를 리턴하므로 복사 제거가 안되고 이동 연산을 한 번 더 하게된다. 어쨌든 결론은 리턴문에 std::move를 쓸 필요가 없다는 것이다.



#### 3.2.3. 오른값 참조 입력 변수 :arrow_right: 지역 변수 할당

std::move가 필요한 희소한 케이스 중 하나다. 아래 예시를 보면 `operator+`는 두 개의 오른값을 오른값 참조 타입으로 받아서 `lhs` 자체에 `rhs`를 더한 뒤 `lhs`를 리턴한다. 이때 `lhs`는 지역변수가 아니고 외부 객체에 대한 참조 변수(왼값)이므로 다른 변수로 들어갈 때 RVO가 일어날 수 없다. 이때 그냥 `lhs`를 리턴하거나 `Vector`에 이동 연산자가 없다면 복사가 일어날 것이다. 하지만 `Vector` 구현에 이동 생성자와 이동 할당 연산자가 있고 `std::move(lhs)`를 리턴하므로 이동 생성자가 실행될 수 있다.

```cpp
Vector operator+(Vector&& lhs, Vector&& rhs) {
    lhs += rhs; // +연산
    return std::move(lhs);
}
int main() {
    Vector v1 = Vector(2) + Vector(2);
    v1 = Vector(3) + Vector(3);
}
```

> [Vector] general constructor: n = 2  
> [Vector] general constructor: n = 2  
> [Vector] move constructor: n = 2  
> [Vector] general constructor: n = 3  
> [Vector] general constructor: n = 3  
> [Vector] move constructor: n = 3  
> [Vector] move assignment: n = 3  



#### 3.2.4. 왼값 참조 입력 변수 :arrow_right: 지역 변수 할당

이번에는 어떤 함수가 왼값 참조를 받아서 어떤 처리과정을 거친 다음 값으로 리턴하는 상황을 생각해보자. 아래 예시에서 `f1`은 그냥 값을 리턴하고 `f2`는 std::move를 적용하여 리턴한다. 그리고 `f1`과 `f2`의 동작 과정을 좀 더 쉽게 이해하기 위해 두 함수를 인라인 코드로 구현한 코드도 main 아래쪽에 작성하였다.

```cpp
Vector f1(Vector &v) {
    // v에 대한 연산
    return v;
}

Vector f2(Vector &v) {
    // v에 대한 연산
    return std::move(v);
}

int main() {
    std::cout << "f1\n";
    Vector v1(2);
    Vector v2(3);
    v2 = f1(v1);
    std::cout << "result: " << v1.n << " " << v2.n << "\n";

    std::cout << "f2\n";
    Vector v3(4);
    Vector v4(5);
    v4 = f2(v3);
    std::cout << "result: " << v3.n << " " << v4.n << "\n";

    std::cout << "inline code like f1\n";
    Vector &t1 = v1;
    Vector t2 = t1;
    Vector t3 = std::move(t2);
    std::cout << "result: " << t1.n << " " << t2.n << " " << t3.n << "\n";

    std::cout << "inline code like f2\n";
    Vector &t4 = v1;
    Vector t5 = std::move(t4);
    Vector t6 = std::move(t5);
    std::cout << "result: " << t4.n << " " << t5.n << " " << t6.n << "\n";
}
```

> f1  
> [Vector] general constructor: n = 2  
> [Vector] general constructor: n = 3  
> [Vector] copy constructor: n = 2  
> [Vector] move assignment: n = 2  
> result: 2 2  
> f2  
> [Vector] general constructor: n = 4  
> [Vector] general constructor: n = 5  
> [Vector] move constructor: n = 4  
> [Vector] move assignment: n = 4  
> result: 0 4  
> inline code like f1  
> [Vector] copy constructor: n = 2  
> [Vector] move constructor: n = 2  
> result: 2 0 2  
> inline code like f2  
> [Vector] move constructor: n = 2  
> [Vector] move constructor: n = 2  
> result: 0 0 2  

결과를 보면 리턴 값을 주는 과정에서 (이동/복사) 생성자 한 번과 할당 연산자 한 번이 실행된다. 리턴하는 변수가 참조 객체이기 때문에 RVO가 일어날 수 없다. std::move로 리턴하는 경우에는 두 번 다 이동이고 다른 경우에는 생성자가 복사 생성자로 실행된다. 그럼 std::move를 써서 이동연산을 하는게 좋을까? 꼭 그렇지는 않다. `f2` 함수에 들어간 객체가 파괴되기 때문이다. 약간의 효율성을 올리려다 심각한 버그를 만들 수 있으므로 신중하게 써야하고 특히 다른 사람이 쓰지 못하게 해야한다.



**생각해볼 것**

- 리턴 타입이 같은데 move()를 하고 안하고 왜 차이 날까? :arrow_right: 예시에서 보여준 함수와 등가의 인라인 코드를 생각해보자.
- 리턴 타입을 오른값 참조(Type&&)로 하고 지역 객체를 리턴하면 함수가 끝나면서 지역 객체가 파괴돼서 dangling reference를 받게됨 :arrow_right: 컴파일러 에러 발생
- 리턴 타입을 오른값 참조(Type&&)로 하려면 입력 받은 왼값 참조나 오른값 참조에 move 적용하여 리턴 가능



## 4. forwarding reference and std::forward()

앞서 오른값 참조와 std::move 함수의 조합을 이용해 이동 연산을 활용하는 법에 대해 알아봤다. 하지만 오른값 참조는 오직 오른값에만 묶일수 있기 때문에 (왼값을 받을 수 없어서) 같은 함수를 입력 인자의 오른값, 왼값 성질에 따라 두 가지로 선언해야 하는 문제가 있다. 다음 함수 예시를 보자. Vector를 받아 이를 정수열로 채운 Vector 객체를 만들어 리턴하는 함수다.

```cpp
Vector set_sequential(const Vector &v) {
    std::cout << "[set_sequential] L-value\n";
    Vector out = v;
    for (int i = 0; i < v.n; ++i)
        out.data[i] = i;
    return out;
}

Vector set_sequential(Vector &&v) {
    std::cout << "[set_sequential] R-value\n";
    Vector out = std::move(v);
    for (int i = 0; i < v.n; ++i)
        out.data[i] = i;
    return out;
}

int main() {
    Vector v1(3);
    set_sequential(v1);
    set_sequential(Vector(3));
}
```

> [Vector] general constructor: n = 3  
> [set_sequential] L-value  
> [Vector] copy constructor: n = 3  
> [Vector] general constructor: n = 3  
> [set_sequential] R-value  
> [Vector] move constructor: n = 3  

사실 첫 번째 함수의 상수 참조만 있어도 lvalue와 rvalue를 모두 받을 수 있긴하다. 하지만 상수 참조로 받으면 이동이 일어날 수 없다. 이동을 사용하려면 두 번째 함수가 있어야 한다. 그럼 이동이 필요한 함수들은 매번 이렇게 두 가지 버전으로 구현을 해야할까? 불편하기도 하고 코드 중복에 의한 위험도 있다. 이걸 하나로 합칠 수 있는 게 바로 전달 참조다. 전달 참조를 이용한 예시를 보자.  

```cpp
template <typename T>
Vector set_sequential(T &&v)
{
    std::cout << "[set_sequential] forwarding reference\n";
    Vector out = std::forward<T>(v);
    for (int i = 0; i < v.n; ++i)
        out.data[i] = i;
    return out;
}

int main()
{
    Vector v1(3);
    set_sequential(v1);
    set_sequential(Vector(3));
}
```

> [Vector] general constructor: n = 3  
> [set_sequential] forwarding reference  
> [Vector] copy constructor: n = 3  
> [Vector] general constructor: n = 3  
> [set_sequential] forwarding reference  
> [Vector] move constructor: n = 3  

전달 참조(forwarding reference)란 템플릿에서 `T&&`와 같은 참조를 말한다. 오른값 참조 같지만 특성이 다르다. *전달*이라는 이름에 걸맞게 입력된 타입을 **그대로** 전달한다는 것이다. cv-qualifier(const, volitile) 뿐만 아니라 왼값, 오른값 속성도 그대로 전달한다. **입력되는 값이 왼값이면 왼값 참조가 되고 오른값이면 오른값 참조가 된다.** 

템플릿 함수에서 전달 참조의 특성을 가지기 위해서는 입력 인자의 타입이 딱 `T&&` 이어야 하고 입력 인자에 대한 형식 연역이 일어나야 한다. 전달 참조인 경우와 아닌 경우를 예시를 통해 비교해보자.

```cpp
template<typename T>
void f(T&& param);		// 1. 전달 참조

template<typename T>
void f(const T&& param);	// 2. 오른값 참조

template<typename T>
void f(std::vector<T>&& param);	// 3. 오른값 참조

template<class T, class Allocater = allocater<T>>
class vector {
    public:
    	void push_back(T&& x);	// 4. 오른값 참조
}
```

1. 전형적인 전달 참조의 모습이다.
2. const가 붙어서 전달 참조가 아니다.
3. 타입이 `T&&`이 아니라 `vector<T>` 타입이라 전달 참조가 아니다.
4. 클래스를 선언할 때 이미 `T`가 결정이 돼서, `push_back` 할 때 형식 연역이 일어나지 않는다.

`set_sequential(Vector &&v)`처럼 오른값 참조로 입력을 받았을 때는 오른값 참조 자체는 왼값이기 때문에 std::move를 이용하여 무조건 오른값으로 변환한다. 전달 참조로 입력을 받았을 때는 입력이 원래 왼값이면 복사를 하고 오른값이면 이동을 하게 하는 것이 이상적이다. 정확히 이 기능을 하는 함수가 **std::forward**다. std::forward의 기능은 원래 입력된 값의 속성을 그대로 전달해주는 것이다. `set_sequential(T &&v)` 함수를 다시 보자.

- 원래 입력 값이 **왼값**이면 v는 왼값 참조가 되고 `std::forward<T>()`를 통과해도 그대로다.
- 원래 입력 값이 **오른값**이면 v는 오른값 참조가 되고 `std::forward<T>()`를 통과하면 마치 std::move 함수를 쓴 것처럼 오른값으로 변환해준다.

왼값에 std:forward가 아닌 std::move 함수를 적용한다면 왼값까지 이동을 시켜버려서 기존 왼값 객체가 파괴될 수 있다. 

**추가할 것**

전달 참조를 받는 함수와 명시적 타입을 받는 함수의 오버로딩

