```cpp
Foo a;
Foo& f1(Foo& foo) { return foo; }
Foo& b = f1(a);
Foo f2(Foo& foo) { return foo; }
Foo c = f2(a);
Foo f3(Foo& foo) { return std::move(foo); }
Foo& c = f3(a);
Foo f3(Foo foo) { return foo; }
```

- f1: 참조를 받아 참조를 리턴하여 복사 없다.
- f2: 참조를 받았는데 값을 리턴할 수 없다. 에러!
- f3: 값을 받아 값을 그대로 리턴하므로 RVO가 일어나거나 오른값으로 리턴된다.







move 개념

stack, heap 때문

lvalue, rvalue 개념



24: 왼값 참조, 오른값 참조, 전달 참조

25: move, forward

26중복적재는 이런 케이스가 있으니 전달 참조는 생성자 같은데서 쓰지 마라.. 정도로 정리



궁금

```
int a = 10;
int b = move(a);
cout << a; ?? 뭐가나오나?
```





default 생성자

https://ozt88.tistory.com/24

value categories

https://en.cppreference.com/w/cpp/language/value_category

https://docs.microsoft.com/ko-kr/cpp/cpp/lvalues-and-rvalues-visual-cpp?view=msvc-160

move

https://docs.microsoft.com/ko-kr/cpp/cpp/move-constructors-and-move-assignment-operators-cpp?view=msvc-160&viewFallbackFrom=vs-2019







```cpp
int a = 1;
int &b = a;     // 왼값 참조는 왼값에 묶인다
int &&c = 1;    // 오른값 참조는 오른값에 묶인다
int &d = c;     // 왼값 참조는 오른값 참조(왼값)에 묶인다
int &e = 1;     // 오류! 오른값을 왼값 참조에 할당
int &&f = a;    // 오류! 왼값을 오른값 참조에 할당
int &&g = c;    // 오류! 오른값 참조(왼값)을 오른값 참조에 할당 
```

위 코드에서 `a`와 `c`는 생성과정이 다르다. `a`는 1이란 임시 객체를 만들고 나서 그 값을 `a`에 덮어쓰는 것이고, `c`는 1이란 임시 객체의 참조를 받아오는 것이다. 만약 왼값 참조에 오른값을 묶거나 (`e`) 오른값 참조에 왼값을 묶으려고 하면 (`f`) 에러가 난다. 재미있는 것은, 왼값 참조에 오른값 참조를 묶을 수 있고 (`d`) 오른값 참조에는 오른값 참조를 묶을 수 없다는 (`g`) 것이다. 오른값의 **참조**는 왼값이기 때문이다.

앞서 설명한 예시에서처럼 이동 연산을 하기위해서는 입력인자에 오른값을 넣어야 하는데 우리가 프로그램에서 가진 데이터가 왼값으로 존재하는 경우가 많기 때문에 왼값을 오른값으로 바꿔주는 연산이 필요하다. 예를 들면 아래와 같은 함수를 만들 수 있다.

```c++
struct Foo {};
Foo lval_to_rval(Foo& foo) { return foo; }
// 이거 rvalue로 리턴되면서 복사되는거 아닌가??
void rval_input(Foo&& foo) {}
int main() {
    Foo foo;
    rval_input(foo);	// 오류!
    rval_input(lval_to_rval(foo));
}
```

`rval_input`은 오른값을 받는 함수이므로 왼값을 입력하면 에러가 난다. 그래서 왼값을 받아 오른값 임시 객체로 반환하는 `lval_to_rval`라는 함수를 만들어 리턴 값을 바로 입력했더니 에러 없이 실행됐다. 그런데 모든 데이터 타입마다 `lval_to_rval` 이런걸 만들어 줄 순 없다. 템플릿 함수로 구현하는 것이 효율적일 것이다. 다행스럽게도 STL에서 이런 함수를 구현해서 제공하는데 그게 바로 **std::move()**다.  

