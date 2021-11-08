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

