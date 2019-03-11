import sys
a = 135.68
b = 15
print("float", type(a), sys.getsizeof(a))
print("integer", type(b), sys.getsizeof(b))

# 기본적인 사칙연산
print(a + b)
print(a - b)
print(a * b)
print(a / b)

# 거듭제곱
print(b ** 2)
# 나누기 후 나머지
print(a % b)
# 나누기 후 몫
print(a // b)


# 연습문제: 16진수 변환
num = 13**3

# 첫째 자리
h1 = num // (16**2)
# 둘째 자리
residue = num % (16**2)
h2 = residue // 16
# 셋째 자리
h3 = num % 16

print(h1, h2, h3)
print(hex(num))

print("if statements")
if 13 ** 3 > 50 **2:
    print("13**2 > 50**2")
elif 13 ** 3 != 2197:
    print("13**2 != 2197")
elif 13 ** 3 >= 30 **2:
    print("13**2 >= 30**2")
else:
    print("13 ** 3 < 30 **2")

print("\nWhat is difference between `is` and `==`?")
print("little difference for basic types(int, float, str)")
print("12 == 12:", 12 == 12)
print("12 is 12:", 12 is 12)
print("12 == True:", 12 == True)
print("12 is True:", 12 is True)

print("big difference for other types")
foo = [1]
bar = [1]
print("foo == bar", foo == bar)
print("foo is bar", foo is bar)
