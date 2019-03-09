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
