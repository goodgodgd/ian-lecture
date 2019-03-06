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
