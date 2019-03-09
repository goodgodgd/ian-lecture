string1 = "Life is too short"
string2 = 'You need python'
print(type(string1), type(string1))
# 문자열 안에 따옴표(', ") 입력
print("Life 'is' too short")
print('You "need" python')
# 특수문자 입력
print("Life \"is\" too short,\nYou \'need\' python")
# 한글도 잘 나온다.
print("안녕? 파이썬")

print("날 " + ("너무" * 3 + " ")*5 + "좋아하면 그때 말해줘")
print("내가 " + ("자꾸" * 3 + " ")*5 + "떠오르면 그때 불러줘")


print("\n" + "="*30)
print("String indexing, slicing")
text = "Life is too short, You need Python"
# Life is too short, You need Python
# 0         1         2         3
# 0123456789012345678901234567890123
print("index t:", text[8], text[16], text[30], text[-4])
print("slice 'Life':", text[:4])
print("slice 'short':", text[12:17])
print("slice 'Python':", text[28:])
print("slice 'need'", text[23:-7])


print("\n" + "="*30)
print("String formatting 1: %")
print("class: %s" % "warrior")
print("HP: %d" % 100)
print("DPS: %f" % 1456.23)

_class = "warrior"
HP = 100
DPS = 1456.23
# 문자열은 10칸 사용, 정수는 5칸 사용, 실수는 10칸에 소수점은 3자리까지 사용
pattern = "class: %10s, HP: %5d, DPS: %10.3f"
char_intro = pattern % (_class, HP, DPS)
print(char_intro)
print(pattern % ("healer", 200, 834.79))


print("\n" + "="*30)
print("String formatting 2: {}.format()")
print("class: {}".format("warrior"))
print("HP: {}".format(100))
print("DPS: {}".format(1456.23))

_class = "warrior"
HP = 100
DPS = 1456.23
# 모두 왼쪽 정렬, 문자열은 10칸, 정수는 5칸, 실수는 10칸에 소수점은 3자리까지 사용
pattern = "class: {:<10}, HP: {:<5}, DPS: {:<10.3f}"
char_intro = pattern.format(_class, HP, DPS)
print(char_intro)
print(pattern.format("healer", 200, 834.79))


print("\n" + "="*30)
print("String formatting 3: f string formatting")
_class = "warrior"
HP = 100
DPS = 1456.23
# 모두 왼쪽 정렬, 문자열은 10칸, 정수는 5칸, 실수는 10칸에 소수점은 3자리까지 사용
char_intro = f"class: {_class:<10}, HP: {HP:<5}, DPS: {DPS:<10.3f}"
print(char_intro)
char_intro = f"class: {'healer':<10}, HP: {200:<5}, DPS: {834.79:<10.3f}"
print(char_intro)


print("\n" + "="*30)
print("str class functions")
# 문자열 함수를 이용하는 두 가지 방법
# 1. 문자열 자체에서 사용, 2. 문자열 변수에서 사용
print("count substring")
print("count '너무':", "날 너무너무너무".count('너무'))
text = "날 " + ("너무" * 3 + " ")*5 + "좋아하면 그때 말해줘"
print("count '너무':", text.count('너무'))


print("\nfind substring")
text = "For the python, of the python, by the python"
# 문자열 위치 찾기 (find)
pyind = text.find("py")
print(f"'py' found at {pyind} in `{text}`")
pyind = text.find("py", pyind+1)
print(f"'py' found at {pyind} in `{text}`")
pyind = text.find("py", pyind+1)
print(f"'py' found at {pyind} in `{text}`")
pyind = text.find("ruby")
print(f"'ruby' found at {pyind} in `{text}`")

pyind = text.index("py")
print(f"'py' indexed at {pyind} in `{text}`")
pyind = text.index("py", pyind+1)
print(f"'py' indexed at {pyind} in `{text}`")
pyind = text.index("py", pyind+1)
print(f"'py' indexed at {pyind} in `{text}`")
try:
    pyind = text.index("ruby")
    print(f"'ruby' found at {pyind} in `{text}`")
except ValueError as ve:
    print("ruby not indexed, value error:", ve)


print("\nuppper and lower case")
mixed = "PYthon"
small = "python"
print(f"compare {mixed} and {small}")
print(f"{mixed} == {small}:", mixed == small)
print(f"{mixed}.lower() == {small}:", mixed.lower() == small)
print(f"{mixed}.upper() == {small}.upper():", mixed.upper() == small.upper())
print(f"{mixed}.lower() is {small}.lower():", mixed.lower() is small.lower())


print("\nwhat is difference between `is` and `==`?")
print("1 is True:", 1 is True)
print("1 == True:", 1 == True)
print("(0 == 1) is False:", (0 == 1) is False)


print("\nstrip string")
wise_saying = ' "Walking on water and developing software ' \
              'from a specification are easy if both are frozen..." '
print(wise_saying)
wise_saying = wise_saying.strip()
print(wise_saying)
wise_saying = wise_saying.strip('\"')
print(wise_saying)
wise_saying = wise_saying.rstrip('.')
print(wise_saying)


print("\nreplace string")
Lincoln_said = "for the people, by the people, of the people"
We_say = Lincoln_said.replace("people", "python")
Simply_say = We_say.replace("the ", "")
print("Lincoln said:", Lincoln_said)
print("We say:", We_say)
print("Simply say:", Simply_say)


print("\nsplit string into list of substrings")
print("split by words:", We_say.split())
print("split by phrase:", We_say.split(","))
