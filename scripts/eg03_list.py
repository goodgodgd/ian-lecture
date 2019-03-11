
empty_list1 = []
empty_list2 = list()
basic_list = ["Hello", 1234, 1.234, True]
depth2_list = ["Hello", 1234, [1.234, True]]
depth3_list = [["Hello"], [1234, [1.234, True]]]

print("\nIndexing: 1.234에 접근하기")
print("basic indexing:", basic_list[2])
print("negative indexing:", basic_list[-2])
print("first indexing:", depth2_list[2])
print("second indexing:", depth2_list[2][0])
print("first indexing:", depth3_list[1])
print("second indexing:", depth3_list[1][1])
print("third indexing:", depth3_list[1][1][0])
try:
    print(depth2_list[5])
except IndexError as ie:
    print("IndexError:", ie)

print("\nSlicing")
print("[start:end]", basic_list[1:3])
print("[start:]", basic_list[2:])
print("[:end]", basic_list[:2])
print("[start:negative_end]", basic_list[1:-1])
print("[:negative_end]", basic_list[:-2])
print("[negative_start:negative_end]", basic_list[-4:-2])
print("[nested list1]", depth2_list[1:3])
print("[nested list2]", depth3_list[0:2])
print("[partially overlap]", basic_list[2:10])
print("[out of range]", basic_list[5:10])

print("practice:", depth2_list[2][:1])

mammal = ["dog", "cat", "human"]
reptile = ["snake", "lizard", "frog"]
bird = ["eagle", "sparrow", "chicken"]
animal = mammal + reptile + bird
print("\nlist concatenation")
print("animal:", animal)

members = ["나연", "정연", "지효"]
tests = ["vocl", "dance", "rap"]
first_row = members[0:1]*3 + members[1:2]*3 + members[2:3]*3
second_row = tests*3
print("\nlist repetition")
print(first_row)
print(second_row)

string = "Hello"
print("\nlen of {}:".format(string), len(string))
mylist = [1, 2, 3, 4]
print("len of {}:".format(mylist), len(mylist))

mylist = [1, 2, 3, 4, 5]
del mylist[2]
print("\nafter deleting [2]:", mylist)
del mylist[2:]
print("after deleting [2:]:", mylist)

mylist = [1, 2, 3, 4, 5]
mylist[0] = "Life"
print("\nchange element by indexing:", mylist)
# => after deleting [2]: [1, 2, 4, 5]
mylist[1:4] = ["is", "too", "short"]
print("change elements by slicing:", mylist)

print("\njoin strings")
path = ["/home", "ian", "work", "ian-lecture"]
print("joined path:", "/".join(path))
time = ["13", "20", "30"]
print("joined time:", ":".join(time))

print("\n'in' operator")
twice = ["나연", "정연", "모모", "사나", "지효", "미나", "다현", "채영", "쯔위"]
if "채영" in twice:
    print("채영은 트와이스 입니다.")
if "채령" not in twice:
    print("채령은 트와이스가 아닙니다.")

print("\nlist functions")
tottenham = ['Kane', 'Moura', 'Lloris', 'Sissoko', 'Alli', 'Rose']
print("Tottenham vs Southampton 2019-03-10 starting line up: \n", tottenham)
# sort(): 원소 알파벳순, 크기순 정렬, in-place 함수기 때문에 아무것도 리턴하지 않는다.
print("sort() is in-place function:", tottenham.sort())
print("sort by name:", tottenham)
# remove(): 입력한 원소를 삭제
tottenham.remove('Moura')
# insert(): 원하는 위치에 원소 삽입
tottenham.insert(1, 'Son')
print("At 72, Moura out Son in:", tottenham)
# pop(): 입력이 없으면 마지막 원소를 삭제하고 pop(index)는 index의 원소를 삭제한다.
print("At 82, pop Alli:", tottenham.pop(0))
del tottenham[-2]
print("At 82, del Rose")
# append(): 원소를 마지막에 추가한다. 두 리스트의 원소들을 합칠 때는 +나 extend()를 쓴다.
tottenham.append('Davies')
tottenham.append('Llorente')
print("At 82, Davies and Llorente in:", tottenham)
# reverse(): 순서를 거꾸로 뒤집는다.
tottenham.reverse()
print("reverse order", tottenham)


itzy = ["예지", "리아", "류진", "채령", "유나"]
print("\nconvert to iter object")
itzy = iter(itzy)
try:
    print("next1:", next(itzy))
    print("next2:", next(itzy))
    print("next3:", next(itzy))
    print("next4:", next(itzy))
    print("next5:", next(itzy))
    print("next6:", next(itzy))
except StopIteration:
    print("error: iterator finished")

print("\nsquare numbers")
numbers = [1, 2, 3, 4, 5]
for n in numbers:
    print("square of {}  is {}".format(n, n*n))
