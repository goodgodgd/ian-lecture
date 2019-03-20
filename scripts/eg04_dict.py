good_for_list = ["pooh", "tigger", "piglet", "rabbit"]
pooh = ["bear", 5, 50]
tigger = ["tiger", 4, 40]
print("pooh's species is", pooh[0])
print("pooh's weight is", pooh[2])
print("tigers's age is", tigger[1])

pooh = {"species": "bear", "age": 5, "weight": 50}
tigger = {"species": "tiger", "age": 4, "weight": 40}
print("\ndict based data management")
print("pooh's species is", pooh["species"])
print("pooh's weight is", pooh["weight"])
print("tigers's age is", tigger["age"])

print("\nhash example")
print("hash of 1:", hash(1))
print("hash of python:", hash("python"))
try:
    print("hash of []:", hash([]))
except TypeError as te:
    print("[TypeError]", te)

print("\nbasic usage")
pooh = {"species": "bear", "age": 5, "weight": 50}
# 데이터 읽기: 특정 `key`에 연결된 `Value`를 읽기 위해서는 `[Key]`하면 된다.
print("pooh's age is", pooh["age"])
# 데이터 수정: 특정 `key`에 연결된 `Value`를 수정할 때는 그냥 값을 키에 넣으면 된다.
pooh["age"] = 10
print("pooh's age is", pooh["age"])
# 데이터 추가: 이미 만들어진 딕셔너리에 `Key:Value` 쌍을 추가하는 방법 역시 그냥 값을 키에 넣으면 된다.
pooh["height"] = 1.2
print("pooh:", pooh)
# 데이터 삭제: 리스트처럼 `del`을 이용한다.
del pooh["weight"]
print("pooh:", pooh)

pooh = {"species": "bear", "age": 5, "weight": 50}
try:
    print("\ntry non-existing key")
    print("pooh's color?", pooh["color"])
except KeyError as ke:
    print("[KeyError]", ke)

print("pooh's color?", pooh.get("color"))
if "color" in pooh:
    print("pooh's color is", pooh["color"])
else:
    print("pooh has no color")

print("\ndict functions")
scores = {"pooh": 80, "tigger": 70, "piglet": 90, "rabbit": 85}
print("names:", scores.keys())
print("scores:", scores.values())
print("items:", scores.items())

print("names:", list(scores.keys()))
print("scores:", list(scores.values()))
print("items:", list(scores.items()))

print("\niterate over keys")
for name in scores.keys():
    print("name:", name)
for score in scores.values():
    print("score:", score)
for name, score in scores.items():
    print("name:score:", name, ":", score)


print("\nHow to use tuple")
empty_tuple1 = ()
empty_tuple2 = tuple()
basic_tuple1 = ("Hello", 1234, 1.234, True)
basic_tuple2 = "Hello", 1234, 1.234, True
depth2_tuple = ("Hello", 1234, (1.234, True))
print("read tuple", basic_tuple1[0])
print("read tuple", basic_tuple2[1])
print("read tuple", basic_tuple1[:3])

print("\ndistribute values")
pooh = "pooh", "bear", 5, 50
name, species, age, weight = pooh
print("tupled pooh info:", name, species, age, weight)

print("\nHow to use set")
entrance_order = ["k3", "aventador", "k3", "a6", "cayenne", "a6"]
car_set = set(entrance_order)
print("car set:", car_set)

print("\nset example: companies")
large_comps = {"samsung", "hyundai", "lg", "sk"}
motor_comps = {"hyundai", "kia", "gm"}
print("intersection set by &:", large_comps & motor_comps)
print("intersection set by function:", large_comps.intersection(motor_comps))
print("union set by |:", large_comps | motor_comps)
print("union set by function:", large_comps.union(motor_comps))
print("difference set by |:", large_comps - motor_comps)
print("difference set by function:", large_comps.difference(motor_comps))



