marvel_heroes = ["iron man", "thor", "hulk", "spider man", "black widow", "capt. america", "capt. marvel"]
dc_heroes = ["batman", "superman", "aquaman", "wonder women", "harley quinn"]
all_heros = marvel_heroes + dc_heroes

print("To which party batman belongs?")
myhero = "batman"
if myhero in marvel_heroes:
    print("{}: Avengers are super cool!".format(myhero))
elif myhero in dc_heroes:
    print("{}: We save martha...".format(myhero))
else:
    print("{}: We are villains!!".format(myhero))

print("\nlist of marvel heroes")
for hero in marvel_heroes:
    print(f"    {hero}")

print("\nlist of dc heroes")
for i in range(len(dc_heroes)):
    print(f"    {dc_heroes[i]}")

i = 0
while i < len(dc_heroes) and dc_heroes[i].endswith('man'):
    print(f"{dc_heroes[i]}: DC's male heroes ends with 'man'")
    i += 1

print("\nlist of marvel heroes")
for hero in marvel_heroes:
    if hero.startswith("spider"):
        print("    Peter Parker by Tobey Maguire was not a kid: \"With great power comes great responsibility.\"")
        continue
    if hero.startswith("capt"):
        print("    one captain is enough. let me stop here")
        break
    print(f"    {hero} is cool")

name = None
print("Press 'q' to quit")
if 0:
    while name != 'q':
        print("type dc hero's name")
        name = input()
        if name is 'q':
            break
        if name not in dc_heroes:
            print(f"{name} is not dc hero")
            continue
        index = dc_heroes.index(name)
        print(f"{name}'s index =", index)


print("\nwhat does enumerate return?", enumerate(marvel_heroes))
print(list(enumerate(marvel_heroes)))
print("what does zip return?", zip(marvel_heroes, dc_heroes))
print(list(zip(marvel_heroes, dc_heroes)))

print("\nprint only first 3 heroes with index")
for index, name in enumerate(marvel_heroes):
    if index >= 3:
        break
    print("marvel hero:", index, name)

print("\nprint pairs of marvel and dc heroes")
for mv, dc in zip(marvel_heroes, dc_heroes):
    print("{} vs {}".format(mv, dc))

print("\nprint only 3 pairs of marvel and dc heroes")
for ind, (mv, dc) in enumerate(zip(marvel_heroes, dc_heroes)):
    if ind >= 3:
        break
    if ind % 2 == 1:
        print("{0} vs {1}: {0} win!".format(mv, dc))
    else:
        print("{0} vs {1}: {1} win!".format(mv, dc))


print("\ncreate a new list from the existing list")
super_heroes = []
for hero in marvel_heroes:
    super_heroes.append(hero + "_super")
print("super heroes", super_heroes)

print("\nextract a subset of the existing list")
man_heroes = []
for hero in marvel_heroes:
    if hero.endswith("man"):
        man_heroes.append(hero)
print("man heroes:", man_heroes)

print("\ncreate a new list by list comprehension")
super_heroes = [hero + "_super" for hero in marvel_heroes]
print("super heroes", super_heroes)

print("\nextract a subset by conditioned list comprehension")
man_heroes = [hero for hero in marvel_heroes if hero.endswith("man")]
print("man heroes:", man_heroes)

print("\nsquare of integers")
int_square = [i ** 2 for i in range(10)]
print(int_square)


print("\ndictionary comprehension")
abilities = ["suit", "Mjölnir", "physical power", "spider web"]
heroes = {name: power for name, power in zip(marvel_heroes, abilities)}
print("hero's ability", heroes)


