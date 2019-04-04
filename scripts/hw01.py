heroes = ["iron man", "thor", "hulk", "captain america", "hawkeye"]
print("original heroes:", heroes)
# 1. 히어로들을 알파벳 역순으로 정렬하시오.
heroes.sort(reverse=True)
print("1. sorted heroes:", heroes)

# 2. 1의 결과에서 첫번째와 마지막 히어로를 제거하고 captain marvel과 spider man을 추가하시오.
heroes.pop()
heroes.pop(0)
heroes.append("captain marvel")
heroes.append("spider man")
print("2. change heroes:", heroes)

# 3. for문을 이용하여 'man'으로 끝나는 히어로 이름에서 'man'을 'woman'으로 바꾼 새 리스트 heroines를 만드시오.
heroines = []
for hero in heroes:
    if hero.endswith("man"):
        hero = hero.replace("man", "woman")
    heroines.append(hero)
print("3. transgender heroines:", heroines)

# 4. list comprehension을 이용하여 'woman'이 들어간 히어로 이름을 'human'으로 바꾼 새 리스트 neutralized_heroes를 만드시오.
neutralized_heroes = [heroine.replace("woman", "human") for heroine in heroines]
print("4. neutralized heroes:", neutralized_heroes)

# 5. 2의 heroes에서 enumerate를 이용하여 홀수번째 히어로만 뽑아낸 새 리스트 odd_heroes를 만드시오.
odd_heroes = []
for i, hero in enumerate(heroes):
    if i % 2 == 0:
        odd_heroes.append(hero)
print("5. odd heroes", odd_heroes)

# 6. for문에 zip 함수를 써서 5의 히어로 이름과 배우 이름을 딕셔너리로 묶은 hero_actors를 만들어 보시오.
actors = ["로다주", "제레미 레너", "톰 홀랜드"]
hero_actors = {}
for hero, actor in zip(odd_heroes, actors):
    hero_actors[hero] = actor
print("6. hero actors", hero_actors)
print("6. thor was played by", hero_actors["hawkeye"])
print("6. spider man was played by", hero_actors["spider man"])

# 7. 2의 heroes를 연기한 배우들의 이름을 6의 hero_actors를 이용해 출력하시오.
# 이름이 있으면 6번과 같이 이름을 출력하고 없으면 "?? has no name"을 출력하시오.
for hero in heroes:
    if hero in hero_actors:
        print("7. {} was played by {}".format(hero, hero_actors[hero]))
    else:
        print("7. {} has no name".format(hero))

