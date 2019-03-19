heroes = ["iron man", "thor", "hulk", "captain america", "hawkeye"]
print("original list:", heroes)
heroes.sort()
print("sorted list:", heroes)

# remove first and last heroes and add two heroes
heroes.pop()
del heroes[0]
heroes.append("captain marvel")
heroes.append("spider man")
print("changed list:", heroes)

# change man to woman
heroines = [hero.replace("man", "woman") for hero in heroes]
print("heroines:", heroines)



