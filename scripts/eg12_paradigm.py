class Dog:
    def __init__(self, name):
        self.name = name
        self.position = 0

    def bark(self):
        print(f"{self.name}: Wal! Wal!")

    def move(self, distance):
        self.position += distance
        print(f"{self.name} is at {self.position}")


puppy = Dog("dangdang")
puppy.bark()
puppy.move(10)
print("current position:", puppy.position)


print("===== object oriented programming")
class Animal:
    def __init__(self, name):
        self.name = name
    def introduce(self):
        print("my name is", self.name)
    def sound(self):
        print("...")
class Cow(Animal):
    def __init__(self, name):
        super().__init__(name)
    def sound(self):
        print("ummer~~~")
class Cat(Animal):
    def __init__(self, name):
        super().__init__(name)
    def sound(self):
        print("nyaong~~")

cow = Cow("cow1")
cow.introduce()
cow.sound()
animals = [Animal("ani"), Cow("cow2"), Cat("cat")]
for ani in animals:
    ani.introduce()
    ani.sound()


print("===== procedural programming")
animal_name = "ani"
cow_name = "cow"
cat_name = "cat"
def introduce(name):
    print("my name is", name)
def animal_sound():
    print("...")
def cow_sound():
    print("ummer~~~")
def cat_sound():
    print("nyaong~~")

introduce(animal_name)
animal_sound()
introduce(cow_name)
cow_sound()
introduce(cat_name)
cat_sound()

ani_type = input()
operation = input()
if ani_type == "animal":
    if operation == "intro":
        introduce(animal_name)
    elif operation == "sound":
        animal_sound()
elif ani_type == "cow":
    if operation == "intro":
        introduce(cow_name)
    elif operation == "sound":
        cow_sound()
elif ani_type == "cat":
    if operation == "intro":
        introduce(cow_name)
    elif operation == "sound":
        cat_sound()


ani_type = input()
operation = input()
if ani_type == "animal":
    ani = Animal("ani")
elif ani_type == "cow":
    ani = Cow("cow")
elif ani_type == "cat":
    ani = Cat("cat")
if operation == "intro":
    ani.introduce()
elif operation == "sound":
    ani.sound()
