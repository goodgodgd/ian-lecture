print("===== procedural programming")
animal_name = "ani"
animal_weight = 20
cow_name = "cow"
cow_weight = 100
cat_name = "cat"
cat_weight = 5

def introduce(name):
    print("my name is", name)
def animal_sound():
    print("...")
def animal_is_fat(weight):
    print("is fat as an animal?", weight > 50)
def cow_sound():
    print("ummer~~~")
def cow_is_fat(weight):
    print("is fat as a cow?", weight > 120)
def cat_sound():
    print("nyaong~~")
def cat_is_fat(weight):
    print("is fat as a cat?", weight > 3)

ani_type = input()
operation = input()
if ani_type == "animal":
    if operation == "intro":
        introduce(animal_name)
    elif operation == "sound":
        animal_sound()
    elif operation == "isfat":
        animal_is_fat(animal_weight)
elif ani_type == "cow":
    if operation == "intro":
        introduce(cow_name)
    elif operation == "sound":
        cow_sound()
    elif operation == "isfat":
        cow_is_fat(cow_weight)
elif ani_type == "cat":
    if operation == "intro":
        introduce(cat_name)
    elif operation == "sound":
        cat_sound()
    elif operation == "isfat":
        cat_is_fat(cow_weight)


print("===== object oriented programming")
class Animal:
    def __init__(self, name, weight):
        self.name = name
        self.weight = weight
    def introduce(self):
        print("my name is", self.name)
    def sound(self):
        print("...")
    def is_fat(self):
        print("is fat as an animal?", self.weight > 50)
class Cow(Animal):
    def __init__(self, name, weight):
        super().__init__(name, weight)
    def sound(self):
        print("ummer~~~")
    def is_fat(self):
        print("is fat as a cow?", self.weight > 120)
class Cat(Animal):
    def __init__(self, name, weight):
        super().__init__(name, weight)
    def sound(self):
        print("nyaong~~")
    def is_fat(self):
        print("is fat as a cat?", self.weight > 3)


ani_type = input()
operation = input()
if ani_type == "animal":
    ani = Animal("ani", 20)
elif ani_type == "cow":
    ani = Cow("cow", 100)
elif ani_type == "cat":
    ani = Cat("cat", 5)
if operation == "intro":
    ani.introduce()
elif operation == "sound":
    ani.sound()
elif operation == "isfat":
    ani.is_fat()

