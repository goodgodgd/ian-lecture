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
