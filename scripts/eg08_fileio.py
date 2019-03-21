fout = open("testfile.txt", "w")
print(fout)
fout.write("I think Microsoft named .Net so it wouldn’t show up in a Unix directory listing.")
fout.close()
print("file was written")

fin = open("testfile.txt", "r")
contents = fin.read()
fin.close()
print(contents)


with open("testfile.txt", "r") as fr:
    data = fr.read()
    print("check closed under 'with':", fr.closed)
    print(data)

print("check closed outside 'with':", fr.closed)

try:
    f = open("nofile.txt", "r")
    f.close()
except FileNotFoundError as fe:
    print(fe)

springx3 = ["봄 봄 봄 봄이 왔네요",
            "우리가 처음 만났던 그때의 향기 그대로",
            "그대가 앉아 있었던 그 벤치 옆에 나무도 아직도 남아있네요",
            "살아가다 보면 잊혀질 거라 했지만",
            "그 말을 하면 안될거란걸 알고 있었소",
            "그대여 너를 처음 본 순간 나는 바로 알았지",
            "그대여 나와 함께 해주오 이 봄이 가기 전에"]

print("\nwrite lyrics into file")
with open("springx3.txt", "w") as f:
    for i, line in enumerate(springx3):
        f.write(f"{i:2}:" + line + "\n")

with open("springx3.txt", "w") as f:
    printlyrics = [f"{i:2}:" + line + "\n" for i, line in enumerate(springx3)]
    f.writelines(printlyrics)


print("use read")
with open("springx3.txt", "r") as f:
    lyrics = f.read()
    print(lyrics)


print("use readline")
with open("springx3.txt", "r") as f:
    lyrics = []
    line = f.readline()
    while line:
        line = line.rstrip("\n")
        lyrics.append(line)
        line = f.readline()
print("\n".join(lyrics))


print("use readlines")
with open("springx3.txt", "r") as f:
    lyrics = f.readlines()
    lyrics = [line.rstrip("\n") for line in lyrics]
    print("\n".join(lyrics))


matrix_str = "4 2\n5 3"
print("\nwrite matrix:", matrix_str)
with open("matrix.txt", "w") as f:
    f.write(matrix_str)

matrix = []
with open("matrix.txt", "r") as f:
    line = f.readline()
    while line:
        line = line.rstrip("\n")
        numbers = line.split(" ")
        numbers = [int(num) for num in numbers]
        matrix.append(numbers)
        line = f.readline()
print("read matrix:", matrix)
print("determinant =", matrix[0][0]*matrix[1][1] - matrix[1][0]*matrix[0][1])
