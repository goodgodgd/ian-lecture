fout = open("testfile.txt", "w")
fout.write("I think Microsoft named .Net so it wouldn’t show up in a Unix directory listing.")
fout.close()
print("file was written")

fin = open("testfile.txt", "r")
contents = fin.read()
fin.close()
print(contents)
