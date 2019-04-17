import os
import glob


def change_lyrics_and_save(srcdir, dstdir, change_terms):
    if not os.path.isdir(srcdir):
        print("wrong source path:", srcdir)
        return
    if not os.path.isdir(dstdir):
        os.mkdir(dstdir)
    srcfiles = glob.glob(os.path.join(srcdir, "*.txt"))
    for file in srcfiles:
        with open(file, "r") as fr:
            lyrics = fr.read()
            for src, dst in change_terms.items():
                lyrics = lyrics.replace(src, dst)
            print("new lyrics:", lyrics)
            newfile = file.replace(srcdir, dstdir)
            print("newfile name:", newfile)
            with open(newfile, "w") as fw:
                fw.write(lyrics)


if __name__ == "__main__":
    srcdir = "D:/NaverCloud/동기화문서/강의자료/2019-1 시스템프로그래밍/HW2/lyrics"
    dstdir = "D:/NaverCloud/동기화문서/강의자료/2019-1 시스템프로그래밍/HW2/lyrics_winter"
    change_terms = {"봄": "겨울", "벚꽃": "눈"}
    change_lyrics_and_save(srcdir, dstdir, change_terms)

