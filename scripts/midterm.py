print("\n" + "="*10 + " problem 3")
me = "나"
you = "너"
like = "좋아하"
print("%s는 %s를 %s고 %s를 %s고" % (me, you, like, you, like))
print("%s도 %s를 %s고 %s를 %s고" % (you, me, like, me, like))
print("우린 서로 %s는데도 그 누구도 말을 안해요." % like)

print("{}는 {}를 {}고 {}를 {}고".format(me, you, like, you, like))
print("{}도 {}를 {}고 {}를 {}고".format(you, me, like, me, like))
print("우린 서로 {}는데도 그 누구도 말을 안해요.".format(like))

print(f"{me}는 {you}를 {like}고 {you}를 {like}고")
print(f"{you}도 {me}를 {like}고 {me}를 {like}고")
print(f"우린 서로 {like}는데도 그 누구도 말을 안해요.")


print("\n" + "="*10 + " problem 4")

all_players = {"Tottenham": {"FW": "Son", "MF": "Eriksen", "GK": "Lloris"},
               "Man.City": {"FW": "Sterling", "MF": "Silva", "DF": "Walker"},
               "Barcelona": {"FW": "Messi", "DF": "Piqué", "GK": "ter Stegen"}}
positions = ["FW", "MF", "DF", "GK"]
pos_names = {}
for pos in positions:
    names = []
    # 각 팀에서 'pos' 포지션이 있는지 확인 후 있으면 그 포지션의 선수 이름 추가
    for team_name, team_players in all_players.items():
        if pos in team_players:
            names.append(team_players[pos])
    print("position and players:", pos, names)
    # 포지션 별 선수 이름들을 딕셔너리에 추가
    pos_names[pos] = names
print("position names:", pos_names)
pos_names["FW"].pop()
pos_names["FW"].insert(0, "Ronaldo")
print("FW list:", pos_names["FW"])


print("\n" + "="*10 + " problem 5")
import os

def change_lyrics_and_save(srcfile, srcterm, dstterm):
    # 파일 존재 여부 확인
    if not os.path.isfile(srcfile):
        print("there is no file:", srcfile)
        return
    # srcfile을 읽기 모드로 열기
    with open(srcfile, "r") as fr:
        # 파일 전체 내용 문자열로 읽기
        lyrics = fr.read()
        # 가사 바꾸기
        new_lyrics = lyrics.replace(srcterm, dstterm)
        print("new lyrics:", new_lyrics)
        # srcfile 경로명과 파일명 분리
        dirname = os.path.dirname(srcfile)
        # 경로명과 새 파일명 합쳐서 새로운 파일경로명 만들기
        newfile = os.path.join(dirname, "homesick.txt")
        print("newfile name:", newfile)
        # newfile을 쓰기 모드로 열기
        with open(newfile, "w") as fw:
            # 바뀐 가사를 파일로 출력
            fw.write(new_lyrics)

if __name__ == "__main__":
    srcfile = "D:/NaverCloud/동기화문서/강의자료/2019-1 시스템프로그래밍/시험/lovesick.txt"
    change_lyrics_and_save(srcfile, "love", "home")


print("\n" + "="*10 + " problem 6")

member_scores = {"나연": {"python": 77, "cpp": 86, "java": 54},
                 "정연": {"python": 96, "cpp": 69, "java": 85},
                 "지효": {"python": 84, "cpp": 47, "java": 36}
                 }

def multi_members_sum(data, *args):
    out = {}
    for member in args:
        sumval = member_sum_return(data, member)
        out[member] = sumval
        print(f"{member} 총점: {sumval}")
    return out

def member_sum_return(data, member):
    sumval = 0
    print(f"{member} 점수:", data[member])
    for score in data[member].values():
        sumval += score
    return sumval

print("지효, 나연, 정연 총점:", multi_members_sum(member_scores, "지효", "나연", "정연"))


print("\n" + "="*10 + " problem 7")

import numpy as np
def find_sum(array, axis, index):
    sumval = 0
    if axis == 0:
        for val in array[:, index]:
            sumval += val
    elif axis == 1:
        for val in array[index, :]:
            sumval += val
    else:
        return 0
    return sumval

array = [[1, 2, 3], [4, 5, 6]]
array = np.array(array)
print("array:\n", array)
print("sum column 1:", find_sum(array, axis=0, index=1))
print("sum row    0:", find_sum(array, axis=1, index=0))
