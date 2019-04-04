import pandas as pd

scores = {"이름": ["정연", "나연", "지효"],
          "python": [46, 89, 93],
          "cpp": [94, 51, 77],
          "java": [67, 73, 61]
          }

scores = pd.DataFrame(scores)
print("original data\n", scores)

subject_mean = scores.mean(axis=0)
print("subject mean\n", subject_mean)
member_mean = scores.mean(axis=1)
member_mean = scores.mean(axis=1)
member_std = scores.std(axis=1)
description = scores.describe()
print("member mean\n", member_mean)
print("member std\n", member_std)
print("subject describe\n", description)
print("subject describe\n", description.round(1))

member_scores = scores.set_index("이름")
member_mean = member_scores.mean(axis=1)
print("member mean\n", member_mean)

sana_score = {"이름": "사나", "python": 79, "cpp": 63, "java": 55}
scores = scores.append(sana_score, ignore_index=True)
print("add row\n", scores)

print("slicing1\n", scores.loc[:, ["python", "cpp"]])
print("slicing2\n", scores.loc[1:3, ["python", "cpp"]])
print("get column\n", scores["python"])
print("get row\n", scores.loc[1, :])

print("add new column")
expand_scores = scores.copy()
expand_scores["ruby"] = 0
print("column operation")
expand_scores["sum"] = expand_scores["python"] + expand_scores["cpp"]
print(expand_scores)

print("concatenate dataframes")
new_scores = {"이름": ["모모", "쯔위", "미나"],
              "python": [85, 42, 56],
              "cpp": [86, 36, 75],
              "java": [58, 73, 49]
              }
new_scores = pd.DataFrame(new_scores)
print("new member scores\n", new_scores)
print("all member scores")
all_scores = pd.concat([scores, new_scores], ignore_index=True)
print(all_scores)

print("save dataframe")
all_scores.to_csv("./scores.csv", index=False)

loaded_scores = pd.read_csv("./scores.csv")
print("loaded_scores\n", loaded_scores)

