member_scores = {"나연": {"python": 77, "cpp": 86, "java": 54},
                 "정연": {"python": 96, "cpp": 69, "java": 85},
                 "지효": {"python": 84, "cpp": 47, "java": 36}
                 }

def subject_average_no_input_data(subject):
    average = 0
    for scores in member_scores.values():
        average += scores[subject]
    average /= len(member_scores.values())
    print(subject, "average:", average)

subject_average_no_input_data("python")


extvar = [10]

def addvar():
    extvar[0] = extvar[0] + 100
    return extvar

print(addvar())
print(extvar)
