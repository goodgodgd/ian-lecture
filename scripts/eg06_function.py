member_scores = {"나연": {"python": 77, "cpp": 86, "java": 54},
                 "정연": {"python": 96, "cpp": 69, "java": 85},
                 "지효": {"python": 84, "cpp": 47, "java": 36}
                 }

print("code without functions")
# python 점수 평균 구하기
python_average = 0
for scores in member_scores.values():
    python_average += scores["python"]
python_average /= len(member_scores.values())
print("python average:", python_average)

# cpp 점수 평균 구하기
cpp_average = 0
for scores in member_scores.values():
    cpp_average += scores["cpp"]
cpp_average /= len(member_scores.values())
print("cpp average:", cpp_average)

# 지효 평균 점수 구하기
jh_average = 0
for score in member_scores["지효"].values():
    jh_average += score
jh_average /= len(member_scores["지효"].values())
print("지효 average:", jh_average)


print("\ncode with functions")
def subject_average_print(data, subject):
    average = 0
    for scores in data.values():
        average += scores[subject]
    average /= len(data.values())
    print(subject, "average:", average)

def member_average_print(data, member):
    average = 0
    for score in data[member].values():
        average += score
    average /= len(data[member].values())
    print(member, "average:", average)

subject_average_print(member_scores, "python")
subject_average_print(member_scores, "cpp")
member_average_print(member_scores, "지효")


print("\nfunction example")
def output_only__create_data():
    data = {}
    data["나연"] = {"python": 77, "cpp": 86, "java": 54}
    data["정연"] = {"python": 96, "cpp": 69, "java": 85}
    data["지효"] = {"python": 84, "cpp": 47, "java": 36}
    print("output only function can be used to replace complex object creation process")
    return data

member_scores_ = output_only__create_data()


def input_only__print_formatted_score(data, name, subject):
    print("{} received {} in {}".format(name, data[name][subject], subject))

input_only__print_formatted_score(member_scores_, "정연", "python")
input_only__print_formatted_score(member_scores_, "지효", "java")


def input_output__change_score(data, name, change):
    for subject in data[name].keys():
        data[name][subject] += change
    print("{}'s score changed by {}".format(name, change))
    return data

print("before change score", member_scores_["나연"])
member_scores_ = input_output__change_score(member_scores_, "나연", 10)
print("after change score", member_scores_["나연"])


def multi_output__get_dimension(data):
    num_members = len(data)
    for key in data.keys():
        num_subjects = len(data[key])
        break
    return num_members, num_subjects

dimensions = multi_output__get_dimension(member_scores)
print("data dimension:", dimensions)
num_members, num_subjects = multi_output__get_dimension(member_scores)
print("num members and subjects:", num_members, num_subjects)


print("\ndesignated input argument")
def print_formatted_score(data, name, subject):
    print("{} received {} in {}".format(name, data[name][subject], subject))

print_formatted_score(member_scores, "지효", "python")
print_formatted_score(member_scores, "지효", subject="python")
print_formatted_score(member_scores, name="지효", subject="python")
print_formatted_score(member_scores, subject="python", name="지효")
# not allowed
# print_formatted_score(member_scores, name="지효", "python")


print("\nvariable number of input arguments")
def multi_subjects_average(data, *args):
    print("input arguments:", args)
    out = {}
    for subject in args:
        out[subject] = subject_average_return(data, subject)
    return out

def subject_average_return(data, subject):
    average = 0
    for scores in data.values():
        average += scores[subject]
    average /= len(data.values())
    return average

print("java 평균:", multi_subjects_average(member_scores, "java"))
print("python, cpp 평균:", multi_subjects_average(member_scores, "python", "cpp"))

def multi_members_average(data, *args):
    print("input arguments:", args)
    out = {}
    for member in args:
        out[member] = member_average_return(data, member)
    return out

def member_average_return(data, member):
    average = 0
    for score in data[member].values():
        average += score
    average /= len(data[member].values())
    return average

print("지효 평균:", multi_members_average(member_scores, "지효"))
print("지효, 나연, 정연 평균:", multi_members_average(member_scores, "지효", "나연", "정연"))
print("tuple:", (1, 2, 3), "stripped tuple:", *(1, 2, 3))


print("\nkeyword arguments")
def average(data, **kwargs):
    if "name" in kwargs:
        return member_average_return(data, kwargs["name"])
    elif "subject" in kwargs:
        return subject_average_return(data, kwargs["subject"])
    else:
        return 0

print("나연 average:", average(member_scores, name="나연"))
print("python average:", average(member_scores, subject="python"))


print("\nvariable scope")
if True:
    var_created = "created"
print("variable created inside block:", var_created)
if False:
    var_not_created = "not created"
try:
    print("variable NOT created inside block:", var_not_created)
except NameError as ne:
    print(ne)


global_var = 10
def add_ten_local():
    local_var = global_var + 10
    print("add_ten_local:", local_var)

def add_ten_global():
    try:
        global_var = global_var + 10
        print("add_ten_global:", global_var)
    except NameError as ne:
        print(ne)

def add_ten_global_two_steps():
    try:
        local_var = global_var + 10
        global_var = local_var
        print("add_ten_global_two_steps:", global_var)
    except NameError as ne:
        print(ne)

def add_ten_global_use_global():
    global global_var
    global_var = global_var + 10
    print("add_ten_global_use_global:", global_var)

add_ten_local()
add_ten_global()
add_ten_global_two_steps()
add_ten_global_use_global()
print("global_var=", global_var)
