def main():
    member_scores = {"나연": {"python": 77, "cpp": 86, "java": 54},
                     "정연": {"python": 96, "cpp": 69, "java": 85},
                     "지효": {"python": 84, "cpp": 47, "java": 36}
                     }
    member_average_print(member_scores, "정연")
    member_average_print(member_scores, "나연")
    print_formatted_score(member_scores, "지효", "python")


def member_average_print(data, member):
    average = 0
    for score in data[member].values():
        average += score
    average /= len(data[member].values())
    print(member, "average:", average)


def print_formatted_score(data, name, subject):
    print("{} received {} in {}".format(name, data[name][subject], subject))


if __name__ == '__main__':
    main()
