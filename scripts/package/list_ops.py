# list_ops.py
def add(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f + b)
    return out


def subtract(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f - b)
    return out


def multiply(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f * b)
    return out


def divide(foo, bar):
    out = []
    for f, b in zip(foo, bar):
        out.append(f / b)
    return out


spam = [51, 23]
ham = [34, 67]
if __name__ == '__main__':
    eggs = add(spam, ham)
