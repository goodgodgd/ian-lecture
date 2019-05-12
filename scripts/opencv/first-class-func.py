def foo():
    print("foo")

class bar:
    def bar_func(self):
        print("bar")

def spam(f1, f2):
    f1()
    f2()

eggs = bar()
spam(foo, eggs.bar_func)
