foo = [1, 2, 3, 4, 5]
bar = [24, 52, 13, 27]

import list_ops

goo = list_ops.add(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
print("list_ops.spam: {}".format(list_ops.spam))
goo = list_ops.multiply(list_ops.spam, list_ops.ham)
print("{} * {} = {}".format(list_ops.spam, list_ops.ham, goo))

try:
    print("list_ops.eggs: {}".format(list_ops.eggs))
except Exception as e:
    print(e)


import list_ops as lo

goo = lo.subtract(foo, bar)
print("{} - {} = {}".format(foo, bar, goo))
goo = lo.divide(bar, foo)
print("{} / {} = {}".format(bar, foo, goo))

from list_ops import add, subtract, spam

goo = add(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
goo = subtract(bar, foo)
print("{} - {} = {}".format(bar, foo, goo))
print("spam = {}".format(spam))

