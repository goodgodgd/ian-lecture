foo = [1, 2, 3, 4, 5]
bar = [24, 52, 13, 27]

import list_ops

goo = list_ops.add(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
goo = list_ops.multiply(foo, bar)
print("{} * {} = {}".format(foo, bar, goo))


import list_ops as lo

goo = lo.substitute(foo, bar)
print("{} - {} = {}".format(foo, bar, goo))
goo = lo.divide(bar, foo)
print("{} / {} = {}".format(bar, foo, goo))

from list_ops import add, substitute

goo = add(foo, bar)
print("{} + {} = {}".format(foo, bar, goo))
goo = substitute(bar, foo)
print("{} - {} = {}".format(bar, foo, goo))

