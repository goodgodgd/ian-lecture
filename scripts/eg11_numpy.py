import numpy as np
array1d = [1, 2, 3, 4]
array2d = [[1, 2], [3, 4]]
array3d = [[[1, 2], [3, 4]], [[5, 6], [7, 8]]]
print("array1d", np.array(array1d, dtype=int))
print("array2d\n", np.array(array2d, dtype=float))
print("array3d\n", np.array(array3d))

print("ones\n", np.ones((2, 4)))
print("zeros\n", np.zeros((3, 2)))
print("identity\n", np.identity(3))
print("identity\n", np.eye(3))
print("linear space:", np.linspace(5, 10, 11))
print("arange:", np.arange(5, 10, 0.5))
print("permutation:\n", np.random.permutation(10))

print("uniform over [0, 1)\n", np.random.rand(3, 4))
print("normal by N(0, 1)\n", np.random.randn(3, 4))
print("random int over [0, 5)\n", np.random.randint(0, 5, size=(2, 3)))

data_list = [[[5, 6, 2], [3, 4, 9]], [[1, 7, 2], [3, 8, 0]]]
data = np.array(data_list)
print("data\n", data)
print("data_list[0] =", data_list[0])
print("data_list[0][1] =", data_list[0][1])
print("data_list[0][1][2] =", data_list[0][1][2])
print("data[0] =\n", data[0])
print("data[0, 1] =", data[0, 1])
print("data[0, 1, 2] =", data[0, 1, 2])

print("\ndata: shape={}\n{}".format(data.shape,  data))
print("1) data[0, :, :]: shape={}\n{}".format(data[0, :, :].shape, data[0, :, :]))
print("2) data[:, :, 1]: shape={}\n{}".format(data[:, :, 1].shape, data[:, :, 1]))
print("3) data[0, :, 1:]: shape={}\n{}".format(data[0, :, 1:].shape, data[0, :, 1:]))
print("4) data[0, 1, :]: shape={}\n{}".format(data[0, 1, :].shape, data[0, 1, :]))
print("5) data[0, 1:, :]: shape={}\n{}".format(data[0, 1:, :].shape, data[0, 1:, :]))
print("6) data[:1, 1:, :]: shape={}\n{}".format(data[:1, 1:, :].shape, data[:1, 1:, :]))

print("\nmatrix operations")
foo = np.array([[9, 3, 2], [1, 3, 9], [1, 6, 8]])
bar = np.array([[1, 4, 2], [3, 3, 4], [2, 1, 3]])
print("foo\n", foo)
print("bar\n", bar)
print("foo + bar\n", foo + bar, "\n", np.add(foo, bar))
print("foo - bar\n", foo - bar, "\n", np.subtract(foo, bar))
print("foo * bar\n", foo * bar, "\n", np.multiply(foo, bar))
print("foo / bar\n", foo / bar, "\n", np.divide(foo, bar))
print("foo ** bar\n", foo ** bar, "\n", np.power(foo, bar))
print("foo // bar\n", foo // bar, "\n", np.floor_divide(foo, bar))
print("foo % bar\n", foo % bar, "\n", np.remainder(foo, bar))
print("foo x bar\n", np.dot(foo, bar))
print("foo.T (transpose)\n", foo.T)

print("\ncompare operations")
print("foo > bar\n", foo > bar, "\n", np.greater(foo, bar))
print("foo[foo > bar]:", foo[foo > bar])
print("foo > bar\n", foo <= bar, "\n", np.less_equal(foo, bar))
print("foo[foo > bar]:", foo[foo <= bar])
print("foo[foo >= 5]:", foo[foo >= 5])
print("foo[bar < 3]:", foo[bar < 3])
print("foo[foo % 2 == 0]:", foo[foo % 2 == 0])

print("\nbasic math functions")
np.set_printoptions(precision=4, suppress=True)
foo = np.random.rand(5)
print("foo", foo)
print("np.sin(foo):", np.sin(foo))
print("np.cos(foo):", np.cos(foo))
print("sin^2 + cos^2 = 1:", np.sin(foo)**2 + np.cos(foo)**2)
print("np.exp(foo):", np.exp(foo))
print("np.log(foo):", np.log(foo))
print("np.log(exp(foo))==foo", np.log(np.exp(foo)))
print("np.sqrt(foo):", np.sqrt(foo))
print("np.sqrt(foo)^2==foo:", np.sqrt(foo)*np.sqrt(foo))

print("\naggregate functions")
foo = np.random.rand(2, 4)
print("data", foo)
print("1) mean over all", np.mean(foo))
print("2) mean over axis 0", np.mean(foo, axis=0))
print("3) mean over axis 1", np.mean(foo, axis=1))

print("sum", np.sum(foo, axis=0))
print("min", np.min(foo, axis=1))
print("max", np.max(foo, axis=0))
print("std", np.std(foo, axis=1))

print("\nmerging arrays")
foo = np.array([[1, 2, 3], [4, 5, 6]])
bar = np.array([[11, 12, 13], [14, 15, 16]])
print("stack axis=0\n", np.stack([foo, bar], axis=0))
print("stack axis=1\n", np.stack([foo, bar], axis=1))
stack1 = np.array([ [[1, 2, 3], [11, 12, 13]],
                    [[4, 5, 6], [14, 15, 16]] ])
print("stack axis=1 equivalent\n", stack1)
print("stack axis=2\n", np.stack([foo, bar], axis=2))
print("concat axis=0\n", np.concatenate([foo, bar], axis=0))
print("concat axis=1\n", np.concatenate([foo, bar], axis=1))

for i in range(len(foo)):
    print("row", i, foo[i])
for row in foo:
    print("row", row)
for row in foo:
    for value in row:
        print("v:", value)

foo = np.ones((3, 4, 2))
print("shape:", foo.shape)
print("ndim:", foo.ndim)

foo = np.arange(0, 6)
print("foo", foo)
print("foo (2,3)\n", foo.reshape(2, 3))
foo3d = foo.reshape(2, 3, 1)
print("foo (2,3,1)\n", foo3d)
print("foo (3,2)\n", foo3d.reshape(3, 2))
print("foo (3,2)\n", foo3d.reshape(2, 3))


def find_max(array, axis=None):
    pass

