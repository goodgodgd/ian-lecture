import package.list_ops as lo
import package.dict_ops as do

weights = [65, 90, 42, 76]
heights = [1.65, 1.78, 1.59, 1.80]
heights_sq = lo.multiply(heights, heights)
bmi = lo.divide(weights, heights_sq)
print("BMI:", bmi)

w_names = ["RM", "Suga", "Jin", "V"]
h_names = ["Jimin", "RM", "Suga", "Jin"]
weights = dict(zip(w_names, weights))
heights = dict(zip(h_names, heights))
print("dict weights:", weights)
print("dict heightss:", heights)
heights_sq = do.multiply(heights, heights)
bmi = do.divide(weights, heights_sq)
print("BMI:", bmi)

import package.prime_number as pn

numbers = [341, 12, 523, 59]
pnresult = pn.check_prime_number(numbers)
print("prime number: {} => {}".format(numbers, pnresult))
