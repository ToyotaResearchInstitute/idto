from bindings import adder

result = adder.add(2, 3)
print("sum of 2 and 3 is ", result)
assert result == 5

result = adder.subtract(2, 3)
print("difference of 2 and 3 is ", result)
assert result == -1
