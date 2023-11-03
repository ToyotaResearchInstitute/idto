import numpy as np
from bindings import example

result = example.add(2, 3)
print("sum of 2 and 3 is ", result)
assert result == 5

result = example.subtract(2, 3)
print("difference of 2 and 3 is ", result)
assert result == -1

my_pet = example.Pet("Fido")
print("My pet's name is", my_pet.getName())
my_pet.setName("Rex")
print("My pet's new name is", my_pet.getName())
assert my_pet.getName() == "Rex"

problem_definition = example.ProblemDefinition()
print(problem_definition.num_steps)
problem_definition.num_steps = 10
print(problem_definition.num_steps)
assert problem_definition.num_steps == 10

print(problem_definition.q_init)
problem_definition.q_init = np.array([1, 2, 3])
print(problem_definition.q_init)

mec = example.MyEigenClass()
M = mec.getMatrix()
print(M.shape)
print(type(M))