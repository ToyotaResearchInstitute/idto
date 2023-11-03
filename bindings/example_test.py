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
