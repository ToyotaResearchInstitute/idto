from my_fun_bindings import MyFunLibrary

from pydrake.multibody.plant import MultibodyPlant

if __name__=="__main__":
    plant = MultibodyPlant(0.0)
    plant.Finalize()
    print("Plant created successfully")
    assert isinstance(plant, MultibodyPlant)
    lib = MyFunLibrary()
    nq_squared = lib.SquarePlantGeneralizedPositions(plant)
    print("nq_squared: ", nq_squared)
    assert nq_squared == 0