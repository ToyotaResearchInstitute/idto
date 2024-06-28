from pyidto import MyFunLibrary

from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.common import FindResourceOrThrow


if __name__=="__main__":
    plant = MultibodyPlant(0.0)
    urdf = FindResourceOrThrow('drake/examples/pendulum/Pendulum.urdf')
    Parser(plant).AddModels(urdf)

    plant.Finalize()
    print("Plant created successfully")
    assert isinstance(plant, MultibodyPlant)
    lib = MyFunLibrary()
    nq_squared = lib.SquarePlantGeneralizedPositions(plant)
    print("nq_squared: ", nq_squared)
    assert nq_squared == 1