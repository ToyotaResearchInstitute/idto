import pytest
from pyidto import TrajectoryOptimizerSolution


def test_optimizer_solution():
    """Smoke test for an IDTO optimizer solution."""
    solution = TrajectoryOptimizerSolution()

    assert len(solution.q) == 0
    assert len(solution.v) == 0
    assert len(solution.tau) == 0

    # We cannot directly assign to the solution
    with pytest.raises(AttributeError):
        solution.q = [1, 2, 3]
