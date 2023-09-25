import unittest

import anzu.traj_opt.cc as mut


class Test(unittest.TestCase):
    def test_basic(self):
        tol = mut.ConvergenceCriteriaTolerances()
        print(tol.rel_cost_reduction)
        print(tol.abs_cost_reduction)
        print(tol.rel_gradient_along_dq)
        print(tol.abs_gradient_along_dq)
        print(tol.rel_state_change)
        print(tol.abs_state_change)


if __name__ == "__main__":
    unittest.main()
