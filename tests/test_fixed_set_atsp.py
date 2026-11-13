import itertools
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'planning_strategy/joint'))
from solve_fixed_set_atsp import solve_fixed_set_atsp, tour_cost


class FixedSetATSPTests(unittest.TestCase):
    def test_asymmetric_subtours_match_enumeration(self):
        rng=np.random.default_rng(17)
        D=rng.integers(3,30,(7,7)).astype(float);np.fill_diagonal(D,0)
        D[0,1]=D[1,0]=D[2,3]=D[3,2]=1
        exact=min(tour_cost(D,[0]+list(p)) for p in itertools.permutations(range(1,7)))
        result=solve_fixed_set_atsp(D,list(range(7)),10)
        self.assertAlmostEqual(result['upper_bound'],exact)
        self.assertTrue(result['certified_optimal'])
        self.assertGreater(result['subtour_cut_count'],0)
        self.assertLess(result['lower_bound_conservative'],exact)

    def test_forbidden_directed_edges(self):
        D=np.full((5,5),np.inf);np.fill_diagonal(D,0)
        for a,b in zip(range(5),[1,2,3,4,0]):D[a,b]=a+1
        result=solve_fixed_set_atsp(D,list(range(5)),10)
        self.assertEqual(result['upper_bound'],15)
        self.assertTrue(result['certified_optimal'])

    def test_no_solver_budget_preserves_feasible_seed_not_false_optimum(self):
        D=np.ones((4,4));np.fill_diagonal(D,0)
        result=solve_fixed_set_atsp(D,list(range(4)),0)
        self.assertEqual(result['upper_bound'],4)
        self.assertFalse(result['certified_optimal'])
        self.assertEqual(result['lower_bound_raw'],0)

if __name__=='__main__':unittest.main()
