import os
import sys
import unittest
from pathlib import Path

import numpy as np

Q3 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Q3))
import touch

URDF = Path(os.environ.get('BAJIE_URDF', Q3.parents[1] / 'src/question2_project/generated/bajie_robot.urdf'))


class ClearanceTests(unittest.TestCase):
    def test_climbs_until_the_toy_is_released_with_a_speed_limit(self):
        start = np.zeros(10)
        top = start.copy()
        top[2] = .08
        sent = []

        def step(q):
            sent.append(q.copy())
            return len(sent) <= 3, []

        frames = touch.execute_clearance(start, [top], 1/60, step, speed=.48)
        self.assertEqual(len(sent), frames)
        np.testing.assert_array_equal(sent[-1], top)
        deltas = np.diff(np.vstack([start, *sent]), axis=0)
        self.assertLessEqual(np.max(np.abs(deltas)), .48/60 + 1e-12)

    def test_unreleased_toy_halts_before_the_next_case(self):
        start = np.zeros(10)
        top = start.copy()
        top[2] = .08
        with self.assertRaisesRegex(touch.ClearanceFailure, 'not released'):
            touch.execute_clearance(start, [top], 1/60, lambda q: (True, []))

    def test_recontact_after_release_and_non_target_contact_halt_the_chain(self):
        start = np.zeros(10)
        top = start.copy()
        top[2] = .08
        flags = iter([False] * 4 + [True])
        with self.assertRaisesRegex(touch.ClearanceFailure, 're-contacted'):
            touch.execute_clearance(start, [top], 1/60,
                                    lambda q: (next(flags, False), []))
        with self.assertRaisesRegex(touch.ClearanceFailure, 'non-target contact'):
            touch.execute_clearance(start, [top], 1/60,
                                    lambda q: (True, [('arm', 'obstacle')]))


class RetreatPlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = touch.load_cases(Q3.parents[1] / 'inputs/q3_demo_input.jsonl')

    def test_retreat_lifts_the_tool_vertically_and_keeps_the_fingers(self):
        for case in self.cases:
            with self.subTest(case=case.name):
                planner = touch.Planner(URDF, case)
                start = np.array(list(case.joints.values()))
                targets = list(planner.retreat(start, lift=.02, step=.004))
                self.assertGreaterEqual(len(targets), 4)
                before = planner.forward(start[2:8])[:3, 3]
                after = planner.forward(targets[-1][2:8])[:3, 3]
                self.assertGreater(after[2], before[2] + .015)
                np.testing.assert_allclose(after[:2], before[:2], atol=.004)
                for target in targets:
                    np.testing.assert_array_equal(target[[0, 1, 8, 9]], start[[0, 1, 8, 9]])


if __name__ == '__main__':
    unittest.main()
