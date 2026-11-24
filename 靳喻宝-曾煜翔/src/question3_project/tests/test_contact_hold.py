import sys
import unittest
from pathlib import Path

import numpy as np

Q3 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Q3))
import touch
from run_touch import visible_finger_contacts


class ContactHoldTests(unittest.TestCase):
    def test_one_frame_and_switching_fingers_do_not_count(self):
        readings = iter(({'left': 'l1'}, {'right': 'r1'}, {'right': 'r2'}, {'right': 'r3'}))
        steps = []

        def observe(command):
            steps.append(command.copy())
            return next(readings)

        result = touch.hold_contact(np.zeros(10), {'left': 'first'}, observe,
                                    lambda _: self.fail('should hold the impact command'), stable_frames=3)
        self.assertEqual(result, ('r3', 3, 0))
        self.assertEqual(len(steps), 4)
        np.testing.assert_array_equal(steps, np.zeros((4, 10)))

    def test_corrects_only_after_loss_and_rechecks_after_correction(self):
        readings = iter(({}, {}, {'left': 'new'}, {'left': 'stable'}))
        commands = []

        def observe(command):
            commands.append(command[-1])
            return next(readings)

        def nudge(n):
            self.assertEqual(n, 1)
            return np.full(10, .003)

        self.assertEqual(touch.hold_contact(np.zeros(10), {'left': 'first'}, observe, nudge,
                                            stable_frames=2, loss_frames=2), ('stable', 2, 1))
        self.assertEqual(commands, [0., 0., .003, .003])

    def test_loss_is_bounded_and_does_not_publish_a_stale_hit(self):
        commands = []
        with self.assertRaises(touch.TouchMiss):
            touch.hold_contact(np.zeros(10), {'left': 'first'},
                               lambda q: commands.append(q[-1]) or {},
                               lambda n: np.full(10, float(n)), loss_frames=2, max_corrections=2)
        self.assertEqual(commands, [0., 0., 1., 1., 2., 2.])

    def test_contact_guard_failure_stops_without_another_command(self):
        def observe(_):
            raise touch.TouchMiss('target moved more than 2 mm')

        with self.assertRaisesRegex(touch.TouchMiss, '2 mm'):
            touch.hold_contact(np.zeros(10), {'left': 'first'}, observe,
                               lambda _: self.fail('should not correct after guard failure'))

    def test_close_correction_only_moves_fingers_and_respects_limit(self):
        impact = np.arange(10, dtype=float)
        impact[-2:] = .02
        result = touch.close_correction(impact, 4, minimum=.01, planned_minimum=.011)
        np.testing.assert_array_equal(result[:-2], impact[:-2])
        np.testing.assert_allclose(result[-2:], [.011, .011])
        np.testing.assert_allclose(impact[-2:], [.02, .02])

    def test_descent_correction_keeps_finger_angle_and_uses_original_tcp(self):
        case = touch.load_cases(Q3.parents[1] / 'inputs/q3_demo_input.jsonl')[0]
        planner = touch.Planner(Q3.parents[1] / 'generated/bajie_robot.urdf', case)
        impact = np.array(list(case.joints.values()))
        result = planner.lower_contact_command(impact, impact, 2, np.eye(4))
        self.assertIsNotNone(result)
        before = planner.forward(impact[2:8])
        after = planner.forward(result[2:8])
        np.testing.assert_allclose(after[:3, 3] - before[:3, 3], [0, 0, -.0005], atol=1e-5)
        np.testing.assert_allclose(after[:3, :3], before[:3, :3], atol=1e-5)
        np.testing.assert_array_equal(result[[0, 1, 8, 9]], impact[[0, 1, 8, 9]])


class VisibleFingerContactTests(unittest.TestCase):
    def test_matching_finger_must_own_both_physical_and_visible_contact(self):
        toy = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
        separated_finger = toy + [0., 0., .00005]
        hull_hit = ('left', 'toy', [0., 0., 0.], -.0002, .002)
        visible_hit = ('toy', 'right', [0., 0., 0.], -.0001, .003)
        meshes = {'left': separated_finger, 'right': toy}
        self.assertEqual(visible_finger_contacts([hull_hit], meshes, meshes.__getitem__, toy), {})
        self.assertEqual(visible_finger_contacts([hull_hit, visible_hit], meshes, meshes.__getitem__, toy),
                         {'right': visible_hit})

    def test_positive_separation_is_not_accepted_even_if_meshes_touch(self):
        toy = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
        hit = ('left', 'toy', [0., 0., 0.], .00005, .002)
        self.assertEqual(visible_finger_contacts([hit], ('left',), lambda _: toy, toy), {})
