"""结构化日志的行为：事件落盘、接触快照、整轮汇总、静默降级。"""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

Q3 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Q3))
from journal import Journal, journal_from_env, pose


def transform(xyz=(0.0, 0.0, 0.0), yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0, xyz[0]],
                     [s, c, 0, xyz[1]],
                     [0, 0, 1, xyz[2]],
                     [0, 0, 0, 1.0]])


class JournalTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.dir = Path(self.folder.name) / 'logs'

    def tearDown(self):
        self.folder.cleanup()

    def test_events_are_jsonl_and_ordered(self):
        journal = Journal(self.dir, run_id='test-run')
        journal.event('info', 'hello', case='touch_toy_1', steps=3)
        journal.event('warn', 'careful', detail='gap=0.001')
        self.assertTrue(str(journal.events_path).startswith(str(self.dir)))
        lines = [json.loads(line) for line in
                 journal.events_path.read_text(encoding='utf-8').splitlines()]
        self.assertEqual([line['msg'] for line in lines], ['run_start', 'hello', 'careful'])
        self.assertEqual(lines[1]['case'], 'touch_toy_1')
        self.assertEqual(lines[1]['steps'], 3)
        self.assertTrue(all(line['run'] == 'test-run' for line in lines))
        self.assertTrue(all('ts' in line and 'mono_s' in line for line in lines))

    def test_level_filter_keeps_file_and_stdout_in_sync(self):
        journal = Journal(self.dir, run_id='quiet', level='warn')
        journal.event('info', 'dropped')
        journal.event('error', 'kept')
        lines = journal.events_path.read_text(encoding='utf-8').splitlines()
        text = '\n'.join(lines)
        self.assertNotIn('dropped', text)
        self.assertIn('kept', text)

    def test_contact_snapshot_has_measured_poses(self):
        journal = Journal(self.dir, run_id='snap')
        journal.case_start('touch_toy_1', 'adaptive', 'Rubik')
        path = journal.contact('touch_toy_1', 'touch', {'joint1': 0.5},
                              transform((1.0, 2.0, 0.0), math.pi),
                              transform((0.5, 0.3, 0.1)),
                              transform((0.6, 0.3, 0.03)),
                              {'paths': ['a', 'b'], 'separation_m': 0.0, 'impulse': 1.0})
        self.assertIsNotNone(path)
        data = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(data['case'], 'touch_toy_1')
        self.assertEqual(data['strategy'], 'adaptive')
        self.assertAlmostEqual(data['joints']['joint1'], 0.5)
        np.testing.assert_allclose(data['base']['xyz'], [1.0, 2.0, 0.0])
        np.testing.assert_allclose(data['toy']['xyz'], [0.6, 0.3, 0.03])
        self.assertEqual(data['contact']['paths'], ['a', 'b'])

    def test_pose_quaternion_is_unit_and_yaw_pi(self):
        result = pose(transform((0.0, 0.0, 0.0), math.pi))
        self.assertAlmostEqual(sum(v * v for v in result['quat_xyzw']), 1.0, places=12)
        self.assertAlmostEqual(abs(result['quat_xyzw'][3]), 0.0, places=9)

    def test_finish_writes_summary_with_outcomes(self):
        journal = Journal(self.dir, run_id='summary')
        journal.case_start('touch_toy_1', 'adaptive')
        journal.case_outcome('touch_toy_1', 'contact', 'touch', command_steps=120)
        path = journal.finish(rows=[{'id': 'touch_toy_1'}], misses=[])
        summary = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual(summary['run'], 'summary')
        self.assertEqual(summary['written_rows'], 1)
        self.assertEqual(summary['cases'][0]['outcome'], 'contact')
        self.assertEqual(summary['cases'][0]['command_steps'], 120)

    def test_failure_summary_marks_error_exit(self):
        journal = Journal(self.dir, run_id='boom')
        path = journal.finish(exit_state='error')
        self.assertEqual(json.loads(path.read_text(encoding='utf-8'))['exit'], 'error')

    def test_no_directory_means_no_files(self):
        journal = Journal(None, run_id='memory-only')
        journal.event('info', 'nothing on disk')
        journal.case_outcome('touch_toy_1', 'miss', 'plan')
        self.assertIsNone(journal.contact('touch_toy_1', 'touch', {}, transform(),
                                          transform(), transform()))
        self.assertIsNone(journal.finish())

    def test_env_defaults(self):
        journal = journal_from_env(None, None)
        self.assertEqual(journal.level, 'info')
        self.assertIsNone(journal.directory)


if __name__ == '__main__':
    unittest.main()
