"""在 ROS 2 下使用安装的官方八界模型验证题一净空，无须启动 RViz。"""
import math
from pathlib import Path
import unittest

from ament_index_python.packages import get_package_share_directory
import xacro

from bajie_tasks.q1_drive import Q1Drive, STAGES_ALL
from bajie_tasks.q1_safety import BODY_CLEARANCE, GROUND_CLEARANCE, ClearanceModel, UnsafeMotion, ZERO


class Q1ClearanceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        share = Path(get_package_share_directory('bajie_robot'))
        model = xacro.process_file(str(share / 'urdf/bajie_robot.urdf.xacro')).toxml()
        cls.clearance = ClearanceModel(model, share)

    def test_official_seed7_pose_violates_clearance_but_is_recorded(self):
        """官方示例位姿同时低于地面与机身冗余；这正是题一把冗余降为记录的原因。"""
        pose = dict(ZERO, lift_joint=0.244767, pitch_joint=-0.174575,
                    joint1=0.407523, joint2=-0.969615, joint3=0.096881,
                    joint4=-0.362640, joint5=-0.954722, joint6=0.020076,
                    arm_finger_joint1=0.286374, arm_finger_joint2=0.286374)
        with self.assertRaisesRegex(UnsafeMotion, '地面'):
            self.clearance.certify(pose, pose)
        with self.assertRaisesRegex(UnsafeMotion, '净空不足'):
            self.clearance.certify(pose, pose, enforce_ground=False)
        ground, body = self.clearance.certify(
            pose, pose, enforce_ground=False, enforce_body=False)
        self.assertLess(ground, GROUND_CLEARANCE)
        self.assertLess(body, BODY_CLEARANCE)

    def test_body_clearance_is_independent_of_ground(self):
        pose = dict(ZERO, joint1=-1.2, joint2=0.6)
        with self.assertRaisesRegex(UnsafeMotion, 'lift_link'):
            self.clearance.certify(pose, pose)

    def test_safe_endpoints_can_have_an_unsafe_middle(self):
        start = dict(ZERO, joint1=1.2, joint3=0.7)
        end = dict(ZERO, joint1=-1.1, joint2=-0.6, joint3=1.0)
        self.clearance.certify(start, start)
        self.clearance.certify(end, end)
        with self.assertRaisesRegex(UnsafeMotion, '地面'):
            self.clearance.certify(start, end)

    def test_seeded_stages_report_clearance_without_blocking(self):
        """全部 seed 都能量出净空下界；硬约束仅在显式开启时生效。"""
        for seed in range(128):
            with self.subTest(seed=seed):
                targets = Q1Drive.sample_targets(seed)
                self.assertEqual(targets, Q1Drive.sample_targets(seed))
                state = ZERO.copy()
                for stage in STAGES_ALL:
                    next_state = {**state, **Q1Drive.goal_for(stage, targets)}
                    ground, body = self.clearance.certify(
                        state, next_state, enforce_ground=False, enforce_body=False)
                    self.assertTrue(math.isfinite(ground) and math.isfinite(body))
                    state = next_state
                ground, body = self.clearance.certify(
                    state, ZERO, enforce_ground=False, enforce_body=False)
                self.assertTrue(math.isfinite(ground) and math.isfinite(body))

    def test_seed7_reproduces_official_example(self):
        """seed=7 的抽样必须与官方 README §2.2 公布的关键帧逐值一致。"""
        targets = Q1Drive.sample_targets(7)
        expected = dict(lift_joint=0.244767, pitch_joint=-0.174575,
                        joint1=0.407523, joint2=-0.969615, joint3=0.096881,
                        joint4=-0.362640, joint5=-0.954722, joint6=0.020076,
                        arm_finger_joint1=0.286374, arm_finger_joint2=0.286374)
        for name, value in expected.items():
            self.assertAlmostEqual(round(targets[name], 6), value, places=6)


if __name__ == '__main__':
    unittest.main()
