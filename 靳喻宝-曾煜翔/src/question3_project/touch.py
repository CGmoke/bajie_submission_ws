"""Pure input, planning and result helpers for the three official touch cases."""
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
from tempfile import NamedTemporaryFile
import xml.etree.ElementTree as ET
from itertools import product

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'question2_project'))
from fk import BajieChain, finite_number, matrix_to_pose, pose_to_matrix, rigid_inverse

ARM = tuple(f'joint{i}' for i in range(1, 7))
JOINTS = ('lift_joint', 'pitch_joint', *ARM, 'arm_finger_joint1', 'arm_finger_joint2')
TOYS = {'rubik_cube': 'Rubik', 'dinosaur': 'toy_yellow_dragon', 'soft_toy': 'spiderman_grasp'}


def finger_pad_offset(urdf, open_angle):
    """左手指尖垫在 objects_link 坐标系中的位置：+X 向下、+Y 侧向、+Z 径向。"""
    root = ET.parse(urdf).getroot()
    distal = None
    for link in root.findall('link'):
        if link.attrib['name'] != 'gripper_left_link':
            continue
        for collision in link.findall('collision'):
            box = collision.find('geometry/box')
            if box is None:
                continue
            origin = collision.find('origin')
            xyz = [float(v) for v in
                   (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            half_x = float(box.attrib['size'].split()[0]) / 2
            candidate = (xyz[0] + half_x, xyz[1], 0.0)
            if distal is None or candidate[0] > distal[0]:
                distal = candidate
    if distal is None:
        raise ValueError('官方 URDF 缺少 gripper_left_link 的 box 碰撞几何')
    to_object = BajieChain.from_urdf(urdf, base='gripper_link', tip='objects_link').forward({})
    to_finger = BajieChain.from_urdf(urdf, base='gripper_link', tip='gripper_left_link').forward(
        {'arm_finger_joint1': open_angle})
    return (np.linalg.inv(to_object) @ to_finger @ np.r_[distal, 1.0])[:3]


@dataclass(frozen=True)
class Case:
    id: str
    name: str
    joints: dict
    base: np.ndarray
    target: np.ndarray


def load_cases(path):
    cases = []
    for number, line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(), 1):
        try:
            row = json.loads(line)
            ident, name, joints = row['id'], row['name'], row['joint_state']
            if not isinstance(ident, str) or not ident or name not in TOYS or set(joints) != set(JOINTS):
                raise ValueError('invalid id, name, or joint names')
            joints = {j: finite_number(joints[j], j) for j in JOINTS}
            base = pose_to_matrix(row['base_footprint_in_world'])
            obj = pose_to_matrix(row['scene_object_in_world'])
        except (KeyError, ValueError, TypeError) as exc:
            raise ValueError(f'{path}:{number}: {exc}') from exc
        cases.append(Case(ident, name, joints, base, rigid_inverse(base) @ obj))
    if len(cases) != 3 or len({c.id for c in cases}) != 3 or {c.name for c in cases} != TOYS.keys():
        raise ValueError('Expected exactly one official case per toy, with distinct IDs')
    return cases


def answer(case, joints, base, hand, obj):
    return {'id': case.id, 'final_joint_state': {j: finite_number(float(joints[j]), j) for j in JOINTS},
            'objects_link_in_base_footprint': matrix_to_pose(rigid_inverse(base) @ hand),
            'scene_object_in_objects_link': matrix_to_pose(rigid_inverse(hand) @ obj)}


def write_answers(path, cases, rows):
    if len(cases) != 3 or [r['id'] for r in rows] != [c.id for c in cases]:
        raise ValueError('All three confirmed contacts are required to write the answer')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = None
    try:
        with NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as f:
            tmp = Path(f.name)
            for row in rows:
                f.write(json.dumps(row, allow_nan=False) + '\n')
        # NamedTemporaryFile 默认 0600。Isaac Sim 的进程在容器里以 root 运行，
        # 若不放宽权限，宿主上的调用者连读都读不了（复刻时会误以为没产出）。
        tmp.chmod(0o644)
        tmp.replace(path)
    finally:
        if tmp is not None:
            tmp.unlink(missing_ok=True)


def smooth_segment(start, stop, dt, speed=.5):
    delta = stop - start
    steps = max(1, math.ceil(1.5 * np.max(np.abs(delta)) / speed / dt))
    for i in range(1, steps + 1):
        t = i / steps
        yield start + (3 * t * t - 2 * t**3) * delta


def contact_pair(a, b, toy, fingers):
    return (a == toy and b in fingers) or (b == toy and a in fingers)


def classify_contact(a, b, toy, fingers, moving_bodies):
    if contact_pair(a, b, toy, fingers):
        return 'target'
    if a in moving_bodies or b in moving_bodies:
        return 'blocked'
    return None


def is_surface_contact(separation, impulse):
    """Reject predictive contacts across a positive gap, even with impulse."""
    return (math.isfinite(separation) and separation <= 0.
            and math.isfinite(impulse) and impulse > 1e-6)


class TouchMiss(RuntimeError):
    """Expected adaptive planning or contact failure; eligible for a clean retry."""


class RecoveredTouchMiss(TouchMiss):
    """A miss after the arm was lifted clear of the toy; the chain can continue."""


class ClearanceFailure(RuntimeError):
    """Halt the chain; a touch is not permission to move past the toy blindly."""


def run_chained_cases(cases, attempt):
    """Run every case in one live session; only a clean, cleared miss may continue."""
    rows, misses = [], []
    for case in cases:
        try:
            rows.append(attempt(case))
        except RecoveredTouchMiss as exc:
            misses.append((case, exc))
    return rows, misses


def execute_clearance(start, targets, dt, step, *, speed=.12, release_frames=4):
    """Climb through pre-solved targets until the touched toy is released.

    step sends one setpoint and returns (touching, blocked) for that frame.
    Any non-target contact, or re-touching the toy after it was released,
    halts the chain instead of dragging the toy into the next case.
    """
    previous = start
    released = 0
    frames = 0

    def checked(setpoint):
        nonlocal released, frames
        touching, blocked = step(setpoint)
        frames += 1
        if blocked:
            raise ClearanceFailure(f'non-target contact while clearing: {blocked[0]}')
        if released >= release_frames and touching:
            raise ClearanceFailure('toy re-contacted while clearing')
        released = 0 if touching else released + 1

    for target in targets:
        for setpoint in smooth_segment(previous, target, dt, speed):
            checked(setpoint)
        previous = target
    # 最后一段可能只有几帧；到顶后保持，等接触报告确认玩具确实离开。
    for _ in range(30):
        if released >= release_frames:
            break
        checked(previous)
    if released < release_frames:
        raise ClearanceFailure('toy was not released before the clearance height')
    return frames


@dataclass(frozen=True)
class Waypoint:
    phase: str
    positions: np.ndarray
    # settle=False：到达该点后不必先等手腕停稳（press 姿态不合指，没有这个前置条件）。
    settle: bool = True


@dataclass(frozen=True)
class TouchPose:
    label: str
    rotation: np.ndarray
    hover: np.ndarray
    approach: np.ndarray
    touch: np.ndarray
    open_angle: float
    touch_angle: float
    # press=True：腕部垂直下压取接触，手指全程保持 open_angle 不合。
    # 适用于侧脸近乎垂直的玩具（恐龙），闭合弧线扫掠会把玩具横向拖走。
    press: bool = False


def check_touch(phase, contacts, blocked):
    if blocked:
        raise TouchMiss(f'non-target contact: {blocked[0]}')
    if contacts and phase != 'touch':
        raise TouchMiss(f'early toy contact in {phase}')
    return bool(contacts)


def execute_adaptive(initial, waypoints, dt, observe_step, wrist_settled=None):
    """One observation per commanded step; never issue another step after a hit."""
    previous = initial
    for waypoint in waypoints:
        if waypoint.phase == 'touch' and wrist_settled is not None and waypoint.settle:
            # Hold the open terminal pose until measured motion has stopped.
            consecutive = 0
            for _ in range(90):
                observe_step(previous, 'approach')
                consecutive = consecutive + 1 if wrist_settled(previous) else 0
                if consecutive == 4:
                    break
            else:
                raise TouchMiss('open wrist did not settle before closing')
        speed = {'clear': .30, 'align': .90, 'open': .30, 'approach': .30, 'touch': .10}[waypoint.phase]
        for positions in smooth_segment(previous, waypoint.positions, dt, speed):
            result = observe_step(positions, waypoint.phase)
            if result is not None:
                return result
        previous = waypoint.positions
    # A final stationary command allows the bridge/drives to catch up; no further closing.
    for _ in range(12):
        result = observe_step(previous, 'touch')
        if result is not None:
            return result
    raise TouchMiss('no verified fingertip contact')


def hold_contact(impact, first, observe, correct, *, stable_frames=6, loss_frames=6,
                 max_corrections=4, max_frames=60):
    """Hold the impact setpoint; count consecutive qualifying frames per finger."""
    command = impact.copy()
    streak = dict.fromkeys(first, 1)
    lost = corrections = 0
    for _ in range(max_frames):
        hits = observe(command)
        streak = {finger: streak.get(finger, 0) + 1 for finger in hits}
        for finger, count in streak.items():
            if count >= stable_frames:
                return hits[finger], count, corrections
        lost = 0 if hits else lost + 1
        if lost >= loss_frames:
            if corrections >= max_corrections:
                break
            candidate = correct(corrections + 1)
            if candidate is None or np.array_equal(candidate, command):
                break
            command = candidate
            lost = 0
            corrections += 1
    raise TouchMiss('visible fingertip contact did not stabilize')


def close_correction(impact, increments, *, minimum, planned_minimum):
    """Recover contact within the planned closing arc and the joint limits."""
    angle = max(minimum, planned_minimum, float(impact[-2]) - .003 * increments)
    if angle >= impact[-2]:
        return None
    result = impact.copy()
    result[-2:] = angle
    return result


def run_strategy(strategy, attempt, report):
    if strategy != 'adaptive-fallback':
        return attempt(strategy)
    try:
        return attempt('adaptive')
    except TouchMiss as exc:
        report(f'adaptive miss ({exc}); lift clear and retry baseline')
    return attempt('baseline')


_BOX_SIGNS = np.array(list(product((-1., 1.), repeat=3)))


def _transform_points(pose, points):
    return points @ pose[:3, :3].T + pose[:3, 3]


class GripperGeometry:
    """Collision box corners in the URDF, with the distal box last on each finger."""

    def __init__(self, urdf):
        root = ET.parse(urdf).getroot()
        links = {link.get('name'): link for link in root.findall('link')}
        joints = {joint.get('name'): joint for joint in root.findall('joint')}
        self.tool = rigid_inverse(BajieChain.from_urdf(
            urdf, base='gripper_link', tip='objects_link').forward({}))
        self.fingers = []
        for i, side in enumerate(('left', 'right'), 1):
            link = f'gripper_{side}_link'
            joint = joints[f'arm_finger_joint{i}']
            origin = self.origin(joint.find('origin'))
            axis = np.array(joint.find('axis').get('xyz').split(), dtype=float)
            self.fingers.append((self.tool @ origin, axis / np.linalg.norm(axis), self.boxes(links[link])))
        self.palm = _transform_points(self.tool, self.boxes(links['gripper_link']).reshape(-1, 3))

    @staticmethod
    def origin(origin):
        return pose_to_matrix({key: list(map(float, (origin.get(key, '0 0 0') if origin is not None
                                                     else '0 0 0').split())) for key in ('xyz', 'rpy')})

    @staticmethod
    def boxes(link):
        boxes = []
        for collision in link.findall('collision'):
            box = collision.find('geometry/box')
            if box is None:
                continue
            half = np.array(box.get('size').split(), dtype=float) / 2
            boxes.append(_transform_points(GripperGeometry.origin(collision.find('origin')), _BOX_SIGNS * half))
        if not boxes:
            raise TouchMiss(f'no URDF collision boxes for {link.get("name")}')
        return np.array(sorted(boxes, key=lambda b: b[:, 0].max()))

    def at(self, angle):
        return np.array([self.finger_points(side, angle, boxes)
                         for side, (_, _, boxes) in enumerate(self.fingers)])

    def finger_points(self, side, angle, points):
        from scipy.spatial.transform import Rotation

        # Official mimic is 1:1. Each joint has its own signed axis.
        origin, axis, _ = self.fingers[side]
        return _transform_points(origin, points @ Rotation.from_rotvec(axis * angle).as_matrix().T)


class Planner:
    """Follow a down-facing gripper pose through incremental IK waypoints."""

    def __init__(self, urdf, case):
        self.case = case
        self.chain = BajieChain.from_urdf(urdf)
        limits = {j.attrib['name']: j.find('limit') for j in ET.parse(urdf).findall('joint')}
        self.lower = np.array([float(limits[n].attrib['lower']) for n in ARM])
        self.upper = np.array([float(limits[n].attrib['upper']) for n in ARM])
        self.seed = np.array([case.joints[n] for n in ARM])
        for n in JOINTS:
            limit = limits[n].attrib
            if not float(limit['lower']) - 1e-6 <= case.joints[n] <= float(limit['upper']) + 1e-6:
                raise ValueError(f'Initial value out of URDF limits: {n}')
        self.pad = finger_pad_offset(urdf, case.joints['arm_finger_joint1'])
        self.urdf = urdf
        self.finger_min = max(float(limits[n].get('lower')) for n in JOINTS[-2:])
        self.finger_max = min(float(limits[n].get('upper')) for n in JOINTS[-2:])
        self.pad_down = float(self.pad[0])
        self.rotation = np.eye(3)
        self.hand_xy = np.zeros(2)

    def forward(self, q):
        return self.chain.forward({**self.case.joints, **dict(zip(ARM, map(float, q)))})

    def solve(self, point, rotation, seed):
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation

        def residual(q):
            pose = self.forward(q)
            return np.r_[pose[:3, 3] - point,
                         .25 * Rotation.from_matrix(rotation.T @ pose[:3, :3]).as_rotvec()]

        fit = least_squares(residual, np.clip(seed, self.lower, self.upper),
                            bounds=(self.lower, self.upper), max_nfev=100, gtol=1e-8)
        if np.linalg.norm(fit.fun) > .004:
            raise ValueError(f'IK could not reach {point.tolist()} (error={np.linalg.norm(fit.fun):.4f})')
        return fit.x

    def descent(self, point, top=.25, bottom=None, step=.004):
        # objects_link +X 指向指尖；把左手指尖垫对准 point，而不是只对准工具原点。
        radial = np.array([point[0], point[1], 0.])
        radial /= np.linalg.norm(radial)
        down = np.array([0., 0., -1.])
        self.rotation = np.column_stack((down, np.cross(radial, down), radial))
        offset = self.rotation @ self.pad
        self.hand_xy = np.array([point[0], point[1]]) - offset[:2]
        floor = max(.004, point[2]) if bottom is None else bottom
        initial = np.array([self.hand_xy[0], self.hand_xy[1], top])
        candidates = []
        for seed in self.approach_seeds():
            try:
                candidates.append(self.solve(initial, self.rotation, seed))
            except ValueError:
                continue
        if not candidates:
            raise ValueError(f'No approach IK solution for {self.case.id}')
        q = min(candidates, key=lambda x: np.linalg.norm(x - self.seed))
        yield q
        for z in np.linspace(top, floor, 1 + math.ceil((top - floor) / step))[1:]:
            q = self.solve(np.array([self.hand_xy[0], self.hand_xy[1], z]), self.rotation, q)
            yield q

    def lower_contact_command(self, impact, previous, increments, base_in_world):
        """Move the impact TCP down 0.25 mm per increment without changing finger angle."""
        origin = self.forward(impact[2:8])
        down = base_in_world[:3, :3].T @ np.array([0., 0., -1.])
        target = origin[:3, 3] + down * (.00025 * increments)
        try:
            arm = self.solve(target, origin[:3, :3], previous[2:8])
        except ValueError:
            return None
        pose = self.forward(arm)
        # General approach IK tolerates millimetres of error; contact IK must
        # actually move by the commanded fraction of a millimetre.
        if (np.linalg.norm(pose[:3, 3] - target) > .00005
                or np.linalg.norm(pose[:3, :3] - origin[:3, :3]) > .001
                or np.max(np.abs(arm - previous[2:8])) > .015):
            return None
        result = impact.copy()
        result[2:8] = arm
        return result

    def retreat(self, positions, lift=.08, step=.004):
        """Lift the tool straight up from the measured pose until the toy is free.

        The chain never replays the outbound path: a short vertical release is
        enough to stop touching, and the next toy is planned from there.
        """
        pose = self.forward(positions[2:8])
        x, y, z = (float(v) for v in pose[:3, 3])
        rotation = pose[:3, :3].copy()
        q = np.array(positions[2:8], dtype=float)
        count = max(1, math.ceil(lift / step))
        for i in range(1, count + 1):
            try:
                q = self.solve(np.array([x, y, z + lift * i / count]), rotation, q)
            except ValueError:
                # 上方姿态超出可达范围时，停在已经爬升的高度；调用方仍会
                # 要求玩具已脱离，否则按 ClearanceFailure 中止。
                if i == 1:
                    raise
                return
            target = np.array(positions, dtype=float)
            target[2:8] = q
            yield target

    def press_poses(self, points, visible_points=None):
        """Pad-down vertical press onto the case target; contact comes from the descent.

        恐龙侧脸近乎垂直：贴着侧面合指时指尖沿圆弧扫掠，会把玩具横向推走
        （实测 2.14 mm，触发 2 mm 保护）；而指尖朝下垂直下压只推动 0.36 mm，
        因为位移沿玩具的支撑方向。这里把「垂直下压」做成自适应的兜底候选：
        手指保持输入姿态不变，接触完全由腕部下降产生，不叠加闭合力。

        软玩具改用居中合指夹取：原来的下压只把左指垫对到玩具上，工具中心
        偏向一侧，两指并没有跨住玩具；现在工具中心对准玩具中心（视觉上等于
        整体向右挪半个指距），张开到最大开度跨过窄边，再合指夹住。
        """
        if self.case.name == 'soft_toy':
            return [self.soft_toy_grasp(points, visible_points)]
        point = np.asarray(self.case.target[:3, 3], dtype=float)
        down = np.array([0., 0., -1.])
        radial = np.array([point[0], point[1], 0.])
        norm = float(np.linalg.norm(radial))
        if norm < 1e-6:
            raise TouchMiss('target is straight above the base origin; no press direction')
        radial /= norm
        rotation = np.column_stack((down, np.cross(radial, down), radial))
        # pad 是左手指尖垫在夹爪坐标系里的偏移；旋转后取其在地面内的分量，
        # 让指尖垫（而不是工具原点）落在目标点正上方。
        offset = rotation @ self.pad
        hand_xy = point[:2] - offset[:2]
        goal_z = float(point[2]) + self.pad_down - .003
        goal = np.array([hand_xy[0], hand_xy[1], goal_z])
        # 接近点必须高于玩具最高点：下压接触发生在下降途中，而接触只允许出现在
        # touch 相位，所以从「最高点上方 30 mm」起就进入 touch 段，一路缓降到 goal。
        ceiling = float(np.max(np.asarray(points, dtype=float)[:, 2]))
        approach = np.array([hand_xy[0], hand_xy[1], max(goal_z + .012, ceiling + .030)])
        hover = np.array([hand_xy[0], hand_xy[1], max(.25, approach[2] + .10)])
        angle = float(self.case.joints['arm_finger_joint1'])
        return [TouchPose(f'{self.case.name}_press', rotation, hover, approach, goal,
                          angle, angle, press=True)]

    def soft_toy_grasp(self, points, visible_points=None):
        """从上方把软玩具夹在两指之间：工具中心对准玩具中心，竖直下降后合指。

        软玩具窄边（实测约 57 mm）在切向，与下压姿态的开合方向一致；张开到
        最大开度（两指内距约 76 mm）可以跨过它，降到躯干高度后合指即可夹住。
        """
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or len(points) < 4 or not np.isfinite(points).all():
            raise TouchMiss('missing soft-toy collision geometry')
        low, high = points.min(axis=0), points.max(axis=0)
        centre = (low + high) / 2
        top = high[2]
        if visible_points is not None:
            visible = np.asarray(visible_points, dtype=float)
            if visible.ndim == 2 and len(visible) and np.isfinite(visible).all():
                top = float(visible[:, 2].max())
        down = np.array([0., 0., -1.])
        radial = np.array([centre[0], centre[1], 0.])
        norm = float(np.linalg.norm(radial))
        if norm < 1e-6:
            raise TouchMiss('soft toy is straight above the base origin; no approach direction')
        radial /= norm
        rotation = np.column_stack((down, np.cross(radial, down), radial))
        # 指尖垫降到玩具顶面以下 24 mm 的躯干处；工具中心与玩具中心同轴。
        goal = np.array([centre[0], centre[1], top - .024 + self.pad_down])
        hover = np.array([centre[0], centre[1], max(.25, top + .12)])
        # 先张到最大开度跨过窄边，再合指；接触会在合指途中触发并停在接触帧。
        opened = self.finger_max - .004
        closed = self.finger_min + .004
        return TouchPose('soft_toy_grasp', rotation, hover, goal, goal.copy(),
                         opened, closed)

    def adaptive_poses(self, points, toy_rotation):
        """Propose in base coordinates; authored geometry ranks poses, PhysX verifies them."""
        points = np.asarray(points, dtype=float)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 4
                or not np.isfinite(points).all() or np.ptp(points[:, 2]) < .004):
            raise TouchMiss('missing or degenerate toy collision geometry')
        grip = GripperGeometry(self.urdf)
        ceiling = float(points[:, 2].max())
        floor = float(points[:, 2].min())
        down = np.array([0., 0., -1.])
        radial = self.case.target[:3, 3].copy()
        radial[2] = 0
        axes = []
        for column in (0, 1):
            axis = toy_rotation[:, column].copy()
            axis[2] = 0
            length = np.linalg.norm(axis)
            if length > 1e-6:
                axes.append(axis / length)
        # For organic shapes the collision footprint is often not aligned with the prim axes.
        if self.case.name != 'rubik_cube':
            _, vectors = np.linalg.eigh(np.cov(points[:, :2], rowvar=False))
            axes.extend(np.r_[vectors[:, i], 0.] for i in (0, 1))
        axes.sort(key=lambda axis: np.ptp(points @ axis) if self.case.name == 'dinosaur'
                  else abs(axis @ radial))
        open_angle = self.finger_max - .004
        sweep = np.linspace(open_angle, self.finger_min + .004, 65)
        fingers_open = grip.at(open_angle)
        spans = []
        for angle in sweep:
            pads = grip.at(angle)[:, -1]
            spans.append(float(pads[0, :, 1].min() - pads[1, :, 1].max()))
        plans = []
        # Test opposite tool rolls as well: a geometrically identical alignment can
        # require a very different arm posture under the official joint limits.
        seen = []
        for axis in axes:
            if any(abs(axis @ prev) > .995 for prev in seen):
                continue
            seen.append(axis)
            for closing in (axis, -axis):
                tangent = np.cross(down, closing)
                rotation = np.column_stack((down, closing, tangent))
                u, v = points @ closing, points @ tangent
                vlo, vhi = float(v.min()), float(v.max())
                mid = (vlo + vhi) / 2
                if self.case.name != 'soft_toy':
                    # Estimate the width around the body, not the dinosaur's head or tail.
                    strip = np.abs(v - mid) <= max(.021, .23 * (vhi - vlo))
                    section = points[strip]
                    if len(section) >= 4:
                        left, right = float(u[strip].max()), float(u[strip].min())
                        width = left - right
                        local_top = float(section[:, 2].max())
                        # The dinosaur's upper contour tapers inward from the body width.
                        # Descend into the upper body and reserve a little extra closure.
                        dinosaur = self.case.name == 'dinosaur'
                        inset = .19 if dinosaur else .12
                        pad_z = local_top - np.clip(inset * (local_top - floor), .006, .016)
                        # 合指宽度必须按夹取高度处的截面算：整段高度会按躯干最宽处
                        # 估算，恐龙实测合指停在 48 mm，够不到 44 mm 的躯干。
                        near = strip & (np.abs(points[:, 2] - pad_z) <= .006)
                        if dinosaur and np.count_nonzero(near) >= 4:
                            left, right = float(u[near].max()), float(u[near].min())
                            width = left - right
                        center = (left + right) / 2
                        overlap = .011 if dinosaur else .003
                        hit = next((i for i, span in enumerate(spans) if span <= width - overlap), None)
                        if hit is not None and hit > 1:
                            # Full open-finger boxes below the local top must clear
                            # the toy corridor throughout the descent, not just at the endpoint.
                            distal = grip.at(float(sweep[hit]))[:, -1]
                            goal = center * closing + mid * tangent
                            goal[2] = pad_z + float(distal[:, :, 0].mean())
                            safe = True
                            for finger in fingers_open:
                                for box in finger:
                                    depth = goal[2] - float(box[:, 0].max())
                                    a, b = mid + box[:, 2].min() - .002, mid + box[:, 2].max() + .002
                                    nearby = (v >= a) & (v <= b) & (points[:, 2] >= depth - .002)
                                    lo = center + box[:, 1].min() - .002
                                    hi = center + box[:, 1].max() + .002
                                    if np.any(nearby & (u >= lo) & (u <= hi)):
                                        safe = False
                            # Ensure the open distal pads actually flank the target.
                            if (safe and spans[0] >= width + .010
                                    and goal[2] - grip.palm[:, 0].max() >= ceiling + .008):
                                hover = goal.copy()
                                hover[2] = max(.25, ceiling + .10)
                                label = 'cube_side' if self.case.name == 'rubik_cube' else 'dinosaur_body'
                                plans.append(TouchPose(label, rotation, hover, goal, goal.copy(),
                                                       open_angle, float(sweep[hit])))
        if self.case.name != 'rubik_cube':
            # Wide toys needn't fit between both fingers. Aim one distal pad at
            # an upper-body patch; the small final descent is itself contact-gated.
            center = (points.min(axis=0) + points.max(axis=0)) / 2
            central = np.linalg.norm(points[:, :2] - center[:2], axis=1) <= .026
            patch = points[central] if np.count_nonzero(central) >= 3 else points
            upper = patch[patch[:, 2] >= np.quantile(patch[:, 2], .8)]
            opened = open_angle
            closed = float(np.clip(self.case.joints[JOINTS[-2]] - .075, self.finger_min + .004, opened))
            for axis in seen[:2]:
                for closing in (axis, -axis):
                    rotation = np.column_stack((down, closing, np.cross(down, closing)))
                    for side in (0, 1):
                        pads = grip.at(closed)
                        pad = pads[side, -1].mean(axis=0)
                        pad[0] = pads[side, -1, :, 0].max()
                        # Try the broad upper patch first, then move that pad toward
                        # the side that leaves the unused finger outside the toy.
                        inward = closing if side == 0 else -closing
                        width = np.ptp(points @ inward)
                        targets = [center - inward * width * fraction for fraction in (0., .32, .46)]
                        for aim in targets:
                            samples = upper if np.array_equal(aim, center) else points[
                                points[:, 2] >= ceiling - .012]
                            target = samples[np.argmin(np.linalg.norm(samples[:, :2] - aim[:2], axis=1))]
                            goal = target - rotation @ pad
                            goal[2] -= .002
                            projected = (points - goal) @ rotation
                            other = np.vstack([grip.at(float(a))[1-side] for a in np.linspace(opened, closed, 5)])
                            if any(np.any(np.all((projected >= box.min(axis=0) - .002)
                                                 & (projected <= box.max(axis=0) + .002), axis=1))
                                   for box in other):
                                continue
                            if goal[2] - grip.palm[:, 0].max() < ceiling + .006:
                                continue
                            hover = goal + np.array([0., 0., max(.25, ceiling + .10) - goal[2]])
                            approach = goal + np.array([0., 0., .010])
                            label = 'soft_upper' if self.case.name == 'soft_toy' else 'dinosaur_single'
                            plans.append(TouchPose(label, rotation, hover, approach, goal, opened, closed))
                            break
        if not plans:
            raise TouchMiss('no collision-geometry candidate with room for the gripper')
        return plans

    def surface_close_poses(self, points, toy_rotation, fingers):
        """Aim a tilted fingertip at an exposed shoulder, not into a concavity."""
        from scipy.spatial import cKDTree

        grip = GripperGeometry(self.urdf)
        tree = cKDTree(points)
        open_angle = self.finger_max - .004
        aim_angle = max(self.finger_min + .05, open_angle - .082)
        limit = aim_angle - .05
        up = np.array([0., 0., 1.])
        # The dinosaur exposes the side of its crest, the soft toy its upper rim.
        column, direction, preferred_side = ((0, 1, 0) if self.case.name == 'dinosaur'
                                              else (1, 1, 1))
        opened = [grip.finger_points(i, open_angle, p) for i, p in enumerate(fingers)]
        aimed = [grip.finger_points(i, aim_angle, p) for i, p in enumerate(fingers)]
        closed = [grip.finger_points(i, limit, p) for i, p in enumerate(fingers)]
        plans = []
        for col, sign in ((column, direction), (column, -direction),
                          (1-column, 1), (1-column, -1)):
            radial = sign * toy_rotation[:, col].copy()
            radial[2] = 0.
            if np.linalg.norm(radial) < 1e-6:
                continue
            radial /= np.linalg.norm(radial)
            for tilt in (15., 25.):
                beta = math.radians(tilt)
                x = -math.cos(beta) * up - math.sin(beta) * radial
                y = math.cos(beta) * radial - math.sin(beta) * up
                elevation = math.radians(55.)
                outward = math.cos(elevation) * radial + math.sin(elevation) * up
                patch = points[np.argmax(points @ outward)]
                for side in (preferred_side, 1-preferred_side):
                    closing = y if side == 0 else -y
                    rotation = np.column_stack((x, closing, np.cross(x, closing)))
                    normal = rotation.T @ outward
                    tip = aimed[side][np.argmin(aimed[side] @ normal)]
                    goal = patch - rotation @ tip
                    if tip[0] < -.006:  # Use a distal pad, never the finger root.
                        continue
                    # A supporting plane clears the selected open pad, so
                    # lowering the wrist cannot be the source of its contact.
                    projection = tip @ normal
                    if ((opened[side] @ normal).min() - projection < .003
                            or projection - (closed[side] @ normal).min() < .001):
                        continue
                    other = 1 - side
                    offside = np.concatenate((opened[other], aimed[other], closed[other], grip.palm))
                    offside = offside @ rotation.T + goal
                    # Leave room for the other finger and palm during approach
                    # and closure. This is a coarse filter; PhysX guards the path.
                    if min(tree.query(offside + dz * up)[0].min()
                           for dz in (0., .008, .016, .032, .064)) < .008:
                        continue
                    low_z = min((p @ rotation[2]).min() for p in (*opened, *closed, grip.palm))
                    if goal[2] + low_z < .006:
                        continue
                    hover = goal.copy()
                    hover[2] = max(.25, goal[2] + .10)
                    plans.append(TouchPose(f'{self.case.name}_surface_close', rotation,
                                           hover, goal, goal, open_angle, limit))
        if not plans:
            raise TouchMiss('no exposed surface with a clear open approach')
        return plans

    def adaptive_waypoints(self, points, toy_rotation, *, visible_points=None, finger_points=None,
                           start=None):
        # start 为空时按输入初始位姿规划（单例行为）；链式执行时传入上一例
        # 实测的十关节姿态，让本例从当前实际位置接续，而不是跳回归位点。
        initial = np.array([self.case.joints[j] for j in JOINTS] if start is None
                           else start, dtype=float)
        last_error = None
        if self.case.name != 'rubik_cube' and visible_points is not None:
            try:
                poses = self.surface_close_poses(visible_points, toy_rotation, finger_points)
            except TouchMiss:
                poses = []
            if self.case.name in ('dinosaur', 'soft_toy'):
                # 软玩具优先用「腕部垂直下压、不合指」取接触：闭合弧线会横向
                # 扫掠把玩具拖走；指尖朝下垂直下压沿支撑方向施力，位移最小。
                # 恐龙改用「从上方跨在躯干窄边两侧再合指」：原来下压姿态的开合
                # 轴正好落在恐龙长轴（实测 228 mm）上，两指合不上；转 90° 后开合
                # 轴落到躯干窄边（实测夹取高度约 40 mm），才能真的夹住恐龙。
                press = self.press_poses(points, visible_points)
                try:
                    adaptive = self.adaptive_poses(points, toy_rotation)
                except TouchMiss:
                    adaptive = []
                poses = ((adaptive + press) if self.case.name == 'dinosaur'
                         else (press + adaptive)) + poses
            if not poses:
                raise TouchMiss('no collision-geometry candidate with room for the gripper')
        else:
            poses = self.adaptive_poses(points, toy_rotation)
            if self.case.name == 'dinosaur':
                poses.sort(key=lambda pose: pose.label != 'dinosaur_single')
        if os.environ.get('Q3_DEBUG_PLANS'):
            print(f'[plans] {self.case.id}: ' +
                  ', '.join(f'{p.label}@{np.round(p.touch, 3).tolist()}' for p in poses),
                  file=sys.stderr)
        for pose in poses:
            starts = []
            for seed in self.approach_seeds():
                try:
                    q = self.solve(pose.hover, pose.rotation, seed)
                    if not any(np.max(np.abs(q - prior)) < .02 for prior in starts):
                        starts.append(q)
                except ValueError as exc:
                    last_error = exc
            # Try the IK branch requiring the least initial joint motion first.
            for q in sorted(starts, key=lambda q: np.max(np.abs(q - self.seed))):
                try:
                    route = []
                    if start is not None:
                        # 链式执行：先保持当前腕姿竖直抬到悬停高度，再横向
                        # 对齐下一玩具，避免贴着上一玩具做关节空间换向。
                        hand = self.forward(initial[2:8])
                        try:
                            clear = self.solve(
                                np.array([hand[0, 3], hand[1, 3],
                                          max(hand[2, 3], pose.hover[2])]),
                                hand[:3, :3], initial[2:8])
                            route.append(Waypoint('clear',
                                                  self._positions(initial, clear, initial[-2])))
                        except ValueError:
                            pass
                    route += [Waypoint('align', self._positions(initial, q, initial[-2])),
                              Waypoint('open', self._positions(initial, q, pose.open_angle))]
                    # press 姿态不靠合指取接触：接近段停在接触点上方 12 mm，
                    # 最后一段纯竖直下压放到 touch 相位（该相位允许目标接触）。
                    # 其余姿态保持原样：先到最终位姿并张开，再原地合指。
                    target = pose.approach if pose.press else pose.touch
                    count = max(1, math.ceil(np.linalg.norm(target - pose.hover) / .010))
                    for i in range(1, count + 1):
                        next_q = self.solve(pose.hover + (target - pose.hover) * (i/count),
                                            pose.rotation, q)
                        if np.max(np.abs(next_q - q)) > .35:
                            raise ValueError('IK branch jumped between adjacent poses')
                        q = next_q
                        route.append(Waypoint('approach', self._positions(initial, q, pose.open_angle)))
                    if pose.press:
                        q = self.solve(pose.touch, pose.rotation, q)
                        if np.max(np.abs(q - route[-1].positions[2:8])) > .35:
                            raise ValueError('IK branch jumped during the final press')
                    route.append(Waypoint('touch', self._positions(initial, q, pose.touch_angle),
                                          settle=not pose.press))
                    return pose.label, route
                except ValueError as exc:
                    last_error = exc
        raise TouchMiss(f'no continuous adaptive IK path: {last_error}')

    @staticmethod
    def _positions(initial, arm, angle):
        positions = initial.copy()
        positions[2:8] = arm
        positions[-2:] = angle
        return positions

    def approach_seeds(self):
        return self.seed, [0, 1, 2, 0, -1.5, 0], [0, 1, -1.4, 0, 1, 0], [0, -1, -1.5, 0, 1, 0]
