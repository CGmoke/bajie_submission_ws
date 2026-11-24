#!/usr/bin/env python3
"""Run the three toy touches back-to-back in one Isaac Sim 4.5 session.

After a verified contact the arm no longer replays its outbound path back to
the initial pose. It lifts straight up until the toy is released and then plans
the next toy from the measured pose, so the three touches are one continuous
motion. The official scene and models are not modified.
"""
import argparse
import json
import os
from pathlib import Path
import time
import traceback

import numpy as np

from touch import (JOINTS, TOYS, Planner, ClearanceFailure, RecoveredTouchMiss, TouchMiss, answer,
                   check_touch, classify_contact, close_correction, execute_adaptive, execute_clearance,
                   hold_contact, is_surface_contact, load_cases, rigid_inverse, run_chained_cases,
                   run_strategy, smooth_segment, write_answers)
from bridge import install_bridge
from journal import Journal, journal_from_env
from surface import VisibleMesh, meshes_touch

ROBOT = '/World/abr_machine/robot_root'
BASE = ROBOT + '/base/base_footprint'
HAND = ROBOT + '/gripper/objects_link'
FINGERS = tuple(f'{ROBOT}/gripper/gripper_{side}_link' for side in ('left', 'right'))
TOY_ROOT = '/World/company/toys/'
DT = 1 / 60


def visible_finger_contacts(contacts, fingers, finger_triangles, toy_triangles):
    """The same finger must provide both physics and visible-surface evidence."""
    hits = {}
    for finger in fingers:
        points = [p for p in contacts if finger in p[:2] and is_surface_contact(p[3], p[4])]
        if points and meshes_touch(finger_triangles(finger), toy_triangles):
            hits[finger] = min(points, key=lambda p: p[3])
    return hits


# 由 main() 装入；未装入时退化为纯 stdout，行为与旧版一致。
JOURNAL: Journal | None = None


def log(message, level='info', case=None, **fields):
    if JOURNAL is None:
        print(f'[q3] {message}', flush=True)
    else:
        JOURNAL.event(level, message, case=case, **fields)


def focus_viewport(stage, center, close_up=False):
    """Point the session camera at the robot or the measured contact point.

    Writes the camera transform directly instead of using the viewport command
    stack, which fails on stages whose up-axis differs from the Kit default.
    """
    try:
        from pxr import Gf, Usd, UsdGeom
        cam = stage.GetPrimAtPath('/OmniverseKit_Persp')
        if not cam:
            log('viewport camera setup skipped: /OmniverseKit_Persp not found')
            return
        eye = center + (np.array([-.28, -.22, .14]) if close_up
                        else np.array([-2.2, -2.2, 1.6]))
        target = center if close_up else center + np.array([0., 0., .5])
        view = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye.tolist()), Gf.Vec3d(*target.tolist()), Gf.Vec3d(0, 0, 1))
        xform = UsdGeom.Xformable(cam)
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            xform.ClearXformOpOrder()
            xform.MakeMatrixXform().Set(view.GetInverse())
        label = 'contact close-up' if close_up else 'viewport'
        log(f'{label} focused at {np.round(center, 4).tolist()}')
    except Exception as exc:
        log(f'viewport camera setup skipped: {exc}')


def collider_points(colliders, base, Usd, UsdGeom):
    """Sample authored collider geometry in robot base coordinates, not render bounds."""
    time_code = Usd.TimeCode.Default()
    transforms = UsdGeom.XformCache(time_code)
    to_base = rigid_inverse(base)
    arrays = []
    for prim in colliders:
        geometry = UsdGeom.Mesh(prim)
        if geometry:
            points = np.array(geometry.GetPointsAttr().Get(time_code), dtype=float)
            if points.size == 0:
                continue
        elif prim.IsA(UsdGeom.Cube):
            half = float(UsdGeom.Cube(prim).GetSizeAttr().Get(time_code)) / 2
            linear = np.linspace(-half, half, 9)
            points = np.stack(np.meshgrid(linear, linear, linear, indexing='ij'), axis=-1).reshape(-1, 3)
            points = points[np.any(np.abs(points) == half, axis=1)]
        else:
            continue
        pose = to_base @ np.asarray(transforms.GetLocalToWorldTransform(prim)).T
        arrays.append(points @ pose[:3, :3].T + pose[:3, 3])
    return np.vstack(arrays) if arrays else np.empty((0, 3))


class TouchSession:
    """One open stage, one ROS bridge and three consecutive touches."""

    def __init__(self, app, scene, first_case, gui=False):
        import carb
        import omni.graph.core as og
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from omni.physx import get_physx_simulation_interface
        from omni.physx.bindings._physx import ContactEventType
        from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        import rclpy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState

        self.app, self.gui = app, gui
        self.kit_app = omni.kit.app.get_app()
        self.rclpy = rclpy
        self.Usd, self.UsdGeom, self.Gf = Usd, UsdGeom, Gf
        self.UsdPhysics, self.PhysxSchema = UsdPhysics, PhysxSchema
        self.ContactEventType = ContactEventType
        self.PhysicsSchemaTools = PhysicsSchemaTools
        self.session_base = None

        ctx = omni.usd.get_context()
        self.tl = omni.timeline.get_timeline_interface()
        self.tl.stop()
        if ctx.get_stage() is not None:
            ctx.close_stage()
            app.update()
        log(f'session: opening {scene}')
        if not ctx.open_stage(str(scene)):
            raise RuntimeError('Official scene failed to open')
        self.stage = ctx.get_stage()
        self.stage.SetEditTarget(self.stage.GetSessionLayer())
        # The scene can include optional camera/lidar and mobility graphs. The task uses only its own bridge.
        for path in [p.GetPath() for p in self.stage.Traverse() if p.GetTypeName() == 'OmniGraph']:
            self.stage.GetPrimAtPath(path).SetActive(False)
        for path in (HAND, BASE, *FINGERS):
            if not self.stage.GetPrimAtPath(path):
                raise RuntimeError(f'Missing official scene prim: {path}')
        for p in self.stage.Traverse():
            if p.IsA(UsdPhysics.Scene):
                api = PhysxSchema.PhysxSceneAPI.Apply(p)
                api.CreateEnableGPUDynamicsAttr().Set(False)
                api.CreateBroadphaseTypeAttr('MBP').Set('MBP')
                api.CreateTimeStepsPerSecondAttr(60).Set(60)
        carb.settings.get_settings().set_bool('/app/player/useFixedTimeStepping', True)
        self.tl.set_target_framerate(60)
        self.tl.set_play_every_frame(True)

        self.ensure_base(first_case.base)
        robot_prims = Usd.PrimRange(self.stage.GetPrimAtPath(ROBOT))
        self.moving = set(FINGERS) | {
            str(p.GetPath()) for p in robot_prims if p.HasAPI(UsdPhysics.RigidBodyAPI)
            and p.GetName() in ('gripper_link', *(f'arm{i}_link' for i in range(1, 7)))}
        for path in sorted(self.moving):
            PhysxSchema.PhysxContactReportAPI.Apply(
                self.stage.GetPrimAtPath(path)).CreateThresholdAttr().Set(0.)

        self.toy = None
        self.contacts = []
        self.blocked = []
        self.target_near = False
        self.closest_positive_gap = float('inf')
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(
            self._received)
        install_bridge(og, BASE)

        self.node = rclpy.create_node('bajie_q3_touch')
        self.states = []
        self.clocks = []
        self.node.create_subscription(JointState, '/q3/joint_states',
                                      lambda msg: self.states.append(msg), 10)
        self.node.create_subscription(Clock, '/q3/clock', lambda msg: self.clocks.append(msg), 10)
        self.pub = self.node.create_publisher(JointState, '/q3/joint_command', 10)
        self.msg = JointState()
        # Finger 2 is USD's mimic of finger 1; position commands only go to the driven joint.
        self.msg.name = list(JOINTS[:-1])

        self.a = SingleArticulation(prim_path=BASE)
        self.tl.play()
        log('session: initializing PhysX and ROS bridge')
        # GUI 的窗口与渲染是异步初始化的，多走几帧再取 physics view。
        for _ in range(30 if gui else 4):
            app.update()
        self.a.initialize()
        self.ids = np.array([self.a.get_dof_index(j) for j in JOINTS])
        init = np.array([first_case.joints[j] for j in JOINTS], dtype=np.float32)
        self.a.set_joint_positions(init, joint_indices=self.ids)
        self.a.set_joint_velocities(np.zeros_like(init), joint_indices=self.ids)
        self.a.apply_action(ArticulationAction(joint_positions=init, joint_indices=self.ids))

        deadline = time.monotonic() + 12
        while not (self.states and self.clocks and self.pub.get_subscription_count()):
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f'ROS handshake failed: states={len(self.states)}, clocks={len(self.clocks)}, '
                    f'cmd_subscribers={self.pub.get_subscription_count()}')
            self.command(init)
            time.sleep(.005)
        log(f'session: ROS feedback ready, start target={np.round(init, 4).tolist()} '
            f'actual={np.round(self.actual(), 4).tolist()} base={self.world(BASE)[:3, 3].tolist()}')
        if np.max(np.abs(self.actual() - init)) > .04:
            raise RuntimeError('Could not establish requested initial joint state')

    # ---------- 场景与机器人 ----------
    def world(self, path):
        m = self.UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(path))
        m = m.RemoveScaleShear()
        return np.asarray(m).T.copy()

    def place(self, prim, desired):
        x = self.UsdGeom.Xformable(prim)
        old = np.asarray(self.UsdGeom.XformCache().GetLocalToWorldTransform(prim)).T
        parent = np.eye(4) if x.GetResetXformStack() else np.asarray(
            self.UsdGeom.XformCache().GetParentToWorldTransform(prim)).T
        desired = desired.copy()
        desired[:3, :3] *= np.linalg.norm(old[:3, :3], axis=0)
        x.MakeMatrixXform().Set(self.Gf.Matrix4d((np.linalg.inv(parent) @ desired).T.tolist()))

    def ensure_base(self, base):
        # 三例的底盘位姿一致时只摆一次；不一致说明底盘要移动，仍需固定在世界里。
        if self.session_base is not None and np.allclose(base, self.session_base, atol=1e-6):
            return
        if self.session_base is not None:
            log('session: base pose changed; moving the anchored base for the next case')
        assembly = '/World/abr_machine'
        self.place(self.stage.GetPrimAtPath(assembly),
                   base @ rigid_inverse(self.world(BASE)) @ self.world(assembly))
        anchor = self.UsdPhysics.FixedJoint.Define(self.stage, '/World/BajieQ3BaseAnchor')
        anchor.CreateBody1Rel().SetTargets([BASE])
        anchor.CreateLocalPos0Attr().Set(self.Gf.Vec3f(*base[:3, 3]))
        quat = self.Gf.Matrix4d(base.T.tolist()).ExtractRotationQuat()
        anchor.CreateLocalRot0Attr().Set(self.Gf.Quatf(quat.GetReal(),
                                                       self.Gf.Vec3f(*quat.GetImaginary())))
        self.session_base = base.copy()

    def prepare(self, case):
        """Place this case's toy and point the contact filter at it."""
        self.ensure_base(case.base)
        toy = TOY_ROOT + TOYS[case.name]
        prim = self.stage.GetPrimAtPath(toy)
        if not prim:
            raise RuntimeError(f'Missing official scene prim: {toy}')
        colliders = [p for p in self.Usd.PrimRange(prim)
                     if p.HasAPI(self.UsdPhysics.CollisionAPI)
                     and self.UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
        if not colliders:
            raise RuntimeError(f'Missing toy colliders; install the complete official models/: {toy}')
        self.place(prim, case.base @ case.target)
        self.PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.)
        self.toy = toy
        self.target_near = False
        self.closest_positive_gap = float('inf')
        # 玩具是刚体：传送到输入位姿后要落稳再记位移基准，否则 1~2 mm 的
        # 自然下落会被当成「触碰把玩具推走」，把正常测例判成失败。
        for _ in range(20):
            self.app.update()
        toy_range = (self.UsdGeom.BBoxCache(self.Usd.TimeCode.Default(),
                                           [self.UsdGeom.Tokens.default_])
                     .ComputeWorldBound(prim).ComputeAlignedRange())
        return toy, colliders, float(toy_range.GetMax()[2])

    # ---------- 通信 ----------
    def _received(self, headers, data):
        for header in headers:
            if header.type not in (self.ContactEventType.CONTACT_FOUND,
                                   self.ContactEventType.CONTACT_PERSIST):
                continue
            a, b = (str(self.PhysicsSchemaTools.intToSdfPath(v))
                    for v in (header.actor0, header.actor1))
            kind = classify_contact(a, b, self.toy, FINGERS, self.moving)
            if kind is None:
                continue
            for i in range(header.contact_data_offset,
                           header.contact_data_offset + header.num_contact_data):
                point = data[i]
                impulse = float(np.linalg.norm(point.impulse))
                gap = float(point.separation)
                if kind == 'target':
                    self.target_near |= gap <= .001
                    if impulse > 1e-6 and 0. < gap:
                        self.closest_positive_gap = min(self.closest_positive_gap, gap)
                # Keep the existing conservative obstacle guard; only target
                # success requires the actual collision surfaces to meet.
                valid = (is_surface_contact(gap, impulse) if kind == 'target'
                         else impulse > 1e-6 and gap <= .001)
                if valid:
                    entries = self.contacts if kind == 'target' else self.blocked
                    entries.append((a, b, list(point.position), gap, impulse))

    def command(self, positions):
        self.msg.position = list(map(float, positions[:-1]))
        self.pub.publish(self.msg)
        self.contacts.clear()
        self.blocked.clear()
        self.target_near = False
        self.app.update()
        self.rclpy.spin_once(self.node, timeout_sec=0)
        self.rclpy.spin_once(self.node, timeout_sec=0)
        if len(self.states) > 16:
            del self.states[:-8]
        if len(self.clocks) > 16:
            del self.clocks[:-8]

    def actual(self):
        return self.a.get_joint_positions(joint_indices=self.ids)

    # ---------- 单例执行 ----------
    def clear_toy(self, case, planner):
        """Open the fingers, then lift straight up until the touched toy is released."""
        # 合指夹住玩具后直接抬升会把玩具一起带走；先张开到最大开度松手。
        release = self.actual().copy()
        release[-2:] = planner.finger_max - .004
        for _ in range(20):
            self.command(release)
        start = self.actual()
        try:
            targets = list(planner.retreat(start))
        except ValueError as exc:
            raise ClearanceFailure(f'no clearance IK above the toy: {exc}') from exc

        def step(positions):
            self.command(positions)
            return self.target_near, self.blocked

        frames = execute_clearance(start, targets, DT, step)
        log(f'{case.id}: clearance=released frames={frames} '
            f'hand_z={self.world(HAND)[2, 3]:.4f}')

    def show_contact(self, hit):
        self.tl.pause()
        focus_viewport(self.stage, np.asarray(hit[2]), close_up=True)
        hold_until = time.monotonic() + 2.5
        while time.monotonic() < hold_until and self.kit_app.is_running():
            self.app.update()
        focus_viewport(self.stage, self.world(BASE)[:3, 3])
        self.tl.play()

    def run_case(self, case, planner, strategy='baseline', *, depart=True):
        toy, colliders, toy_top = self.prepare(case)
        start = self.actual().copy().astype(float)
        start_fk = planner.forward(start[2:8])
        actual_hand = rigid_inverse(self.world(BASE)) @ self.world(HAND)
        if np.linalg.norm(start_fk[:3, 3] - actual_hand[:3, 3]) > .035:
            raise RuntimeError(f'Official FK does not match sim: predicted={start_fk[:3,3]} '
                               f'actual={actual_hand[:3,3]}')
        start_toy = self.world(toy)
        reference = start_toy.copy()
        max_shift = 0.
        steps = 0
        blocked_steps = 0
        log(f'{case.id}: strategy={strategy} start={np.round(start, 4).tolist()} '
            f'toy={np.round(start_toy[:3, 3], 4).tolist()}')

        def advance(positions):
            nonlocal max_shift, steps, blocked_steps, reference
            self.command(positions)
            steps += 1
            blocked_steps += bool(self.blocked)
            obj = self.world(toy)
            # 指尖还没进入玩具上方 3 cm 时只刷新基准：刚体传送后的落稳不算
            # 「被推走」；进入接触区后才把位移计入 2 mm 拖拽判定。
            hand = planner.forward(positions[2:8])
            tip_z = float((hand @ np.r_[planner.pad, 1.0])[2])
            if tip_z <= toy_top + .03:
                max_shift = max(max_shift, float(np.linalg.norm(obj[:3, 3] - reference[:3, 3])))
            else:
                reference = obj


        def contact_detail(hit):
            return {'paths': [hit[0], hit[1]], 'position_m': list(hit[2]),
                    'separation_m': float(hit[3]), 'impulse': float(hit[4])}

        def report(outcome, phase, detail='', **evidence):
            gap = (float(1e3 * self.closest_positive_gap)
                   if np.isfinite(self.closest_positive_gap) else None)
            fields = dict(command_steps=steps, max_toy_shift_mm=round(1000 * max_shift, 3),
                          non_target_steps=blocked_steps,
                          closest_positive_gap_mm=None if gap is None else round(gap, 6),
                          **evidence)
            if detail:
                fields['detail'] = detail
            if JOURNAL is None:
                tail = ' '.join(f'{k}={v}' for k, v in fields.items())
                log(f'{case.id}: strategy={strategy} outcome={outcome} phase={phase} {tail}')
            else:
                JOURNAL.event('info' if outcome == 'contact' else 'warn', 'case_outcome',
                              case=case.id, outcome=outcome, phase=phase, strategy=strategy,
                              **fields)
                JOURNAL.cases.setdefault(case.id, {'id': case.id}).update(
                    dict(outcome=outcome, phase=phase, strategy=strategy, **fields))

        phase = 'plan'
        collision_only = False
        # 合指类动作的防拖拽线；垂直下压是有意的轻碰，允许玩具动得稍多。
        shift_limit = .002
        row = hit = None
        try:
            try:
                visible = {path: VisibleMesh(self.stage.GetPrimAtPath(path), self.Usd, self.UsdGeom)
                           for path in (toy, *FINGERS)}
            except ValueError as exc:
                raise TouchMiss(str(exc)) from exc

            def observe_contact(positions, current_phase):
                nonlocal collision_only
                advance(positions)
                if os.environ.get('Q3_DEBUG_PLANS') and self.contacts:
                    log(f'{case.id}: [hit] phase={current_phase} '
                        f'cmd_z={planner.forward(positions[2:8])[2, 3]:.4f} '
                        f'hand_z={self.world(HAND)[2, 3]:.4f} contacts={len(self.contacts)}')
                if max_shift > shift_limit:
                    raise TouchMiss(f'target moved more than {1e3 * shift_limit:.0f} mm; '
                                    'stop approaching')
                if not check_touch(current_phase, self.contacts, self.blocked):
                    return {}
                cache = self.UsdGeom.XformCache()
                hits = visible_finger_contacts(self.contacts, FINGERS,
                    lambda finger: visible[finger].triangles(cache),
                    visible[toy].triangles(cache))
                if not hits and not collision_only:
                    log(f'{case.id}: collision hull touched before visible surfaces')
                    collision_only = True
                return hits

            def confirm_contact(impact, first, *, closing=False, planned_minimum=None):
                nonlocal phase
                phase = 'touch'
                hold_base = self.world(BASE)
                previous = impact

                def correct(increments):
                    nonlocal previous
                    if closing:
                        return close_correction(impact, increments, minimum=planner.finger_min,
                                                planned_minimum=planned_minimum)
                    candidate = planner.lower_contact_command(impact, previous, increments,
                                                              hold_base)
                    if candidate is not None:
                        previous = candidate
                    return candidate

                hit, stable, corrections = hold_contact(
                    impact, first, lambda q: observe_contact(q, 'touch'), correct)
                # Record only the final verified physics frame, never the initial hit.
                q = self.actual()
                obj, base, hand = self.world(toy), self.world(BASE), self.world(HAND)
                joints = dict(zip(JOINTS, q))
                row = answer(case, joints, base, hand, obj)
                evidence = dict(visible_surface='touch', stable_contact_frames=stable,
                                contact_corrections=corrections)
                report('contact', phase, f'contact={hit} actual={np.round(q, 5).tolist()}',
                       **evidence)
                if JOURNAL is not None:
                    JOURNAL.contact(case.id, phase, joints, base, hand, obj,
                                    contact_detail(hit), **evidence)
                return row, hit

            if strategy == 'adaptive':
                base = self.world(BASE)
                points = collider_points(colliders, base, self.Usd, self.UsdGeom)
                planning_cache = self.UsdGeom.XformCache()

                def local_vertices(path, frame):
                    vertices = visible[path].triangles(planning_cache).reshape(-1, 3)
                    inv = rigid_inverse(frame)
                    return np.unique(vertices @ inv[:3, :3].T + inv[:3, 3], axis=0)

                label, route = planner.adaptive_waypoints(
                    points, (rigid_inverse(base) @ start_toy)[:3, :3],
                    visible_points=local_vertices(toy, base),
                    finger_points=[local_vertices(finger, self.world(finger))
                                   for finger in FINGERS],
                    start=start)
                final = route[-1]
                opened = next(waypoint for waypoint in route if waypoint.phase == 'open')
                if label.endswith('_press') or label.endswith('_grasp'):
                    # 垂直下压与跨窄边合指都是有意轻碰：玩具会沿支撑方向动一下。
                    shift_limit = .006
                log(f'{case.id}: profile={label} geometry_points={len(points)} '
                    f'open_rad={opened.positions[-2]:.3f} '
                    f'limit_rad={final.positions[-2]:.3f} waypoints={len(route)}')

                def observe(positions, current_phase):
                    nonlocal phase
                    if phase != current_phase:
                        phase = current_phase
                        log(f'{case.id}: adaptive phase={phase}')
                    hits = observe_contact(positions, phase)
                    if hits:
                        return confirm_contact(positions, hits, closing=final.settle,
                                               planned_minimum=final.positions[-2])
                    return None

                def wrist_settled(target):
                    error = np.abs(self.actual() - target)
                    speed = np.abs(self.a.get_joint_velocities(joint_indices=self.ids))
                    return (error[0] <= .001 and np.max(error[1:]) <= .006
                            and np.max(speed) <= .025)

                row, hit = execute_adaptive(start, route, DT, observe, wrist_settled)
            else:
                phase = 'descent'
                aim = case.target[:3, 3].copy()
                if case.name in ('dinosaur', 'soft_toy'):
                    # 竖直下探同样是有意的轻碰，允许玩具轻微位移。
                    shift_limit = .006
                # Keep the existing vertical path; a hull-only hit is not a
                # result and cannot trigger hold corrections or a snapshot.
                bottom = max(.004, min(toy_top, aim[2]) + planner.pad_down - .003)
                log(f'{case.id}: toy_top={toy_top:.4f} m aim={np.round(aim, 4).tolist()} '
                    f'pad_down={planner.pad_down:.4f} m descent_bottom={bottom:.4f} m')
                previous = start
                result = None
                for n, arm in enumerate(planner.descent(aim, bottom=bottom, step=.004)):
                    target = start.copy()
                    target[2:8] = arm
                    for positions in smooth_segment(previous, target, DT,
                                                    speed=.60 if n == 0 else .25):
                        hits = observe_contact(positions, 'touch')
                        if hits:
                            result = confirm_contact(positions, hits)
                            break
                    if result is not None:
                        break
                    previous = target
                if result is None:
                    raise TouchMiss('no verified fingertip contact')
                row, hit = result
        except (TouchMiss, ValueError) as exc:
            report('miss', phase, f'reason={exc} hand={self.world(HAND)[:3, 3].round(5).tolist()} '
                   f'toy={self.world(toy)[:3, 3].round(5).tolist()} '
                   f'actual={self.actual().round(5).tolist()}')
            if steps:
                self.clear_toy(case, planner)
            raise RecoveredTouchMiss(str(exc)) from exc

        if self.gui and hit is not None:
            self.show_contact(hit)
        if depart:
            self.clear_toy(case, planner)
        return row

    def close(self):
        # Leave the arm at the last physical pose; pausing stops the timeline
        # without resetting the simulated state.
        self.tl.pause()
        self.subscription = None
        self.node.destroy_node()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('scene', 'urdf', 'input'):
        p.add_argument(f'--{name}', type=Path, required=True)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument('--output', type=Path, help='Write all three verified rows atomically')
    group.add_argument('--case', choices=list(TOYS), help='Verify one toy; never write the final answer')
    p.add_argument('--gui', action='store_true', help='Open an Isaac Sim window and render the scene')
    # 默认 adaptive-fallback：先跑自适应轻合；某例未取得「可见表面接触」时，
    # 抬升脱离该玩具并回退到同一题的竖直下探再试一次，避免因几微米的正间隙假阴性
    # （实测恐龙 0.003 mm）让整轮作答作废。日志会标明实际生效的 strategy。
    p.add_argument('--strategy', default='adaptive-fallback',
                   choices=('baseline', 'adaptive', 'adaptive-fallback'),
                   help='adaptive 优先，未取得可见表面接触时回退竖直下探')
    p.add_argument('--log-dir', type=Path, default=None,
                   help='结构化日志目录（events.jsonl / contact-<case>.json / run.json）；'
                        '默认取 Q3_LOG_DIR，两者都没有时只打印不落盘')
    p.add_argument('--log-level', default=None, choices=('debug', 'info', 'warn', 'error'),
                   help='stdout 与 events.jsonl 的级别下限，默认 info（或 Q3_LOG_LEVEL）')
    args = p.parse_args()
    global JOURNAL
    JOURNAL = journal_from_env(args.log_dir, args.log_level)
    cases = load_cases(args.input)
    selected = [c for c in cases if args.case is None or args.case == c.name]
    planners = [Planner(args.urdf, c) for c in selected]
    if not args.scene.is_file():
        p.error('scene file not found')
    import scipy.optimize  # Load before starting Kit, to surface missing dependencies early.
    isaac = Path(os.environ.get('ISAAC_PATH', '/isaac-sim'))
    log(f"starting minimal Isaac Sim 4.5 (CPU PhysX, ROS 2, {'GUI' if args.gui else 'no viewport'})")
    from isaacsim import AppFramework
    kit = ((isaac / 'apps' / 'isaacsim.exp.base.kit') if args.gui
           else Path(__file__).with_name('physics.kit').resolve())
    argv = [str(kit), '--portable']
    if args.gui:
        # GPU 显存与容器内存都有限，降低渲染分辨率以容纳三例连续运行。
        argv += ['--/app/renderer/resolution/width=960',
                 '--/app/renderer/resolution/height=540']
    else:
        argv.insert(1, '--no-window')
    # 只挂载实际存在的扩展目录：Launcher 版与 tarball 版的目录集合不完全一致，
    # 传入不存在的路径会让 Kit 在启动阶段报错。
    for folder in ('exts', 'extscache', 'extsPhysics', 'extsDeprecated'):
        path = isaac / folder
        if path.is_dir():
            argv.extend(('--ext-folder', str(path)))
        else:
            log(f'extension folder not present, skipped: {path}', level='warn')
    app = AppFramework('bajie-q3', argv)
    session = None
    try:
        if args.gui:
            import omni.kit.app
            omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
                'isaacsim.ros2.bridge', True)
            from omni.kit.viewport.window import ViewportWindow, get_viewport_window_instances
            windows = list(get_viewport_window_instances())
            if not windows:
                windows = [ViewportWindow('Viewport', width=1280, height=720)]
            for win in windows:
                win.visible = True
            for _ in range(5):
                app.update()
            log(f'viewport windows ready: {len(windows)}')
        import rclpy
        rclpy.init(args=[])
        try:
            session = TouchSession(app, args.scene.resolve(), selected[0], args.gui)

            def attempt_case(item):
                case, planner = item
                last = case is selected[-1]

                def attempt(strategy):
                    JOURNAL.case_start(case.id, strategy, TOYS.get(case.name))
                    return session.run_case(case, planner, strategy, depart=not last)

                return run_strategy(args.strategy, attempt,
                                    lambda text: log(f'{case.id}: {text}'))

            rows, misses = run_chained_cases(zip(selected, planners), attempt_case)
        finally:
            if session is not None:
                session.close()
            rclpy.shutdown()
        if misses:
            raise TouchMiss('no answer written; unverified cases: ' + '; '.join(
                f'{case.name}: {exc}' for (case, _), exc in misses))
        if args.output:
            write_answers(args.output, cases, rows)
            log(f'wrote three verified contacts: {args.output}')
        else:
            log('single-case verified (no final answer written): ' + json.dumps(rows[0], allow_nan=False))
        JOURNAL.finish(rows=rows, misses=misses, output=str(args.output) if args.output else None,
                       strategy=args.strategy, gui=args.gui)
    except BaseException:
        traceback.print_exc()
        if JOURNAL is not None:
            JOURNAL.finish(exit_state='error')
        # Kit shutdown may exit the process itself; keep failures nonzero.
        os._exit(1)
    else:
        app.close()


if __name__ == '__main__':
    main()
