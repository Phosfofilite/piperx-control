import math
import time
import unittest
from unittest.mock import Mock
from bridge import Robot
from piper_client import Piper

class ControlTests(unittest.TestCase):
    def test_simulation_requires_consent_and_rejects_bad_targets(self):
        p = Piper()
        with self.assertRaises(ValueError):
            p.arm()
        with self.assertRaises(ValueError):
            p.jog(1, .5)
        p.arm(confirmed=True)
        for target in ([math.nan]*6, [0]*5, [5,45,-45,0,0,0], [True]*6):
            with self.assertRaises(ValueError):
                p.move_joints(target)
        p.jog(1, .5)
        self.assertEqual(p.state()['joints'][0], .5)
        p.armed_until = time.monotonic()-1
        with self.assertRaises(ValueError):
            p.jog(1, .5)

    def fake(self, **overrides):
        import threading
        r = Robot.__new__(Robot)
        r.guard = threading.RLock()
        r.sdk = Mock()
        r.active = True
        r.armed_until = time.monotonic()+15
        r.live_axis=None
        r.live_target=None
        r.live_direction=0
        r.live_deadline=0
        r.target_joints=None
        r.target_gripper=None
        r.target_deadline=0
        r.target_result=None
        r.target_progress=None
        r.speed_deg_s=2.0
        r.acceleration_deg_s2=4.0
        r.gripper_speed_mm_s=5.0
        r.gripper_effort_nm=1.0
        r.current_speed=0.0
        r.current_gripper_speed=0.0
        s = dict(fresh=True, mode=1, fault=0, error_code=0,
                 enabled=[True]*6, joints=[0,45,-45,0,0,0],gripper_mm=10,
                 gripper_mode=0,gripper_fault=0,gripper_fresh=True)
        s.update(overrides)
        r.state = lambda: s
        return r

    def test_real_guard_blocks_without_sending(self):
        for state in [dict(fresh=False), dict(mode=2), dict(fault=7), dict(error_code=1), dict(enabled=[False]*6)]:
            r = self.fake(**state)
            with self.assertRaises(ValueError):
                r.move([.5,45,-45,0,0,0])
            r.sdk.JointCtrl.assert_not_called()
            r.sdk.MotionCtrl_2.assert_not_called()

    def test_real_guard_units_and_maximum_step(self):
        r = self.fake()
        r.move([.5,45,-45,0,0,0])
        r.sdk.JointCtrl.assert_called_once_with(500,45000,-45000,0,0,0)
        r.sdk.reset_mock()
        for target in ([3,45,-45,0,0,0], [0,45,-45,90,0,0], [float('inf')]*6):
            with self.assertRaises(ValueError):
                r.move(target)
        r.sdk.JointCtrl.assert_not_called()

    def test_stop_latches_and_sends_no_resume(self):
        r = self.fake()
        r.stop()
        self.assertFalse(r.active)
        r.sdk.MotionCtrl_1.assert_called_once_with(1,0,0)
        with self.assertRaises(ValueError):
            r.move([.5,45,-45,0,0,0])

    def test_requires_each_feedback_pair(self):
        r = self.fake()
        now = time.monotonic()
        r.seen = {i:now for i in [0x2A1,0x2A5,0x2A6,0x2A7,*range(0x261,0x267)]}
        self.assertTrue(r.fresh())
        r.seen[0x2A6] = now-1
        self.assertFalse(r.fresh())

    def test_live_joint_heartbeat_and_release(self):
        r=self.fake()
        r.live(1,1)
        r.control_tick(.02)
        r.sdk.JointCtrl.assert_called_with(2,45000,-45000,0,0,0)
        r.live_deadline=time.monotonic()-1
        r.control_tick(.02)
        self.assertIsNone(r.live_axis)
        r.sdk.JointCtrl.assert_called_with(0,45000,-45000,0,0,0)
        r.sdk.reset_mock()
        r.control_tick(.02)
        r.sdk.JointCtrl.assert_not_called()

    def test_gripper_units_limits_and_modes(self):
        r=self.fake()
        r.gripper(11)
        r.sdk.GripperCtrl.assert_called_once_with(11000,1000,1,0)
        for state in [dict(gripper_fresh=False),dict(gripper_mode=1),dict(gripper_fault=2)]:
            r=self.fake(**state)
            with self.assertRaises(ValueError):
                r.gripper(11)
            r.sdk.GripperCtrl.assert_not_called()
        r=self.fake()
        for v in [float('nan'),True,-1,51,13]:
            with self.assertRaises(ValueError):
                r.gripper(v)

    def test_live_gripper_does_not_command_joints(self):
        r=self.fake()
        r.live(7,1)
        r.control_tick(.02)
        r.sdk.GripperCtrl.assert_called_with(10004,1000,1,0)
        r.sdk.JointCtrl.assert_not_called()
        r.live(direction=0)
        r.sdk.GripperCtrl.assert_called_with(10000,1000,1,0)

    def test_sim_live_gripper_and_release(self):
        p=Piper()
        p.arm(confirmed=True)
        p.live(7,1)
        time.sleep(.05)
        self.assertGreater(p.state()['gripper_mm'],10)
        p.hold()
        width=p.state()['gripper_mm']
        time.sleep(.03)
        self.assertEqual(width,p.state()['gripper_mm'])

    def test_ended_teach_is_allowed_only_for_explicit_v3_transition(self):
        r=self.fake(mode=2,teach_status=2)
        r.protocol='V3'
        with self.assertRaises(ValueError):
            r.check()
        self.assertEqual(r.check(allow_ended_teach=True)['teach_status'],2)
        r.protocol='V2'
        with self.assertRaises(ValueError):
            r.check(allow_ended_teach=True)
        r=self.fake(mode=2,teach_status=1)
        r.protocol='V3'
        with self.assertRaises(ValueError):
            r.check(allow_ended_teach=True)

    def test_absolute_target_is_ramped_not_sent_as_a_jump(self):
        r=self.fake()
        r.goto(joints=[30,45,-45,0,0,0])
        r.sdk.JointCtrl.assert_not_called()
        r.control_tick(.02)
        r.sdk.JointCtrl.assert_called_once_with(2,45000,-45000,0,0,0)
        for _ in range(100):
            r.control_tick(.02)
        # A blocked arm must never accumulate more than half a degree of target lead.
        self.assertLessEqual(r.sdk.JointCtrl.call_args.args[0],500)

    def test_absolute_target_requires_heartbeat_and_honors_cancel(self):
        r=self.fake()
        r.goto(joints=[30,45,-45,0,0,0])
        r.target_deadline=time.monotonic()-1
        r.control_tick(.02)
        self.assertIsNone(r.target_joints)
        self.assertEqual(r.target_result,'heartbeat_expired')
        r.goto(width_mm=30)
        r.hold()
        self.assertIsNone(r.target_gripper)
        self.assertEqual(r.target_result,'cancelled')

    def test_absolute_target_rejects_invalid_values_without_sending(self):
        r=self.fake()
        for values in ([200,45,-45,0,0,0],[math.nan]*6,[0]*5,[True]*6):
            with self.assertRaises(ValueError):
                r.goto(joints=values)
        for width in [-1,51,math.inf,True]:
            with self.assertRaises(ValueError):
                r.goto(width_mm=width)
        r.sdk.JointCtrl.assert_not_called()
        r.sdk.GripperCtrl.assert_not_called()

    def test_sim_library_absolute_joint_and_gripper_and_timeout(self):
        p=Piper()
        p.arm(confirmed=True)
        result=p.set_joint_angle(1,.3,timeout=2)
        self.assertEqual(result['target_result'],'reached')
        self.assertLessEqual(abs(p.state()['joints'][0]-.3),.1)
        self.assertEqual(p.state()['joints'][1:],[45,-45,0,0,0])
        result=p.set_gripper_width(11,timeout=2)
        self.assertEqual(result['target_result'],'reached')
        with self.assertRaises(ValueError):
            p.set_joint_angle(1,20,timeout=-1)
        self.assertFalse(p.state()['moving_to_target'])
        with self.assertRaises(TimeoutError):
            p.set_joint_angle(1,30,timeout=.02)
        self.assertFalse(p.state()['moving_to_target'])

    def test_sim_async_target_expires_without_keepalive(self):
        p=Piper()
        p.arm(confirmed=True)
        p.set_joint_angle(1,30,wait=False)
        time.sleep(.35)
        s=p.state()
        self.assertFalse(s['moving_to_target'])
        self.assertEqual(s['target_result'],'heartbeat_expired')
        self.assertLessEqual(s['joints'][0],.61)

    def test_absolute_target_reports_arrival_and_no_progress(self):
        r=self.fake()
        r.goto(joints=[0,45,-45,0,0,0])
        r.control_tick(.02)
        self.assertEqual(r.target_result,'reached')
        self.assertIsNone(r.target_joints)
        r.goto(joints=[30,45,-45,0,0,0])
        r.target_progress=(30,time.monotonic()-4)
        r.control_tick(.02)
        self.assertEqual(r.target_result,'no_progress')
        self.assertIsNone(r.target_joints)

    def test_settings_validate_atomically_without_motor_commands(self):
        r=self.fake()
        r.configure(speed_deg_s=4,acceleration_deg_s2=8,gripper_effort_nm=.5)
        self.assertEqual(r.speed_deg_s,4)
        self.assertEqual(r.gripper_effort_nm,.5)
        for values in [dict(speed_deg_s=11),dict(acceleration_deg_s2=0),dict(gripper_effort_nm=2),dict(speed_deg_s=math.nan)]:
            with self.assertRaises(ValueError):
                r.configure(**values)
        with self.assertRaises(ValueError):
            r.configure(speed_deg_s=3,gripper_speed_mm_s=99)
        self.assertEqual(r.speed_deg_s,4)
        r.sdk.JointCtrl.assert_not_called()
        r.sdk.GripperCtrl.assert_not_called()

    def test_multi_joint_targets_share_progress_and_accelerate(self):
        r=self.fake()
        r.goto(joints=[30,60,-45,0,0,0])
        r.control_tick(.02)
        first=r.live_target[:]
        self.assertAlmostEqual(first[0],2*(first[1]-45))
        self.assertAlmostEqual(r.current_speed,.08)
        r.control_tick(.02)
        self.assertGreater(r.live_target[0]-first[0],first[0])
        self.assertAlmostEqual(r.live_target[0],2*(r.live_target[1]-45))

    def test_simulation_settings_and_synchronized_targets(self):
        p=Piper()
        p.configure(speed_deg_s=4,acceleration_deg_s2=8)
        p.arm(confirmed=True)
        p.set_joint_angles([30,60,-45,0,0,0],wait=False)
        time.sleep(.05)
        s=p.keepalive()
        self.assertAlmostEqual(s['joints'][0],2*(s['joints'][1]-45))
        self.assertEqual(s['settings']['speed_deg_s'],4)
        p.hold()

class ClosingConfirmationTests(unittest.TestCase):
    def test_denied_close_does_not_interrupt_anything(self):
        from app import App
        from types import SimpleNamespace
        from unittest.mock import patch
        fake=SimpleNamespace(_finish_close=Mock())
        with patch('app.messagebox.askyesno',return_value=False):
            App.quit_app(fake)
        fake._finish_close.assert_not_called()

    def test_confirmed_close_calls_finalizer(self):
        from app import App
        from types import SimpleNamespace
        from unittest.mock import patch
        fake=SimpleNamespace(_finish_close=Mock())
        with patch('app.messagebox.askyesno',return_value=True):
            App.quit_app(fake)
        fake._finish_close.assert_called_once()

    def test_denied_disconnect_does_not_release_controls(self):
        from app import App
        from types import SimpleNamespace
        from unittest.mock import patch
        fake=SimpleNamespace(robot=Mock(),end_hold=Mock())
        with patch('app.messagebox.askyesno',return_value=False):
            App.disconnect(fake)
        fake.end_hold.assert_not_called()
        fake.robot.close.assert_not_called()

if __name__ == '__main__':
    unittest.main()
