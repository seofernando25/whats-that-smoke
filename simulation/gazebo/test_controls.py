"""Run with python3 -m unittest discover -s simulation/gazebo -p test_controls.py."""
import unittest
from control_state import ControlState

class ControlsTest(unittest.TestCase):
    def setUp(self):
        self.now = 1
        self.state = ControlState(lambda: self.now)

    def drive(self, keys, seq=1, speed=.12, client='panel'):
        return self.state.drive(client, seq, keys, speed)

    def test_keys_match_existing_dashboard_and_pivot_takes_priority(self):
        self.drive(['KeyW', 'KeyA'])
        self.assertEqual(self.state.command(), (.12, .6))
        self.drive(['KeyS','KeyA','KeyE'], seq=2)
        self.assertEqual(self.state.command(), (-.12, -.6))

    def test_missing_heartbeat_and_disabled_simulation_stop_motion(self):
        self.drive(['KeyW'])
        self.assertEqual(self.state.command(False), (0, 0))
        self.now += .401
        self.assertEqual(self.state.command(), (0, 0))

    def test_delayed_request_cannot_override_newer_release(self):
        self.drive(['KeyW'], seq=1)
        self.drive([], seq=3)
        self.assertFalse(self.drive(['KeyW'], seq=2))
        self.assertEqual(self.state.command(), (0, 0))

    def test_speed_bounds_and_invalid_numeric_values(self):
        self.drive(['KeyW', 'KeyQ'], speed=400)
        self.assertEqual(self.state.command(), (.3, 1))
        with self.assertRaises(ValueError):
            self.drive(['KeyW'], seq=2, speed=float('nan'))

    def test_second_panel_cannot_interrupt_active_owner(self):
        self.drive(['KeyW'])
        with self.assertRaises(ValueError):
            self.drive(['KeyS'], client='other')
        self.now += .401
        self.drive(['KeyS'], client='other')
        self.assertEqual(self.state.command(), (-.12, 0))

    def test_idle_panel_release_does_not_claim_ownership(self):
        self.drive([], client='idle')
        self.drive(['KeyW'])
        self.drive([], seq=2, client='idle')
        self.assertEqual(self.state.command(), (.12, 0))

    def test_pwm_combinations_match_real_wheel_mix_and_deadman(self):
        self.state.drive('panel',1,['KeyW','KeyA'],.12,pwm=1800)
        self.assertEqual(self.state.duty_command(),(0,1800/4095))
        self.state.drive('panel',2,['KeyS','KeyE'],.12,pwm=1300)
        self.assertEqual(self.state.duty_command(),(0,-1300/4095))
        self.now+=.401
        self.assertEqual(self.state.duty_command(),(0,0))
        with self.assertRaises(ValueError):
            self.state.drive('panel',3,['KeyW'],.12,pwm=4095)

if __name__ == '__main__':
    unittest.main()
