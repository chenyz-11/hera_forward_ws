import math
import unittest

from hera_forward.formation_switch_logic import formation_templates
from hera_forward.recovery_scenario import make_scenario


class RecoveryScenarioTest(unittest.TestCase):
    def setUp(self):
        wide, _ = formation_templates(15.0, 15.0)
        self.initial = {i: (*wide[i], 0.0) for i in range(1, 7)}

    def test_phases_roles_and_relative_formation(self):
        p = make_scenario(self.initial)
        pos, t = p['positions'], p['times_s']
        lost, rescue = p['dropout'], p['rescuer']
        self.assertEqual((p['assignment'][lost], p['assignment'][rescue]), (5, 6))
        self.assertEqual(pos[2][lost], pos[3][lost])
        self.assertAlmostEqual(pos[3][rescue][0] - pos[2][rescue][0], 20.0)
        self.assertAlmostEqual(pos[4][rescue][0], pos[4][lost][0])
        self.assertAlmostEqual(pos[4][lost][1] - pos[4][rescue][1], 15.0)
        for r in self.initial:
            self.assertAlmostEqual(pos[5][r][0] - pos[1][r][0],
                                   pos[5][1][0] - pos[1][1][0])
            self.assertAlmostEqual(pos[6][r][0] - pos[5][r][0], 50.0)
        self.assertTrue(all(a < b for a, b in zip(t, t[1:])))
        self.assertGreaterEqual(p['minimum_separation'][0], 5.0)

    def test_rejects_unsafe_or_invalid_inputs(self):
        close = dict(self.initial)
        close[2] = close[1]
        with self.assertRaisesRegex(ValueError, 'separation'):
            make_scenario(close)
        with self.assertRaises(ValueError):
            make_scenario(self.initial, catchup_speed=1.0)
        with self.assertRaises(ValueError):
            make_scenario(self.initial, cruise_speed=math.nan)


if __name__ == '__main__':
    unittest.main()
