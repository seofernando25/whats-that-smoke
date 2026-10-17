"""Regression checks for motor units, generated force actuation, and tire axes."""
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as E
from motor_physics import load_profile

HERE=Path(__file__).resolve().parent

class MotorModelTest(unittest.TestCase):
    def setUp(self):
        self.profile=load_profile()
        self.model=E.parse(HERE/'freenove_world.sdf').find('./world/model[@name="freenove"]')

    def test_reference_units_are_output_shaft_units(self):
        self.assertAlmostEqual(self.profile['stall_nm'],.0784532)
        self.assertAlmostEqual(self.profile['omega0'],250*2*math.pi/60)

    def test_velocity_actuator_cannot_override_motor_forces(self):
        plugins={p.get('name') for p in self.model.findall('plugin')}
        self.assertIn('ugv::TTMotorSystem',plugins)
        self.assertNotIn('gz::sim::systems::DiffDrive',plugins)
        joints=self.model.findall('./joint[@type="revolute"]')
        self.assertEqual(len(joints),4)
        for joint in joints:
            self.assertAlmostEqual(float(joint.findtext('axis/limit/effort')),self.profile['stall_nm'])

    def test_rubber_contacts_have_lateral_axis_and_radial_compliance(self):
        springs=self.model.findall('./joint[@type="prismatic"]')
        self.assertEqual(len(springs),4)
        for link in self.model.findall('link'):
            if not link.get('name').endswith('_wheel'):
                continue
            contact=link.find('collision/surface/friction/ode')
            self.assertEqual(contact.findtext('fdir1'),'0 0 1')
            self.assertEqual(link.findtext('collision/pose').split()[3],str(math.pi/2))
            self.assertGreater(float(contact.findtext('slip1')),float(contact.findtext('slip2')))
            self.assertGreater(float(contact.findtext('mu')),0)

if __name__=='__main__':
    unittest.main()
