"""Measured total, component accounting and physically valid composite inertia."""
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as E
from assembly import assembly
from mass_properties import properties, combine, box

HERE = Path(__file__).resolve().parent

class MassPropertiesTest(unittest.TestCase):
    def test_generated_links_match_measured_total_including_carriers(self):
        model = E.parse(HERE/'freenove_world.sdf').find('./world/model[@name="freenove"]')
        self.assertEqual(len(model.findall('link')), 9)
        self.assertAlmostEqual(sum(float(l.findtext('inertial/mass')) for l in model.findall('link')), .5774)
        self.assertAlmostEqual(float(model.findtext('./link[@name="chassis"]/inertial/mass')), .4454)

    def test_tensor_is_positive_and_mass_distribution_stays_low(self):
        dims = json.loads((HERE/'dimensions.json').read_text())
        links, total = properties(dims, assembly(dims))
        self.assertLess(abs(total['center'][0]), .01)
        self.assertLess(abs(total['center'][1]), .001)
        self.assertGreater(total['center'][2], .0254)
        self.assertLess(total['center'][2], .04445)
        for p in [*links.values(), total]:
            a,b,c,d,e,f = p['inertia']
            self.assertGreater(a, 0)
            self.assertGreater(a*b-d*d, 0)
            self.assertGreater(a*b*c+2*d*e*f-a*f*f-b*e*e-c*d*d, 0)
            # Inertia eigenvalues obey the triangle inequality: trace(I)/2 I - I >= 0.
            half = (a+b+c)/2
            u,v,w = half-a,half-b,half-c
            self.assertGreater(u, 0)
            self.assertGreater(u*v-d*d, 0)
            self.assertGreater(u*v*w-2*d*e*f-u*f*f-v*e*e-w*d*d, 0)

    def test_parallel_axis_theorem_and_products(self):
        cube = box(1, [1,2,0], [1,1,1])
        opposite = box(1, [-1,-2,0], [1,1,1])
        combined = combine([cube, opposite])
        self.assertEqual(combined['center'], [0,0,0])
        self.assertAlmostEqual(combined['inertia'][0], 8+1/3)
        self.assertAlmostEqual(combined['inertia'][3], -4)

if __name__ == '__main__':
    unittest.main()
