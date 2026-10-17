"""Estimated component distribution constrained to the measured total mass.

Coordinates are model-frame metres, tensor values kg m². Component masses,
centres and envelopes are estimates, not individual measurements.
"""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent


def box(mass, center, size):
    x, y, z = size
    return dict(mass=mass, center=center,
                inertia=[mass*(y*y+z*z)/12, mass*(x*x+z*z)/12,
                         mass*(x*x+y*y)/12, 0, 0, 0])


def combine(parts):
    mass = sum(p['mass'] for p in parts)
    center = [sum(p['mass']*p['center'][i] for p in parts)/mass for i in range(3)]
    tensor = [0.0]*6
    for p in parts:
        x, y, z = [p['center'][i]-center[i] for i in range(3)]
        shifts = [y*y+z*z, x*x+z*z, x*x+y*y, -x*y, -x*z, -y*z]
        for i in range(6):
            tensor[i] += p['inertia'][i] + p['mass']*shifts[i]
    return dict(mass=mass, center=center, inertia=tensor)


def properties(dimensions, groups):
    budget = json.loads((HERE/'mass_distribution.json').read_text())
    entries = budget['rigid_components']
    parts = []
    for item in entries:
        for center in item['centers_m']:
            parts.append(box(item['each_mass_g']/1000, center, item['envelope_m']))
    rigid = combine(parts)
    # Annular rubber tread plus a plastic hub; rotation axis is link Y.
    radius = dimensions['wheel_diameter_in']*.0254/2
    width = (dimensions['overall_width_in']-dimensions['chassis_width_in'])*.0254/2
    wheel = budget['wheel']
    inner = wheel['rubber_inner_radius_m_assumed']
    rubber, plastic = wheel['rubber_mass_g']/1000, wheel['plastic_mass_g']/1000
    spin = rubber*(radius*radius+inner*inner)/2 + plastic*inner*inner/2
    radial = (rubber*(3*(radius*radius+inner*inner)+width*width)/12
              + plastic*(3*inner*inner+wheel['hub_width_m_assumed']**2)/12)
    wheel_mass = rubber+plastic
    result = {'chassis': rigid}
    assembly_parts = [rigid]
    for name, group in groups.items():
        if name == 'chassis':
            continue
        result[name] = dict(mass=wheel_mass, center=[0, 0, 0],
                            inertia=[radial, spin, radial, 0, 0, 0])
        carrier = box(budget['each_carrier_mass_g']/1000, [0, 0, 0], [.01]*3)
        result[name+'_carrier'] = carrier
        for component in (result[name], carrier):
            assembly_parts.append(dict(component, center=group['origin']))
    total = combine(assembly_parts)
    if abs(total['mass']-budget['measured_total_mass_g']/1000) > 1e-10:
        raise ValueError('Component budget does not match the measured car mass')
    return result, total
