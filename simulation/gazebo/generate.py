"""Generate a photo-informed Freenove visual model; stdlib only, SI in SDF."""
from assembly import assembly, COLORS
from environment import add_environment
from pathlib import Path
import json
import math
import xml.etree.ElementTree as E

HERE = Path(__file__).resolve().parent

def node(parent, tag, text=None, **attrs):
    el = E.SubElement(parent, tag, attrs)
    if text is not None:
        el.text = str(text)
    return el

def main():
    p = json.loads((HERE / 'dimensions.json').read_text())
    inch = .0254
    length, width, outer, diameter, overall = [p[k] * inch for k in
        ('chassis_length_in', 'chassis_width_in', 'overall_width_in', 'wheel_diameter_in', 'overall_length_in')]
    radius = diameter / 2
    tire = (outer - width) / 2
    track = outer - tire
    wheelbase = overall - diameter
    height = .06
    groups = assembly(p)
    root = E.Element('sdf', version='1.9')
    world = node(root, 'world', name='freenove_lab')
    physics = node(world, 'physics', name='default', type='ignored')
    node(physics, 'max_step_size', .001)
    node(physics, 'real_time_factor', 1)
    for name, cls in [('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')]:
        node(world, 'plugin', filename=f'gz-sim-{name}-system', name=f'gz::sim::systems::{cls}')
    add_environment(world, node)
    model = node(world, 'model', name='freenove')
    node(model, 'pose', '0 0 0.001 0 0 0')

    for name, group in groups.items():
        link = node(model, 'link', name=name)
        node(link, 'pose', ' '.join(map(str, (*group['origin'], 0, 0, 0))))
        inertial = node(link, 'inertial')
        mass = p['chassis_mass_kg_placeholder'] if name == 'chassis' else p['wheel_mass_kg_placeholder']
        node(inertial, 'mass', mass)
        if name == 'chassis':
            node(inertial, 'pose', '0 0 .045 0 0 0')
            values = (mass*(width**2+height**2)/12, mass*(length**2+height**2)/12, mass*(length**2+width**2)/12)
        else:
            radial = mass*(3*radius**2+tire**2)/12
            values = (radial, mass*radius**2/2, radial)
        inertia = node(inertial, 'inertia')
        for tag, value in zip(('ixx','iyy','izz','ixy','ixz','iyz'), (*values,0,0,0)):
            node(inertia, tag, value)
        for part in group['parts']:
            for kind in (('visual','collision') if part['collision'] else ('visual',)):
                obj=node(link, kind, name=part['name']+'_'+kind)
                node(obj, 'pose', ' '.join(map(str, (*part['pos'],*part['rot']))))
                geom=node(obj, 'geometry')
                shape=node(geom, part['shape'])
                if part['shape']=='box':
                    node(shape,'size',' '.join(map(str,part['size'])))
                else:
                    node(shape,'radius',part['size'][0])
                    # Collision reaches the full measured wheel radius; treads are visual.
                    if kind=='collision' and name!='chassis':
                        shape.find('radius').text=str(radius)
                    node(shape,'length',part['size'][1])
                if kind=='visual':
                    color=' '.join(map(str,(*COLORS[part['color']],1)))
                    mat=node(obj,'material')
                    node(mat,'ambient',color)
                    node(mat,'diffuse',color)
        if name!='chassis':
            joint=node(model,'joint',name=name+'_joint',type='revolute')
            node(joint,'parent','chassis')
            node(joint,'child',name)
            axis=node(joint,'axis')
            node(axis,'xyz','0 1 0')
            limit=node(axis,'limit')
            node(limit,'lower','-1e16')
            node(limit,'upper','1e16')
    drive = node(model, 'plugin', filename='gz-sim-diff-drive-system', name='gz::sim::systems::DiffDrive')
    for side in ('left', 'right'):
        for end in ('front', 'rear'):
            node(drive, side+'_joint', f'{end}_{side}_wheel_joint')
    node(drive, 'wheel_separation', track)
    node(drive, 'wheel_radius', radius)
    node(drive, 'topic', '/model/freenove/cmd_vel')
    node(drive, 'odom_topic', '/model/freenove/odometry')
    node(drive, 'max_linear_velocity', .3)
    node(drive, 'max_angular_velocity', 1)
    E.indent(root)
    E.ElementTree(root).write(HERE / 'freenove_world.sdf', encoding='utf-8', xml_declaration=True)
    print(f'Generated: wheelbase={wheelbase*1000:.2f}mm, track={track*1000:.2f}mm, tire width={tire*1000:.2f}mm')

if __name__ == '__main__':
    main()
