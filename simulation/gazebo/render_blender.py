from pathlib import Path
import sys, math, json
import bpy
from mathutils import Vector, Euler
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from assembly import assembly, COLORS
from motor_physics import load_profile
profile=load_profile()
args=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else []
out=Path(args[0]).resolve() if args else HERE/'preview'
out.mkdir(parents=True,exist_ok=True)
p=json.loads((HERE/'dimensions.json').read_text())
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
mats={}
for name,color in COLORS.items():
    m=bpy.data.materials.new(name)
    m.diffuse_color=(*color,1)
    m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF')
    bs.inputs['Base Color'].default_value=(*color,1)
    bs.inputs['Roughness'].default_value=profile['rubber_roughness_visual'] if name=='rubber' else .43
    bs.inputs['Metallic'].default_value=.75 if name in ('silver','gold') else 0
    mats[name]=m
assembly_collection=bpy.data.collections.new('Freenove car - photo approximation')
bpy.context.scene.collection.children.link(assembly_collection)
for group,definition in assembly(p).items():
    origin=Vector(definition['origin'])
    for part in definition['parts']:
        pos=origin+Vector(part['pos'])
        if part['shape']=='box':
            bpy.ops.mesh.primitive_cube_add(size=1,location=pos,rotation=part['rot'])
            obj=bpy.context.object
            obj.dimensions=part['size']
            bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
        else:
            bpy.ops.mesh.primitive_cylinder_add(vertices=48,radius=part['size'][0],depth=part['size'][1],location=pos,rotation=part['rot'])
            obj=bpy.context.object
        obj.name=group+' / '+part['name']
        obj.data.materials.append(mats[part['color']])
        for coll in list(obj.users_collection): coll.objects.unlink(obj)
        assembly_collection.objects.link(obj)
        if not part['name'].startswith(('tread','nano_pin')):
            bevel=obj.modifiers.new('Edge highlight','BEVEL')
            bevel.width=.00025 if part['shape']=='box' else .00015
            bevel.segments=2
            obj.modifiers.new('Normals','WEIGHTED_NORMAL')

# Discreet component labels, approximate rather than a copied PCB layout.
def label(text,pos,size,rotation=(0,0,0)):
    bpy.ops.object.text_add(location=pos,rotation=rotation)
    ob=bpy.context.object
    ob.name='Label '+text
    ob.data.body=text
    ob.data.size=size
    ob.data.align_x='CENTER'
    ob.data.materials.append(mats['white'])
label('FREENOVE',(.028,.023,.04455),.0045)
label('4WD SMART CAR',(.015,-.025,.04455),.0023)
label('Pi',(-.028,-.012,.06135),.004)

floor=bpy.data.materials.new('Studio floor')
floor.diffuse_color=(.22,.25,.29,1)
bpy.ops.mesh.primitive_plane_add(size=200,location=(0,0,-.0015))
ground=bpy.context.object
ground.name='Studio ground'
ground.data.materials.append(floor)
for loc,power,size in [((.2,-.3,.5),4,.35),((-.25,.2,.3),3,.25),((.1,.3,.5),3,.3)]:
    bpy.ops.object.light_add(type='AREA',location=loc)
    lamp=bpy.context.object
    lamp.data.energy=power
    lamp.data.shape='DISK'
    lamp.data.size=size
    lamp.rotation_euler=(Vector((0,0,.03))-lamp.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add(location=(.32,-.38,.30))
cam=bpy.context.object
cam.rotation_euler=(Vector((0,0,.040))-cam.location).to_track_quat('-Z','Y').to_euler()
cam.data.type='ORTHO'
cam.data.ortho_scale=.34
scene=bpy.context.scene
scene.camera=cam
scene.render.engine='CYCLES'
scene.cycles.samples=40
scene.render.resolution_x=1440
scene.render.resolution_y=1080
scene.render.resolution_percentage=100
scene.world.use_nodes=True
bg=scene.world.node_tree.nodes.get('Background')
bg.inputs['Color'].default_value=(.55,.60,.7,1)
bg.inputs['Strength'].default_value=.35
scene.unit_settings.system='METRIC'
scene.view_settings.view_transform='Standard'
scene.view_settings.exposure=-.7
scene.render.filepath=str(out/'freenove-car-preview-v3.png')
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type=='VIEW_3D': area.spaces.active.region_3d.view_perspective='CAMERA'
bpy.ops.object.select_all(action='DESELECT')
bpy.ops.wm.save_as_mainfile(filepath=str(out/'freenove-car-v3.blend'))
bpy.ops.render.render(write_still=True)
# Rotate just the assembly to show its belly, resting on the tire edges again.
for obj in list(assembly_collection.objects):
    obj.location.y=-obj.location.y
    obj.location.z=.0635-obj.location.z
    obj.rotation_euler=(Euler((math.pi,0,0)).to_matrix()@obj.rotation_euler.to_matrix()).to_euler()
for obj in list(bpy.data.objects):
    if obj.type=='FONT': obj.hide_render=True
scene.render.filepath=str(out/'freenove-car-underside-v3.png')
bpy.ops.render.render(write_still=True)

# A detail view of one TT motor, made from the same Gazebo visual parts.
for obj in list(assembly_collection.objects):
    obj.location.y=-obj.location.y
    obj.location.z=.0635-obj.location.z
    obj.rotation_euler=(Euler((math.pi,0,0)).to_matrix()@obj.rotation_euler.to_matrix()).to_euler()
    obj.hide_render=not obj.name.startswith('chassis / front_left_')
ground.location.z=.0249
cam.location=(.135,-.075,.105)
cam.rotation_euler=(Vector((.044,.0366,.034))-cam.location).to_track_quat('-Z','Y').to_euler()
cam.data.ortho_scale=.10
scene.render.filepath=str(out/'tt-motor-detail.png')
bpy.ops.render.render(write_still=True)
