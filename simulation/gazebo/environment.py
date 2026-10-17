"""A small, well-lit lab floor with a visual-only metric test mat."""

def add_environment(world, node):
    scene=node(world,'scene')
    node(scene,'ambient','.65 .65 .65 1')
    node(scene,'background','.84 .88 .9 1')
    node(scene,'shadows','true')
    node(scene,'grid','false')
    for name,direction,diffuse in (
        ('key_light','-.4 .3 -1','.85 .85 .85 1'),
        ('fill_light','.4 -.5 -1','.35 .38 .42 1')):
        light=node(world,'light',name=name,type='directional')
        node(light,'pose','0 0 3 0 0 0')
        node(light,'diffuse',diffuse)
        node(light,'specular','.05 .05 .05 1')
        node(light,'direction',direction)
        node(light,'cast_shadows','true' if name=='key_light' else 'false')

    ground=node(world,'model',name='ground')
    node(ground,'static','true')
    link=node(ground,'link',name='ground_link')
    for kind in ('visual','collision'):
        obj=node(link,kind,name='floor_'+kind)
        plane=node(node(obj,'geometry'),'plane')
        node(plane,'normal','0 0 1')
        node(plane,'size','200 200')
        if kind=='visual':
            material=node(obj,'material')
            node(material,'ambient','.64 .69 .7 1')
            node(material,'diffuse','.64 .69 .7 1')
            node(material,'specular','0 0 0 1')
    pad=node(link,'visual',name='metric_test_pad')
    node(pad,'pose','0 0 .00025 0 0 0')
    node(pad,'cast_shadows','false')
    plane=node(node(pad,'geometry'),'plane')
    node(plane,'normal','0 0 1')
    node(plane,'size','1.6 1.6')
    mat=node(pad,'material')
    node(mat,'ambient','1 1 1 1')
    node(mat,'diffuse','1 1 1 1')
    node(mat,'specular','0 0 0 1')
    metal=node(node(mat,'pbr'),'metal')
    node(metal,'albedo_map','assets/test_pad.png')
    node(metal,'roughness',1)
    node(metal,'metalness',0)
