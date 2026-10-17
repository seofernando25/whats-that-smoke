"""TT form-factor geometry: estimated subpart dimensions fitted to the user's car.

The 70x22x18mm reference envelope guides proportions. Mounting heights and
output wheel axes follow the user's measurements; this is not a vendor CAD model.
"""
import math

def add_tt_motor(parts, tag, x, y, direction, side, bottom, wheel_z):
    def add(name, shape, pos, size, color, rot=(0,0,0), collision=False):
        parts.append(dict(name=tag+'_'+name,shape=shape,pos=pos,size=size,
                          color=color,rot=rot,collision=collision))
    def box(name, pos, size, color, rot=(0,0,0), collision=False):
        add(name,'box',pos,size,color,rot,collision)
    def cyl(name, pos, radius, length, color, rot=(0,0,0), collision=False):
        add(name,'cylinder',pos,(radius,length),color,rot,collision)
    # Two moulded shell halves, raised gear cover, edge seam and rounded end cap.
    center=x+direction*.007
    z=bottom+.0085
    for half in (-1,1):
        box('gearbox_half_'+str(half),(center,y+half*.00555,z),(.036,.0109,.017),
            'yellow',collision=True)
    box('case_seam',(center,y,z),(.0363,.0005,.0172),'yellow_dark')
    box('raised_gear_cover',(x+direction*.009,y,z+.0078),(.025,.020,.0023),'yellow')
    for s in (-1,1):
        cyl('output_boss_'+str(s),(x,y+s*.0114,wheel_z),.0052,.0013,'yellow',(math.pi/2,0,0))
        # Dual output shafts are vertical-flat plastic tabs, represented as D-like
        # flat plastic tabs, instead of a metal rod.
        box('shaft_flat_'+str(s),(x,y+s*.015,wheel_z),(.0054,.007,.0036),'white')
        for t in (-1,1):
            sx=x+direction*(.017+t*.004)
            cyl('screw_'+str((s,t)),(sx,y+s*.0117,z+.001),.00165,.0009,'silver',(math.pi/2,0,0))
            box('screw_slot_'+str((s,t)),(sx,y+s*.0122,z+.001),(.0023,.0003,.00045),'chip')
    # 130-size flattened can: metal sides, rounded corners and black brush cap.
    can_x=x+direction*.038
    can_z=bottom+.007
    box('motor_can',(can_x,y,can_z),(.024,.0084,.014),'silver',collision=True)
    for s in (-1,1):
        cyl('can_round_'+str(s),(can_x,y+s*.0042,can_z),.0068,.024,'silver',(0,math.pi/2,0))
    box('brush_cap',(x+direction*.051,y,can_z),(.004,.018,.0125),'chip')
    cyl('rear_bearing',(x+direction*.0535,y,can_z),.0038,.0015,'silver',(0,math.pi/2,0))
    # Side retention plates, tabs and two electrical terminals.
    for s in (-1,1):
        box('retaining_clip_'+str(s),(x+direction*.036,y+s*.0112,can_z),(.029,.0007,.006),'silver')
        box('clip_tab_'+str(s),(x+direction*.022,y+s*.0128,can_z),(.004,.004,.006),'silver')
        box('terminal_'+str(s),(x+direction*.054,y+s*.005,can_z-.004),(.003,.002,.0006),'gold')
    box('red_lead',(x+direction*.048,y-.009,can_z-.005),(.012,.0011,.0011),'red',(0,0,.12))
    box('black_lead',(x+direction*.048,y+.009,can_z-.005),(.012,.0011,.0011),'chip',(0,0,-.12))
    # Small mounting ear beyond the gearbox outer end.
    box('mounting_ear',(x-direction*.020,y,z),(.006,.010,.003),'yellow')
