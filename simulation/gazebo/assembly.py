"""Photo-informed visual parts shared by Gazebo and Blender. All values in metres.

X points toward the camera/front, Y left, Z up. Small components are approximate.
"""
import math

COLORS = {
    'pcb': (.018, .024, .03), 'green': (.025, .26, .10),
    'blue': (.025, .11, .32), 'rubber': (.016, .019, .022),
    'yellow': (.95, .55, .008), 'silver': (.52, .56, .59),
    'gold': (.56, .34, .07), 'chip': (.012, .014, .018),
    'orange': (.78, .28, .025), 'white': (.82, .84, .79),
    'red': (.55, .018, .012), 'lens': (.02, .045, .065),
}

def assembly(p):
    inch = .0254
    L=p['chassis_length_in']*inch
    W=p['chassis_width_in']*inch
    R=p['wheel_diameter_in']*inch/2
    tire=(p['overall_width_in']*inch-W)/2
    track=p['overall_width_in']*inch-tire
    wb=p['overall_length_in']*inch-2*R
    board=p['main_board_height_in']*inch
    thick=p['board_thickness_mm_assumed']/1000
    motorbottom=p['motor_bottom_clearance_in']*inch
    groups={'chassis': {'origin': (0,0,0), 'parts': []}}

    def part(group,name,shape,pos,size,color,rot=(0,0,0),collision=False):
        groups[group]['parts'].append(dict(name=name,shape=shape,pos=pos,size=size,
                                         color=color,rot=rot,collision=collision))
    def box(name,pos,size,color,rot=(0,0,0),collision=False):
        part('chassis',name,'box',pos,size,color,rot,collision)
    def cyl(name,pos,r,h,color,rot=(0,0,0),collision=False):
        part('chassis',name,'cylinder',pos,(r,h),color,rot,collision)

    box('main_board',(0,0,board-thick/2),(L,W,thick),'pcb',collision=True)
    # Four TT gearmotors: yellow gearbox and inboard silver can on each side.
    for end,x in [('front',wb/2),('rear',-wb/2)]:
        direction=-1 if end=='front' else 1
        for side,s in [('left',1),('right',-1)]:
            tag=end+'_'+side
            y=s*(W/2-.011)
            box(tag+'_gearbox',(x,y,motorbottom+.0085),(.025,.020,.017),'yellow',collision=True)
            cyl(tag+'_motor',(x+direction*.022,y,motorbottom+.00635),.00635,.022,'silver',(0,math.pi/2,0),True)
            cyl(tag+'_endcap',(x+direction*.034,y,motorbottom+.00635),.0058,.003,'chip',(0,math.pi/2,0))
            box(tag+'_bracket',(x,y,board-.003),(.012,.023,.003),'silver')
            box(tag+'_strap',(x+direction*.015,y,board-.006),(.002,.021,.011),'white')
            cyl(tag+'_axle',(x,s*(W/2+.001),R),.002,.009,'silver',(math.pi/2,0,0))
            group=tag+'_wheel'
            groups[group]={'origin':(x,s*track/2,R),'parts':[]}
            # Low-radius core plus outer tread blocks leave a visible tire pattern.
            part(group,'tire','cylinder',(0,0,0),(R-.0008,tire),'rubber',(math.pi/2,0,0),True)
            for a in range(44):
                theta=2*math.pi*a/44
                for row in range(4):
                    t=theta+(row%2)*.024
                    part(group,f'tread_{a}_{row}','box',
                         ((R-.0006)*math.sin(t),(row-1.5)*tire/4,(R-.0006)*math.cos(t)),
                         (.0037,tire/4-.00065,.0012),'rubber',(0,t,0))
            # Yellow inset rim and spokes on both faces, just inside the tire width.
            for face in (-1,1):
                yy=face*(tire/2-.0005)
                part(group,f'rim_{face}','cylinder',(0,yy,0),(.0238,.001),'yellow',(math.pi/2,0,0))
                part(group,f'inset_{face}','cylinder',(0,yy+face*.00055,0),(.0208,.0005),'rubber',(math.pi/2,0,0))
                for j in range(6):
                    t=j*math.pi/3
                    part(group,f'spoke_{face}_{j}','box',(.012*math.sin(t),yy+face*.001,.012*math.cos(t)),
                         (.005,.002,.020),'yellow',(0,t+.12,0))
                part(group,f'hub_{face}','cylinder',(0,yy+face*.001,0),(.007,.003),'yellow',(math.pi/2,0,0))

    # Two-cell holder visible underneath, with open slots and end springs.
    box('battery_holder_base',(-.002,0,.0365),(.066,.036,.002),'chip',collision=True)
    for y in (-.018,.018):
        box('battery_side_'+str(y),(-.002,y,.029),(.066,.002,.014),'chip')
    box('battery_divider',(-.002,0,.029),(.066,.0015,.014),'chip')
    for x in (-.035,.031):
        box('battery_end_'+str(x),(x,0,.029),(.002,.036,.014),'chip')
        for y in (-.009,.009):
            cyl('spring_'+str((x,y)),(x,y,.029),.004,.003,'silver',(0,math.pi/2,0))

    # Raised Raspberry Pi near the rear; positions estimated from top and side views.
    pi_z=board+.016
    for x in (-.082,-.020):
        for y in (-.025,.025):
            cyl('standoff_'+str((x,y)),(x,y,board+.007),.002,.014,'gold')
            cyl('pi_screw_'+str((x,y)),(x,y,pi_z+.0013),.0025,.0015,'silver')
    box('raspberry_pi',(-.048,0,pi_z),(.085,.056,.0016),'green',collision=True)
    box('pi_processor',(-.045,-.002,pi_z+.0025),(.015,.015,.003),'chip')
    for i,y in enumerate((-.018,.004)):
        box('usb_stack_'+str(i),(-.082,y,pi_z+.008),(.018,.016,.015),'silver')
        box('usb_opening_'+str(i),(-.0913,y,pi_z+.008),(.0008,.012,.010),'chip')
    box('ethernet',(-.08,.024,pi_z+.008),(.021,.016,.015),'silver')
    for x in (-.059,-.040):
        box('hdmi_'+str(x),(x,-.026,pi_z+.003),(.009,.007,.005),'silver')
    box('gpio_header',(-.041,.022,pi_z+.005),(.050,.005,.008),'chip')
    # Arduino Nano on tall pin headers over the Pi.
    nano_z=pi_z+.022
    box('nano_board',(-.054,.003,nano_z),(.043,.018,.0016),'blue',collision=True)
    box('nano_radio',(-.047,.002,nano_z+.0025),(.012,.010,.003),'silver')
    box('nano_usb',(-.075,.003,nano_z+.003),(.006,.008,.004),'silver')
    for x in range(12):
        for y in (-.005,.011):
            box('nano_pin_'+str((x,y)),(-.072+x*.003,y,nano_z-.009),(.0006,.0006,.018),'gold')
    # Servo, white horn and taped mount shown at the front.
    box('front_servo',(.052,0,board+.009),(.023,.013,.017),'chip',collision=True)
    cyl('servo_pivot',(.052,0,board+.019),.003,.005,'white')
    box('servo_horn',(.052,0,board+.022),(.004,.022,.002),'white')
    box('camera_mount',(.092,0,.031),(.005,.034,.022),'chip')
    # Camera faces +X, mounted below the leading board edge.
    box('camera_pcb',(.099,0,.030),(.0016,.025,.024),'green',collision=True)
    cyl('camera_lens_barrel',(.105,0,.028),.008,.011,'chip',(0,math.pi/2,0),True)
    cyl('camera_glass',(.111,0,.028),.0055,.0007,'lens',(0,math.pi/2,0))
    for y in (-.010,.010):
        cyl('camera_screw_'+str(y),(.1005,y,.039),.002,.002,'silver',(0,math.pi/2,0))
    # Broad flexible orange ribbon draped over the board, with a short raised bend.
    for name,pos,size,rot in [
        ('ribbon_rear',(-.025,-.003,pi_z+.004),(.030,.015,.0003),(0,.25,0)),
        ('ribbon_middle',(.010,-.003,board+.012),(.048,.015,.0003),(0,.22,0)),
        ('ribbon_front',(.055,-.003,board+.006),(.044,.015,.0003),(0,-.04,0)),
        ('ribbon_fold',(.082,-.003,board+.003),(.014,.015,.0003),(0,.5,0))]:
        box(name,pos,size,'orange',rot)
    box('black_mount_tape',(.074,-.003,board+.005),(.020,.022,.0006),'chip')
    cyl('buzzer',(.071,-.030,board+.0055),.006,.011,'chip')
    cyl('buzzer_label',(.071,-.030,board+.0112),.0053,.0003,'white')
    for x in (-.026,-.011):
        box('switch_base_'+str(x),(x,-.037,board+.003),(.009,.008,.005),'white')
        box('switch_blue_'+str(x),(x,-.037,board+.006),(.004,.004,.002),'blue')
    for x in (-.082,-.048,-.014,.025,.080):
        for y in (-.039,.039):
            box('led_'+str((x,y)),(x,y,board+.001),(.0035,.0035,.0015),'white')
    for x in (-.085,.078):
        for y in (-.033,.033):
            cyl('board_screw_'+str((x,y)),(x,y,board+.001),.0025,.002,'silver')
    for i in range(12):
        box('front_gpio_'+str(i),(-.003+i*.0025,.029,board+.004),(.001,.002,.007),'gold')
    return groups
