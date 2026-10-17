"""Optional texture regeneration: uv run --with pillow make_test_pad.py."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
N = 2048
SPAN = 1.6  # metres; must match the plane in environment.py
image = Image.new('RGB', (N, N), '#e9ece9')
draw = ImageDraw.Draw(image)

def px(m):
    return round(m / SPAN * N)

def font(size):
    for name in ('/System/Library/Fonts/Supplemental/Arial.ttf', 'DejaVuSans.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default(size=size)

def text(x, y, value, size, color='#29414b'):
    draw.text((x, y), value, font=font(size), fill=color)

# Pale 10cm squares establish scale without overpowering the small robot.
for i in range(17):
    c = px(i * .1)
    draw.line((c, 0, c, N), fill='#ccd5d2', width=2)
    draw.line((0, c, N, c), fill='#ccd5d2', width=2)
draw.rounded_rectangle((22, 22, N-23, N-23), radius=30,
                       outline='#29414b', width=14)
draw.rectangle((50, 50, 820, 240), fill='#e9ece9')
text(74, 68, 'FREENOVE', 67)
text(78, 154, '4WD  /  TEST PAD', 35)
text(1390, 80, '1.6 m x 1.6 m', 35)
text(1390, 133, 'GRID: 10 cm', 30)

# A labelled home position is a stable reference when the car drives away.
cx=cy=N//2
draw.rounded_rectangle((cx-px(.15),cy-px(.10),cx+px(.15),cy+px(.10)),
                       radius=15, outline='#2e857f', width=6)
text(cx-px(.145),cy+px(.115),'START / HOME',29,'#246c68')
draw.line((cx-px(.022),cy,cx+px(.022),cy),fill='#2e857f',width=3)
draw.line((cx,cy-px(.022),cx,cy+px(.022)),fill='#2e857f',width=3)

# X points forward from the car's camera; Y is left of the car at the start.
ox,oy=cx+px(.23),cy
draw.line((ox,oy,ox+px(.18),oy),fill='#cb6b32',width=12)
draw.polygon([(ox+px(.18),oy),(ox+px(.145),oy-px(.018)),
              (ox+px(.145),oy+px(.018))],fill='#cb6b32')
text(ox,oy+24,'+X / FRONT',30,'#a95426')
ox,oy=cx,cy-px(.23)
draw.line((ox,oy,ox,oy-px(.18)),fill='#2e857f',width=12)
draw.polygon([(ox,oy-px(.18)),(ox-px(.018),oy-px(.145)),
              (ox+px(.018),oy-px(.145))],fill='#2e857f')
text(ox+25,oy-px(.16),'+Y / LEFT',30,'#246c68')

# A short ruler and four differently coloured corner blocks anchor orientation.
left,bottom=90,N-160
for i in range(5):
    draw.rectangle((left+px(i*.1),bottom,left+px((i+1)*.1),bottom+18),
                   fill='#29414b' if i%2==0 else '#a9b7b6')
    text(left+px(i*.1)-8,bottom+30,str(i*10),24)
text(left+px(.5)-12,bottom+30,'50 cm',24)
for x,y,c in [(45,45,'#2e857f'),(N-80,45,'#cb6b32'),
              (45,N-80,'#516687'),(N-80,N-80,'#cfb459')]:
    draw.rectangle((x,y,x+34,y+34),fill=c)
assets=HERE/'assets'
assets.mkdir(exist_ok=True)
image.save(assets/'test_pad.png',optimize=True)
print('Wrote assets/test_pad.png')
