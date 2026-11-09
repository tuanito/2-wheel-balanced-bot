import os as _os; WS = (_os.environ.get("BOT_WS") or ("/home/claude" if _os.path.isdir("/home/claude/case") else _os.path.expanduser("~/bot-ws"))).rstrip("/") + "/"   # workspace root (tools/make_workspace.sh)
import os; os.environ["MUJOCO_GL"]="osmesa"
import mujoco, numpy as np, pathlib
from PIL import Image, ImageDraw, ImageFont
D=pathlib.Path(WS + "case/assembly")
parts=[l.split("|") for l in open(D/"parts.txt").read().splitlines()]
def xml(skip=()):
    a="".join(f'<mesh name="{k}" file="{D}/{k}.stl" scale="0.001 0.001 0.001"/>' for k,_ in parts if k not in skip)
    g="".join(f'<geom type="mesh" mesh="{k}" rgba="{c}"/>' for k,c in parts if k not in skip)
    return f'''<mujoco><asset><texture type="skybox" builtin="flat" rgb1="1 1 1" rgb2="1 1 1" width="16" height="16"/>{a}</asset><visual><global offwidth="1200" offheight="1000"/><headlight ambient="0.45 0.45 0.45" diffuse="0.55 0.55 0.55"/></visual>
    <worldbody><light pos="0.3 -0.3 0.6" dir="-0.4 0.4 -1"/><light pos="-0.3 0.3 0.4" dir="0.4 -0.4 -0.6" diffuse="0.3 0.3 0.3"/>
    <geom type="plane" size="0.5 0.5 0.01" pos="0 0 -0.0543" rgba="0.93 0.93 0.93 1"/>{g}</worldbody></mujoco>'''
def shot(skip,az,el,dist,label,look=(0,0,0.04)):
    m=mujoco.MjModel.from_xml_string(xml(skip)); d=mujoco.MjData(m); mujoco.mj_forward(m,d)
    r=mujoco.Renderer(m,900,1000); cam=mujoco.MjvCamera(); cam.lookat[:]=look; cam.azimuth=az; cam.elevation=el; cam.distance=dist
    r.update_scene(d,cam); im=Image.fromarray(r.render()); ImageDraw.Draw(im).text((20,20),label,fill=(20,20,20),font=ImageFont.load_default(size=30)); return im
ims=[shot((),-150,-15,0.55,"Front: camera, sonar, lidar on top"),
     shot(("cover",),30,-20,0.55,"Back, cover removed: perfboard on standoffs, battery below"),
     shot(("cover","c_perfboardESP32CANboard","wheel+1","wheel-1","c_lidarLD19"),35,-30,0.42,"Inside: perfboard, wheels, lidar hidden",look=(0,0,0.03))]
W=sum(i.width for i in ims); out=Image.new("RGB",(W,1000),"white"); x=0
for i in ims: out.paste(i,(x,0)); x+=i.width
out.save(WS + "case/case_preview.png"); print(out.size)
