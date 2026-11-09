import sys, json, base64, struct, numpy as np
sys.path.insert(0,'/home/claude/case'); sys.path.insert(0,'/home/claude/wheel_adapter')
import cadquery as cq
exec(open('/home/claude/case/assemble.py').read().split("print(\"== interference")[0])  # reuse part construction
def mesh(shape, tol):
    v,t=shape.tessellate(tol,0.35)
    V=np.array([[p.x,p.y,p.z] for p in v],dtype=np.float64); T=np.array(t,dtype=np.uint32)
    lo=V.min(0); hi=V.max(0); sc=(hi-lo); sc[sc==0]=1
    Q=np.round((V-lo)/sc*65535).astype(np.uint16)
    idx=T.astype(np.uint16 if len(V)<65536 else np.uint32)
    return dict(lo=lo.round(4).tolist(), sc=sc.round(4).tolist(), v=base64.b64encode(Q.tobytes()).decode(),
                i=base64.b64encode(idx.tobytes()).decode(), i32=bool(len(V)>=65536), ntri=len(T))
parts=[]
def add(name,label,shape,color,group,explode,tol=0.25):
    m=mesh(shape,tol); m.update(name=name,label=label,color=color,group=group,ex=explode); parts.append(m)
    print(f"{name:12s} {m['ntri']:7d} tris")
add("body","Body (one print)",body,"#3d7cc9","case",[0,0,0])
add("cover","Back cover",cover,"#5b9be0","case",[-70,0,0])
add("front","Front panel",front,"#5b9be0","case",[70,0,0])
add("shelf","Battery shelf",shelf,"#6fa8e8","case",[-40,0,40])
add("cap","Contact plate",cap,"#5b9be0","case",[-110,0,0])
add("tray","Electronics tray",tray,"#f0a040","case",[0,0,-70])
add("ret_l","Eye retainer",ret_l,"#f0a040","case",[40,0,0])
add("ret_r","Eye retainer",ret_r,"#f0a040","case",[40,0,0])
add("plate_l","Carrier plate",plate_l,"#f0a040","case",[0,60,0])
add("ring","Speaker ring + amp tab",ring,"#f0a040","case",[0,-70,0])
add("micc_l","Mic clamp",micc_l,"#f0a040","case",[50,0,0])
add("micc_r","Mic clamp",micc_r,"#f0a040","case",[50,0,0])
lret_l, lret_r = bc.build_light_retainer(1).val(), bc.build_light_retainer(-1).val()
add("lret_l","Headlight retainer",lret_l,"#f0a040","case",[45,0,0])
add("lret_r","Headlight retainer",lret_r,"#f0a040","case",[45,0,0])
add("irhood","IR receiver hoods (4 x TSOP38238)",bc.ir_hoods_placed().val(),"#2a2a2e","case",[0,0,25])
for s in (1,-1):
    add(f"motor{s:+d}","MF6015 motor",place(motor,s),"#a3a9b0","drive",[0,45*s,0],tol=0.35)
    add(f"adapter{s:+d}","Wheel adapter",place(adapter,s),"#e08a2e","drive",[0,85*s,0])
    add(f"wheel{s:+d}","Wheel (proxy, 108.6 mm)",wheel(s),"#222326","wheels",[0,125*s,0],tol=0.4)
cols={"brass":"#d4a72c","USB-C PD":"#5a5ab0","USB-C panel":"#333333","charge":"#888888","regulator":"#2b6cb0","PDB":"#4caf50","IMU":"#d9534f","receiver coil":"#b87333","wireless":"#6a9a4c","CC/CV":"#4d80c9","6S":"#9a4d9a","LED":"#7fe3ff","battery":"#e3c832","power":"#4caf50","IMU":"#d9534f","sonar":"#35a0a0","Pi":"#2e7d32","Camera":"#7b7be0",
      "perfboard":"#3f6e3f","arm":"#e53935","XT60":"#f2d600","lidar LD19":"#2a2a2a","lidar adapter":"#777777",
      "I2S amp":"#8e44ad","speaker":"#3a3a40","mic":"#c0392b","motor plugs":"#e8e2c8"}
for name,(g,shp) in comps.items():
    c=next(v for k,v in cols.items() if name.startswith(k))
    ex=[0,0,60] if name.startswith("lidar LD19") else ([-40,0,40] if name.split()[0] in ("battery","lidar") and "adapter" in name or name.startswith("battery") else None)
    if ex is None and name.split()[0] in ("regulator","PDB","IMU"): ex=[0,0,-70]
    if ex is None and name.startswith("CC/CV"): ex=[0,60,0]
    if ex is None and name.startswith("wireless"): ex=[0,-60,0]
    if ex is None and name.startswith("brass"): ex=[-110,0,0]
    if ex is None and name.startswith("USB-C PD"): ex=[0,60,0]
    if ex is None and (name.startswith("speaker") or name.startswith("I2S amp")): ex=[0,-70,0]
    if ex is None and name.startswith("mic"): ex=[70,0,0]
    if ex is None and name.startswith("motor plugs"): ex=[0,45 if "left" in name else -45,0]
    if ex is None: ex=[0,0,60] if name.startswith("lidar LD19") else ([-70,0,0] if name.split()[0] in ("perfboard","Pi") else
       ([70,0,0] if name.split()[0] in ("LED","Camera","sonar") else [0,0,0]))
    add("c_"+name.split()[0],name,shp,c,"parts",ex)
# vertical charging dock: the viewer leans the robot back DOCK_LEAN deg about the axle (root at the axle, floor z = -R)
R = bc.WHEEL_R
LIE = bc.DOCK_LEAN
dock = bc.build_dock("foot").val().translate((0, 0, -R))
add("mat","Charging dock: foot (floor plate + lower column)",dock,"#e7e9ee","mat",[0,0,0])
parts[-1]["lie"] = LIE
add("mat_head","Charging dock: head (shelf + face slab + upper column; ring + 2 M3 onto the foot)",bc.build_dock("head").val().translate((0, 0, -R)),"#e7e9ee","mat",[0,0,0])
mast = bc.build_dock("mast").val().translate((0, 0, -R))
add("mat_mast","Charging dock: mast (top of the column, plugs into the head)",mast,"#e7e9ee","mat",[0,0,0])
wl = bc.build_dock("wing").val().translate((0, 0, -R))
add("mat_wl","Charging dock: side wing",wl,"#cfd6e0","mat",[0,0,0])
add("mat_wr","Charging dock: side wing",wl.mirror("XZ"),"#cfd6e0","mat",[0,0,0])
rg = bc.build_dock("ridge").val().translate((0, 0, -R))
add("mat_rl","Charging dock: wheel track frame (PETG)",rg,"#cfd6e0","mat",[0,0,0])
add("mat_rr","Charging dock: wheel track frame (PETG)",rg.mirror("XZ"),"#cfd6e0","mat",[0,0,0])
ri = bc.build_dock("ridge_insert").val().translate((0, 0, -R))
add("mat_il","Charging dock: saddle insert, ribbed TPU (the wheel sits in it)",ri,"#3a3a40","mat",[0,0,0])
add("mat_ir","Charging dock: saddle insert, ribbed TPU (the wheel sits in it)",ri.mirror("XZ"),"#3a3a40","mat",[0,0,0])
add("mat_tape","Charging dock: 2 retro-reflective tape strips (lidar target)",bc.build_dock("tape").val().translate((0, 0, -R)),"#f2f4f8","mat",[0,0,0])
add("mat_board","Dock controller board (ATtiny85: IR beacon + safe rod switch), in the mast's hollow",bc.build_dock("board").val().translate((0, 0, -R)),"#2e7d32","mat",[0,0,0])
add("mat_rods","Charging dock: 2 sprung brass contact rods (Ø4, + = left looking at the face)",bc.build_dock("rods").val().translate((0, 0, -R)),"#d4a52a","mat",[0,0,0])
add("mat_jack","Charging dock: 5.5/2.1 DC jack (25.2 V charger in)",bc.build_dock("jack").val().translate((0, 0, -R)),"#303030","mat",[0,0,0])
add("mat_wpos","Charging dock wiring: + (jack -> controller -> + rod), 22 AWG red",bc.build_dock("wires_pos").val().translate((0, 0, -R)),"#d32f2f","mat",[0,0,0])
add("mat_wneg","Charging dock wiring: - (jack -> controller -> - rod), 22 AWG black",bc.build_dock("wires_neg").val().translate((0, 0, -R)),"#212121","mat",[0,0,0])
add("mat_wled","Charging dock wiring: controller -> 5 IR LEDs, 26 AWG pairs",bc.build_dock("wires_led").val().translate((0, 0, -R)),"#8e24aa","mat",[0,0,0])
add("mat_irled","Charging dock: IR beacon LEDs (4 all round + 1 centre tube, TSAL6200)",bc.build_dock("ir_leds").val().translate((0, 0, -R)),"#c0392b","mat",[0,0,0])
json.dump(parts,open('/home/claude/case/viewer_parts_v11.json','w'))
import os; print(os.path.getsize('/home/claude/case/viewer_parts_v11.json')/1e6,"MB")
