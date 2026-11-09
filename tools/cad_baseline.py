import pathlib as _pl, sys as _sys
_sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[1] / "cad_src" / "case"))
import robot_paths as RP            # where everything is on this machine (BOT_WS still honoured)
import sys, json
RP.add_sources()
import body_case as bc, wheel_adapter as wa
parts = {"body": bc.build_body(), "shelf": bc.build_shelf(), "front": bc.build_front(), "cover": bc.build_cover(),
         "contact_plate": bc.build_contact_plate(), "tray": bc.build_tray(), "eye_retainer": bc.build_eye_retainer(1),
         "clamp_bar": bc.build_clamp_bar(), "carrier_plate": bc.build_carrier_plate(), "speaker_ring": bc.build_speaker_ring(),
         "mic_clamp": bc.build_mic_clamp(1), "light_retainer": bc.build_light_retainer(1), "diffuser": bc.build_diffuser(),
         "ir_hood": bc.build_ir_hood(), "adapter": wa.build()}
for p in ("tower", "mast", "wing", "ridge", "ridge_insert", "tape", "ir_leds", "board", "rods", "jack", "wires_pos", "wires_neg", "wires_led"):
    parts["dock_" + p] = bc.build_dock(p)
out = {}
for k, v in parts.items():
    s = v.val(); b = s.BoundingBox()
    out[k] = [round(s.Volume(), 2)] + [round(x, 3) for x in (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)]
json.dump(out, open(sys.argv[1], "w"), indent=0)
print(len(out), "parts")
