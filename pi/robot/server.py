"""Web app: live video, joystick, buttons, live map + lidar/sonar obstacles. Open http://<robot>.local:8080 on the phone."""
import asyncio, io, json, math, pathlib, threading, time
import numpy as np
from aiohttp import web

STATIC = pathlib.Path(__file__).parent / "static"


def make_app(brain, camera):
    app = web.Application()

    async def index(_):
        return web.FileResponse(STATIC / "index.html")

    async def stream(request):
        resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame",
                                           "Cache-Control": "no-cache"})
        await resp.prepare(request)
        count = -1
        loop = asyncio.get_running_loop()
        try:
            while True:
                frame, count = await loop.run_in_executor(None, camera.output.wait, count)
                if frame is None: continue
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                 str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n")
        except (ConnectionResetError, asyncio.CancelledError):
            pass
        return resp

    async def map_png(_):
        from PIL import Image, ImageDraw
        g = brain.mapper.grid
        L = g.L
        img = (255 * (1 - 1 / (1 + np.exp(-L)))).astype(np.uint8)[::-1]
        im = Image.fromarray(img).convert("RGB")
        d = ImageDraw.Draw(im)
        tr = brain.mapper.trajectory[-3000:]
        if len(tr) > 1:
            c = [((p[0] - g.origin) / g.res, g.n - 1 - (p[1] - g.origin) / g.res) for p in tr]
            d.line(c, fill=(220, 60, 60), width=1)
        x, y, th = brain.mapper.pose
        cx, cy = (x - g.origin) / g.res, g.n - 1 - (y - g.origin) / g.res
        d.ellipse([cx - 3, cy - 3, cx + 3, cy + 3], fill=(0, 120, 255))
        d.line([cx, cy, cx + 8 * math.cos(th), cy - 8 * math.sin(th)], fill=(0, 120, 255), width=2)
        b = io.BytesIO(); im.save(b, "PNG")
        return web.Response(body=b.getvalue(), content_type="image/png", headers={"Cache-Control": "no-store"})

    async def map_info(_):
        g = brain.mapper.grid
        return web.json_response(dict(origin=g.origin, res=g.res, n=g.n))

    async def ws(request):
        sock = web.WebSocketResponse(heartbeat=5)
        await sock.prepare(request)

        async def pusher():
            while not sock.closed:
                st = brain.state()
                pts = brain.scan_pts[::3]
                st["scan"] = [[round(float(a), 2), round(float(b), 2)] for a, b in pts]
                await sock.send_str(json.dumps(st))
                await asyncio.sleep(0.1)

        task = asyncio.create_task(pusher())
        try:
            async for msg in sock:
                if msg.type != web.WSMsgType.TEXT: continue
                m = json.loads(msg.data)
                if m.get("type") == "joy": brain.joystick(m.get("v", 0), m.get("w", 0))
                elif m.get("type") == "talk" and brain.voice: brain.voice.push_to_talk()
                elif m.get("type") == "ask" and brain.voice and m.get("text"):
                    threading.Thread(target=brain.voice.handle_text, args=(str(m["text"])[:500],), daemon=True).start()
                elif m.get("type") == "cmd":
                    name = m.pop("name"); m.pop("type", None); brain.command(name, **m)
        finally:
            task.cancel()
        return sock

    app.router.add_get("/", index)
    app.router.add_get("/stream.mjpg", stream)
    app.router.add_get("/map.png", map_png)
    app.router.add_get("/map.json", map_info)
    async def people(request):
        """POST multipart: field 'name' + one or more image files -> enrolled for face ID."""
        if brain.vision is None: return web.json_response(dict(ok=False, error="no vision"), status=503)
        rd = await request.multipart(); name, imgs = None, []
        async for part in rd:
            if part.name == "name": name = (await part.text()).strip()
            elif part.filename: imgs.append(await part.read(decode=False))
        if not name or not name.replace(" ", "").replace("-", "").isalnum() or not imgs:
            return web.json_response(dict(ok=False, error="need a name (letters/digits) and at least one photo"), status=400)
        n = brain.vision.enroll(name, imgs, brain.data / "people")
        return web.json_response(dict(ok=True, name=name, faces_found=n, photos=len(imgs)))

    app.router.add_post("/api/people", people)
    app.router.add_get("/ws", ws)
    app.router.add_static("/static", STATIC)
    return app
