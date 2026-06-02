#!/usr/bin/env python3
# Remote Display — serves the printer's physical display over HTTP
# with touch input support.  Appears as a camera in Mainsail/Fluidd.
#
# Configuration:
#   [remote_display]
#   port: 8092
#   bind: 0.0.0.0
#   framebuffer_device: /dev/fb0
#   touch_device: /dev/input/event0
#   stream_fps: 5

import ctypes
import fcntl
import hashlib
import logging
import mmap
import os
import struct
import threading
import time
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from io import BytesIO
from urllib.parse import urlparse, parse_qs

FBIOGET_VSCREENINFO = 0x4600
FBIOGET_FSCREENINFO = 0x4602

EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03
SYN_REPORT = 0x00
BTN_TOUCH = 0x14A
ABS_X = 0x00
ABS_Y = 0x01
ABS_MT_SLOT = 0x2F
ABS_MT_TRACKING_ID = 0x39
ABS_MT_POSITION_X = 0x35
ABS_MT_POSITION_Y = 0x36

MJPEG_BOUNDARY = b"frameboundary"


class FbVarScreeninfo(ctypes.Structure):
    _fields_ = [
        ("xres", ctypes.c_uint32),
        ("yres", ctypes.c_uint32),
        ("xres_virtual", ctypes.c_uint32),
        ("yres_virtual", ctypes.c_uint32),
        ("xoffset", ctypes.c_uint32),
        ("yoffset", ctypes.c_uint32),
        ("bits_per_pixel", ctypes.c_uint32),
        ("grayscale", ctypes.c_uint32),
        ("red", ctypes.c_uint32 * 3),
        ("green", ctypes.c_uint32 * 3),
        ("blue", ctypes.c_uint32 * 3),
        ("transp", ctypes.c_uint32 * 3),
        ("nonstd", ctypes.c_uint32),
        ("activate", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("width", ctypes.c_uint32),
        ("accel_flags", ctypes.c_uint32),
        ("pixclock", ctypes.c_uint32),
        ("left_margin", ctypes.c_uint32),
        ("right_margin", ctypes.c_uint32),
        ("upper_margin", ctypes.c_uint32),
        ("lower_margin", ctypes.c_uint32),
        ("hsync_len", ctypes.c_uint32),
        ("vsync_len", ctypes.c_uint32),
        ("sync", ctypes.c_uint32),
        ("vmode", ctypes.c_uint32),
        ("rotate", ctypes.c_uint32),
        ("colorspace", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 4),
    ]


class FbFixScreeninfo(ctypes.Structure):
    _fields_ = [
        ("id", ctypes.c_char * 16),
        ("smem_start", ctypes.c_ulong),
        ("smem_len", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("type_aux", ctypes.c_uint32),
        ("visual", ctypes.c_uint32),
        ("xpanstep", ctypes.c_uint16),
        ("ypanstep", ctypes.c_uint16),
        ("ywrapstep", ctypes.c_uint16),
        ("line_length", ctypes.c_uint32),
        ("mmio_start", ctypes.c_ulong),
        ("mmio_len", ctypes.c_uint32),
        ("accel", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint16),
        ("reserved", ctypes.c_uint16 * 2),
    ]


class Framebuffer:
    def __init__(self, device):
        self.device = device
        self.fd = None
        self.mm = None
        self.width = 0
        self.height = 0
        self.bpp = 0
        self.line_length = 0
        self._virtual_height = 0
        self._cache_hash = None
        self._cache_jpeg = None
        self._cache_png = None
        self._lock = threading.Lock()

    def open(self):
        self.fd = os.open(self.device, os.O_RDONLY)
        vinfo = FbVarScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vinfo)
        finfo = FbFixScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_FSCREENINFO, finfo)
        self.width = vinfo.xres
        self.height = vinfo.yres
        self._virtual_height = vinfo.yres_virtual
        self.bpp = vinfo.bits_per_pixel
        self.line_length = finfo.line_length
        size = self.line_length * self._virtual_height
        self.mm = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ)

    def _read_raw(self):
        vinfo = FbVarScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vinfo)
        offset = vinfo.yoffset * self.line_length
        self.mm.seek(offset)
        return self.mm.read(self.line_length * self.height)

    def _raw_to_image(self, raw):
        try:
            from PIL import Image
        except ImportError:
            raise RuntimeError(
                "remote_display requires Pillow. "
                "Install with: pip install Pillow")
        if self.bpp == 32:
            img = Image.frombytes(
                'RGBA', (self.width, self.height), raw,
                'raw', 'BGRA', self.line_length)
            return img.convert('RGB')
        elif self.bpp == 16:
            return Image.frombytes(
                'RGB', (self.width, self.height), raw,
                'raw', 'BGR;16', self.line_length)
        return Image.frombytes(
            'RGB', (self.width, self.height), raw,
            'raw', 'BGR', self.line_length)

    def get_snapshot_png(self, client_etag=None):
        with self._lock:
            raw = self._read_raw()
            raw_hash = hashlib.md5(raw).hexdigest()[:16]
            if client_etag and client_etag == raw_hash:
                return raw_hash, None
            if raw_hash == self._cache_hash and self._cache_png:
                return raw_hash, self._cache_png
            img = self._raw_to_image(raw)
            buf = BytesIO()
            img.save(buf, 'PNG', compress_level=6)
            self._cache_hash = raw_hash
            self._cache_png = buf.getvalue()
            self._cache_jpeg = None
            return raw_hash, self._cache_png

    def get_snapshot_jpeg(self, quality=80):
        with self._lock:
            raw = self._read_raw()
            raw_hash = hashlib.md5(raw).hexdigest()[:16]
            if raw_hash == self._cache_hash and self._cache_jpeg:
                return raw_hash, self._cache_jpeg
            img = self._raw_to_image(raw)
            buf = BytesIO()
            img.save(buf, 'JPEG', quality=quality)
            self._cache_hash = raw_hash
            self._cache_jpeg = buf.getvalue()
            self._cache_png = None
            return raw_hash, self._cache_jpeg

    def close(self):
        if self.mm:
            self.mm.close()
            self.mm = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class TouchInput:
    def __init__(self, device, fb_width, fb_height):
        self.device = device
        self.fd = None
        self.fb_width = fb_width
        self.fb_height = fb_height
        self.touch_max_x = fb_width
        self.touch_max_y = fb_height

    def open(self):
        self.fd = os.open(self.device, os.O_WRONLY)
        self._detect_range()

    def _detect_range(self):
        def eviocgabs(axis):
            return 0x80184540 + axis
        try:
            buf = bytearray(24)
            fcntl.ioctl(self.fd, eviocgabs(ABS_MT_POSITION_X), buf)
            self.touch_max_x = struct.unpack('iiiii', buf[:20])[2]
            fcntl.ioctl(self.fd, eviocgabs(ABS_MT_POSITION_Y), buf)
            self.touch_max_y = struct.unpack('iiiii', buf[:20])[2]
        except OSError:
            try:
                buf = bytearray(24)
                fcntl.ioctl(self.fd, eviocgabs(ABS_X), buf)
                self.touch_max_x = struct.unpack('iiiii', buf[:20])[2]
                fcntl.ioctl(self.fd, eviocgabs(ABS_Y), buf)
                self.touch_max_y = struct.unpack('iiiii', buf[:20])[2]
            except OSError:
                pass

    def _write_event(self, ev_type, code, value):
        if self.fd is None:
            return
        tv_sec = int(time.time())
        tv_usec = int((time.time() % 1) * 1_000_000)
        os.write(self.fd, struct.pack('llHHi', tv_sec, tv_usec,
                                      ev_type, code, value))

    def _scale(self, x, y):
        return (int(x * self.touch_max_x / self.fb_width),
                int(y * self.touch_max_y / self.fb_height))

    def tap(self, x, y):
        if self.fd is None:
            return
        tx, ty = self._scale(x, y)
        self._write_event(EV_ABS, ABS_MT_SLOT, 0)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, 1)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, tx)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, ty)
        self._write_event(EV_KEY, BTN_TOUCH, 1)
        self._write_event(EV_SYN, SYN_REPORT, 0)
        time.sleep(0.05)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, -1)
        self._write_event(EV_KEY, BTN_TOUCH, 0)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_down(self, x, y):
        if self.fd is None:
            return
        tx, ty = self._scale(x, y)
        self._write_event(EV_ABS, ABS_MT_SLOT, 0)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, 1)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, tx)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, ty)
        self._write_event(EV_KEY, BTN_TOUCH, 1)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_move(self, x, y):
        if self.fd is None:
            return
        tx, ty = self._scale(x, y)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, tx)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, ty)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_up(self):
        if self.fd is None:
            return
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, -1)
        self._write_event(EV_KEY, BTN_TOUCH, 0)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


def _make_handler(fb, touch, stream_fps, logger):
    class DisplayHandler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send_cors(self):
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods',
                             'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers',
                             'If-None-Match')

        def do_OPTIONS(self):
            self.send_response(204)
            self._send_cors()
            self.end_headers()

        def do_GET(self):
            path = urlparse(self.path).path.rstrip('/')
            if path in ('/snapshot', '/snapshot.png'):
                self._handle_snapshot_png()
            elif path == '/snapshot.jpg':
                self._handle_snapshot_jpg()
            elif path in ('/stream', '/stream.mjpg'):
                self._handle_mjpeg_stream()
            elif path == '' or path == '/index.html':
                self._handle_viewer()
            else:
                self.send_error(404)

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path.rstrip('/') == '/touch':
                self._handle_touch(parsed.query)
            else:
                self.send_error(404)

        def _handle_snapshot_png(self):
            try:
                client_etag = self.headers.get('If-None-Match', '').strip('"')
                etag, data = fb.get_snapshot_png(client_etag)
                if data is None:
                    self.send_response(304)
                    self.send_header('ETag', f'"{etag}"')
                    self._send_cors()
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Content-Length', len(data))
                self.send_header('ETag', f'"{etag}"')
                self.send_header('Cache-Control', 'no-cache')
                self._send_cors()
                self.end_headers()
                self.wfile.write(data)
            except Exception as e:
                logger.debug(f"Snapshot error: {e}")
                self.send_error(500)

        def _handle_snapshot_jpg(self):
            try:
                _, data = fb.get_snapshot_jpeg()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(data))
                self.send_header('Cache-Control',
                                 'no-cache, no-store, must-revalidate')
                self._send_cors()
                self.end_headers()
                self.wfile.write(data)
            except Exception as e:
                logger.debug(f"JPEG snapshot error: {e}")
                self.send_error(500)

        def _handle_mjpeg_stream(self):
            self.send_response(200)
            self.send_header(
                'Content-Type',
                'multipart/x-mixed-replace; boundary=' +
                MJPEG_BOUNDARY.decode())
            self.send_header('Cache-Control',
                             'no-cache, no-store, must-revalidate')
            self._send_cors()
            self.end_headers()
            interval = 1.0 / max(stream_fps, 1)
            last_hash = None
            try:
                while True:
                    etag, data = fb.get_snapshot_jpeg(quality=70)
                    if data is not None and etag != last_hash:
                        header = (
                            b"--" + MJPEG_BOUNDARY + b"\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Content-Length: " +
                            str(len(data)).encode() + b"\r\n\r\n"
                        )
                        self.wfile.write(header)
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                        last_hash = etag
                    time.sleep(interval)
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:
                logger.debug(f"MJPEG stream ended: {e}")

        def _handle_touch(self, query):
            if touch is None or touch.fd is None:
                resp = b'{"status":"error","message":"touch not available"}'
                self.send_response(503)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', len(resp))
                self._send_cors()
                self.end_headers()
                self.wfile.write(resp)
                return
            try:
                params = parse_qs(query)
                action = params.get('a', ['tap'])[0]
                x = int(params.get('x', [0])[0])
                y = int(params.get('y', [0])[0])
                if action == 'down':
                    touch.touch_down(x, y)
                elif action == 'move':
                    touch.touch_move(x, y)
                elif action == 'up':
                    touch.touch_up()
                else:
                    touch.tap(x, y)
                resp = (f'{{"status":"ok","a":"{action}",'
                        f'"x":{x},"y":{y}}}').encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', len(resp))
                self._send_cors()
                self.end_headers()
                self.wfile.write(resp)
            except Exception as e:
                resp = f'{{"status":"error","message":"{e}"}}'.encode()
                self.send_response(500)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', len(resp))
                self.end_headers()
                self.wfile.write(resp)

        def _handle_viewer(self):
            html = VIEWER_HTML.replace(
                '{{WIDTH}}', str(fb.width)).replace(
                '{{HEIGHT}}', str(fb.height)).replace(
                '{{TOUCH}}', 'true' if touch and touch.fd else 'false')
            data = html.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', len(data))
            self.end_headers()
            self.wfile.write(data)

    return DisplayHandler


VIEWER_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Remote Display</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
html,body{width:100%;height:100%;overflow:hidden;background:#1a1a1a;
  color:#eee;font-family:system-ui,sans-serif}
body{display:flex;flex-direction:column;align-items:center}
#bar{width:100%;padding:6px 16px;background:#222;display:flex;
  align-items:center;justify-content:space-between;font-size:13px;
  border-bottom:1px solid #333;flex-shrink:0}
#bar .title{font-weight:600;font-size:15px}
#bar .status{color:#888}
#bar .status.connected{color:#4caf50}
#bar .status.error{color:#f44336}
body.iframe #bar{display:none}
#wrap{flex:1;display:flex;align-items:center;justify-content:center;
  width:100%;padding:8px;overflow:hidden}
body.iframe #wrap{padding:0}
#screen{cursor:crosshair;max-width:100%;max-height:100%;
  image-rendering:auto;border:1px solid #333;display:block}
body.iframe #screen{border:none;width:100%;height:100%;
  object-fit:contain}
#touch-indicator{position:fixed;width:24px;height:24px;border-radius:50%;
  background:rgba(255,100,100,0.6);border:2px solid rgba(255,100,100,0.9);
  pointer-events:none;transform:translate(-50%,-50%);display:none;z-index:99}
</style>
</head>
<body>
<div id="bar">
  <span class="title">Remote Display</span>
  <span id="info">{{WIDTH}}&times;{{HEIGHT}}</span>
  <span id="status" class="status">connecting&hellip;</span>
</div>
<div id="wrap">
  <img id="screen" draggable="false">
</div>
<div id="touch-indicator"></div>
<script>
(function(){
  var inIframe = false;
  try { inIframe = window.self !== window.top; } catch(e) { inIframe = true; }
  if (inIframe) document.body.classList.add('iframe');

  var base = '';
  var loc = window.location.pathname;
  if (loc && loc.length > 1) {
    base = loc.replace(/\/[^\/]*$/, '') + '/';
    if (base === '/') base = '';
  }

  const img = document.getElementById('screen');
  const status = document.getElementById('status');
  const indicator = document.getElementById('touch-indicator');
  const TOUCH = {{TOUCH}};
  const FB_W = parseInt('{{WIDTH}}') || 1024;
  const FB_H = parseInt('{{HEIGHT}}') || 600;
  let etag = '';
  let errorCount = 0;
  let dragging = false;

  if (!TOUCH) img.style.cursor = 'default';

  function setStatus(text, cls) {
    status.textContent = text;
    status.className = 'status ' + (cls || '');
  }

  function poll() {
    const headers = {};
    if (etag) headers['If-None-Match'] = '"' + etag + '"';
    fetch(base + 'snapshot', { headers })
      .then(resp => {
        if (resp.status === 304) {
          errorCount = 0;
          setStatus('live', 'connected');
          setTimeout(poll, 200);
          return;
        }
        if (!resp.ok) throw new Error(resp.status);
        const newEtag = (resp.headers.get('ETag') || '').replace(/"/g, '');
        if (newEtag) etag = newEtag;
        return resp.blob();
      })
      .then(blob => {
        if (!blob) return;
        const url = URL.createObjectURL(blob);
        const prev = img.src;
        img.src = url;
        if (prev && prev.startsWith('blob:')) URL.revokeObjectURL(prev);
        errorCount = 0;
        setStatus('live', 'connected');
        setTimeout(poll, 100);
      })
      .catch(e => {
        errorCount++;
        setStatus('disconnected', 'error');
        setTimeout(poll, Math.min(errorCount * 500, 5000));
      });
  }

  function imgCoords(e) {
    const rect = img.getBoundingClientRect();
    const scaleX = FB_W / rect.width;
    const scaleY = FB_H / rect.height;
    return {
      x: Math.round((e.clientX - rect.left) * scaleX),
      y: Math.round((e.clientY - rect.top) * scaleY)
    };
  }

  function sendTouch(action, x, y) {
    if (!TOUCH) return;
    fetch(base + 'touch?a=' + action + '&x=' + x + '&y=' + y,
      { method: 'POST' }).catch(() => {});
  }

  function showIndicator(clientX, clientY) {
    indicator.style.left = clientX + 'px';
    indicator.style.top = clientY + 'px';
    indicator.style.display = 'block';
  }

  function hideIndicator() {
    indicator.style.display = 'none';
  }

  img.addEventListener('mousedown', function(e) {
    if (!TOUCH) return;
    e.preventDefault();
    const c = imgCoords(e);
    dragging = true;
    sendTouch('down', c.x, c.y);
    showIndicator(e.clientX, e.clientY);
  });

  img.addEventListener('mousemove', function(e) {
    if (!dragging) return;
    const c = imgCoords(e);
    sendTouch('move', c.x, c.y);
    showIndicator(e.clientX, e.clientY);
  });

  window.addEventListener('mouseup', function(e) {
    if (!dragging) return;
    dragging = false;
    sendTouch('up', 0, 0);
    hideIndicator();
  });

  img.addEventListener('touchstart', function(e) {
    if (!TOUCH) return;
    e.preventDefault();
    const t = e.touches[0];
    const c = imgCoords(t);
    dragging = true;
    sendTouch('down', c.x, c.y);
  }, { passive: false });

  img.addEventListener('touchmove', function(e) {
    e.preventDefault();
    if (!dragging) return;
    const t = e.touches[0];
    const c = imgCoords(t);
    sendTouch('move', c.x, c.y);
  }, { passive: false });

  img.addEventListener('touchend', function(e) {
    e.preventDefault();
    if (!dragging) return;
    dragging = false;
    sendTouch('up', 0, 0);
  }, { passive: false });

  poll();
})();
</script>
</body>
</html>
"""


class RemoteDisplay:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.logger = logging.getLogger('remote_display')
        self.port = config.getint('port', 8092)
        self.bind = config.get('bind', '0.0.0.0')
        self.fb_device = config.get('framebuffer_device', '/dev/fb0')
        self.touch_device = config.get('touch_device', '/dev/input/event0')
        self.stream_fps = config.getint('stream_fps', 5)
        self._server = None
        self._thread = None
        self._fb = None
        self._touch = None
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'REMOTE_DISPLAY_STATUS', self.cmd_STATUS,
            desc="Show remote display server status")
        gcode.register_command(
            'REMOTE_DISPLAY_TAP', self.cmd_TAP,
            desc="Simulate a tap on the printer display")
        self.printer.register_event_handler(
            "klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)

    def _handle_ready(self):
        try:
            self._fb = Framebuffer(self.fb_device)
            self._fb.open()
            self.logger.info(
                "Framebuffer: %dx%d @ %dbpp (%s)",
                self._fb.width, self._fb.height,
                self._fb.bpp, self.fb_device)
        except Exception as e:
            self.logger.error("Failed to open framebuffer %s: %s",
                              self.fb_device, e)
            return
        try:
            self._touch = TouchInput(
                self.touch_device, self._fb.width, self._fb.height)
            self._touch.open()
            self.logger.info(
                "Touch input: %s (range %dx%d)",
                self.touch_device,
                self._touch.touch_max_x, self._touch.touch_max_y)
        except Exception as e:
            self.logger.warning(
                "Touch input unavailable (%s): %s — "
                "display will be view-only", self.touch_device, e)
            self._touch = None
        handler_cls = _make_handler(
            self._fb, self._touch, self.stream_fps, self.logger)
        try:
            self._server = ThreadingHTTPServer(
                (self.bind, self.port), handler_cls)
            self._server.daemon_threads = True
            self._thread = threading.Thread(
                target=self._server.serve_forever, daemon=True)
            self._thread.start()
            self.logger.info(
                "Remote display server on http://%s:%d",
                self.bind, self.port)
            self.logger.info(
                "  MJPEG camera: http://<host>:%d/stream.mjpg",
                self.port)
            self.logger.info(
                "  Iframe camera (with touch): "
                "http://<host>:%d/", self.port)
            self.logger.info(
                "  Snapshot: http://<host>:%d/snapshot",
                self.port)
        except Exception as e:
            self.logger.error("Failed to start HTTP server: %s", e)

    def _handle_disconnect(self):
        if self._server:
            self._server.shutdown()
            self._server = None
        if self._fb:
            self._fb.close()
            self._fb = None
        if self._touch:
            self._touch.close()
            self._touch = None

    def cmd_STATUS(self, gcmd):
        if self._server and self._fb:
            gcmd.respond_info(
                f"Remote display: http://{self.bind}:{self.port}\n"
                f"Framebuffer: {self._fb.width}x{self._fb.height} "
                f"@ {self._fb.bpp}bpp ({self.fb_device})\n"
                f"Touch: {'available' if self._touch and self._touch.fd else 'unavailable'}\n"
                f"MJPEG stream: /stream.mjpg ({self.stream_fps} fps)\n"
                f"Snapshot: /snapshot.jpg or /snapshot.png\n"
                f"Viewer: /")
        else:
            gcmd.respond_info("Remote display: not running")

    def cmd_TAP(self, gcmd):
        if not self._touch or self._touch.fd is None:
            gcmd.respond_info("Touch input not available")
            return
        x = gcmd.get_int('X', None)
        y = gcmd.get_int('Y', None)
        if x is None or y is None:
            gcmd.respond_info("Usage: REMOTE_DISPLAY_TAP X=<x> Y=<y>")
            return
        self._touch.tap(x, y)
        gcmd.respond_info(f"Tapped at ({x}, {y})")

    def get_status(self, eventtime):
        running = self._server is not None
        return {
            'running': running,
            'port': self.port,
            'width': self._fb.width if self._fb else 0,
            'height': self._fb.height if self._fb else 0,
            'touch_available': (self._touch is not None
                                and self._touch.fd is not None),
        }


def load_config(config):
    return RemoteDisplay(config)
