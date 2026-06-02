#!/usr/bin/env python3

import argparse
import ctypes
import fcntl
import hashlib
import mmap
import os
import struct
import subprocess
import time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from io import BytesIO
from urllib.parse import urlparse, parse_qs

from PIL import Image

FBIOGET_VSCREENINFO = 0x4600
FBIOGET_FSCREENINFO = 0x4602

# ── DRM/KMS constants ──────────────────────────────────────────────

def _iowr(type_val, nr, size):
    return (3 << 30) | (size << 16) | (type_val << 8) | nr

def _iow(type_val, nr, size):
    return (1 << 30) | (size << 16) | (type_val << 8) | nr

_DRM_IOCTL_MAP_DUMB = _iowr(0x64, 0xB3, 16)
_DRM_IOCTL_GEM_CLOSE = _iow(0x64, 0x09, 8)


class _DrmMapDumb(ctypes.Structure):
    _fields_ = [('handle', ctypes.c_uint32), ('pad', ctypes.c_uint32),
                ('offset', ctypes.c_uint64)]


class _DrmGemClose(ctypes.Structure):
    _fields_ = [('handle', ctypes.c_uint32), ('pad', ctypes.c_uint32)]


class _DrmModeModeInfo(ctypes.Structure):
    _fields_ = [
        ('clock', ctypes.c_uint32),
        ('hdisplay', ctypes.c_uint16), ('hsync_start', ctypes.c_uint16),
        ('hsync_end', ctypes.c_uint16), ('htotal', ctypes.c_uint16),
        ('hskew', ctypes.c_uint16), ('vdisplay', ctypes.c_uint16),
        ('vsync_start', ctypes.c_uint16), ('vsync_end', ctypes.c_uint16),
        ('vtotal', ctypes.c_uint16), ('vscan', ctypes.c_uint16),
        ('vrefresh', ctypes.c_uint32), ('flags', ctypes.c_uint32),
        ('type', ctypes.c_uint32), ('name', ctypes.c_char * 32)]


class _DrmModeRes(ctypes.Structure):
    _fields_ = [
        ('count_fbs', ctypes.c_int),
        ('fbs', ctypes.POINTER(ctypes.c_uint32)),
        ('count_crtcs', ctypes.c_int),
        ('crtcs', ctypes.POINTER(ctypes.c_uint32)),
        ('count_connectors', ctypes.c_int),
        ('connectors', ctypes.POINTER(ctypes.c_uint32)),
        ('count_encoders', ctypes.c_int),
        ('encoders', ctypes.POINTER(ctypes.c_uint32)),
        ('min_width', ctypes.c_uint32), ('max_width', ctypes.c_uint32),
        ('min_height', ctypes.c_uint32), ('max_height', ctypes.c_uint32)]


class _DrmModeCrtc(ctypes.Structure):
    _fields_ = [
        ('crtc_id', ctypes.c_uint32), ('buffer_id', ctypes.c_uint32),
        ('x', ctypes.c_uint32), ('y', ctypes.c_uint32),
        ('width', ctypes.c_uint32), ('height', ctypes.c_uint32),
        ('mode_valid', ctypes.c_int), ('mode', _DrmModeModeInfo),
        ('gamma_size', ctypes.c_int)]


class _DrmModeFB(ctypes.Structure):
    _fields_ = [
        ('fb_id', ctypes.c_uint32),
        ('width', ctypes.c_uint32), ('height', ctypes.c_uint32),
        ('pitch', ctypes.c_uint32), ('bpp', ctypes.c_uint32),
        ('depth', ctypes.c_uint32), ('handle', ctypes.c_uint32)]

# ── fbdev structures ───────────────────────────────────────────────

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

# ── Input constants ────────────────────────────────────────────────

EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03
SYN_REPORT = 0x00
BTN_TOUCH = 0x14a
ABS_X = 0x00
ABS_Y = 0x01
ABS_MT_SLOT = 0x2f
ABS_MT_TRACKING_ID = 0x39
ABS_MT_POSITION_X = 0x35
ABS_MT_POSITION_Y = 0x36

def log(msg):
    ts = time.strftime('%H:%M:%S')
    print(f"[{ts}] {msg}", flush=True)

# ── Display backends ───────────────────────────────────────────────

class Framebuffer:
    def __init__(self, device='/dev/fb0'):
        self.device = device
        self.fd = None
        self.mm = None
        self.width = 0
        self.height = 0
        self.virtual_width = 0
        self.virtual_height = 0
        self.bpp = 0
        self.line_length = 0
        self._cache_hash = None
        self._cache_png = None
        self._open()

    def _open(self):
        self.fd = os.open(self.device, os.O_RDONLY)
        vinfo = FbVarScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vinfo)
        finfo = FbFixScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_FSCREENINFO, finfo)

        self.width = vinfo.xres
        self.virtual_width = vinfo.xres_virtual
        self.height = vinfo.yres
        self.virtual_height = vinfo.yres_virtual
        self.bpp = vinfo.bits_per_pixel
        self.line_length = finfo.line_length

        size = self.line_length * self.virtual_height
        self.mm = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ)
        log(f"fbdev: {self.width}x{self.height} @ {self.bpp}bpp ({self.device})")

    def _read_raw(self):
        vinfo = FbVarScreeninfo()
        fcntl.ioctl(self.fd, FBIOGET_VSCREENINFO, vinfo)
        offset = vinfo.yoffset * self.line_length
        self.mm.seek(offset)
        return self.mm.read(self.line_length * self.height)

    def _raw_to_image(self, raw):
        if self.bpp == 32:
            img = Image.frombytes('RGBA', (self.width, self.height), raw,
                                  'raw', 'BGRA', self.line_length)
            return img.convert('RGB')
        elif self.bpp == 16:
            return Image.frombytes('RGB', (self.width, self.height), raw,
                                   'raw', 'BGR;16', self.line_length)
        return Image.frombytes('RGB', (self.width, self.height), raw,
                               'raw', 'BGR', self.line_length)

    def get_snapshot(self, client_etag=None):
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
        return raw_hash, self._cache_png

    def close(self):
        if self.mm:
            self.mm.close()
        if self.fd:
            os.close(self.fd)


class DRMFramebuffer:
    def __init__(self, device=None):
        self.device = device
        self.fd = None
        self.width = 0
        self.height = 0
        self.bpp = 0
        self.line_length = 0
        self._crtc_id = None
        self._fb_cache = {}
        self._drm = None
        self._cache_hash = None
        self._cache_png = None
        self._open()

    def _open(self):
        try:
            self._drm = ctypes.CDLL('libdrm.so.2')
        except OSError:
            raise RuntimeError(
                "libdrm not found — install with: sudo apt install libdrm2")
        self._setup_libdrm()
        if self.device:
            self._try_open(self.device)
        else:
            for i in range(8):
                path = f'/dev/dri/card{i}'
                if not os.path.exists(path):
                    continue
                try:
                    self._try_open(path)
                    self.device = path
                    log(f"DRM: {self.width}x{self.height} @ {self.bpp}bpp ({path})")
                    return
                except Exception:
                    if self.fd is not None:
                        os.close(self.fd)
                        self.fd = None
            raise RuntimeError("No active DRM display found")
        log(f"DRM: {self.width}x{self.height} @ {self.bpp}bpp ({self.device})")

    def _setup_libdrm(self):
        d = self._drm
        d.drmModeGetResources.restype = ctypes.POINTER(_DrmModeRes)
        d.drmModeGetResources.argtypes = [ctypes.c_int]
        d.drmModeFreeResources.argtypes = [ctypes.c_void_p]
        d.drmModeGetCrtc.restype = ctypes.POINTER(_DrmModeCrtc)
        d.drmModeGetCrtc.argtypes = [ctypes.c_int, ctypes.c_uint32]
        d.drmModeFreeCrtc.argtypes = [ctypes.c_void_p]
        d.drmModeGetFB.restype = ctypes.POINTER(_DrmModeFB)
        d.drmModeGetFB.argtypes = [ctypes.c_int, ctypes.c_uint32]
        d.drmModeFreeFB.argtypes = [ctypes.c_void_p]

    def _try_open(self, path):
        self.fd = os.open(path, os.O_RDWR)
        res = self._drm.drmModeGetResources(self.fd)
        if not res:
            raise RuntimeError(f"drmModeGetResources failed on {path}")
        try:
            for i in range(res.contents.count_crtcs):
                crtc_id = res.contents.crtcs[i]
                crtc = self._drm.drmModeGetCrtc(self.fd, crtc_id)
                if not crtc:
                    continue
                try:
                    if crtc.contents.buffer_id and crtc.contents.mode_valid:
                        self._crtc_id = crtc_id
                        self.width = crtc.contents.width
                        self.height = crtc.contents.height
                        fb = self._drm.drmModeGetFB(
                            self.fd, crtc.contents.buffer_id)
                        if fb:
                            self.bpp = fb.contents.bpp
                            self.line_length = fb.contents.pitch
                            if not fb.contents.handle:
                                self._drm.drmModeFreeFB(fb)
                                raise RuntimeError(
                                    "DRM buffer handle not accessible. "
                                    "Run with CAP_SYS_ADMIN or as root.")
                            self._close_gem(fb.contents.handle)
                            self._drm.drmModeFreeFB(fb)
                        return
                finally:
                    self._drm.drmModeFreeCrtc(crtc)
        finally:
            self._drm.drmModeFreeResources(res)
        raise RuntimeError(f"No active CRTC on {path}")

    def _get_fb_buffer(self):
        crtc = self._drm.drmModeGetCrtc(self.fd, self._crtc_id)
        if not crtc:
            raise RuntimeError("Failed to get CRTC")
        try:
            fb_id = crtc.contents.buffer_id
        finally:
            self._drm.drmModeFreeCrtc(crtc)
        if not fb_id:
            raise RuntimeError("No framebuffer on CRTC")
        if fb_id in self._fb_cache:
            entry = self._fb_cache[fb_id]
            self.line_length = entry[1]
            self.bpp = entry[2]
            return entry[0], entry[3]
        fb = self._drm.drmModeGetFB(self.fd, fb_id)
        if not fb:
            raise RuntimeError("Failed to get FB")
        try:
            handle = fb.contents.handle
            pitch = fb.contents.pitch
            h = fb.contents.height
            bpp = fb.contents.bpp
            size = pitch * h
            if not handle:
                raise RuntimeError("DRM buffer handle=0 (no permission)")
            req = _DrmMapDumb()
            req.handle = handle
            req.pad = 0
            req.offset = 0
            fcntl.ioctl(self.fd, _DRM_IOCTL_MAP_DUMB, req)
            mm = mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ,
                           offset=req.offset)
            self._fb_cache[fb_id] = (mm, pitch, bpp, size, handle)
            self.line_length = pitch
            self.bpp = bpp
            if len(self._fb_cache) > 4:
                old_id = next(iter(self._fb_cache))
                old = self._fb_cache.pop(old_id)
                try:
                    old[0].close()
                except Exception:
                    pass
                self._close_gem(old[4])
            return mm, size
        finally:
            self._drm.drmModeFreeFB(fb)

    def _close_gem(self, handle):
        try:
            req = _DrmGemClose()
            req.handle = handle
            req.pad = 0
            fcntl.ioctl(self.fd, _DRM_IOCTL_GEM_CLOSE, req)
        except Exception:
            pass

    def _read_raw(self):
        mm, size = self._get_fb_buffer()
        mm.seek(0)
        return mm.read(size)

    def _raw_to_image(self, raw):
        if self.bpp == 32:
            img = Image.frombytes('RGBA', (self.width, self.height), raw,
                                  'raw', 'BGRA', self.line_length)
            return img.convert('RGB')
        elif self.bpp == 16:
            return Image.frombytes('RGB', (self.width, self.height), raw,
                                   'raw', 'BGR;16', self.line_length)
        return Image.frombytes('RGB', (self.width, self.height), raw,
                               'raw', 'BGR', self.line_length)

    def get_snapshot(self, client_etag=None):
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
        return raw_hash, self._cache_png

    def close(self):
        for entry in self._fb_cache.values():
            try:
                entry[0].close()
            except Exception:
                pass
            self._close_gem(entry[4])
        self._fb_cache.clear()
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

# ── Touch input ────────────────────────────────────────────────────

class TouchInput:
    def __init__(self, device='/dev/input/event0', fb_width=1024, fb_height=600):
        self.device = device
        self.fd = None
        self.fb_width = fb_width
        self.fb_height = fb_height
        self.touch_max_x = 1024
        self.touch_max_y = 600
        self._open()

    def _open(self):
        try:
            self.fd = os.open(self.device, os.O_WRONLY)
            self._get_abs_info()
            log(f"Touch device: {self.device}, range: {self.touch_max_x}x{self.touch_max_y}")
        except OSError as e:
            log(f"Failed to open touch device: {e}")
            self.fd = None

    def _get_abs_info(self):
        EVIOCGABS = lambda axis: 0x80184540 + axis
        try:
            buf = bytearray(24)
            fcntl.ioctl(self.fd, EVIOCGABS(ABS_MT_POSITION_X), buf)
            self.touch_max_x = struct.unpack('iiiii', buf[:20])[2]
            fcntl.ioctl(self.fd, EVIOCGABS(ABS_MT_POSITION_Y), buf)
            self.touch_max_y = struct.unpack('iiiii', buf[:20])[2]
        except OSError:
            try:
                fcntl.ioctl(self.fd, EVIOCGABS(ABS_X), buf)
                self.touch_max_x = struct.unpack('iiiii', buf[:20])[2]
                fcntl.ioctl(self.fd, EVIOCGABS(ABS_Y), buf)
                self.touch_max_y = struct.unpack('iiiii', buf[:20])[2]
            except OSError:
                pass

    def _write_event(self, ev_type, code, value):
        if self.fd is None:
            return
        tv_sec = int(time.time())
        tv_usec = int((time.time() % 1) * 1000000)
        event = struct.pack('llHHi', tv_sec, tv_usec, ev_type, code, value)
        os.write(self.fd, event)

    def _scale(self, x, y):
        touch_x = int(x * self.touch_max_x / self.fb_width)
        touch_y = int(y * self.touch_max_y / self.fb_height)
        return touch_x, touch_y

    def tap(self, x, y):
        if self.fd is None:
            log(f"Touch device not available, would tap at ({x}, {y})")
            return
        touch_x, touch_y = self._scale(x, y)
        log(f"Tap at ({x}, {y}) -> touch ({touch_x}, {touch_y})")
        self._write_event(EV_ABS, ABS_MT_SLOT, 0)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, 1)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, touch_x)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, touch_y)
        self._write_event(EV_KEY, BTN_TOUCH, 1)
        self._write_event(EV_SYN, SYN_REPORT, 0)
        time.sleep(0.05)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, -1)
        self._write_event(EV_KEY, BTN_TOUCH, 0)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_down(self, x, y):
        if self.fd is None:
            return
        touch_x, touch_y = self._scale(x, y)
        log(f"Touch down at ({x}, {y})")
        self._write_event(EV_ABS, ABS_MT_SLOT, 0)
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, 1)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, touch_x)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, touch_y)
        self._write_event(EV_KEY, BTN_TOUCH, 1)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_move(self, x, y):
        if self.fd is None:
            return
        touch_x, touch_y = self._scale(x, y)
        self._write_event(EV_ABS, ABS_MT_POSITION_X, touch_x)
        self._write_event(EV_ABS, ABS_MT_POSITION_Y, touch_y)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def touch_up(self):
        if self.fd is None:
            return
        log(f"Touch up")
        self._write_event(EV_ABS, ABS_MT_TRACKING_ID, -1)
        self._write_event(EV_KEY, BTN_TOUCH, 0)
        self._write_event(EV_SYN, SYN_REPORT, 0)

    def close(self):
        if self.fd:
            os.close(self.fd)

# ── HTTP handler ───────────────────────────────────────────────────

class ScreenHandler(SimpleHTTPRequestHandler):
    framebuffer = None
    touch_input = None
    html_dir = None

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=self.html_dir, **kwargs)

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/snapshot':
            self.handle_snapshot()
        else:
            super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/touch':
            self.handle_touch(parsed.query)
        else:
            self.send_error(404, 'Not Found')

    def handle_snapshot(self):
        try:
            client_etag = self.headers.get('If-None-Match', '').strip('"')
            etag, png_data = self.framebuffer.get_snapshot(client_etag)
            if png_data is None:
                self.send_response(304)
                self.send_header('ETag', f'"{etag}"')
                self.end_headers()
                return
            log(f"Snapshot: {len(png_data)} bytes, etag={etag}")
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', len(png_data))
            self.send_header('ETag', f'"{etag}"')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(png_data)
        except Exception as e:
            log(f"Snapshot error: {e}")
            self.send_error(500, str(e))

    def handle_touch(self, query):
        try:
            params = parse_qs(query)
            action = params.get('a', ['tap'])[0]
            x = int(params.get('x', [0])[0])
            y = int(params.get('y', [0])[0])
            if action == 'down':
                self.touch_input.touch_down(x, y)
            elif action == 'move':
                self.touch_input.touch_move(x, y)
            elif action == 'up':
                self.touch_input.touch_up()
            else:
                self.touch_input.tap(x, y)
            response = f'{{"status":"ok","a":"{action}","x":{x},"y":{y}}}'.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(response))
            self.end_headers()
            self.wfile.write(response)
        except Exception as e:
            log(f"Touch error: {e}")
            response = f'{{"status":"error","message":"{e}"}}'.encode()
            self.send_response(500)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(response))
            self.end_headers()
            self.wfile.write(response)

# ── Main ───────────────────────────────────────────────────────────

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    local_html_dir = os.path.join(script_dir, 'html')
    installed_html_dir = '/usr/share/fb-http/html'

    if os.path.isdir(local_html_dir):
        default_html_dir = local_html_dir
    else:
        default_html_dir = installed_html_dir

    parser = argparse.ArgumentParser(description='Framebuffer HTTP Server')
    parser.add_argument('-p', '--port', type=int, default=8092, help='HTTP port')
    parser.add_argument('--bind', default='0.0.0.0', help='Bind address')
    parser.add_argument('--backend', default='auto', choices=['auto', 'drm', 'fbdev'],
                        help='Display backend (default: auto)')
    parser.add_argument('--drm-device', default=None,
                        help='DRM device path (e.g. /dev/dri/card1)')
    parser.add_argument('--fb', default='/dev/fb0', help='Framebuffer device (fbdev mode)')
    parser.add_argument('--touch', default='/dev/input/event0', help='Touch input device')
    parser.add_argument('--html-dir', default=default_html_dir, help='Path to HTML directory')
    args = parser.parse_args()

    fb = None
    backend = args.backend

    if backend == 'auto':
        try:
            result = subprocess.run(['pgrep', '-x', 'helixscreen'],
                                    capture_output=True, timeout=2)
            if result.returncode == 0:
                log("helixscreen detected, using DRM backend")
                backend = 'drm'
            else:
                backend = 'fbdev'
        except Exception:
            backend = 'fbdev'

    if backend == 'drm':
        fb = DRMFramebuffer(args.drm_device)

    if fb is None:
        fb = Framebuffer(args.fb)

    touch = TouchInput(args.touch, fb.width, fb.height)

    ScreenHandler.framebuffer = fb
    ScreenHandler.touch_input = touch
    ScreenHandler.html_dir = os.fspath(args.html_dir)

    server = ThreadingHTTPServer((args.bind, args.port), ScreenHandler)
    log(f"Server running on http://{args.bind}:{args.port}")
    log(f"  HTML directory: {args.html_dir}")
    log(f"  GET  /           - HTML viewer")
    log(f"  GET  /snapshot   - PNG snapshot")
    log(f"  POST /touch?x=&y= - Send touch event")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("Shutting down...")
    finally:
        fb.close()
        touch.close()

if __name__ == '__main__':
    main()
