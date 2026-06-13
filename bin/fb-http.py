#!/usr/bin/env python3

import argparse
import ctypes
import fcntl
import glob
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


# drmModeGetFB2 — reads framebuffers created with drmModeAddFB2 (modern GPU
# compositors like helixscreen), which legacy drmModeGetFB can't (it returns
# bpp=0 / NULL). Layout matches libdrm's drmModeFB2 (uint64 modifier is
# 8-byte-aligned right after the four leading uint32s, so no padding).
class _DrmModeFB2(ctypes.Structure):
    _fields_ = [
        ('fb_id', ctypes.c_uint32),
        ('width', ctypes.c_uint32), ('height', ctypes.c_uint32),
        ('pixel_format', ctypes.c_uint32),   # fourcc
        ('modifier', ctypes.c_uint64),
        ('flags', ctypes.c_uint32),
        ('handles', ctypes.c_uint32 * 4),
        ('pitches', ctypes.c_uint32 * 4),
        ('offsets', ctypes.c_uint32 * 4)]


def _fourcc(s):
    return ord(s[0]) | (ord(s[1]) << 8) | (ord(s[2]) << 16) | (ord(s[3]) << 24)

# DRM fourcc -> bits per pixel (only the formats a display scans out).
_DRM_FORMAT_BPP = {}
for _f in ('RG16', 'BG16'):
    _DRM_FORMAT_BPP[_fourcc(_f)] = 16
for _f in ('XR24', 'AR24', 'XB24', 'AB24', 'RX24', 'RA24', 'BX24', 'BA24'):
    _DRM_FORMAT_BPP[_fourcc(_f)] = 32

_DRM_FORMAT_MOD_LINEAR = 0
_DRM_FORMAT_MOD_INVALID = (1 << 56) - 1  # 0x00ffffffffffffff

# DRM_FORMAT_* fourcc -> PIL 'raw' decoder mode (channel order, RRGGBB read).
_DRM_FORMAT_RAWMODE = {
    _fourcc('RG16'): ('RGB', 'BGR;16'),   # RGB565
    _fourcc('BG16'): ('RGB', 'RGB;16'),
    _fourcc('XR24'): ('RGBA', 'BGRA'),    # XRGB8888
    _fourcc('AR24'): ('RGBA', 'BGRA'),    # ARGB8888
    _fourcc('XB24'): ('RGBA', 'RGBA'),    # XBGR8888
    _fourcc('AB24'): ('RGBA', 'RGBA'),    # ABGR8888
}

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

# ── Touchscreen auto-detection (self-healing device resolution) ──────
# Find the touchscreen by what it IS (evdev capabilities), not a hardcoded
# eventN that can renumber across reboots/driver loads.
INPUT_PROP_DIRECT = 0x01

def _eviocgbit(ev_type, length):
    return (2 << 30) | (length << 16) | (0x45 << 8) | (0x20 + ev_type)

def _eviocgprop(length):
    return (2 << 30) | (length << 16) | (0x45 << 8) | 0x09

def _eviocgname(length):
    return (2 << 30) | (length << 16) | (0x45 << 8) | 0x06

def _test_bit(bitmap, bit):
    idx = bit // 8
    return idx < len(bitmap) and bool(bitmap[idx] & (1 << (bit % 8)))

def _device_is_touchscreen(path):
    """Return (is_touchscreen, name_str) by querying evdev capabilities."""
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError:
        return False, None
    try:
        name = None
        try:
            buf = bytearray(256)
            fcntl.ioctl(fd, _eviocgname(len(buf)), buf)
            name = bytes(buf).split(b'\x00', 1)[0].decode('utf-8', 'replace')
        except OSError:
            pass
        abs_bits = bytearray(8)
        try:
            fcntl.ioctl(fd, _eviocgbit(EV_ABS, len(abs_bits)), abs_bits)
        except OSError:
            return False, name
        has_xy = (_test_bit(abs_bits, ABS_MT_POSITION_X)
                  or (_test_bit(abs_bits, ABS_X) and _test_bit(abs_bits, ABS_Y)))
        if not has_xy:
            return False, name
        key_bits = bytearray(96)
        has_touch = False
        try:
            fcntl.ioctl(fd, _eviocgbit(EV_KEY, len(key_bits)), key_bits)
            has_touch = _test_bit(key_bits, BTN_TOUCH)
        except OSError:
            pass
        prop_bits = bytearray(4)
        is_direct = False
        try:
            fcntl.ioctl(fd, _eviocgprop(len(prop_bits)), prop_bits)
            is_direct = _test_bit(prop_bits, INPUT_PROP_DIRECT)
        except OSError:
            pass
        return (has_touch or is_direct), name
    finally:
        os.close(fd)

def _prefer_by_path(dev):
    """Return a stable /dev/input/by-path symlink for dev if one exists."""
    bp_dir = '/dev/input/by-path'
    try:
        target = os.path.realpath(dev)
        for link in sorted(os.listdir(bp_dir)):
            full = os.path.join(bp_dir, link)
            try:
                if os.path.realpath(full) == target:
                    return full
            except OSError:
                pass
    except OSError:
        pass
    return dev

def _find_touchscreen():
    for dev in sorted(glob.glob('/dev/input/event*')):
        ok, name = _device_is_touchscreen(dev)
        if ok:
            return _prefer_by_path(dev), name
    return None, None

def resolve_touch_device(configured):
    """A configured path that really is a touchscreen wins (mapped to its
    stable by-path symlink); otherwise auto-detect, so a moved eventN or wrong
    --touch corrects itself."""
    if configured and os.path.exists(configured):
        ok, _name = _device_is_touchscreen(configured)
        if ok:
            return _prefer_by_path(configured)
        log(f"Configured touch device {configured} is not a touchscreen — auto-detecting")
    elif configured:
        log(f"Configured touch device {configured} not found — auto-detecting")
    found, name = _find_touchscreen()
    if found:
        log(f"Auto-detected touchscreen: {found}" + (f" ({name})" if name else ""))
        return found
    log(f"No touchscreen auto-detected; falling back to {configured!r}")
    return configured

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
        self._fmt = 0          # DRM fourcc of the current FB (0 = unknown)
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
        # Optional (newer libdrm): drmModeGetFB2 for addfb2/GPU buffers, and
        # PRIME export to map GPU buffers that aren't dumb-mappable.
        if hasattr(d, 'drmModeGetFB2'):
            d.drmModeGetFB2.restype = ctypes.POINTER(_DrmModeFB2)
            d.drmModeGetFB2.argtypes = [ctypes.c_int, ctypes.c_uint32]
            d.drmModeFreeFB2.argtypes = [ctypes.c_void_p]
        if hasattr(d, 'drmPrimeHandleToFD'):
            d.drmPrimeHandleToFD.restype = ctypes.c_int
            d.drmPrimeHandleToFD.argtypes = [
                ctypes.c_int, ctypes.c_uint32, ctypes.c_uint32,
                ctypes.POINTER(ctypes.c_int)]

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
                        info = self._fb_info(crtc.contents.buffer_id)
                        if info is None:
                            raise RuntimeError(
                                "DRM framebuffer not readable (need libdrm "
                                "GetFB2 and/or run as root/CAP_SYS_ADMIN)")
                        handle, pitch, bpp, _h, fmt = info
                        if not handle:
                            raise RuntimeError(
                                "DRM buffer handle not accessible. "
                                "Run with CAP_SYS_ADMIN or as root.")
                        self.bpp = bpp
                        self.line_length = pitch
                        self._fmt = fmt
                        self._close_gem(handle)
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
            self._fmt = entry[5]
            return entry[0], entry[3]
        info = self._fb_info(fb_id)
        if info is None:
            raise RuntimeError("Failed to get FB")
        handle, pitch, bpp, h, fmt = info
        if not handle:
            raise RuntimeError("DRM buffer handle=0 (no permission)")
        size = pitch * h
        mm = self._map_handle(handle, size)
        if mm is None:
            self._close_gem(handle)
            raise RuntimeError("Failed to map DRM buffer (dumb + PRIME failed)")
        self._fb_cache[fb_id] = (mm, pitch, bpp, size, handle, fmt)
        self.line_length = pitch
        self.bpp = bpp
        self._fmt = fmt
        if len(self._fb_cache) > 4:
            old_id = next(iter(self._fb_cache))
            old = self._fb_cache.pop(old_id)
            try:
                old[0].close()
            except Exception:
                pass
            self._close_gem(old[4])
        return mm, size

    def _fb_info(self, fb_id):
        """Return (handle, pitch, bpp, height, fourcc) for an FB id, or None.

        Prefer drmModeGetFB2 — it reads framebuffers made with drmModeAddFB2
        (GPU compositors), which legacy drmModeGetFB can't (bpp=0/NULL). Fall
        back to legacy GetFB for plain dumb buffers."""
        getfb2 = getattr(self._drm, 'drmModeGetFB2', None)
        if getfb2 is not None:
            fb2 = getfb2(self.fd, fb_id)
            if fb2:
                try:
                    handle = fb2.contents.handles[0]
                    pitch = fb2.contents.pitches[0]
                    fmt = fb2.contents.pixel_format
                    mod = fb2.contents.modifier
                    height = fb2.contents.height
                    bpp = _DRM_FORMAT_BPP.get(fmt, 0)
                    if handle and pitch and bpp:
                        if mod not in (_DRM_FORMAT_MOD_LINEAR,
                                       _DRM_FORMAT_MOD_INVALID):
                            log("DRM: FB %d non-linear modifier 0x%x — image "
                                "may be garbled" % (fb_id, mod))
                        return handle, pitch, bpp, height, fmt
                finally:
                    self._drm.drmModeFreeFB2(fb2)
        fb = self._drm.drmModeGetFB(self.fd, fb_id)
        if fb:
            try:
                handle = fb.contents.handle
                pitch = fb.contents.pitch
                bpp = fb.contents.bpp
                height = fb.contents.height
                if handle and pitch and bpp:
                    return handle, pitch, bpp, height, 0
            finally:
                self._drm.drmModeFreeFB(fb)
        return None

    def _map_handle(self, handle, size):
        """mmap a GEM handle for CPU read. Try MAP_DUMB (dumb buffers); on
        failure fall back to PRIME export + dma-buf mmap (GPU buffers)."""
        try:
            req = _DrmMapDumb()
            req.handle = handle
            req.pad = 0
            req.offset = 0
            fcntl.ioctl(self.fd, _DRM_IOCTL_MAP_DUMB, req)
            return mmap.mmap(self.fd, size, mmap.MAP_SHARED, mmap.PROT_READ,
                             offset=req.offset)
        except OSError:
            pass
        prime = getattr(self._drm, 'drmPrimeHandleToFD', None)
        if prime is not None:
            prime_fd = ctypes.c_int(-1)
            # DRM_CLOEXEC (0x80000) | DRM_RDWR (0x8) so the dma-buf is mappable.
            ret = prime(self.fd, handle, 0x80000 | 0x8, ctypes.byref(prime_fd))
            if ret == 0 and prime_fd.value >= 0:
                try:
                    return mmap.mmap(prime_fd.value, size, mmap.MAP_SHARED,
                                     mmap.PROT_READ)
                except OSError as e:
                    log("DRM: PRIME mmap failed: %s" % e)
                finally:
                    os.close(prime_fd.value)
        return None

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
        # Prefer the exact DRM pixel format (correct channel order) when known;
        # otherwise fall back to a bpp-based guess.
        rawmode = _DRM_FORMAT_RAWMODE.get(self._fmt)
        if rawmode is not None:
            mode, decoder = rawmode
            img = Image.frombytes(mode, (self.width, self.height), raw,
                                  'raw', decoder, self.line_length)
            return img.convert('RGB') if mode != 'RGB' else img
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
            last_err = None
            for flags in (os.O_RDWR, os.O_WRONLY):
                try:
                    self.fd = os.open(self.device, flags)
                    last_err = None
                    break
                except OSError as e:
                    last_err = e
            if last_err is not None:
                raise last_err
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
    parser.add_argument('--touch', default='', help='Touch input device (blank = auto-detect the touchscreen)')
    parser.add_argument('--html-dir', default=default_html_dir, help='Path to HTML directory')
    args = parser.parse_args()

    fb = None
    backend = args.backend

    if backend == 'auto':
        # The U1's on-screen UI (the native Snapmaker UI, or helixscreen if it's
        # running) renders through DRM/KMS, which leaves fbdev (/dev/fb0) black.
        # So prefer DRM whenever there's an active DRM display — don't gate on a
        # specific process name (pgrep 'helix-screen' missed the native UI and
        # any differently-named or not-yet-started helixscreen, falling back to
        # a black fbdev). Fall back to fbdev only when no usable DRM CRTC found.
        try:
            fb = DRMFramebuffer(args.drm_device)
            log("Auto: active DRM display found, using DRM backend")
            backend = 'drm'
        except Exception as e:
            log(f"Auto: no usable DRM display ({e}); using fbdev")
            fb = None
            backend = 'fbdev'

    if backend == 'drm' and fb is None:
        # Explicit --backend drm: still fall back to fbdev on failure rather
        # than crashing the server (which leaves Mainsail's box empty).
        try:
            fb = DRMFramebuffer(args.drm_device)
        except Exception as e:
            log(f"DRM backend failed ({e}); falling back to fbdev")
            fb = None

    if fb is None:
        fb = Framebuffer(args.fb)

    touch = TouchInput(resolve_touch_device(args.touch), fb.width, fb.height)

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
