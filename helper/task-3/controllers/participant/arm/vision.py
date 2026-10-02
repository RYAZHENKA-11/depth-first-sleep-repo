"""Детекция цветного пятна и глубины в кадре запястной камеры."""
import numpy as np


def _frame_bgr(w, h, raw):
    if not raw or w <= 0 or h <= 0:
        return None
    arr = np.frombuffer(raw, dtype=np.uint8)
    if arr.size < w * h * 4:
        return None
    return arr[: w * h * 4].reshape((h, w, 4))[:, :, :3]


def detect_blob(w, h, raw, target_rgb, tol=45, min_frac=0.004):
    """-> (найден, u, v, доля кадра); u,v в [-1,1] от центра кадра."""
    bgr = _frame_bgr(w, h, raw)
    if bgr is None:
        return False, 0.0, 0.0, 0.0
    tr, tg, tb = (int(round(c * 255)) for c in target_rgb)
    b = bgr[:, :, 0].astype(np.int16)
    g = bgr[:, :, 1].astype(np.int16)
    r = bgr[:, :, 2].astype(np.int16)
    mask = (np.abs(r - tr) <= tol) & (np.abs(g - tg) <= tol) & (np.abs(b - tb) <= tol)

    n = int(mask.sum())
    area_frac = n / float(w * h)
    if area_frac < min_frac:
        return False, 0.0, 0.0, 0.0

    ys, xs = np.nonzero(mask)
    cx, cy = float(xs.mean()), float(ys.mean())
    u = (cx - w / 2.0) / (w / 2.0)
    v = (cy - h / 2.0) / (h / 2.0)
    return True, u, v, area_frac


def sample_min_depth_region(w, h, raw, half_extent=0.4):
    """Минимум глубины в центре кадра: RGB и depth разнесены, точный пиксель не совпадает,
    а пол вокруг объекта всегда дальше самого объекта.
    """
    if not raw or w <= 0 or h <= 0:
        return None
    arr = np.frombuffer(raw, dtype="<f4")
    if arr.size < w * h:
        return None
    arr = arr[: w * h].reshape((h, w))
    x0 = int(round((1.0 - half_extent) / 2.0 * w))
    x1 = w - x0
    y0 = int(round((1.0 - half_extent) / 2.0 * h))
    y1 = h - y0
    window = arr[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
    valid = window[np.isfinite(window) & (window > 0.0)]
    if valid.size == 0:
        return None
    return float(np.min(valid))
