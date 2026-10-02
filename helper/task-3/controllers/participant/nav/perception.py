"""Поиск цветных напольных маркеров в кадре фронтальной камеры."""
import numpy as np

YELLOW_RGB = (0.95, 0.85, 0.1)
PURPLE_RGB = (0.6, 0.2, 0.9)


# Маркер светится: emissiveIntensity 3.0 умножает цвет, сверху добавляется общий свет
# сцены. Замером по кадру камеры: жёлтый (242,217,26) приходит как (253,252,153).
# Сравнивать надо с тем, что реально рендерится, иначе промах по тёмному каналу.
GLOW = 3.0
AMBIENT = 77


def _rgb255(rgb):
    return tuple(min(255, int(round(c * 255 * GLOW + AMBIENT))) for c in rgb)


def _frame_bgr(robot):
    w, h, fov = robot.camera_info()
    raw = robot.image()
    if not raw or w <= 0 or h <= 0:
        return None, w, h, fov
    arr = np.frombuffer(raw, dtype=np.uint8)
    if arr.size < w * h * 4:
        return None, w, h, fov
    arr = arr[: w * h * 4].reshape((h, w, 4))
    return arr[:, :, :3], w, h, fov


def detect(robot, target_rgb, tol=45, min_frac=0.0008):
    """-> (найден, пеленг в системе робота, доля кадра)."""
    bgr, w, h, fov = _frame_bgr(robot)
    if bgr is None or w <= 0:
        return False, 0.0, 0.0
    tr, tg, tb = _rgb255(target_rgb)
    b = bgr[:, :, 0].astype(np.int16)
    g = bgr[:, :, 1].astype(np.int16)
    r = bgr[:, :, 2].astype(np.int16)
    mask = (np.abs(r - tr) <= tol) & (np.abs(g - tg) <= tol) & (np.abs(b - tb) <= tol)

    # у маркера тёмный канал заметно ниже светящихся; без этой проверки белые стены,
    # у которых все каналы одинаково яркие, попадают в допуск вместе с ним
    spread = max(tr, tg, tb) - min(tr, tg, tb)
    if spread > 30:
        hi = np.maximum(np.maximum(r, g), b)
        lo = np.minimum(np.minimum(r, g), b)
        mask &= (hi - lo) >= spread // 2

    n = int(mask.sum())
    area_frac = n / float(w * h)
    if area_frac < min_frac:
        return False, 0.0, 0.0

    ys, xs = np.nonzero(mask)
    cx = float(xs.mean())
    bearing = (w / 2.0 - cx) / (w / 2.0) * (fov / 2.0)
    return True, bearing, area_frac
