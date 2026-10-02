import math
import os

TRUNK_HEIGHT = 0.33
THIGH_ANGLE = 0.8
CALF_ANGLE = -1.5
LEG_LENGTH = 0.213

LEGS = {
    "FL": ((0.1934, 0.0465), 1, (0.0, 0.0, 0.0)),
    "FR": ((0.1934, -0.0465), -1, (math.pi, 0.0, 0.0)),
    "RL": ((-0.1934, 0.0465), 1, (0.0, math.pi, 0.0)),
    "RR": ((-0.1934, -0.0465), -1, (0.0, 0.0, math.pi)),
}


def _mesh(mesh_dir, name, rpy=(0.0, 0.0, 0.0), fallback=""):
    if mesh_dir:
        uri = "file://" + os.path.join(mesh_dir, name)
        geometry = f'<mesh filename="{uri}"/>'
    else:
        geometry = fallback
    r, p, y = rpy
    return (f'<visual><origin xyz="0 0 0" rpy="{r} {p} {y}"/><geometry>{geometry}</geometry>'
            f'<material name="go2"><color rgba="0.75 0.75 0.78 1"/></material></visual>')


def _link(name, visual=""):
    return f'<link name="{name}">{visual}</link>'


def _joint(name, parent, child, xyz, rpy=(0.0, 0.0, 0.0)):
    return (f'<joint name="{name}" type="fixed"><parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/></joint>')


def build_urdf(mesh_dir=None):
    if mesh_dir and not os.path.isfile(os.path.join(mesh_dir, "base.dae")):
        mesh_dir = None
    parts = ['<?xml version="1.0"?>', '<robot name="go2">', _link("base_link")]
    parts.append(_link("trunk", _mesh(mesh_dir, "base.dae",
                                      fallback='<box size="0.38 0.094 0.114"/>')))
    parts.append(_joint("trunk_joint", "base_link", "trunk", (0.0, 0.0, TRUNK_HEIGHT)))
    parts.append(_link("lidar_link"))
    parts.append(_joint("lidar_joint", "trunk", "lidar_link", (0.2, 0.0, 0.16)))

    leg_box = f'<box size="0.04 0.04 {LEG_LENGTH}"/>'
    for leg, ((hx, hy), side, hip_rpy) in LEGS.items():
        mirror = "_mirror" if side < 0 else ""
        hip, thigh, calf, foot = (f"{leg}_{n}" for n in ("hip", "thigh", "calf", "foot"))
        parts.append(_link(hip, _mesh(mesh_dir, "hip.dae", hip_rpy,
                                      fallback='<cylinder radius="0.046" length="0.04"/>')))
        parts.append(_joint(f"{leg}_hip_joint", "trunk", hip, (hx, hy, 0.0)))
        parts.append(_link(thigh, _mesh(mesh_dir, f"thigh{mirror}.dae", fallback=leg_box)))
        parts.append(_joint(f"{leg}_thigh_joint", hip, thigh, (0.0, side * 0.0955, 0.0),
                            (0.0, THIGH_ANGLE, 0.0)))
        parts.append(_link(calf, _mesh(mesh_dir, f"calf{mirror}.dae", fallback=leg_box)))
        parts.append(_joint(f"{leg}_calf_joint", thigh, calf, (0.0, 0.0, -LEG_LENGTH),
                            (0.0, CALF_ANGLE, 0.0)))
        parts.append(_link(foot, _mesh(mesh_dir, "foot.dae",
                                       fallback='<sphere radius="0.022"/>')))
        parts.append(_joint(f"{leg}_foot_joint", calf, foot, (0.0, 0.0, -LEG_LENGTH)))
    parts.append("</robot>")
    return "\n".join(parts)
