import os
import xml.etree.ElementTree as ET

from go2_sim.urdf import build_urdf

MESH_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..",
                                        "helper", "task-3", "common", "protos", "dae"))


def _parse(urdf):
    root = ET.fromstring(urdf.split("\n", 1)[1])
    links = {link.get("name") for link in root.findall("link")}
    joints = root.findall("joint")
    return root, links, joints


def test_structure_is_a_tree_rooted_at_base_link():
    root, links, joints = _parse(build_urdf(None))
    children = [j.find("child").get("link") for j in joints]
    parents = {j.find("parent").get("link") for j in joints}
    assert len(children) == len(set(children))
    assert set(children) | {"base_link"} == links
    assert parents <= links
    assert "base_link" not in children


def test_four_legs():
    _, links, _ = _parse(build_urdf(None))
    for leg in ("FL", "FR", "RL", "RR"):
        for part in ("hip", "thigh", "calf", "foot"):
            assert f"{leg}_{part}" in links


def test_fallback_without_meshes():
    urdf = build_urdf("/nonexistent")
    assert "<mesh" not in urdf and "<box" in urdf


def test_uses_meshes_when_present():
    if not os.path.isfile(os.path.join(MESH_DIR, "base.dae")):
        return
    urdf = build_urdf(MESH_DIR)
    assert urdf.count("<mesh") == 1 + 4 * 4
    assert "thigh_mirror.dae" in urdf and "calf_mirror.dae" in urdf
