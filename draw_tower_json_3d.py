import argparse
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple, TypeAlias

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection


Point3D: TypeAlias = Tuple[float, float, float]
NodeMap: TypeAlias = Dict[str, Point3D]

AXES = ("X", "Y", "Z")
AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}
SYMMETRY_TAIL_MAP = {
    1: {"0": "1", "1": "0", "2": "3", "3": "2"},
    2: {"0": "2", "2": "0", "1": "3", "3": "1"},
    3: {"0": "3", "3": "0", "1": "2", "2": "1"},
}


def _as_float(value: object) -> float:
    return float(value)


def _plus_suffix(node_id: str, delta: int) -> str:
    suffix = node_id[-2:] if len(node_id) >= 2 else node_id
    if not suffix.isdigit():
        return node_id
    return f"{node_id[:-2]}{int(suffix) + delta:02d}"


def _symmetry_point(point: Point3D, symmetry_type: int) -> Point3D:
    x, y, z = point
    if symmetry_type == 1:
        return -x, y, z
    if symmetry_type == 2:
        return x, -y, z
    if symmetry_type == 3:
        return -x, -y, z
    return point


def _symmetry_deltas(symmetry_type: int) -> List[int]:
    if symmetry_type == 1:
        return [1]
    if symmetry_type == 2:
        return [2]
    if symmetry_type == 3:
        return [3]
    if symmetry_type == 4:
        return [1, 2, 3]
    return []


def _add_node_with_symmetry(nodes: NodeMap, node_id: str, point: Point3D, symmetry_type: int) -> None:
    nodes[node_id] = point
    for delta in _symmetry_deltas(symmetry_type):
        nodes[_plus_suffix(node_id, delta)] = _symmetry_point(point, delta)


def _decode_node_reference(value: object) -> Optional[str]:
    if not isinstance(value, str) or not value.startswith("1"):
        return None
    candidate = value[1:]
    if not candidate or "." in candidate:
        return None
    return candidate


def _read_real_node(row: dict) -> Point3D:
    return _as_float(row["X"]), _as_float(row["Y"]), _as_float(row["Z"])


def _compute_reference_node(row: dict, nodes: NodeMap) -> Optional[Point3D]:
    references: List[str] = []
    real_axis = ""
    real_value = 0.0

    for axis in AXES:
        reference_id = _decode_node_reference(row.get(axis))
        if reference_id is None:
            real_axis = axis
            try:
                real_value = _as_float(row[axis])
            except (KeyError, TypeError, ValueError):
                return None
        else:
            references.append(reference_id)

    if len(references) != 2 or not real_axis:
        return None
    if references[0] not in nodes or references[1] not in nodes:
        return None

    start = nodes[references[0]]
    end = nodes[references[1]]
    axis_index = AXIS_INDEX[real_axis]
    span = end[axis_index] - start[axis_index]
    if math.isclose(span, 0.0, abs_tol=1e-9):
        return None

    t = (real_value - start[axis_index]) / span
    coords = [start[i] + t * (end[i] - start[i]) for i in range(3)]
    coords[axis_index] = real_value
    return float(coords[0]), float(coords[1]), float(coords[2])


def build_expanded_nodes(raw_nodes: Iterable[dict]) -> Tuple[NodeMap, List[dict]]:
    nodes: NodeMap = {}
    pending: List[dict] = []

    for row in raw_nodes:
        node_id = str(row.get("node_id", ""))
        if not node_id:
            continue
        symmetry_type = int(row.get("symmetry_type", 0))
        if int(row.get("node_type", 0)) == 11:
            _add_node_with_symmetry(nodes, node_id, _read_real_node(row), symmetry_type)
        else:
            pending.append(row)

    unresolved = pending[:]
    while unresolved:
        next_unresolved = []
        resolved_this_round = 0

        for row in unresolved:
            point = _compute_reference_node(row, nodes)
            if point is None:
                next_unresolved.append(row)
                continue
            _add_node_with_symmetry(
                nodes,
                str(row["node_id"]),
                point,
                int(row.get("symmetry_type", 0)),
            )
            resolved_this_round += 1

        if resolved_this_round == 0:
            break
        unresolved = next_unresolved

    return nodes, unresolved


def _map_node_id(node_id: str, symmetry_type: int) -> Optional[str]:
    mapping = SYMMETRY_TAIL_MAP.get(symmetry_type)
    if not mapping or not node_id:
        return None
    tail = node_id[-1]
    if tail not in mapping:
        return None
    return f"{node_id[:-1]}{mapping[tail]}"


def build_expanded_members(raw_members: Iterable[dict]) -> List[dict]:
    members: List[dict] = []
    seen = set()

    for row in raw_members:
        member_id = str(row.get("member_id", ""))
        node1_id = str(row.get("node1_id", ""))
        node2_id = str(row.get("node2_id", ""))
        symmetry_type = int(row.get("symmetry_type", 0))

        candidates = [(node1_id, node2_id)]
        for delta in _symmetry_deltas(symmetry_type):
            mapped1 = _map_node_id(node1_id, delta)
            mapped2 = _map_node_id(node2_id, delta)
            if mapped1 and mapped2:
                candidates.append((mapped1, mapped2))

        for start_id, end_id in candidates:
            key = tuple(sorted((start_id, end_id)))
            if key in seen:
                continue
            seen.add(key)
            members.append(
                {
                    "member_id": member_id,
                    "node1_id": start_id,
                    "node2_id": end_id,
                }
            )

    return members


def _set_axes_to_data(ax, points: List[Point3D]) -> None:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    zs = [point[2] for point in points]

    def limits(values: List[float]) -> Tuple[float, float, float]:
        low = min(values)
        high = max(values)
        span = max(high - low, 1.0)
        pad = span * 0.06
        return low - pad, high + pad, span + pad * 2

    xmin, xmax, xspan = limits(xs)
    ymin, ymax, yspan = limits(ys)
    zmin, zmax, zspan = limits(zs)

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)
    ax.set_box_aspect((xspan, yspan, zspan))


def plot_tower_body(
    json_path: Path,
    save_path: Optional[Path],
    show: bool,
    elev: float,
    azim: float,
    show_nodes: bool,
) -> Tuple[int, int, int, int]:
    with json_path.open("r", encoding="utf-8") as file:
        data = json.load(file)

    nodes, unresolved_nodes = build_expanded_nodes(data.get("jiedian", []))
    members = build_expanded_members(data.get("ganjian", []))

    segments = []
    used_points: List[Point3D] = []
    missing_members = 0
    for member in members:
        start = nodes.get(str(member["node1_id"]))
        end = nodes.get(str(member["node2_id"]))
        if start is None or end is None:
            missing_members += 1
            continue
        segments.append([start, end])
        used_points.extend((start, end))

    if not segments:
        raise ValueError("No drawable members found in the JSON result.")

    fig = plt.figure(figsize=(9, 11))
    ax = fig.add_subplot(111, projection="3d")

    collection = Line3DCollection(segments, colors="#1f5a96", linewidths=0.75, alpha=0.92)
    ax.add_collection3d(collection)

    if show_nodes:
        xs = [point[0] for point in nodes.values()]
        ys = [point[1] for point in nodes.values()]
        zs = [point[2] for point in nodes.values()]
        ax.scatter(xs, ys, zs, s=4, c="#d64f3a", alpha=0.35)

    _set_axes_to_data(ax, used_points)
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Tower Body 3D View")
    try:
        ax.set_proj_type("ortho")
    except Exception:
        pass

    fig.tight_layout()

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=220)
    if show:
        plt.show()
    else:
        plt.close(fig)

    return len(nodes), len(unresolved_nodes), len(segments), missing_members


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Draw a 3D tower body view from tower_output.json.")
    parser.add_argument("json_path", nargs="?", default="tower_output.json")
    parser.add_argument("--save", default=None, help="PNG path. Defaults to <json_stem>_3d.png.")
    parser.add_argument("--show", action="store_true", help="Open an interactive matplotlib window.")
    parser.add_argument("--hide-nodes", action="store_true", help="Draw members only.")
    parser.add_argument("--elev", type=float, default=18.0)
    parser.add_argument("--azim", type=float, default=-58.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    json_path = Path(args.json_path)
    save_path = Path(args.save) if args.save else json_path.with_name(f"{json_path.stem}_3d.png")

    node_count, unresolved_count, segment_count, missing_count = plot_tower_body(
        json_path=json_path,
        save_path=save_path,
        show=args.show,
        elev=args.elev,
        azim=args.azim,
        show_nodes=not args.hide_nodes,
    )

    print(f"expanded nodes: {node_count}")
    print(f"drawable members: {segment_count}")
    print(f"unresolved reference nodes: {unresolved_count}")
    print(f"members skipped for missing endpoints: {missing_count}")
    print(f"saved: {save_path}")


if __name__ == "__main__":
    main()
