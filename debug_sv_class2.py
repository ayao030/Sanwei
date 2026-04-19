import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Tuple

import io_utils as rw
import sv_class2_transform as sv2


Node = Dict[str, Any]
Member = Dict[str, Any]


def _classify_nodes(nodes: List[Node]) -> Tuple[List[Node], List[Node]]:
    type11 = [node for node in nodes if int(node.get("node_type", 0)) == 11]
    type12 = [node for node in nodes if int(node.get("node_type", 0)) == 12]
    return type11, type12


def _print_summary(txt_path: Path, members: List[Member], nodes: List[Node]) -> None:
    type11, type12 = _classify_nodes(nodes)

    print("\n" + "=" * 60)
    print("sv_class2_transform 调试汇总")
    print("=" * 60)
    print(f"输入文件: {txt_path}")
    print(f"生成杆件数: {len(members)}")
    print(f"生成节点数: {len(nodes)}")
    print(f"Type 11 节点数: {len(type11)}")
    print(f"Type 12 节点数: {len(type12)}")

    if members:
        print("\n前 5 根杆件:")
        for item in members[:5]:
            print(json.dumps(item, ensure_ascii=False))

    if type11:
        print("\n前 5 个 Type 11 节点:")
        for item in type11[:5]:
            print(json.dumps(item, ensure_ascii=False))

    if type12:
        print("\n前 5 个 Type 12 节点:")
        for item in type12[:5]:
            print(json.dumps(item, ensure_ascii=False))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="单独调试 sv_class2_transform.single_view0201 的输出"
    )
    parser.add_argument("txt_path", help="单视图 txt 文件路径")
    parser.add_argument(
        "--dump-json",
        dest="dump_json",
        help="可选：把结果写到指定 json 文件",
    )
    args = parser.parse_args()

    txt_path = Path(args.txt_path)
    if not txt_path.exists():
        raise FileNotFoundError(f"找不到输入文件: {txt_path}")

    line_coord = rw.read_coords(str(txt_path))
    if not isinstance(line_coord, dict):
        raise ValueError("read_coords 解析失败，未拿到 coordinatesFront_data")

    print("=" * 60)
    print("开始执行 sv_class2_transform 单文件调试")
    print("=" * 60)
    print(f"输入文件: {txt_path}")
    print(f"读取到线段数: {len(line_coord)}")

    members, nodes = sv2.single_view0201(line_coord)

    _print_summary(txt_path, members, nodes)

    if args.dump_json:
        out_path = Path(args.dump_json)
        payload = {"ganjian": members, "jiedian": nodes}
        out_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(f"\n结果已写入: {out_path}")


if __name__ == "__main__":
    main()
