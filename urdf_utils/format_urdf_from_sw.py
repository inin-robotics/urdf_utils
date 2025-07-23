import argparse
import csv as csvmod
import sys
from pathlib import Path

from .format_collision import format_collision_info
from .format_package import format_template, format_links_and_joints
from .format_visual_and_inertial import format_visual_inertial_info


def check_sw_urdf_package(input_pkg_path):
    """
    检查输入的SolidWorks导出URDF包目录结构和关键文件
    """
    input_path = Path(input_pkg_path)
    urdf_dir = input_path / "urdf"
    meshes_dir = input_path / "meshes"

    required_dirs = [urdf_dir, meshes_dir]
    for d in required_dirs:
        if not d.is_dir():
            raise RuntimeError(f"缺少目录: {d}")

    # 检查urdf目录下的urdf、csv、yaml文件
    urdf_files = list(urdf_dir.glob("*.urdf"))
    csv_files = list(urdf_dir.glob("*.csv"))

    if not urdf_files:
        raise RuntimeError("urdf目录下缺少.urdf文件")
    if not csv_files:
        raise RuntimeError("urdf目录下缺少.csv文件")

    # 检查csv中每个link的mesh文件是否存在（支持STL、stl、dae等格式）
    csv_path = csv_files[0]
    content = csv_path.read_text(encoding="utf-8")
    lines = content.strip().split("\n")
    reader = csvmod.reader(lines)
    header = next(reader)
    # 假设有link_name列
    link_idx = None
    for i, h in enumerate(header):
        if "link" in h.lower():
            link_idx = i
            break
    if link_idx is None:
        raise RuntimeError("csv文件未找到link列")
    for row in reader:
        link_name = row[link_idx]
        # 检查多种mesh文件格式
        mesh_extensions = ["*.STL", "*.stl", "*.dae", "*.DAE"]
        mesh_found = False
        for ext in mesh_extensions:
            mesh_files = list(meshes_dir.glob(f"{link_name}.{ext.split('.')[-1]}"))
            if mesh_files:
                mesh_found = True
                break
        if not mesh_found:
            raise RuntimeError(f"缺少mesh文件: {link_name}.*")
    print("输入包结构检查通过！")


def main():
    parser = argparse.ArgumentParser(description="清洗SolidWorks导出的URDF包")
    parser.add_argument("--output_path", required=True, help="输出路径")
    parser.add_argument("--output_pkg_name", required=True, help="输出包名称")
    parser.add_argument("--output_macro_name", required=True, help="输出宏名称")
    parser.add_argument(
        "--input_pkg_path", required=True, help="输入的SolidWorks导出包路径"
    )
    args = parser.parse_args()

    input_path = Path(args.input_pkg_path)
    if not input_path.is_dir():
        print(f"输入路径不存在: {args.input_pkg_path}")
        sys.exit(1)
    # 第一步：检查输入包结构
    check_sw_urdf_package(args.input_pkg_path)

    # 第二步：复制模板并修改包名称
    format_template(args.output_path, args.output_pkg_name, args.output_macro_name)

    # 第三步：提取视觉和惯量信息
    output_pkg_path = Path(args.output_path) / args.output_pkg_name
    format_visual_inertial_info(args.input_pkg_path, output_pkg_path)

    # 第四步：提取碰撞信息
    format_collision_info(args.input_pkg_path, output_pkg_path)

    # 第五步：利用utils.xacro生成xacro文件
    format_links_and_joints(
        args.input_pkg_path, output_pkg_path, args.output_macro_name
    )


if __name__ == "__main__":
    main()
