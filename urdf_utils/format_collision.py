import csv as csvmod
import shutil
from pathlib import Path

import yaml

from .mesh_convex_decomposition import batch_convex_decomposition
from .mesh_bounding import get_bounding_geometry


def format_collision_info(input_pkg_path, output_pkg_path):
    """
    第四步：提取碰撞信息到输出包的config文件夹，并复制对应mesh
    """
    input_path = Path(input_pkg_path)
    output_path = Path(output_pkg_path)

    # 找到输入包中的CSV文件
    urdf_dir = input_path / "urdf"
    meshes_dir = input_path / "meshes"
    csv_files = list(urdf_dir.glob("*.csv"))
    if not csv_files:
        raise RuntimeError("输入包中未找到CSV文件")

    # 创建输出目录
    output_collision_dir = output_path / "meshes" / "collision"
    output_collision_dir.mkdir(parents=True, exist_ok=True)
    output_config_dir = output_path / "config"
    output_config_dir.mkdir(parents=True, exist_ok=True)

    # 进行凸分解
    result = batch_convex_decomposition(
        str(meshes_dir), str(output_collision_dir), verbose=True
    )
    if not result["successful"]:
        raise RuntimeError("凸分解失败")

    # 读取CSV文件获取link信息
    csv_path = csv_files[0]
    content = csv_path.read_text(encoding="utf-8")
    lines = content.strip().split("\n")
    reader = csvmod.reader(lines)
    header = next(reader)  # 获取表头

    # 找到需要的列索引
    required_names = [
        "link name",
        "collision x",
        "collision y",
        "collision z",
        "collision roll",
        "collision pitch",
        "collision yaw",
    ]
    column_indices = {}
    for i, h in enumerate(header):
        for search_term in required_names:
            if search_term in h.lower():
                column_indices[search_term] = i
                break
    if "link name" not in column_indices:
        raise RuntimeError("csv文件未找到Link Name列")

    print("开始提取碰撞信息...")
    collision_data = {}
    for row in reader:
        if len(row) <= column_indices.get("link name", 0):
            raise RuntimeError(f"Invalid row format: {row}")
        link_name = row[column_indices["link name"]].strip()
        if not link_name:
            raise RuntimeError(f"Empty link name in row: {row}")
        print(f"处理link: {link_name}")

        # 获取collision位姿信息
        def from_row(row, key):
            idx = column_indices.get(key)
            if idx is not None and idx < len(row) and row[idx]:
                try:
                    return float(row[idx])
                except ValueError:
                    return 0.0
            return 0.0

        collision_xyz = [
            from_row(row, "collision x"),
            from_row(row, "collision y"),
            from_row(row, "collision z"),
        ]
        collision_rpy = [
            from_row(row, "collision roll"),
            from_row(row, "collision pitch"),
            from_row(row, "collision yaw"),
        ]
        collision_xyz = " ".join(map(str, collision_xyz))
        collision_rpy = " ".join(map(str, collision_rpy))

        # 查找对应的mesh文件
        mesh_extensions = ["STL", "stl", "dae", "DAE"]
        source_mesh_path = None

        for ext in mesh_extensions:
            potential_path = meshes_dir / f"{link_name}.{ext}"
            if potential_path.exists():
                source_mesh_path = potential_path
                break

        if source_mesh_path is None:
            raise RuntimeError(f"未找到link {link_name} 的mesh文件，跳过")

        # 生成convex_mesh信息
        mesh_relative_path = (
            f"package://{output_path.name}/meshes/collision/{source_mesh_path.name}"
        )

        # 构建collision数据
        link_collision = {
            "convex_mesh": {
                "origin": {"xyz": collision_xyz, "rpy": collision_rpy},
                "path": mesh_relative_path,
            }
        }

        # 调用get_bounding_geometry计算包围几何体
        bounding_geom = get_bounding_geometry(
            str(source_mesh_path), collision_xyz, collision_rpy
        )
        if bounding_geom is None:
            raise RuntimeError(f"警告: 无法计算link {link_name} 的包围几何体")
        link_collision.update(bounding_geom)

        collision_data[link_name] = link_collision
        print(f"已处理link {link_name} 的碰撞信息")

    # 写入collision.yaml文件
    collision_yaml_path = output_config_dir / "collision.yaml"
    collision_yaml_path.write_text(
        yaml.dump(collision_data, allow_unicode=True, sort_keys=True)
    )

    print("碰撞配置与Mesh相关处理完成！")
