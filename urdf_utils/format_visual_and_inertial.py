#!/usr/bin/env python3
"""
从CSV文件中提取可视化和惯量信息并生成YAML配置文件的工具

使用方法:
python sw_csv2yaml.py input.csv package_path
"""

import csv
from pathlib import Path

import yaml


def format_inertial_info_from_csv(csv_file_path, output_yaml_path):
    """
    从CSV文件中提取惯量信息并生成YAML配置文件

    Args:
        csv_file_path (str): CSV文件路径
        output_yaml_path (str): 输出YAML文件路径
    """
    csv_path = Path(csv_file_path)
    output_path = Path(output_yaml_path)

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV文件不存在: {csv_path}")

    # 确保输出目录存在
    output_path.parent.mkdir(parents=True, exist_ok=True)

    inertial_data = {}

    with csv_path.open("r", encoding="utf-8") as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            link_name = row["Link Name"].strip()

            # 跳过空的link名称
            if not link_name:
                continue

            # 提取质心坐标信息
            com_x = float(row["Center of Mass X"]) if row["Center of Mass X"] else 0.0
            com_y = float(row["Center of Mass Y"]) if row["Center of Mass Y"] else 0.0
            com_z = float(row["Center of Mass Z"]) if row["Center of Mass Z"] else 0.0
            com_roll = (
                float(row["Center of Mass Roll"]) if row["Center of Mass Roll"] else 0.0
            )
            com_pitch = (
                float(row["Center of Mass Pitch"])
                if row["Center of Mass Pitch"]
                else 0.0
            )
            com_yaw = (
                float(row["Center of Mass Yaw"]) if row["Center of Mass Yaw"] else 0.0
            )

            # 提取质量
            mass = float(row["Mass"]) if row["Mass"] else 0.0

            # 提取惯性矩阵信息
            ixx = float(row["Moment Ixx"]) if row["Moment Ixx"] else 0.0
            iyy = float(row["Moment Iyy"]) if row["Moment Iyy"] else 0.0
            izz = float(row["Moment Izz"]) if row["Moment Izz"] else 0.0
            ixy = float(row["Moment Ixy"]) if row["Moment Ixy"] else 0.0
            ixz = float(row["Moment Ixz"]) if row["Moment Ixz"] else 0.0
            iyz = float(row["Moment Iyz"]) if row["Moment Iyz"] else 0.0

            # 构建YAML数据结构
            inertial_data[link_name] = {
                "origin": {
                    "xyz": f"{com_x} {com_y} {com_z}",
                    "rpy": f"{com_roll} {com_pitch} {com_yaw}",
                },
                "mass": mass,
                "inertia": {
                    "xx": ixx,
                    "yy": iyy,
                    "zz": izz,
                    "xy": ixy,
                    "xz": ixz,
                    "yz": iyz,
                },
            }

    # 写入YAML文件
    with output_path.open("w", encoding="utf-8") as yamlfile:
        yaml.dump(
            inertial_data,
            yamlfile,
            default_flow_style=False,
            allow_unicode=True,
            sort_keys=True,
        )

    print(f"成功生成惯量YAML文件: {output_path}")
    print(f"包含 {len(inertial_data)} 个link的惯量信息")


def format_visual_info_from_csv(csv_file_path, output_yaml_path, mesh_parent_dir=None):
    """
    从CSV文件中提取可视化信息并生成YAML配置文件

    Args:
        csv_file_path (str): CSV文件路径
        output_yaml_path (str): 输出YAML文件路径
        mesh_parent_dir (str, optional): STL文件的父目录。如果为空则使用CSV中的路径
    """
    csv_path = Path(csv_file_path)
    output_path = Path(output_yaml_path)

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV文件不存在: {csv_path}")

    # 确保输出目录存在
    output_path.parent.mkdir(parents=True, exist_ok=True)

    visual_data = {}

    with csv_path.open("r", encoding="utf-8") as csvfile:
        reader = csv.DictReader(csvfile)

        for row in reader:
            link_name = row["Link Name"].strip()

            # 跳过空的link名称
            if not link_name:
                continue

            # 提取可视化坐标信息
            visual_x = float(row["Visual X"]) if row["Visual X"] else 0.0
            visual_y = float(row["Visual Y"]) if row["Visual Y"] else 0.0
            visual_z = float(row["Visual Z"]) if row["Visual Z"] else 0.0
            visual_roll = float(row["Visual Roll"]) if row["Visual Roll"] else 0.0
            visual_pitch = float(row["Visual Pitch"]) if row["Visual Pitch"] else 0.0
            visual_yaw = float(row["Visual Yaw"]) if row["Visual Yaw"] else 0.0

            # 提取颜色信息
            color_red = float(row["Color Red"]) if row["Color Red"] else 1.0
            color_green = float(row["Color Green"]) if row["Color Green"] else 1.0
            color_blue = float(row["Color Blue"]) if row["Color Blue"] else 1.0
            color_alpha = float(row["Color Alpha"]) if row["Color Alpha"] else 1.0

            # 处理mesh文件路径
            mesh_filename = row["Mesh Filename"].strip()
            if mesh_filename:
                if mesh_parent_dir:
                    # 如果指定了父目录，则替换路径
                    # 提取原始文件名
                    original_filename = Path(mesh_filename).name
                    mesh_path = f"{mesh_parent_dir}/{original_filename}"
                else:
                    # 使用原始路径
                    mesh_path = mesh_filename
            else:
                print(f"警告: {link_name} 没有mesh文件名，跳过")
                continue

            # 构建YAML数据结构
            visual_data[link_name] = {
                "path": mesh_path,
                "origin": {
                    "xyz": f"{visual_x} {visual_y} {visual_z}",
                    "rpy": f"{visual_roll} {visual_pitch} {visual_yaw}",
                },
                "rgba": f"{color_red} {color_green} {color_blue} {color_alpha}",
            }

    # 写入YAML文件
    with output_path.open("w", encoding="utf-8") as yamlfile:
        yaml.dump(
            visual_data,
            yamlfile,
            default_flow_style=False,
            allow_unicode=True,
            sort_keys=True,
        )

    print(f"成功生成可视化YAML文件: {output_path}")
    print(f"包含 {len(visual_data)} 个link的可视化信息")


def copy_visual_mesh(input_pkg_path, output_pkg_path):
    """
    从输入包中复制可视化mesh到输出包的visual目录

    Args:
        input_pkg_path (str): 输入包路径
        output_pkg_path (str): 输出包路径
    """
    input_path = Path(input_pkg_path)
    output_path = Path(output_pkg_path)

    meshes_dir = input_path / "meshes"
    output_visual_dir = output_path / "meshes" / "visual"
    output_visual_dir.mkdir(parents=True, exist_ok=True)

    for mesh_file in meshes_dir.glob("*"):
        if mesh_file.is_file() and mesh_file.suffix.lower() in [".stl", ".dae"]:
            dest_file = output_visual_dir / mesh_file.name
            dest_file.write_bytes(mesh_file.read_bytes())
            print(f"已复制可视化mesh: {mesh_file.name}")


def format_visual_inertial_info(input_pkg_path, output_package_path):
    input_path = Path(input_pkg_path)
    urdf_dir = input_path / "urdf"
    csv_files = list(urdf_dir.glob("*.csv"))
    if not csv_files:
        print("输入包中未找到CSV文件")
        return False
    csv_path = Path(csv_files[0])  # 使用第一个CSV文件
    output_package_path = Path(output_package_path)

    if not csv_path.exists():
        raise FileNotFoundError(f"CSV文件不存在: {csv_path}")

    if not output_package_path.exists():
        raise FileNotFoundError(f"包路径不存在: {output_package_path}")

    # 设置输出路径
    config_dir = output_package_path / "config"
    visual_yaml_path = config_dir / "visual.yaml"
    inertial_yaml_path = config_dir / "inertial.yaml"

    # 设置mesh路径
    mesh_parent_dir = f"package://{output_package_path.name}/meshes/visual"

    # 复制可视化mesh到输出包
    copy_visual_mesh(input_pkg_path, output_package_path)

    # 提取可视化信息
    format_visual_info_from_csv(csv_path, visual_yaml_path, mesh_parent_dir)

    # 提取惯量信息
    format_inertial_info_from_csv(csv_path, inertial_yaml_path)

    print("\n惯量与视觉配置文件处理完成!")
