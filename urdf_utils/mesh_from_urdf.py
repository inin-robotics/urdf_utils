#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import numpy as np
import open3d as o3d
from urdf_parser_py.urdf import URDF
import rospkg

def create_transform_matrix(xyz, rpy):
    """
    根据xyz和rpy（roll, pitch, yaw）创建一个4x4的齐次变换矩阵。
    使用 Open3D 的内置函数。

    Args:
        xyz (list[float]): [x, y, z] 平移。
        rpy (list[float]): [roll, pitch, yaw] 欧拉角。

    Returns:
        numpy.ndarray: 4x4的变换矩阵。
    """
    translation_matrix = np.eye(4)
    translation_matrix[:3, 3] = xyz

    # Open3D 使用 XYZ 顺序的欧拉角来创建旋转矩阵
    # URDF 的 RPY 也是 extrinsic XYZ
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_xyz(rpy)
    
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    
    # 结合平移和旋转
    return translation_matrix @ transform


def resolve_ros_path(path_string):
    """
    解析 'package://' 格式的路径为绝对文件路径。

    Args:
        path_string (str): 包含 'package://' 的路径。

    Returns:
        str: 绝对文件路径，如果无法解析则返回 None。
    """
    if not path_string.startswith("package://"):
        # 如果不是ROS包路径，假设是普通路径
        return path_string
    
    try:
        # 去掉 'package://' 前缀
        path = path_string[len("package://"):]
        parts = path.split('/', 1)
        package_name = parts[0]
        relative_path = parts[1] if len(parts) > 1 else ""

        # 使用rospkg找到包的路径
        rospack = rospkg.RosPack()
        package_path = rospack.get_path(package_name)

        return os.path.join(package_path, relative_path)
    except (rospkg.ResourceNotFound, IndexError) as e:
        print(f"Error resolving ROS path: {path_string}. Reason: {e}")
        return None


def get_link_world_transforms(robot):
    """
    计算在默认关节角度（0）下，每个link相对于世界（根link）的变换。

    Args:
        robot (urdf_parser_py.urdf.URDF): 已解析的URDF机器人模型。

    Returns:
        dict: 一个字典，键是link名，值是4x4的numpy变换矩阵。
    """
    link_transforms = {}
    root_link = robot.get_root()
    
    # 使用一个队列进行广度优先搜索来遍历运动学树
    # 队列中的每个元素是 (link_name, parent_world_transform)
    queue = [(root_link, np.identity(4))]
    
    # 将根link的变换存入字典
    link_transforms[root_link] = np.identity(4)

    # 已经处理过的link集合，防止在树中循环（虽然URDF不应该有）
    processed_links = {root_link}
    
    while queue:
        parent_name, parent_transform = queue.pop(0)
        
        # 寻找以此link为parent的所有joint
        for joint in robot.joints:
            if joint.parent == parent_name:
                child_name = joint.child
                if child_name not in processed_links:
                    
                    # 1. 获取从父link到子link的静态变换（joint的origin）
                    joint_origin_xyz = joint.origin.xyz if joint.origin else [0, 0, 0]
                    joint_origin_rpy = joint.origin.rpy if joint.origin else [0, 0, 0]
                    T_parent_joint = create_transform_matrix(joint_origin_xyz, joint_origin_rpy)

                    # 2. 获取关节本身的变换（对于默认位置，非固定关节为0）
                    T_joint_child = np.identity(4)
                    if joint.joint_type != 'fixed':
                        # 对于 'revolute' 或 'prismatic' 等, 默认位置值为0
                        # 旋转轴或平移轴
                        if joint.joint_type == 'revolute':
                            # 角度为0，所以是单位矩阵
                            pass 
                        elif joint.joint_type == 'prismatic':
                            # 位移为0，所以是单位矩阵
                            pass
                    
                    # 3. 计算子link的世界坐标
                    # T_world_child = T_world_parent * T_parent_joint * T_joint_child
                    child_transform = parent_transform @ T_parent_joint @ T_joint_child
                    link_transforms[child_name] = child_transform
                    
                    # 将子link加入队列继续处理
                    queue.append((child_name, child_transform))
                    processed_links.add(child_name)
                    
    return link_transforms


def main():
    """主执行函数"""
    parser = argparse.ArgumentParser(description="Read a URDF from a ROS package, combine all collision meshes at their default positions, and save the result as a single mesh file.")
    parser.add_argument("package_name", type=str, help="The name of the ROS package containing the URDF file (e.g., 'urdf_tutorial').")
    parser.add_argument("urdf_path_in_package", type=str, help="The relative path to the URDF file within the package (e.g., 'urdf/01-myfirst.urdf').")
    parser.add_argument("output_file", type=str, help="Path to save the combined mesh (e.g., 'combined_mesh.ply'). Recommended formats: .ply, .stl, .obj")

    args = parser.parse_args()

    # 1. 找到并加载 URDF 文件
    urdf_full_path = resolve_ros_path(f"package://{args.package_name}/{args.urdf_path_in_package}")
    if not urdf_full_path or not os.path.exists(urdf_full_path):
        print(f"Error: Could not find or access URDF file at '{urdf_full_path}'")
        return

    try:
        print(f"Loading URDF from: {urdf_full_path}")
        robot = URDF.from_xml_file(urdf_full_path)
    except Exception as e:
        print(f"Error parsing URDF file: {e}")
        return

    # 2. 计算所有link在世界坐标系下的变换
    print("Calculating link transforms at default joint positions...")
    link_world_transforms = get_link_world_transforms(robot)
    print(f"Found and calculated transforms for {len(link_world_transforms)} links.")

    # 3. 遍历所有link，加载、变换并合并其collision mesh
    combined_mesh = o3d.geometry.TriangleMesh()
    
    for link_name, T_world_link in link_world_transforms.items():
        link = robot.link_map[link_name]
        
        if not link.collisions:
            continue
        
        print(f"Processing link: {link.name}")

        for i, collision in enumerate(link.collisions):
            # Check if collision geometry is a mesh
            if not hasattr(collision.geometry, 'filename'):
                print(f"  - Skipping non-mesh collision geometry in link '{link.name}'.")
                continue
            
            # a. 获取mesh文件路径并解析
            mesh_path = resolve_ros_path(collision.geometry.filename)
            if not mesh_path or not os.path.exists(mesh_path):
                print(f"  - WARNING: Mesh file not found for collision in link '{link.name}': {collision.geometry.filename}")
                continue

            # b. 加载mesh
            try:
                mesh = o3d.io.read_triangle_mesh(mesh_path)
                # 有些mesh可能没有正确计算法线，这会影响显示效果
                if not mesh.has_vertex_normals():
                    mesh.compute_vertex_normals()
                print(f"  - Loaded mesh: {os.path.basename(mesh_path)}")
            except Exception as e:
                print(f"  - ERROR: Failed to load mesh '{mesh_path}' with Open3D. Reason: {e}")
                continue
                
            # c. 获取collision相对于其link的变换
            coll_origin_xyz = collision.origin.xyz if collision.origin else [0, 0, 0]
            coll_origin_rpy = collision.origin.rpy if collision.origin else [0, 0, 0]
            T_link_collision = create_transform_matrix(coll_origin_xyz, coll_origin_rpy)
            
            # d. 计算mesh在世界坐标系下的最终变换
            # T_final = T_world_link * T_link_collision
            final_transform = T_world_link @ T_link_collision
            
            # e. 应用变换
            mesh.transform(final_transform)
            
            # f. 合并到总mesh中
            combined_mesh += mesh

    # 4. 保存最终的合并mesh
    if not combined_mesh.has_triangles():
        print("\nWarning: No collision meshes were found or processed. The output file will be empty.")
    else:
        print(f"\nSaving combined mesh to: {args.output_file}")
        if args.output_file.endswith('.stl'):
            combined_mesh.compute_vertex_normals()  # 确保STL文件有法线
        o3d.io.write_triangle_mesh(args.output_file, combined_mesh)
        print("Done.")


if __name__ == "__main__":
    main()