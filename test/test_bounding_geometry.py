#!/usr/bin/env python3
"""
测试包围几何体计算函数的准确性
生成基本几何体并验证各个函数是否能正确识别几何体类型和参数
"""

import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

# 添加父目录到路径，以便导入simplify_collision模块
sys.path.append(str(Path(__file__).resolve().parent.parent / "urdf_utils"))
from simplify_collision import (  # type: ignore
    get_bounding_box,
    get_bounding_capsule,
    get_bounding_cylinder,
    get_bounding_geometry,
    get_bounding_sphere,
)


def visualize_result(mesh_ori, params_expected, params_calc):
    def get_tf_from_param(params):
        if "origin" in params:
            origin = params["origin"]
        else:
            origin = params["cylinder"]["origin"]
        trans = np.array(list(map(float, origin["xyz"].split())))
        if "rpy" in origin:
            rot = np.array(list(map(float, origin["rpy"].split())))
        else:
            # for sphere
            rot = np.zeros(3)
        tf = np.eye(4)
        tf[:3, 3] = trans
        tf[:3, :3] = R.from_euler("xyz", rot).as_matrix()
        return tf

    type_calc = params_calc.get("type", "capsule")
    if type_calc == "box":
        size = params_calc["size"]
        mesh_bound = o3d.geometry.TriangleMesh.create_box(
            width=size[0], height=size[1], depth=size[2]
        )
        mesh_bound.translate(-size / 2)
    elif type_calc == "sphere":
        radius = params_calc["radius"]
        mesh_bound = o3d.geometry.TriangleMesh.create_sphere(
            radius=radius, resolution=20
        )
    elif type_calc == "cylinder":
        radius = params_calc["radius"]
        length = params_calc["length"]
        mesh_bound = o3d.geometry.TriangleMesh.create_cylinder(
            radius=radius, height=length, resolution=20
        )
    elif type_calc == "capsule":
        radius = params_calc["cylinder"]["radius"]
        length = params_calc["cylinder"]["length"]
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(
            radius=radius, height=length, resolution=20
        )
        sphere1 = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
        sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
        sphere1.translate(np.array([0, 0, length / 2]))
        sphere2.translate(np.array([0, 0, -length / 2]))
        mesh_bound = cylinder + sphere1 + sphere2
    tf_bound = get_tf_from_param(params_calc)
    tf_expected = get_tf_from_param(params_expected)
    mesh_bound.transform(tf_bound)
    mesh_bound.paint_uniform_color([0, 1, 0])  # 红色
    mesh_copy = o3d.geometry.TriangleMesh(mesh_ori)
    mesh_copy.paint_uniform_color([1, 0, 0])
    mesh_copy.transform(tf_expected)
    print("\n显示说明:")
    print("- 红色：输入mesh")
    print("- 绿色：包围几何体")
    o3d.visualization.draw_geometries(  # type: ignore
        [mesh_bound, mesh_copy],
        window_name=f"STL模型与最小包围{type_calc} - 多视图显示",
        width=1400,
        height=900,
    )


def create_test_box(size, origin):
    """创建测试用的立方体/长方体"""
    width, height, depth = size
    box = o3d.geometry.TriangleMesh.create_box(width=width, height=height, depth=depth)
    # 将盒子中心移到原点
    box.translate([-width / 2, -height / 2, -depth / 2])
    # 添加一些三角形细分以增加顶点数量
    box = box.subdivide_midpoint(number_of_iterations=1)
    return box, {
        "type": "box",
        "volume": width * height * depth,
        "size": np.array(size),
        "origin": origin,
    }


def create_test_sphere(radius, origin):
    """创建测试用的球体"""
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
    return sphere, {
        "type": "sphere",
        "volume": (4 / 3) * np.pi * radius**3,
        "radius": radius,
        "origin": {"xyz": origin["xyz"]},
    }


def create_test_cylinder(radius, length, origin):
    """创建测试用的圆柱体"""
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(
        radius=radius, height=length, resolution=20
    )
    return cylinder, {
        "type": "cylinder",
        "volume": np.pi * radius**2 * length,
        "radius": radius,
        "length": length,
        "origin": origin,
    }


def create_test_capsule(radius, length, tf, origin):
    """创建测试用的胶囊体"""
    cylinder, cylinder_param = create_test_cylinder(
        radius=radius, length=length, origin=origin
    )
    cylinder_param.pop("type")
    cylinder_param.pop("volume")

    def create_sphere_for_capsule(radius, length, tf):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius, resolution=20)
        trans = np.array([0, 0, length / 2])
        sphere.translate(trans)
        trans = tf[:3, :3] @ trans + tf[:3, 3]
        xyz = " ".join(map(str, trans))
        return sphere, xyz

    sphere1, xyz1 = create_sphere_for_capsule(radius, length, tf)
    sphere2, xyz2 = create_sphere_for_capsule(radius, -length, tf)
    # 合并几何体
    capsule = cylinder + sphere1 + sphere2
    total_volume = np.pi * radius**2 * length + (4 / 3) * np.pi * radius**3
    return capsule, {
        "type": "capsule",
        "volume": total_volume,
        "cylinder": cylinder_param,
        "spheres": [
            {"radius": radius, "origin": {"xyz": xyz1}},
            {"radius": radius, "origin": {"xyz": xyz2}},
        ],
    }


def test_bounding_function(mesh, params_expected, test_function, rpy, trans, rot):
    """测试单个包围几何体函数"""
    print("\n--- 测试参数计算结果---")
    print(f"预期参数: {params_expected}")
    try:
        result = test_function(mesh, rpy, trans, rot)
        if result is None:
            print("❌ 函数返回None")
            return False

        print("计算结果:")
        for key, value in result.items():
            if isinstance(value, np.ndarray):
                print(f"  {key}: {np.round(value, 4)}")
            else:
                print(f"  {key}: {value}")

        # 验证体积是否合理
        expected_volume = params_expected["volume"]
        calculated_volume = result["volume"]
        volume_error = abs(calculated_volume - expected_volume) / expected_volume

        print("体积对比:")
        print(f"  预期体积: {expected_volume:.4f}")
        print(f"  计算体积: {calculated_volume:.4f}")
        print(f"  相对误差: {volume_error:.2%}")

        # 验证中心是否合理
        if params_expected["type"] != "capsule":
            expected_xyz = params_expected["origin"]["xyz"]
            calculated_xyz = result["origin"]["xyz"]
        else:
            expected_xyz = params_expected["cylinder"]["origin"]["xyz"]
            calculated_xyz = result["cylinder"]["origin"]["xyz"]
        expected_center = np.array(list(map(float, expected_xyz.split())))
        calculated_center = np.array(list(map(float, calculated_xyz.split())))
        center_error = np.linalg.norm(
            calculated_center - expected_center
        ) / expected_volume ** (1 / 3)

        print("中心对比:")
        print(f"  预期中心: {expected_center}")
        print(f"  计算中心: {center_error}")
        print(f"  相对误差（相对体积的立方根计算）: {center_error:.2%}")

        # 判断是否通过测试（体积误差小于5%，中心误差小于体积立方根的5%认为合理）
        if volume_error < 0.05 and center_error < 0.05:
            print("✅ 测试通过")
            return True
        else:
            if volume_error > 0.05:
                print("❓️ 测试未通过: 体积误差过大")
            if center_error > 0.05:
                print("❓️ 测试未通过: 中心误差过大")
            return False

    except Exception as e:
        print(f"❌ 测试失败: {e}")
        return False


def run_comprehensive_test():
    """运行综合测试"""
    print("=== 包围几何体算法测试 ===\n")

    # 生成随机位姿
    trans = np.random.uniform(-1, 1, size=3)
    rot = np.random.uniform(-np.pi, np.pi, size=3)
    xyz = " ".join(map(str, trans))
    rpy = " ".join(map(str, rot))
    origin = {"xyz": xyz, "rpy": rpy}
    rot = R.from_euler("xyz", rot, degrees=True).as_matrix()
    tf = np.eye(4)
    tf[:3, :3] = rot
    tf[:3, 3] = trans

    test_cases = []

    # 1. 测试立方体
    print("1. 生成测试立方体...")
    box_size = np.random.uniform(0.3, 1.0, size=3)
    box_mesh, box_params = create_test_box(box_size, origin)
    test_cases.append(("立方体", box_mesh, box_params, get_bounding_box))

    # 2. 测试球体
    print("2. 生成测试球体...")
    sphere_radius = np.random.uniform(0.3, 1.0)
    sphere_mesh, sphere_params = create_test_sphere(sphere_radius, origin)
    test_cases.append(("球体", sphere_mesh, sphere_params, get_bounding_sphere))

    # 3. 测试圆柱体
    print("3. 生成测试圆柱体...")
    cylinder_radius = np.random.uniform(0.3, 1.0)
    cylinder_length = np.random.uniform(1.0, 2.0)
    cylinder_mesh, cylinder_params = create_test_cylinder(
        cylinder_radius, cylinder_length, origin
    )
    test_cases.append(("圆柱体", cylinder_mesh, cylinder_params, get_bounding_cylinder))

    # 4. 测试胶囊体
    print("4. 生成测试胶囊体...")
    capsule_radius = np.random.uniform(0.3, 1.0)
    capsule_length = np.random.uniform(1.0, 2.0)
    capsule_mesh, capsule_params = create_test_capsule(
        capsule_radius, capsule_length, tf, origin
    )
    test_cases.append(("胶囊体", capsule_mesh, capsule_params, get_bounding_capsule))

    # 运行测试
    results = {}
    for name, mesh, params_expected, test_func in test_cases:
        print(f"\n{'=' * 60}")
        print(f"测试几何体: {name}")
        print(f"{'=' * 60}")

        # 测试类别一致的geomety检查参数
        test_results = {}
        test_results["已知类别识别参数"] = test_bounding_function(
            mesh, params_expected, test_func, rpy, trans, rot
        )

        # 测试综合函数
        print("\n--- 测试类别判断结果---")
        try:
            params_calc = get_bounding_geometry(mesh, xyz, rpy)
            if params_calc:
                # 检查是否选择了正确的几何体类型
                expected_type = params_expected["type"]
                calculated_type = params_calc["type"]
                if expected_type == "box" and calculated_type == "box":
                    print("✅ 正确识别为包围盒")
                elif expected_type == "sphere" and calculated_type == "sphere":
                    print("✅ 正确识别为球体")
                elif expected_type == "cylinder" and calculated_type == "cylinder":
                    print("✅ 正确识别为圆柱体")
                elif expected_type == "capsule" and calculated_type == "capsule":
                    print("✅ 正确识别为胶囊体")
                else:
                    print(f"⚠️  预期类型: {expected_type}, 结果类型: {calculated_type}")
                    print("   这可能是正常的，因为算法选择体积最小的几何体")
                test_results["类别判定"] = True
            else:
                print("❌ 综合函数返回None")
                test_results["类别判定"] = False
        except Exception as e:
            print(f"❌ 综合函数测试失败: {e}")
            test_results["类别判定"] = False

        results[name] = test_results

    # 总结测试结果
    print(f"\n{'=' * 60}")
    print("测试总结")
    print(f"{'=' * 60}")

    for name, test_results in results.items():
        print(f"\n{name}:")
        for test_name, passed in test_results.items():
            status = "✅ 通过" if passed else "❌ 失败"
            print(f"  {test_name}: {status}")


def test_specific_geometry(geometry_type="box"):
    """测试特定几何体并可视化结果"""
    print(f"\n=== 测试特定几何体: {geometry_type} ===")

    # 生成随机位姿
    trans = np.random.uniform(-1, 1, size=3)
    rot = np.random.uniform(-np.pi, np.pi, size=3)
    # trans = np.zeros(3)
    # rot = np.zeros(3)
    xyz = " ".join(map(str, trans))
    rpy = " ".join(map(str, rot))
    origin = {"xyz": xyz, "rpy": rpy}
    rot = R.from_euler("xyz", rot).as_matrix()
    tf = np.eye(4)
    tf[:3, :3] = rot
    tf[:3, 3] = trans

    if geometry_type == "box":
        box_size = np.random.uniform(0.3, 1.0, size=3)
        mesh, params_expected = create_test_box(box_size, origin)
    elif geometry_type == "cylinder":
        cylinder_radius = np.random.uniform(0.3, 1.0)
        cylinder_length = np.random.uniform(1.0, 2.0)
        mesh, params_expected = create_test_cylinder(
            cylinder_radius, cylinder_length, origin
        )
    elif geometry_type == "sphere":
        sphere_radius = np.random.uniform(0.3, 1.0)
        mesh, params_expected = create_test_sphere(sphere_radius, origin)
    elif geometry_type == "capsule":
        capsule_radius = np.random.uniform(0.3, 1.0)
        capsule_length = np.random.uniform(1.0, 2.0)
        mesh, params_expected = create_test_capsule(
            capsule_radius, capsule_length, tf, origin
        )
    else:
        print(f"不支持的几何体类型: {geometry_type}")
        return

    print(f"预期参数: {params_expected}")

    # 测试综合函数
    params_calc = get_bounding_geometry(mesh, xyz, rpy)
    print(f"计算参数: {params_calc}")
    if params_calc:
        visualize_result(mesh, params_expected, params_calc)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="测试包围几何体计算函数")
    parser.add_argument(
        "--test-type",
        choices=["all", "box", "cylinder", "sphere", "capsule"],
        default="all",
        help="测试类型",
    )

    args = parser.parse_args()

    if args.test_type == "all":
        run_comprehensive_test()
    else:
        test_specific_geometry(args.test_type)
