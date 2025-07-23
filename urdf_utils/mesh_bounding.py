import numpy as np
import open3d as o3d
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R


def get_bounding_box(mesh, rpy, trans_stl, rot_stl):
    vertices = np.asarray(mesh.vertices)

    # 计算包围盒的尺寸
    min_coords = np.min(vertices, axis=0)
    max_coords = np.max(vertices, axis=0)
    size = max_coords - min_coords

    # 计算体积
    volume = np.prod(size)

    # 计算中心
    center = (min_coords + max_coords) / 2
    center = rot_stl @ center + trans_stl
    xyz = " ".join(map(str, center.tolist()))

    return {
        "type": "box",
        "volume": volume,
        "size": " ".join(map(str, size)),
        "origin": {"xyz": xyz, "rpy": rpy},
    }


def get_bounding_sphere(mesh, _, trans_stl, rot_stl):
    vertices = np.asarray(mesh.vertices)

    # 计算最小包围球
    # 使用优化方法找到最小半径的球心
    center_guess = np.mean(vertices, axis=0)

    def objective_func(center):
        # 计算所有点到球心的最大距离
        distances = np.linalg.norm(vertices - center, axis=1)
        return np.max(distances)

    # 使用优化算法
    result = minimize(objective_func, center_guess, method="Nelder-Mead")

    center = result.x
    radius = result.fun

    # 验证所有点都在球内
    distances = np.linalg.norm(vertices - center, axis=1)
    if np.any(distances > radius):
        radius = np.max(distances)

    # 计算体积
    volume = (4 / 3) * np.pi * radius**3

    # 格式化中心
    center = rot_stl @ center + trans_stl
    xyz = " ".join(map(str, center.tolist()))

    radius = float(radius)
    return {
        "type": "sphere",
        "volume": volume,
        "radius": radius,
        "origin": {"xyz": xyz},
    }


def get_enclosing_circle(points):
    """
    使用 scipy.optimize.minimize 求解二维点集的最小外接圆。

    参数:
    points (numpy.ndarray): 一个 shape 为 (n, 2) 的数组，代表 n 个二维点。

    返回:
    tuple: 一个包含圆心 (x, y) 和半径 r 的元组。
    """
    # 将点集转换为 numpy 数组
    points = np.asarray(points)
    n_points = len(points)
    if n_points == 0:
        return (0, 0), 0

    # 找到所有点的中心点，作为优化的初始猜测值
    center_guess = np.mean(points, axis=0)

    # 我们要优化的函数是半径的平方 r^2。
    # 优化变量是圆心的坐标 c = (x, y)。
    def objective_func(c):
        # 计算所有点到中心 c 的距离的平方
        distances_sq = np.sum((points - c) ** 2, axis=1)
        # 半径的平方是这些距离中的最大值
        return np.max(distances_sq)

    # 使用 'Nelder-Mead' 或 'Powell' 等不需要梯度的优化算法
    result = minimize(objective_func, center_guess, method="Nelder-Mead")

    # 优化得到的最优圆心
    center = result.x
    # 对应的最小半径
    radius_sq = result.fun
    radius = np.sqrt(radius_sq)
    # 检查所有点是否在半径内
    dists = np.linalg.norm(points - center, axis=1)
    if np.any(dists > radius):
        radius = np.max(dists)

    return center, radius


def get_bounding_cylinder(mesh, _, trans_stl, rot_stl):
    vertices = np.asarray(mesh.vertices)

    cylinders = []

    # 3. 分别沿X, Y, Z轴计算圆柱体参数
    for axis_index in range(3):  # 0: X, 1: Y, 2: Z
        # 获取当前轴的投影平面（其他两个轴构成的平面）
        other_axes = [i for i in range(3) if i != axis_index]
        projected_points = vertices[:, other_axes]

        # 计算最小外接圆
        center_2d, radius = get_enclosing_circle(projected_points)

        # 获取沿当前轴的范围
        axis_coords = vertices[:, axis_index]
        min_coord = np.min(axis_coords)
        max_coord = np.max(axis_coords)
        length = max_coord - min_coord

        # 构造圆柱体中心
        center = np.zeros(3)
        center[other_axes] = center_2d
        center[axis_index] = (min_coord + max_coord) / 2
        center = rot_stl @ center + trans_stl
        xyz = " ".join(map(str, center.tolist()))

        # 计算体积
        volume = np.pi * radius**2 * length

        # 计算rpy
        if axis_index == 0:  # X轴
            rot_cylinder = R.from_euler("y", np.pi / 2).as_matrix() @ rot_stl
        elif axis_index == 1:  # Y轴
            rot_cylinder = R.from_euler("x", np.pi / 2).as_matrix() @ rot_stl
        else:  # Z轴
            rot_cylinder = rot_stl
        rpy = " ".join(map(str, R.from_matrix(rot_cylinder).as_euler("xyz")))

        radius = float(radius)
        length = float(length)
        cylinders.append(
            {
                "type": "cylinder",
                "volume": volume,
                "radius": radius,
                "length": length,
                "origin": {"xyz": xyz, "rpy": rpy},
            }
        )

    # 4. 选择体积最小的圆柱体
    best_cylinder = min(cylinders, key=lambda x: x["volume"])

    return best_cylinder


def get_approx_fixed_radius_bounding_sphere(
    end_points,
    is_max_end,
    cylinder_radius,
    cylinder_center_2d,
    axis_idx,
    other_axes_idx,
    tolerance=1e-2,
):
    """
    使用二分查找找到最优球心位置

    参数:
    - end_points: 端部的顶点
    - is_max_end: 是否为最大端（True）或最小端（False）
    - cylinder_radius: 圆柱半径R
    - cylinder_center_2d: 圆柱在投影平面的中心
    - axis_idx: 当前轴的索引
    - other_axes_idx: 其他两个轴的索引
    """
    if len(end_points) == 0:
        return None

    # 获取端部顶点在当前轴上的坐标
    axis_coords = end_points[:, axis_idx]

    if is_max_end:
        right = np.max(axis_coords)
        left = right - cylinder_radius
    else:
        left = np.min(axis_coords)
        right = left + cylinder_radius

    while (right - left) > tolerance:
        # 当前球心的轴向坐标
        mid = (left + right) / 2.0
        # 构造球心坐标
        sphere_center = np.zeros(3)
        sphere_center[other_axes_idx] = cylinder_center_2d
        sphere_center[axis_idx] = mid
        # 计算所有端部顶点到球心的距离
        if is_max_end:
            mask = end_points[:, axis_idx] >= mid
        else:
            mask = end_points[:, axis_idx] <= mid
        distances = np.linalg.norm(end_points[mask] - sphere_center, axis=1)
        max_distance = np.max(distances)

        # 判断是否有点超出圆柱半径
        if max_distance > cylinder_radius:
            # 有点超出，需要调整球心位置
            if is_max_end:
                # 对于最大端，球心需要向右移动（增大轴坐标）
                left = mid
            else:
                # 对于最小端，球心需要向左移动（减小轴坐标）
                right = mid
        else:
            # 所有点都在半径内，可以尝试优化位置
            if is_max_end:
                # 对于最大端，可以尝试向左移动（减小轴坐标）以最小化球心偏移
                right = mid
            else:
                # 对于最小端，可以尝试向右移动（增大轴坐标）以最小化球心偏移
                left = mid

    return right if is_max_end else left


def get_bounding_capsule(mesh, rpy, trans_stl, rot_stl):
    vertices = np.asarray(mesh.vertices)

    capsules = []

    # 3. 分别沿X, Y, Z轴计算胶囊体参数
    for axis_index in range(3):  # 0: X, 1: Y, 2: Z
        # I 确定胶囊体半径R及圆柱轴线位置：将所有顶点投影到圆柱底面并求其最小外接圆

        # 获取当前轴的投影平面（其他两个轴构成的平面）
        other_axes = [i for i in range(3) if i != axis_index]
        projected_points = vertices[:, other_axes]

        # 计算最外接圆半径初值，使用简化方法：最大距离的一半作为半径）
        center_2d, radius = get_enclosing_circle(projected_points)

        # II 确定两端球心位置：取出轴线两端R范围内的顶点，通过搜索或其他更好的方式确定球心位置

        # 获取沿当前轴的坐标
        axis_coords = vertices[:, axis_index]
        min_coord = np.min(axis_coords)
        max_coord = np.max(axis_coords)

        # 找到轴线两端的点
        # 对于最小端，找到在半径范围内的点，并计算其在轴向的最小值
        min_end_mask = axis_coords <= (min_coord + radius)
        min_end_points = vertices[min_end_mask]

        # 对于最大端，找到在半径范围内的点，并计算其在轴向的最大值
        max_end_mask = axis_coords >= (max_coord - radius)
        max_end_points = vertices[max_end_mask]

        # 计算球心位置

        # 计算最小端球心
        min_sphere_center_coord = get_approx_fixed_radius_bounding_sphere(
            min_end_points, False, radius, center_2d, axis_index, other_axes
        )

        # 计算最大端球心
        max_sphere_center_coord = get_approx_fixed_radius_bounding_sphere(
            max_end_points, True, radius, center_2d, axis_index, other_axes
        )

        # 如果无法计算球心，使用默认值
        if min_sphere_center_coord is None:
            print(f"警告: 无法计算最小端球心，使用默认值 {min_coord}。")
            min_sphere_center_coord = min_coord
        if max_sphere_center_coord is None:
            print(f"警告: 无法计算最大端球心，使用默认值 {max_coord}。")
            max_sphere_center_coord = max_coord

        # 构造球心坐标
        endpoint1 = np.zeros(3)
        endpoint2 = np.zeros(3)

        # 在投影平面上使用计算出的中心
        endpoint1[other_axes] = center_2d
        endpoint2[other_axes] = center_2d

        # 在轴向上设置球心位置
        endpoint1[axis_index] = min_sphere_center_coord
        endpoint2[axis_index] = max_sphere_center_coord

        # 计算圆柱长度（两个球心之间的距离）
        length = max_sphere_center_coord - min_sphere_center_coord

        # 计算圆柱中心（两个球心的中点）
        center = (endpoint1 + endpoint2) / 2

        # III 计算体积
        # 胶囊体体积 = 圆柱体积 + 两个半球体积 = π*r²*h + (4/3)*π*r³
        cylinder_volume = np.pi * radius**2 * length
        sphere_volume = (4 / 3) * np.pi * radius**3
        volume = cylinder_volume + sphere_volume

        # IV 格式化origin
        center = rot_stl @ center + trans_stl
        xyz_cylinder = " ".join(map(str, center.tolist()))
        endpoint1 = rot_stl @ endpoint1 + trans_stl
        endpoint2 = rot_stl @ endpoint2 + trans_stl
        xyz_sphere1 = " ".join(map(str, endpoint1.tolist()))
        xyz_sphere2 = " ".join(map(str, endpoint2.tolist()))

        # V 计算圆柱rpy
        if axis_index == 0:  # X轴
            rot_cylinder = R.from_euler("y", np.pi / 2).as_matrix() @ rot_stl
        elif axis_index == 1:  # Y轴
            rot_cylinder = R.from_euler("x", np.pi / 2).as_matrix() @ rot_stl
        else:  # Z轴
            rot_cylinder = rot_stl
        rpy = " ".join(map(str, R.from_matrix(rot_cylinder).as_euler("xyz")))

        # VI 整理结果
        radius = float(radius)
        length = float(length)
        capsules.append(
            {
                "type": "capsule",
                "volume": volume,
                "cylinder": {
                    "radius": radius,
                    "length": length,
                    "origin": {"xyz": xyz_cylinder, "rpy": rpy},
                },
                "spheres": [
                    {"radius": radius, "origin": {"xyz": xyz_sphere1}},
                    {"radius": radius, "origin": {"xyz": xyz_sphere2}},
                ],
            }
        )

    # 4. 比较并选择体积最小的胶囊体
    best_capsule = min(capsules, key=lambda x: x["volume"])

    return best_capsule


def get_bounding_geometry(mesh, xyz, rpy):
    if isinstance(mesh, str):
        stl_path = mesh
        try:
            mesh = o3d.io.read_triangle_mesh(stl_path)
            if not mesh.has_vertices():
                print(f"错误: 无法从 {stl_path} 加载模型或模型没有顶点。")
                return None
        except Exception as e:
            print(f"加载STL文件时出错: {e}")
            return None
    trans_stl = list(map(float, xyz.split()))
    rot_stl = R.from_euler("xyz", list(map(float, rpy.split()))).as_matrix()
    funcs = [
        get_bounding_box,
        get_bounding_cylinder,
        get_bounding_sphere,
        get_bounding_capsule,
    ]
    geometries = []
    for func in funcs:
        try:
            geometries.append(func(mesh, rpy, trans_stl, rot_stl))
        except Exception as e:
            print(f"计算 {func.__name__} 时出错: {e}")
    if not geometries:
        print("错误: 无法计算任何包围几何体")
        return None
    best_geom = min(geometries, key=lambda x: x["volume"])
    best_geom.pop("volume")
    best_type = best_geom.pop("type")
    return {best_type: best_geom}


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="获取STL模型的包围几何体")
    parser.add_argument("--input_stl_path", required=True, help="输入STL文件路径")
    parser.add_argument("--xyz", help="STL模型的位移 (x y z)", default="0 0 0")
    parser.add_argument("--rpy", help="STL模型的旋转 (roll pitch yaw)", default="0 0 0")
    args = parser.parse_args()
    res = get_bounding_geometry(args.input_stl_path, args.xyz, args.rpy)
    print(res)
