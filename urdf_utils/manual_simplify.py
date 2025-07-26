import numpy as np
import open3d as o3d  # type: ignore
import yaml  # type: ignore
from open3d.visualization import O3DVisualizer  # type: ignore
from scipy.spatial.transform import Rotation as R  # type: ignore

if o3d.__version__ < "0.18.0":
    raise ImportError(
        "Open3D version 0.18.0 or higher is required for this script. "
        "Please update your Open3D installation."
    )


class GeomPrim:
    def __init__(self, mesh: o3d.geometry.TriangleMesh, type_name: str):
        self._transform = np.eye(4)
        self._shape_prop: dict[str, str] = {}
        self._mesh = mesh
        self._type_name = type_name
        if not self._mesh.has_vertex_normals():
            self._mesh.compute_vertex_normals()

    @staticmethod
    def tf_inv(tf):
        """Inverse of a transformation matrix."""
        inv_tf = np.eye(4)
        inv_tf[:3, :3] = tf[:3, :3].T
        inv_tf[:3, 3] = -inv_tf[:3, :3] @ tf[:3, 3]
        return inv_tf

    def inc_transform(self, tf_inc):
        tf = self._transform @ tf_inc @ self.tf_inv(self._transform)
        self._mesh.transform(tf)
        self._transform = self._transform @ tf_inc

    def inc_rotate(self, rotation_matrix):
        tf_inc = np.eye(4)
        tf_inc[:3, :3] = rotation_matrix
        self.inc_transform(tf_inc)

    def inc_translate(self, translation):
        tf_inc = np.eye(4)
        tf_inc[:3, 3] = translation
        self.inc_transform(tf_inc)

    def update_transform(self, transform):
        if not isinstance(transform, np.ndarray) or transform.shape != (4, 4):
            raise ValueError("Transform must be a 4x4 numpy array.")
        self._mesh.transform(transform)
        self._transform = transform

    def get_transform(self):
        return self._transform

    def add_properties(self, prop_dict):
        self._shape_prop.update(prop_dict)

    def get_properties(self):
        return self._shape_prop

    def get_property(self, key):
        return self._shape_prop.get(key, None)

    def as_o3d(self):
        return self._mesh

    def get_type(self):
        return self._type_name

    @staticmethod
    def _arr3_to_str(arr):
        return f"{arr[0]:.6f} {arr[1]:.6f} {arr[2]:.6f}"

    def __repr__(self):
        xyz_values = self._transform[:3, 3]
        rotation_matrix = self._transform[:3, :3]
        rpy_values = R.from_matrix(rotation_matrix).as_euler("xyz", degrees=False)
        xyz_str = self._arr3_to_str(xyz_values)
        rpy_str = self._arr3_to_str(rpy_values)

        if self._type_name == "sphere":
            radius = self.get_property("radius") or 0.0
            center_str = self._arr3_to_str(xyz_values)
            data = {"type": "sphere", "center": center_str, "radius": radius}
        elif self._type_name == "box":
            width = self.get_property("width") or 0.0
            height = self.get_property("height") or 0.0
            depth = self.get_property("depth") or 0.0
            size_str = self._arr3_to_str([width, height, depth])
            data = {
                "type": "box",
                "origin": {"rpy": rpy_str, "xyz": xyz_str},
                "size": size_str,
            }
        elif self._type_name == "cylinder":
            radius = self.get_property("radius") or 0.0
            length = self.get_property("length") or 0.0
            data = {
                "type": "cylinder",
                "origin": {"rpy": rpy_str, "xyz": xyz_str},
                "radius": radius,
                "length": length,
            }
        elif self._type_name == "capsule":
            radius = self.get_property("radius") or 0.0
            length = self.get_property("length") or 0.0
            if isinstance(length, str):
                length = float(length)
            if isinstance(radius, str):
                radius = float(radius)
            sphere1_center = xyz_values + rotation_matrix[:3, 2] * (-length / 2)
            sphere2_center = xyz_values + rotation_matrix[:3, 2] * (length / 2)
            sphere1_center_str = self._arr3_to_str(sphere1_center)
            sphere2_center_str = self._arr3_to_str(sphere2_center)
            data = {
                "type": "capsule",
                "cylinder": {
                    "origin": {"rpy": rpy_str, "xyz": xyz_str},
                    "radius": radius,
                    "length": length,
                },
                "spheres": [
                    {"center": sphere1_center_str, "radius": radius},
                    {"center": sphere2_center_str, "radius": radius},
                ],
            }
        else:
            data = {
                "type": self._type_name,
                "origin": {"rpy": rpy_str, "xyz": xyz_str},
                "properties": self._shape_prop,
            }

        return yaml.dump(data, default_flow_style=False, allow_unicode=True)


def to_geom_prim(mesh, type_name, prop_dict=None):
    if not isinstance(mesh, GeomPrim):
        mesh = GeomPrim(mesh, type_name)
    if prop_dict:
        mesh.add_properties(prop_dict)
    return mesh


def get_coordinate_frame(size):
    return to_geom_prim(
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=size), "coord"
    )


def get_sphere_prim(radius=0.05):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius)
    return to_geom_prim(sphere, "sphere", {"radius": radius})


def get_cylinder_prim(radius=0.05, length=0.1):
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius, length)
    return to_geom_prim(cylinder, "cylinder", {"radius": radius, "length": length})


def get_capsule_prim(radius=0.05, length=0.1):
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius, length)
    sphere1 = o3d.geometry.TriangleMesh.create_sphere(radius)
    sphere2 = o3d.geometry.TriangleMesh.create_sphere(radius)
    sphere1.translate(np.array([0, 0, -length / 2]))
    sphere2.translate(np.array([0, 0, length / 2]))
    capsule = cylinder + sphere1 + sphere2
    return to_geom_prim(capsule, "capsule", {"radius": radius, "length": length})


def get_box_prim(width=0.1, height=0.1, depth=0.1):
    box = o3d.geometry.TriangleMesh.create_box(width, height, depth)
    box.translate(np.array([-width / 2, -height / 2, -depth / 2]))
    return to_geom_prim(box, "box", {"width": width, "height": height, "depth": depth})


def get_prim_by_type(type_name, **kwargs):
    if type_name == "sphere":
        return get_sphere_prim(**kwargs)
    elif type_name == "cylinder":
        return get_cylinder_prim(**kwargs)
    elif type_name == "capsule":
        return get_capsule_prim(**kwargs)
    elif type_name == "box":
        return get_box_prim(**kwargs)
    else:
        raise ValueError(f"Unknown type name: {type_name}")


def get_material(shader, color=None, transparency=None):
    mat = o3d.visualization.rendering.MaterialRecord()  # type: ignore
    mat.shader = shader
    if transparency:
        if color is None:
            color = [0.5, 0.5, 0.5, transparency or 1.0]
        if len(color) == 3:
            color = np.array(color + [transparency or 1.0])
        mat.base_color = np.array(color)
        mat.base_roughness = 0.0
        mat.base_reflectance = 0.0
        mat.base_clearcoat = 1.0
        mat.thickness = 1.0
        mat.transmission = 1.0
        mat.absorption_distance = 10
        mat.absorption_color = np.array([0.5, 0.5, 0.5])
    return mat


def load_mesh_from_file(file_path):
    mesh = o3d.io.read_triangle_mesh(file_path, enable_post_processing=True)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh


def wrap_for_vis(name, mesh, shader, color=None, transparency=None):
    return {
        "name": name,
        "geometry": mesh,
        "material": get_material(shader, color, transparency),
    }


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Manual mesh simplification tool.")
    parser.add_argument(
        "--input_file",
        type=str,
        required=True,
        help="Path to the input mesh file.",
    )
    args = parser.parse_args()
    mesh_vis = wrap_for_vis(
        "target_mesh",
        load_mesh_from_file(args.input_file),
        "defaultLitTransparency",
        [0.5, 0.5, 0.5],
        transparency=0.7,
    )
    mesh_coord_vis = wrap_for_vis(
        "coord",
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2),
        "defaultUnlit",
    )
    prim_list = []
    coord_list = []
    prim_idx = -1

    def add_prim(o3dvis, prim_type):
        nonlocal prim_idx, prim_list, coord_list
        if prim_idx >= 0:
            change_current_prim_color(o3dvis, [0.5, 0.5, 0.5])
        prim = get_prim_by_type(prim_type)
        max_prim_id = 0
        for p in prim_list:
            name = p.get_property("name")
            type_name = p.get_type()
            if type_name == prim_type:
                max_prim_id = max(max_prim_id, int(name.rsplit("_")[-1]))
        prim_name = f"{prim_type}_{max_prim_id + 1}"
        prim.add_properties({"name": prim_name})
        prim_vis = wrap_for_vis(
            prim_name,
            prim.as_o3d(),
            "defaultLitTransparency",
            [0, 0.5, 0],
            transparency=0.7,
        )
        o3dvis.add_geometry(prim_vis)
        prim_list.append(prim)
        prim_coord = get_coordinate_frame(0.1)
        prim_coord.add_properties({"name": "coord_" + prim_name})
        prim_coord_vis = wrap_for_vis(
            prim_coord.get_property("name"), prim_coord.as_o3d(), "defaultUnlit"
        )
        o3dvis.add_geometry(prim_coord_vis)
        coord_list.append(prim_coord)
        prim_idx += 1

    def remove_current_prim(o3dvis):
        nonlocal prim_idx, prim_list, coord_list
        if prim_idx < 0:
            return
        o3dvis.remove_geometry(prim_list[prim_idx].get_property("name"))
        o3dvis.remove_geometry(coord_list[prim_idx].get_property("name"))
        prim_list.pop(prim_idx)
        coord_list.pop(prim_idx)
        prim_idx -= 1
        if len(prim_list) == 0:
            return
        prim_idx %= len(prim_list)
        change_current_prim_color(o3dvis, [0.0, 0.5, 0.0])

    def change_current_prim_color(o3dvis, color):
        nonlocal prim_idx, prim_list
        if prim_idx < 0 or prim_idx >= len(prim_list):
            return
        # get the properties of the current prim
        prim_ori = prim_list[prim_idx]
        prim_name = prim_ori.get_property("name")
        # remove the current geometry from the visualizer
        o3dvis.remove_geometry(prim_name)
        # add the geometry back with the new color
        prim_new = wrap_for_vis(
            prim_name,
            prim_ori.as_o3d(),
            "defaultLitTransparency",
            color=color,
            transparency=0.7,
        )
        o3dvis.add_geometry(prim_new)

    def move_to_next_prim(o3dvis):
        nonlocal prim_idx, prim_list, coord_list
        if len(prim_list) < 2:
            return
        change_current_prim_color(o3dvis, [0.5, 0.5, 0.5])
        prim_idx += 1
        prim_idx %= len(prim_list)
        change_current_prim_color(o3dvis, [0.0, 0.5, 0.0])

    def adjust_property(o3dvis, prop_name, diff):
        nonlocal prim_idx, prim_list
        if prop_name not in prim_list[prim_idx].get_properties().keys():
            print(
                f"Property '{prop_name}' not found in the current primitive '{prim_list[prim_idx].get_property('name')}'."
            )
            return
        prim_ori = prim_list[prim_idx]
        prop = prim_ori.get_properties()
        type_name = prim_ori.get_type()
        transform = prim_ori.get_transform()
        name = prop.pop("name")
        prop[prop_name] += diff
        if prop[prop_name] < 0:
            reset_value = 0.001
            print(f"Property '{prop_name}' cannot be negative. Resetting to {reset_value}")
            prop[prop_name] = reset_value
        prim_new = get_prim_by_type(type_name, **prop)
        prim_new.add_properties({"name": name})
        prim_new.update_transform(transform)
        prim_new_vis = wrap_for_vis(
            name,
            prim_new.as_o3d(),
            "defaultLitTransparency",
            color=[0.0, 0.5, 0.0],
            transparency=0.7,
        )
        o3dvis.remove_geometry(name)
        o3dvis.add_geometry(prim_new_vis)
        prim_list[prim_idx] = prim_new

    def inc_transform_prim(o3dvis, dof, val):
        nonlocal prim_idx, prim_list
        trans_keys = ["x", "y", "z"]
        rot_keys = ["rx", "ry", "rz"]
        assert dof in trans_keys or dof in rot_keys, (
            f"Invalid dof: {dof}. Must be one of {trans_keys + rot_keys}"
        )
        if prim_idx < 0 or prim_idx >= len(prim_list):
            return
        prim_cur = prim_list[prim_idx]
        coord_cur = coord_list[prim_idx]
        if dof in trans_keys:
            trans = np.zeros(3)
            trans[trans_keys.index(dof)] = val
            prim_cur.inc_translate(trans)
            coord_cur.inc_translate(trans)
        else:
            rot = np.zeros(3)
            rot[rot_keys.index(dof)] = val
            rotation_matrix = R.from_euler("xyz", rot, degrees=True).as_matrix()
            prim_cur.inc_rotate(rotation_matrix)
            coord_cur.inc_rotate(rotation_matrix)
        # have not found a way to update the visualizer without removing and adding
        name_prim_cur = prim_cur.get_property("name")
        o3dvis.remove_geometry(name_prim_cur)
        prim_cur_vis = wrap_for_vis(
            name_prim_cur,
            prim_cur.as_o3d(),
            "defaultLitTransparency",
            [0, 0.5, 0],
            transparency=0.7,
        )
        o3dvis.add_geometry(prim_cur_vis)
        name_coord_cur = coord_cur.get_property("name")
        o3dvis.remove_geometry(name_coord_cur)
        coord_cur_vis = wrap_for_vis(name_coord_cur, coord_cur.as_o3d(), "defaultUnlit")
        o3dvis.add_geometry(coord_cur_vis)

    app = o3d.visualization.gui.Application.instance  # type: ignore
    app.initialize()
    viewer = O3DVisualizer("Mesh Simplification", width=1920, height=1080)
    for panel in ["Mouse Controls", "Scene", "Properties", "Geometries"]:
        viewer.set_panel_open(panel, False)
    for geom_vis in [mesh_vis, mesh_coord_vis]:
        viewer.add_geometry(geom_vis)
    for action in [
        ("add sphere", lambda o3dvis: add_prim(o3dvis, "sphere")),
        ("add cylinder", lambda o3dvis: add_prim(o3dvis, "cylinder")),
        ("add capsule", lambda o3dvis: add_prim(o3dvis, "capsule")),
        ("add box", lambda o3dvis: add_prim(o3dvis, "box")),
        ("remove current", remove_current_prim),
        ("next one", move_to_next_prim),
        ("inc radius 1cm", lambda o3dvis: adjust_property(o3dvis, "radius", 0.01)),
        ("inc radius 1mm", lambda o3dvis: adjust_property(o3dvis, "radius", 0.001)),
        ("dec radius 1cm", lambda o3dvis: adjust_property(o3dvis, "radius", -0.01)),
        ("inc length 1cm", lambda o3dvis: adjust_property(o3dvis, "length", 0.01)),
        ("inc length 1mm", lambda o3dvis: adjust_property(o3dvis, "length", 0.001)),
        ("dec length 1cm", lambda o3dvis: adjust_property(o3dvis, "length", -0.01)),
        ("inc width 1cm", lambda o3dvis: adjust_property(o3dvis, "width", 0.01)),
        ("inc width 1mm", lambda o3dvis: adjust_property(o3dvis, "width", 0.001)),
        ("dec width 1cm", lambda o3dvis: adjust_property(o3dvis, "width", -0.01)),
        ("inc height 1cm", lambda o3dvis: adjust_property(o3dvis, "height", 0.01)),
        ("inc height 1mm", lambda o3dvis: adjust_property(o3dvis, "height", 0.001)),
        ("dec height 1cm", lambda o3dvis: adjust_property(o3dvis, "height", -0.01)),
        ("inc depth 1cm", lambda o3dvis: adjust_property(o3dvis, "depth", 0.01)),
        ("inc depth 1mm", lambda o3dvis: adjust_property(o3dvis, "depth", 0.001)),
        ("dec depth 1cm", lambda o3dvis: adjust_property(o3dvis, "depth", -0.01)),
        ("inc x 1cm", lambda o3dvis: inc_transform_prim(o3dvis, "x", 0.01)),
        ("inc x 1mm", lambda o3dvis: inc_transform_prim(o3dvis, "x", 0.001)),
        ("dec x 1cm", lambda o3dvis: inc_transform_prim(o3dvis, "x", -0.01)),
        ("inc y 1cm", lambda o3dvis: inc_transform_prim(o3dvis, "y", 0.01)),
        ("inc y 1mm", lambda o3dvis: inc_transform_prim(o3dvis, "y", 0.001)),
        ("dec y 1cm", lambda o3dvis: inc_transform_prim(o3dvis, "y", -0.01)),
        ("inc z 1cm", lambda o3dvis: inc_transform_prim(o3dvis, "z", 0.01)),
        ("inc z 1mm", lambda o3dvis: inc_transform_prim(o3dvis, "z", 0.001)),
        ("dec z 1cm", lambda o3dvis: inc_transform_prim(o3dvis, "z", -0.01)),
        ("inc rx 5deg", lambda o3dvis: inc_transform_prim(o3dvis, "rx", 5)),
        ("inc rx 1deg", lambda o3dvis: inc_transform_prim(o3dvis, "rx", 1)),
        ("dec rx 5deg", lambda o3dvis: inc_transform_prim(o3dvis, "rx", -5)),
        ("inc ry 5deg", lambda o3dvis: inc_transform_prim(o3dvis, "ry", 5)),
        ("inc ry 1deg", lambda o3dvis: inc_transform_prim(o3dvis, "ry", 1)),
        ("dec ry 5deg", lambda o3dvis: inc_transform_prim(o3dvis, "ry", -5)),
        ("inc rz 5deg", lambda o3dvis: inc_transform_prim(o3dvis, "rz", 5)),
        ("inc rz 1deg", lambda o3dvis: inc_transform_prim(o3dvis, "rz", 1)),
        ("dec rz 5deg", lambda o3dvis: inc_transform_prim(o3dvis, "rz", -5)),
    ]:
        viewer.add_action(action[0], action[1])
    app.add_window(viewer)
    app.run()

    print("=" * 18)
    print("List of primitives")
    print("=" * 18)
    for prim in prim_list:
        print(prim)
        print("-" * 18)

    confirm = input("Review the primitives? (y/n)")
    if confirm.lower() != "y":
        print("Exiting without visualization.")
        return
    vis_list = [mesh_vis]
    prim_fin_list = []
    for prim in prim_list:
        type_name = prim.get_type()
        transform = prim.get_transform()
        prop = prim.get_properties()
        prim_fin_name = prop.pop("name")
        prim_fin = get_prim_by_type(type_name, **prop)
        prim_fin.update_transform(transform)
        prim_fin_list.append(prim_fin)
        prim_vis = wrap_for_vis(
            prim_fin_name,
            prim_fin.as_o3d(),
            "defaultLitTransparency",
            [0, 0, 0.5],
            transparency=0.7,
        )
        vis_list.append(prim_vis)
    o3d.visualization.draw(vis_list)  # type: ignore


if __name__ == "__main__":
    main()
