import csv as csvmod
import shutil
from collections import OrderedDict
from pathlib import Path


def format_template(output_path, output_pkg_name, output_macro_name):
    """
    复制模板包到指定位置并修改包名称
    """
    # 使用相对路径找到模板目录
    script_dir = Path(__file__).parent.parent
    template_dir = script_dir / "package_template"

    if not template_dir.exists():
        raise RuntimeError(f"模板目录不存在: {template_dir}")

    output_dir = Path(output_path) / output_pkg_name

    # 复制整个模板目录
    if output_dir.exists():
        raise RuntimeError(f"输出目录已存在: {output_dir}")

    shutil.copytree(template_dir, output_dir)
    print(f"已复制模板到: {output_dir}")

    # 修改xacro文件名称
    xacro_file = output_dir / "urdf" / "macro_template.xacro"
    xacro_file_new = xacro_file.with_name(output_macro_name + ".xacro")
    xacro_file.rename(xacro_file_new)
    xacro_urdf_file = output_dir / "urdf" / "macro_template.urdf.xacro"
    xacro_urdf_file_new = xacro_urdf_file.with_name(output_macro_name + ".urdf.xacro")
    xacro_urdf_file.rename(xacro_urdf_file_new)
    transmission_file = output_dir / "urdf" / "macro_template_transmission.xacro"
    transmission_file_new = transmission_file.with_name(
        output_macro_name + "_transmission.xacro"
    )
    transmission_file.rename(transmission_file_new)

    # 修改文件中的包名称
    files_to_modify = [
        output_dir / "launch" / "display.launch",
        output_dir / "CMakeLists.txt",
        output_dir / "package.xml",
        xacro_file_new,
        xacro_urdf_file_new,
        transmission_file_new,
    ]
    for file_path in files_to_modify:
        content = file_path.read_text(encoding="utf-8")

        content = content.replace("package_template_name", output_pkg_name)
        content = content.replace("macro_template_name", output_macro_name)

        file_path.write_text(content, encoding="utf-8")


def format_links_and_joints(input_pkg_path, output_pkg_path, output_macro_name):
    input_pkg_path = Path(input_pkg_path)
    output_pkg_path = Path(output_pkg_path)
    # * 找到输入包中的CSV文件
    urdf_dir = input_pkg_path / "urdf"
    csv_files = list(urdf_dir.glob("*.csv"))
    if not csv_files:
        raise RuntimeError("输入包中未找到CSV文件")

    # * 找到root link并修改xacro_file
    # 找到parent列
    csv_path = csv_files[0]
    content = csv_path.read_text(encoding="utf-8")
    lines = content.strip().split("\n")
    reader = csvmod.reader(lines)
    header = next(reader)  # 获取表头
    parent_idx = None
    for i, h in enumerate(header):
        if "parent" in h.lower():
            parent_idx = i
            break
    if parent_idx is None:
        raise RuntimeError("csv文件未找到Parent列")
    # 找到parent列中第一个为空的link作为root link
    root_link_name = ""
    for row in reader:
        if not row[parent_idx].strip():
            root_link_name = row[0].strip()
            break
    if not root_link_name:
        raise RuntimeError("未找到root link")
    # 进行替换
    xacro_file = output_pkg_path / "urdf" / f"{output_macro_name}.xacro"
    xacro_content = xacro_file.read_text(encoding="utf-8")
    xacro_content = xacro_content.replace(
        "root_link_template_name", output_pkg_path.name
    )
    xacro_file.write_text(xacro_content, encoding="utf-8")

    # * 添加link和joint

    # 读取CSV文件内容
    reader = csvmod.reader(lines)
    header = next(reader)  # 获取表头

    # 获取列索引
    def get_column_indices(names):
        indices = OrderedDict()
        for name in names:
            for i, h in enumerate(header):
                if name.lower() == h.lower():
                    indices[name] = i
        return indices

    names_single = [
        "link name",
        "joint name",
        "joint type",
        "parent",
    ]
    names_limits = [
        "limit lower",
        "limit upper",
        "limit effort",
        "limit velocity",
    ]
    names_triple = {
        "xyz": ["joint origin x", "joint origin y", "joint origin z"],
        "rpy": ["joint origin roll", "joint origin pitch", "joint origin yaw"],
        "axis": ["joint axis x", "joint axis y", "joint axis z"],
    }
    indices_single = get_column_indices(names_single + names_limits)
    indices_triple = {k: get_column_indices(v) for k, v in names_triple.items()}

    # 收集所有链接和关节信息
    links = []
    joints = []

    # 检查必需的列索引
    if indices_single["link name"] is None:
        raise RuntimeError("csv文件未找到Link Name列")

    # 重新读取CSV数据
    reader = csvmod.reader(lines)
    next(reader)  # 跳过表头

    for row in reader:
        if (
            len(row) > indices_single["link name"]
            and row[indices_single["link name"]].strip()
        ):
            link_name = row[indices_single["link name"]].strip()
            links.append(link_name)
            if row[indices_single["joint name"]]:
                try:
                    joint_info = {"child": link_name}
                    for name in names_single + names_limits:
                        joint_info[name] = row[indices_single[name]].strip()
                    for name, ind_ordered_dict in indices_triple.items():
                        tri_data = [row[idx] for _, idx in ind_ordered_dict.items()]
                        tri_data = " ".join(map(str, tri_data))
                        joint_info[name] = tri_data
                except Exception as e:
                    raise RuntimeError(f"处理关节 {link_name} 时出错: {e}")
                joints.append(joint_info)

    # 读取现有的xacro文件内容
    xacro_content = xacro_file.read_text(encoding="utf-8")

    # 生成links的xacro代码
    links_code = []
    for link in links:
        links_code.append(
            f'    <xacro:config_link prefix="${{prefix}}" name="{link}" sim="${{sim}}" />'
        )

    # 生成joints的xacro代码
    joints_code = []
    joints_transmission = []
    for joint_info in joints:
        if not joint_info["joint type"]:  # 如果没有关节类型，跳过
            raise RuntimeError(
                f"关节 {joint_info['joint name']} 没有指定类型，请检查CSV文件"
            )
        joint_xacro = []
        joint_xacro.append(
            f'    <joint name="${{prefix}}{joint_info["joint name"]}" type="{joint_info["joint type"]}">'
        )
        joint_xacro.append(
            f'      <origin xyz="{joint_info["xyz"]}" rpy="{joint_info["rpy"]}" />'
        )
        joint_xacro.append(f'      <parent link="${{prefix}}{joint_info["parent"]}" />')
        joint_xacro.append(f'      <child link="${{prefix}}{joint_info["child"]}" />')
        # 为continuous、revolute和prismatic类型添加axis
        if joint_info["joint type"].lower() in ["continuous", "revolute", "prismatic"]:
            joint_xacro.append(f'      <axis xyz="{joint_info["axis"]}" />')
        # 为revolute和prismatic类型添加limit
        if joint_info["joint type"].lower() in ["revolute", "prismatic"]:
            limit_parts = []
            if joint_info["limit lower"] == joint_info["limit upper"]:
                print(
                    f"\033[31mWARN: {joint_info['joint name']}关节没有限位信息，配置-pi~pi给旋转关节，0~0.1给移动关节\033[0m"
                )
                if joint_info["joint type"].lower() == "revolute":
                    limit_parts.append('lower="-3.14159"')
                    limit_parts.append('upper="3.14159"')
                elif joint_info["joint type"].lower() == "prismatic":
                    limit_parts.append('lower="0"')
                    limit_parts.append('upper="0.1"')
                joint_xacro.append(
                    "      <!-- WARN: No limit info, using default but not tested values ! -->"
                )
                names_limits.remove("limit lower")
                names_limits.remove("limit upper")
            if joint_info["limit effort"] == "0":
                print(
                    f"\033[31mWARN: {joint_info['joint name']}关节没有限位力信息，配置为1000\033[0m"
                )
                limit_parts.append('effort="1000"')
                names_limits.remove("limit effort")
            if joint_info["limit velocity"] == "0":
                print(
                    f"\033[31mWARN: {joint_info['joint name']}关节没有限位速度信息，配置为100\033[0m"
                )
                limit_parts.append('velocity="100"')
                names_limits.remove("limit velocity")
            for name in names_limits:
                if joint_info[name]:
                    limit_parts.append(
                        f'{name.rsplit(" ", 1)[-1]}="{joint_info[name]}"'
                    )
            joint_xacro.append(f"      <limit {' '.join(limit_parts)} />")
        joint_xacro.append("    </joint>")
        joints_code.extend(joint_xacro)
        joints_transmission.append(
            f'<xacro:config_transmission joint_name="${{prefix}}{joint_info["joint name"]}" interface_name="${{interface_name}}" />'
        )

    # 将生成的代码插入到xacro文件中
    # 找到注释结束的位置并在其后插入新代码
    lines = xacro_content.split("\n")
    insert_pos = -1
    for i, line in enumerate(lines):
        if "<!-- auto generated urdf below -->" in line:
            insert_pos = i + 1
            break
    # 如果找不到注释结束位置，在</xacro:macro>前插入
    if insert_pos == -1:
        for i, line in enumerate(lines):
            if "</xacro:macro>" in line:
                insert_pos = i
                break
    if insert_pos == -1:
        raise RuntimeError("未找到插入位置，请检查template xacro文件格式")

    # 插入links代码
    for link_code in links_code:
        lines.insert(insert_pos, link_code)
        insert_pos += 1

    # 插入joints代码
    for joint_code in joints_code:
        lines.insert(insert_pos, joint_code)
        insert_pos += 1

    # 写回文件
    xacro_content = "\n".join(lines)
    xacro_file.write_text(xacro_content, encoding="utf-8")
    
    # 插入transmission代码
    transmission_file = output_pkg_path / "urdf" / f"{output_macro_name}_transmission.xacro"
    lines = transmission_file.read_text(encoding="utf-8").split("\n")
    insert_pos = -1
    for i, line in enumerate(lines):
        if "<!-- auto generated urdf below -->" in line:
            insert_pos = i + 1
            break
    if insert_pos == -1:
        for i, line in enumerate(lines):
            if "</xacro:macro>" in line:
                insert_pos = i
                break
    if insert_pos == -1:
        raise RuntimeError("未找到插入位置，请检查transmission template xacro文件格式")
    for transmission_code in joints_transmission:
        lines.insert(insert_pos, transmission_code)
        insert_pos += 1
    transmission_content = "\n".join(lines)
    transmission_file.write_text(transmission_content, encoding="utf-8")

    print("已完成link和joint的格式化！")
