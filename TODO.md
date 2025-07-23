## solidworks自动导出urdf格式清洗

流程：

1. 命令行指定**输出路径**，**输出包名称**和**输入的escription package的路径**，输入的package应有类似如下的格式（sw插件输出格式）
    ```
    robot_description_pkg/
    ├── urdf/
    │   ├── robot_name.urdf
    │   ├── robot_name.csv
    │   └── robot_name_inertial.yaml
    ├── meshes/
    │   ├── your_robot_base_link.STL
    │   └── ... 
    ├── launch/
    ├── config/
    ├── package.xml
    └── CMakeLists.txt
    ```
    检查关键文件是否存在，包括上述示例中的`robot_name.csv`、`robot_name_inertial.yaml`（名字不一定相同，主要检查后缀）以及各个csv中各个link同名的STL mesh
2. 根据输出包名称，将`/workspace/src/urdf_utils/package_template`复制到指定位置，并将对应`launch/display.launch`、`CMakelists.txt`和`package.xml`中的"package_template_name"修改为输出包名称
3. 使用`/workspace/src/urdf_utils/urdf_utils/extract_inertial_visual.py`中的extract_visual_info_from_csv方法将输入包中的视觉和惯量信息提取到输出包下的`config`文件夹
4. 使用/workspace/src/urdf_utils/urdf_utils/simplify_collision.py中的方法提取碰撞信息到输出包下的`config`文件夹，并复制对应mesh
5. 根据csv中的关节信息，结合utils.xacro，生成.xacro和urdf.xacro文件