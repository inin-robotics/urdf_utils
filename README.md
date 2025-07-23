# URDF处理工具

## TODO

- [ ] 待python模版完成后，将虚拟环境整合到项目中

## SW导出格式清洗

SW输出的缺少简化mesh用于仿真和自碰撞检测，本包的目的即在于添加两种简化程度的mesh，并通过macro提高可读性。使用步骤：

1. 使用SW的urdf插件将装配体转换为urdf package，然后将package应有类似如下的格式（sw插件输出格式）
    ```
    robot_description_pkg/
    ├── urdf/
    │   ├── robot_name.urdf
    │   ├── robot_name.csv  # 元数据，应该包含各种名称和惯性、视觉、碰撞等很多属性
    │   └── robot_name_inertial.yaml
    ├── meshes/
    │   ├── your_robot_base_link.STL
    │   └── ... 
    ├── launch/
    ├── config/
    ├── package.xml
    └── CMakeLists.txt
    ```
2. 进入当前项目文件夹，然后运行
  ```shell
  python -m urdf_utils.format_urdf_from_sw --output_path <输出路径> --output_pkg_name <输出包名称> --output_pkg_name <输出包名称> --output_macro_name <输出机器人macro名称>
  ```
3. 因为本包还没有充分测试，最好再人工核对以下各个信息

## 包围几何体模块

### 使用脚本测试包围集合体算法
```bash
cd /workspace/src/urdf_utils/test
./run_test.sh
```