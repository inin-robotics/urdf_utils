#!/bin/bash
# 测试包围几何体算法的运行脚本

# ! 激活虚拟环境: 根据需要修改
source /workspace/urdf/bin/activate

# 设置工作目录
cd /workspace/src/urdf_utils/test

# 检查依赖包是否安装
echo "检查Python环境和依赖包..."
python -c "import numpy; print(f'NumPy版本: {numpy.__version__}')"
python -c "import open3d; print(f'Open3D版本: {open3d.__version__}')"
python -c "import scipy; print(f'SciPy版本: {scipy.__version__}')"

echo ""
echo "=== 开始运行包围几何体测试 ==="
echo ""

# 运行综合测试
echo "运行综合测试..."
python test_bounding_geometry.py --test-type all

echo ""
echo "=== 测试完成 ==="

# 询问是否运行特定几何体测试
echo ""
echo "是否要测试特定几何体并可视化? (y/n)"
read -r response
if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
    echo "选择要测试的几何体:"
    echo "1) box (立方体)"
    echo "2) cylinder (圆柱体)" 
    echo "3) sphere (球体)"
    echo "4) capsule (胶囊体)"
    echo "请输入数字 (1-4):"
    read -r choice
    
    case $choice in
        1)
            echo "测试立方体..."
            python test_bounding_geometry.py --test-type box
            ;;
        2)
            echo "测试圆柱体..."
            python test_bounding_geometry.py --test-type cylinder
            ;;
        3)
            echo "测试球体..."
            python test_bounding_geometry.py --test-type sphere
            ;;
        4)
            echo "测试胶囊体..."
            python test_bounding_geometry.py --test-type capsule
            ;;
        *)
            echo "无效选择"
            ;;
    esac
fi

echo ""
echo "测试完成！"
