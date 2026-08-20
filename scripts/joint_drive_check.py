"""
检查关节驱动配置 (Isaac Sim 4.5 修复版)
将结果保存到文件
"""
import omni.usd
from pxr import UsdPhysics
import os
from datetime import datetime

# 定义输出文件路径
output_dir = "/home/auto/isaacsim4.5/issacsim_excavator"
output_file = os.path.join(output_dir, "joint_drive_check.txt")

# 确保目录存在
os.makedirs(output_dir, exist_ok=True)

# 获取 stage
stage = omni.usd.get_context().get_stage()

joint_paths = [
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/wheel_joint/FRwheel_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/wheel_joint/FLwheel_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/wheel_joint/RRwheel_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/wheel_joint/RLwheel_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/chassis_joint/chassis_body_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/boom_joint/boom_pivot_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/bucket_joint/bucket_pivot_revolute",
]

# 准备输出内容
output_lines = []

def add_line(text=""):
    """添加一行到输出"""
    output_lines.append(text)
    print(text)

# 添加报告头部
add_line("=" * 60)
add_line("关节驱动配置检查报告")
add_line("=" * 60)
add_line(f"检查时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
add_line(f"Stage: {stage.GetRootLayer().identifier if stage else 'None'}")
add_line("=" * 60)

# 统计变量
valid_joints = []
joint_summary = {}

for path in joint_paths:
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        add_line(f"\n✗ 不存在: {path}")
        continue
    
    valid_joints.append(path)
    name = prim.GetName()
    
    # 检查关节类型
    joint_type = prim.GetTypeName()
    
    # 获取关节属性
    attrs = {}
    
    # UsdPhysics Drive 属性 (标准 USD Physics)
    # 对于旋转关节 (RevoluteJoint)
    if prim.HasAttribute("drive:angular:physics:damping"):
        attrs["damping"] = prim.GetAttribute("drive:angular:physics:damping").Get()
    else:
        attrs["damping"] = None
    
    if prim.HasAttribute("drive:angular:physics:stiffness"):
        attrs["stiffness"] = prim.GetAttribute("drive:angular:physics:stiffness").Get()
    else:
        attrs["stiffness"] = None
    
    if prim.HasAttribute("drive:angular:physics:maxForce"):
        attrs["maxForce"] = prim.GetAttribute("drive:angular:physics:maxForce").Get()
    else:
        attrs["maxForce"] = None
    
    if prim.HasAttribute("drive:angular:physics:targetPosition"):
        attrs["targetPosition"] = prim.GetAttribute("drive:angular:physics:targetPosition").Get()
    else:
        attrs["targetPosition"] = None
    
    if prim.HasAttribute("drive:angular:physics:targetVelocity"):
        attrs["targetVelocity"] = prim.GetAttribute("drive:angular:physics:targetVelocity").Get()
    else:
        attrs["targetVelocity"] = None
    
    # 保存到摘要
    joint_summary[name] = {
        "path": path,
        "type": joint_type,
        "stiffness": attrs["stiffness"],
        "damping": attrs["damping"],
        "maxForce": attrs["maxForce"],
        "targetPosition": attrs["targetPosition"],
        "targetVelocity": attrs["targetVelocity"]
    }
    
    # 显示信息
    add_line(f"\n{name} ({joint_type}):")
    add_line(f"  路径: {path}")
    
    if attrs["stiffness"] is not None:
        add_line(f"  stiffness: {attrs['stiffness']}")
    else:
        add_line(f"  stiffness: 未设置")
    
    if attrs["damping"] is not None:
        add_line(f"  damping: {attrs['damping']}")
    else:
        add_line(f"  damping: 未设置")
    
    if attrs["maxForce"] is not None:
        add_line(f"  maxForce: {attrs['maxForce']}")
    else:
        add_line(f"  maxForce: 未设置")
    
    # 判断驱动模式
    if attrs["stiffness"] is not None and attrs["damping"] is not None:
        if attrs["stiffness"] > 0:
            add_line(f"  → 驱动模式: 位置控制 (stiffness={attrs['stiffness']})")
        elif attrs["damping"] > 0:
            add_line(f"  → 驱动模式: 速度控制 (damping={attrs['damping']})")
        else:
            add_line(f"  → 驱动模式: 无驱动 (stiffness=0, damping=0)")
    else:
        add_line(f"  → 驱动模式: 未配置")
    
    # 列出所有属性 (调试用)
    drive_attrs = []
    for attr in prim.GetAttributes():
        attr_name = attr.GetName()
        if "drive" in attr_name.lower():
            value = attr.Get()
            drive_attrs.append(f"    {attr_name} = {value}")
    
    if drive_attrs:
        add_line(f"  所有驱动相关属性:")
        for attr_line in drive_attrs:
            add_line(attr_line)

# 添加统计摘要
add_line("\n" + "=" * 60)
add_line("统计摘要")
add_line("=" * 60)

total_joints = len(joint_paths)
valid_count = len(valid_joints)
invalid_count = total_joints - valid_count

add_line(f"总关节数: {total_joints}")
add_line(f"有效关节数: {valid_count}")
add_line(f"无效关节数: {invalid_count}")

# 统计有驱动的关节
with_drive = []
for name, info in joint_summary.items():
    if info["stiffness"] is not None and info["damping"] is not None:
        if info["stiffness"] > 0 or info["damping"] > 0:
            with_drive.append(name)

add_line(f"\n有驱动配置的关节 ({len(with_drive)}):")
for name in with_drive:
    info = joint_summary[name]
    if info["stiffness"] > 0:
        add_line(f"  - {name}: 位置控制 (K={info['stiffness']}, D={info['damping']})")
    else:
        add_line(f"  - {name}: 速度控制 (D={info['damping']})")

# 统计无驱动的关节
without_drive = [name for name in joint_summary.keys() if name not in with_drive]
if without_drive:
    add_line(f"\n无驱动配置的关节 ({len(without_drive)}):")
    for name in without_drive:
        add_line(f"  - {name}: 需要添加驱动")

# 生成 ActionGraph 配置建议
add_line("\n" + "=" * 60)
add_line("ActionGraph 配置建议")
add_line("=" * 60)

add_line("\n关节名称列表 (用于 ROS2SubscribeJointState):")
add_line("jointNames = [")
for name in joint_summary.keys():
    add_line(f'    "{name}",')
add_line("]")

# 检查是否需要添加驱动
add_line("\n" + "=" * 60)
if without_drive:
    add_line("⚠️  警告: 以下关节缺少驱动配置，将无法通过 ArticulationController 控制:")
    for name in without_drive:
        add_line(f"  - {name}")
    add_line("\n建议运行以下代码为关节添加驱动:")
    add_line("from pxr import PhysxSchema")
    add_line()
    for name in without_drive:
        info = joint_summary[name]
        add_line(f"# 为 {name} 添加驱动")
        add_line(f"prim = stage.GetPrimAtPath('{info['path']}')")
        add_line(f"drive = PhysxSchema.DriveAPI.Apply(prim, 'angular')")
        add_line(f"drive.GetStiffnessAttr().Set(1000000.0)  # 刚度")
        add_line(f"drive.GetDampingAttr().Set(100000.0)     # 阻尼")
        add_line(f"drive.GetMaxForceAttr().Set(1000000.0)   # 最大力")
        add_line()
else:
    add_line("✅ 所有关节都已配置驱动，可以正常控制。")

# 添加关节路径列表供参考
add_line("\n" + "=" * 60)
add_line("关节路径列表 (供参考)")
add_line("=" * 60)
for name, info in joint_summary.items():
    add_line(f"{name}: {info['path']}")

# 添加时间戳
add_line("\n" + "=" * 60)
add_line(f"报告生成完成: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
add_line("=" * 60)

# 写入文件
with open(output_file, 'w', encoding='utf-8') as f:
    f.write("\n".join(output_lines))

print()
print("=" * 60)
print(f"✅ 关节驱动配置检查报告已保存到:")
print(f"   {output_file}")
print("=" * 60)

# 输出关键信息摘要
print()
print("📋 快速摘要:")
print("-" * 40)
print(f"总关节数: {total_joints}")
print(f"有效关节数: {valid_count}")
print(f"有驱动的关节: {len(with_drive)}")
print(f"无驱动的关节: {len(without_drive)}")
if without_drive:
    print()
    print("⚠️  需要添加驱动的关节:")
    for name in without_drive:
        print(f"  - {name}")