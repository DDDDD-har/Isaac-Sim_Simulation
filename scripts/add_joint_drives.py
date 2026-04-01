"""
为动臂和铲斗关节添加驱动配置
"""
import omni.usd
from pxr import Sdf

stage = omni.usd.get_context().get_stage()

# 要配置的关节
joints_to_configure = {
    "boom_pivot_revolute": {
        "path": "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/boom_joint/boom_pivot_revolute",
        "stiffness": 100000.0,   # 较高刚度，适合位置控制
        "damping": 10000.0,      # 适中阻尼，避免震荡
        "maxForce": 1e10,        # 足够大的力
    },
    "bucket_pivot_revolute": {
        "path": "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/bucket_joint/bucket_pivot_revolute",
        "stiffness": 100000.0,
        "damping": 10000.0,
        "maxForce": 1e10,
    },
}

print("=" * 60)
print("为动臂和铲斗关节添加驱动")
print("=" * 60)

def set_drive_attributes(prim, stiffness, damping, max_force):
    """设置关节驱动属性"""
    # stiffness
    attr = prim.GetAttribute("drive:angular:physics:stiffness")
    if not attr:
        attr = prim.CreateAttribute("drive:angular:physics:stiffness", Sdf.ValueTypeNames.Float)
    attr.Set(float(stiffness))
    
    # damping
    attr = prim.GetAttribute("drive:angular:physics:damping")
    if not attr:
        attr = prim.CreateAttribute("drive:angular:physics:damping", Sdf.ValueTypeNames.Float)
    attr.Set(float(damping))
    
    # maxForce
    attr = prim.GetAttribute("drive:angular:physics:maxForce")
    if not attr:
        attr = prim.CreateAttribute("drive:angular:physics:maxForce", Sdf.ValueTypeNames.Float)
    attr.Set(float(max_force))
    
    # targetPosition (初始化为 0)
    attr = prim.GetAttribute("drive:angular:physics:targetPosition")
    if not attr:
        attr = prim.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float)
    attr.Set(0.0)
    
    # targetVelocity (初始化为 0)
    attr = prim.GetAttribute("drive:angular:physics:targetVelocity")
    if not attr:
        attr = prim.CreateAttribute("drive:angular:physics:targetVelocity", Sdf.ValueTypeNames.Float)
    attr.Set(0.0)
    
    # drive type
    attr = prim.GetAttribute("drive:angular:physics:type")
    if not attr:
        attr = prim.CreateAttribute("drive:angular:physics:type", Sdf.ValueTypeNames.Token)
    attr.Set("force")

for name, config in joints_to_configure.items():
    print(f"\n配置 {name}:")
    
    prim = stage.GetPrimAtPath(config["path"])
    
    if not prim or not prim.IsValid():
        print(f"  ✗ 错误: 关节不存在 {config['path']}")
        continue
    
    # 设置驱动属性
    set_drive_attributes(
        prim,
        config["stiffness"],
        config["damping"],
        config["maxForce"]
    )
    
    print(f"  ✓ 已添加驱动:")
    print(f"    stiffness = {config['stiffness']}")
    print(f"    damping = {config['damping']}")
    print(f"    maxForce = {config['maxForce']}")

print("\n" + "=" * 60)
print("✓ 配置完成!")
print("=" * 60)
print("\n下一步:")
print("  1. 保存场景 (Ctrl+S)")
print("  2. Stop 并重新 Play")
print("  3. 运行键盘控制程序")
print("=" * 60)