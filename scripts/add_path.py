import omni.usd
from pxr import UsdGeom, Gf
import math

stage = omni.usd.get_context().get_stage()

root_path = "/World/DebugTrajectory"

# 删除旧路径
if stage.GetPrimAtPath(root_path):
    stage.RemovePrim(root_path)

UsdGeom.Xform.Define(stage, root_path)

# 路径参数（必须与 C++ launch 一致）
x_start = 0.0
x_end = 200.0
dx = 0.5
amplitude = 3.0
frequency = 0.1

i = 0
x = x_start
points = []

while x <= x_end:
    y_offset = 2.7581
    y = y_offset + amplitude * math.sin(frequency * x)
    z = 0.3
    points.append((x, y, z))
    x += dx

for i, (x, y, z) in enumerate(points):
    sphere_path = f"{root_path}/pt_{i}"
    sphere = UsdGeom.Sphere.Define(stage, sphere_path)
    sphere.GetRadiusAttr().Set(0.12)

    # 默认红色
    color = Gf.Vec3f(1.0, 0.0, 0.0)

    # 起点绿色
    if i == 0:
        color = Gf.Vec3f(0.0, 1.0, 0.0)

    # 终点蓝色
    elif i == len(points) - 1:
        color = Gf.Vec3f(0.0, 0.0, 1.0)

    sphere.GetDisplayColorAttr().Set([color])

    prim = stage.GetPrimAtPath(sphere_path)
    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

print(f"Created {len(points)} path markers under {root_path}")
print("Start point = green, middle points = red, end point = blue")




