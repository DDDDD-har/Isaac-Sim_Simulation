# ------------------------------------------------------------
#  在当前Isaac Sim场景中获取：
#   1. 质心(base_link)位置与yaw_rear
#   2. 铰链中心位置 = base_link 前向 L1 米
#   3. 前体朝向 yaw_front = yaw_rear + δ
#  并在 /World/Debug 下生成可视化参考球体
# ------------------------------------------------------------

import omni.usd
from pxr import UsdGeom, Gf
import math

# === 配置路径，根据你的场景调整 ===
base_prim_path   = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/base_link"
front_body_path  = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/front_body"

# base_link 到 铰链中心的距离（根据模型测量调整）
L1 = 1.3

# === 获取 USD Stage ===
stage = omni.usd.get_context().get_stage()

base_prim  = stage.GetPrimAtPath(base_prim_path)
front_prim = stage.GetPrimAtPath(front_body_path)

if not base_prim or not front_prim:
    print("❌ 无法找到 prim，请检查路径是否正确。")
else:
    base_xform  = UsdGeom.Xformable(base_prim)
    front_xform = UsdGeom.Xformable(front_prim)

    base_world  = base_xform.ComputeLocalToWorldTransform(0)
    front_world = front_xform.ComputeLocalToWorldTransform(0)

    # --------------------------------------------------------
    # 函数：从4x4矩阵提取yaw角
    # --------------------------------------------------------
    def get_yaw_from_matrix(mat):
        # x轴在世界坐标下的方向
        x_axis = mat.TransformDir(Gf.Vec3d(1, 0, 0))
        # yaw = atan2(y成分, x成分)
        yaw = math.atan2(x_axis[1], x_axis[0])
        return yaw

    # 获取 yaw_rear 与 yaw_front
    yaw_rear  = get_yaw_from_matrix(base_world)
    yaw_front = get_yaw_from_matrix(front_world)
    delta     = yaw_front - yaw_rear

    # 提取 base_link 世界坐标
    base_pos = base_world.ExtractTranslation()

    # 铰链中心坐标 = base_link + L1 * (cos(yaw_rear), sin(yaw_rear))
    hinge_x = base_pos[0] + L1 * math.cos(yaw_rear)
    hinge_y = base_pos[1] + L1 * math.sin(yaw_rear)
    hinge_z = base_pos[2]

    # --------------------------------------------------------
    # 打印调试信息
    # --------------------------------------------------------
    print("========== 当前装载机几何信息 ==========")
    print(f"质心 base_link 坐标 : ({base_pos[0]:.3f}, {base_pos[1]:.3f}, {base_pos[2]:.3f})")
    print(f"rear_yaw (base_link 朝向 φ后) : {yaw_rear:.3f} rad")
    print(f"铰链中心坐标 : ({hinge_x:.3f}, {hinge_y:.3f}, {hinge_z:.3f}) [L1 = {L1} m]")
    print(f"front_yaw (前车体 φ前) : {yaw_front:.3f} rad")
    print(f"铰接角 δ = φ前 - φ后 = {delta:.3f} rad")
    print("=========================================")
     # --------------------------------------------------------
    # 可视化关键点
    # --------------------------------------------------------
    """
    def draw_sphere(name, pos, color):
        path = f"/World/Debug/{name}"
        if stage.GetPrimAtPath(path):
            stage.RemovePrim(path)
        s = UsdGeom.Sphere.Define(stage, path)
        s.GetRadiusAttr().Set(0.15)
        s.AddTranslateOp().Set(Gf.Vec3d(*pos))
        s.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

    draw_sphere("base_link_center", (base_pos[0], base_pos[1], base_pos[2] + 0.3), (0.0, 1.0, 0.0))  # 质心
    draw_sphere("hinge_center", (hinge_x, hinge_y, hinge_z + 0.3), (1.0, 1.0, 0.0))                 # 铰链中心
    draw_sphere("front_dir_tip",
                (hinge_x + 1.0 * math.cos(yaw_front),
                 hinge_y + 1.0 * math.sin(yaw_front),
                 hinge_z + 0.3),
                (1.0, 0.0, 0.0))                                                                    # 前体朝向端点

    print("已生成可视化球：绿色=质心，黄色=铰链，红色=前体朝向端点")
    """
