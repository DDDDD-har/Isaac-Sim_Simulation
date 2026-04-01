import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni.usd

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# =========================================================
# 固定配置
# =========================================================
FL_WHEEL = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/FLwheel"
FR_WHEEL = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/FRwheel"
RL_WHEEL = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/RLwheel"
RR_WHEEL = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/RRwheel"
HINGE_JOINT = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/chassis_joint/chassis_body_revolute"

FRAME_ID = "world"
TOPIC_FRONT = "/wheel_loader/front_axle_pose"
TOPIC_REAR  = "/wheel_loader/rear_axle_pose"
TOPIC_HINGE = "/wheel_loader/hinge_pose"

# =========================================================
# 工具函数
# =========================================================
def get_stage():
    return omni.usd.get_context().get_stage()

def get_world_transform(prim_path):
    stage = get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[wheel_loader_state_ag_scriptnode] invalid prim: {prim_path}")
        return None
    xformable = UsdGeom.Xformable(prim)
    return xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())

def get_world_pos(prim_path):
    xf = get_world_transform(prim_path)
    if xf is None:
        return None
    return xf.ExtractTranslation()

def midpoint(a, b):
    return Gf.Vec3d(
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2]),
    )

def yaw_from_points(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    return math.atan2(dy, dx)

def quat_from_yaw(yaw):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))

def transform_point(xf, p):
    return xf.Transform(Gf.Vec3d(p[0], p[1], p[2]))

def get_hinge_world_pos(joint_path):
    stage = get_stage()
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim.IsValid():
        print(f"[wheel_loader_state_ag_scriptnode] invalid joint: {joint_path}")
        return None

    joint = UsdPhysics.RevoluteJoint(joint_prim)
    body0_targets = joint.GetBody0Rel().GetTargets()
    body1_targets = joint.GetBody1Rel().GetTargets()
    local_pos0 = joint.GetLocalPos0Attr().Get()
    local_pos1 = joint.GetLocalPos1Attr().Get()

    if body0_targets:
        body0_path = str(body0_targets[0])
        xf0 = get_world_transform(body0_path)
        if xf0 is not None:
            return transform_point(xf0, local_pos0)

    if body1_targets:
        body1_path = str(body1_targets[0])
        xf1 = get_world_transform(body1_path)
        if xf1 is not None:
            return transform_point(xf1, local_pos1)

    return None

def publish_pose(node, pub, frame_id, pos, yaw):
    msg = PoseStamped()
    msg.header = Header()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id

    msg.pose.position.x = float(pos[0])
    msg.pose.position.y = float(pos[1])
    msg.pose.position.z = float(pos[2])

    qx, qy, qz, qw = quat_from_yaw(yaw)
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw

    pub.publish(msg)

# =========================================================
# ScriptNode 生命周期函数
# =========================================================
def setup(db):
    try:
        rclpy.init(args=None)
    except Exception:
        pass

    db.per_instance_state.node = Node("wheel_loader_state_ag_publisher")
    db.per_instance_state.pub_front = db.per_instance_state.node.create_publisher(PoseStamped, TOPIC_FRONT, 10)
    db.per_instance_state.pub_rear  = db.per_instance_state.node.create_publisher(PoseStamped, TOPIC_REAR, 10)
    db.per_instance_state.pub_hinge = db.per_instance_state.node.create_publisher(PoseStamped, TOPIC_HINGE, 10)

    db.per_instance_state.counter = 0
    print("[wheel_loader_state_ag_scriptnode] setup done")

def compute(db):
    try:
        node = db.per_instance_state.node
        pub_front = db.per_instance_state.pub_front
        pub_rear  = db.per_instance_state.pub_rear
        pub_hinge = db.per_instance_state.pub_hinge

        rclpy.spin_once(node, timeout_sec=0.0)

        fl = get_world_pos(FL_WHEEL)
        fr = get_world_pos(FR_WHEEL)
        rl = get_world_pos(RL_WHEEL)
        rr = get_world_pos(RR_WHEEL)
        hinge = get_hinge_world_pos(HINGE_JOINT)

        if fl is None or fr is None or rl is None or rr is None or hinge is None:
            return True

        front_axle = midpoint(fl, fr)
        rear_axle = midpoint(rl, rr)

        yaw_front = yaw_from_points(hinge, front_axle)
        yaw_rear  = yaw_from_points(rear_axle, hinge)

        publish_pose(node, pub_front, FRAME_ID, front_axle, yaw_front)
        publish_pose(node, pub_rear,  FRAME_ID, rear_axle,  yaw_rear)
        publish_pose(node, pub_hinge, FRAME_ID, hinge,      yaw_rear)

        db.per_instance_state.counter += 1
        if db.per_instance_state.counter % 200 == 0:
            print("[wheel_loader_state_ag_scriptnode] publishing ok")

        return True

    except Exception as e:
        print("[wheel_loader_state_ag_scriptnode] compute exception:", e)
        return True

def cleanup(db):
    try:
        if hasattr(db.per_instance_state, "node"):
            db.per_instance_state.node.destroy_node()
    except Exception as e:
        print("[wheel_loader_state_ag_scriptnode] cleanup exception:", e)

    print("[wheel_loader_state_ag_scriptnode] cleanup done")


    