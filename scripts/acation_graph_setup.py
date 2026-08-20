#!/usr/bin/env python3
"""
WheelLoader ROS2 Action Graph 创建脚本 (修复版)
==============================================
直接复制粘贴到 Isaac Sim 的 Script Editor 中运行
"""

import omni.graph.core as og
import omni.usd
from pxr import Sdf

# ============================================================
# 配置 - 根据你的场景修改
# ============================================================
ROBOT_PATH = "/World/Robots/WheelLoaderURDF_full"
ART_PATH = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF"
BASE_LINK_PATH = f"{ART_PATH}/base_link"
GRAPH_PATH = "/World/Robots/WheelLoaderURDF_full/ROS2_Control"

# ============================================================
# 工具函数
# ============================================================
def get_stage():
    return omni.usd.get_context().get_stage()

def prim_exists(path: str) -> bool:
    stage = get_stage()
    prim = stage.GetPrimAtPath(path)
    return prim and prim.IsValid()

def remove_existing_graph(graph_path: str):
    stage = get_stage()
    prim = stage.GetPrimAtPath(graph_path)
    if prim and prim.IsValid():
        stage.RemovePrim(graph_path)
        print(f"[setup] 已删除旧 Graph: {graph_path}")

def set_relationship(node_path: str, attr_name: str, target_paths: list):
    """设置 relationship 类型属性"""
    stage = get_stage()
    node_prim = stage.GetPrimAtPath(node_path)
    if not node_prim or not node_prim.IsValid():
        print(f"[setup] 警告: 节点不存在 {node_path}")
        return False
    
    rel = node_prim.GetRelationship(attr_name)
    if not rel:
        rel = node_prim.CreateRelationship(attr_name)
    
    targets = [Sdf.Path(p) for p in target_paths if p]
    rel.SetTargets(targets)
    return True

def list_robot_joints(art_path: str):
    """列出机器人的所有关节"""
    stage = get_stage()
    print(f"\n关节列表 ({art_path}):")
    print("-" * 50)
    
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if art_path in path:
            type_name = prim.GetTypeName()
            if "Joint" in type_name or "joint" in prim.GetName().lower():
                print(f"  {prim.GetName()} ({type_name})")
    
    print("-" * 50)

# ============================================================
# 主函数
# ============================================================
def setup_wheel_loader_ros2():
    print("=" * 60)
    print("创建 WheelLoader ROS2 Action Graph")
    print("=" * 60)
    
    # 1. 检查路径
    if not prim_exists(ROBOT_PATH):
        print(f"[setup] 错误: 机器人不存在 {ROBOT_PATH}")
        return False
    print(f"[setup] 找到机器人: {ROBOT_PATH}")
    
    # 检查 Articulation 路径
    if prim_exists(ART_PATH):
        print(f"[setup] Articulation: {ART_PATH}")
    else:
        print(f"[setup] 警告: ART_PATH 不存在，尝试使用 ROBOT_PATH")
    
    # 检查 base_link
    if prim_exists(BASE_LINK_PATH):
        print(f"[setup] Base Link: {BASE_LINK_PATH}")
    else:
        print(f"[setup] 警告: BASE_LINK_PATH 不存在")
    
    # 列出关节（帮助调试）
    list_robot_joints(ART_PATH if prim_exists(ART_PATH) else ROBOT_PATH)
    
    # 2. 删除旧 Graph
    remove_existing_graph(GRAPH_PATH)
    
    # 3. 使用的 Articulation 路径
    art_path = ART_PATH if prim_exists(ART_PATH) else ROBOT_PATH
    base_link = BASE_LINK_PATH if prim_exists(BASE_LINK_PATH) else art_path
    
    # 4. 节点定义
    node_definitions = [
        ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
        ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
        ("ros2_qos", "isaacsim.ros2.bridge.ROS2QoSProfile"),
        ("read_sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("subscribe_joint_cmd", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        ("articulation_controller", "isaacsim.core.nodes.IsaacArticulationController"),
        ("publish_joint_state", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ("publish_tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
        ("compute_odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("publish_odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
    ]
    
    # 5. 属性值
    set_values = [
        ("ros2_context.inputs:domain_id", 0),
        ("ros2_context.inputs:useDomainIDEnvVar", False),
        ("ros2_qos.inputs:createProfile", "Default for publishers/subscribers"),
        ("subscribe_joint_cmd.inputs:topicName", "/joint_command"),
        ("articulation_controller.inputs:robotPath", art_path),
        ("publish_joint_state.inputs:topicName", "/joint_states"),
        ("publish_tf.inputs:topicName", "/tf"),
        ("publish_tf.inputs:staticPublisher", False),
        ("publish_odom.inputs:topicName", "/odom"),
        ("publish_odom.inputs:chassisFrameId", "base_link"),
        ("publish_odom.inputs:odomFrameId", "odom"),
        ("publish_clock.inputs:topicName", "/clock"),
    ]
    
    # 6. 连接
    connections = [
        # 执行流
        ("on_playback_tick.outputs:tick", "subscribe_joint_cmd.inputs:execIn"),
        ("on_playback_tick.outputs:tick", "articulation_controller.inputs:execIn"),
        ("on_playback_tick.outputs:tick", "publish_joint_state.inputs:execIn"),
        ("on_playback_tick.outputs:tick", "publish_tf.inputs:execIn"),
        ("on_playback_tick.outputs:tick", "compute_odom.inputs:execIn"),
        ("on_playback_tick.outputs:tick", "publish_odom.inputs:execIn"),
        ("on_playback_tick.outputs:tick", "publish_clock.inputs:execIn"),
        
        # ROS2 Context
        ("ros2_context.outputs:context", "subscribe_joint_cmd.inputs:context"),
        ("ros2_context.outputs:context", "publish_joint_state.inputs:context"),
        ("ros2_context.outputs:context", "publish_tf.inputs:context"),
        ("ros2_context.outputs:context", "publish_odom.inputs:context"),
        ("ros2_context.outputs:context", "publish_clock.inputs:context"),
        
        # QoS
        ("ros2_qos.outputs:qosProfile", "subscribe_joint_cmd.inputs:qosProfile"),
        ("ros2_qos.outputs:qosProfile", "publish_joint_state.inputs:qosProfile"),
        ("ros2_qos.outputs:qosProfile", "publish_tf.inputs:qosProfile"),
        ("ros2_qos.outputs:qosProfile", "publish_odom.inputs:qosProfile"),
        
        # 时间戳
        ("read_sim_time.outputs:simulationTime", "publish_joint_state.inputs:timeStamp"),
        ("read_sim_time.outputs:simulationTime", "publish_tf.inputs:timeStamp"),
        ("read_sim_time.outputs:simulationTime", "publish_odom.inputs:timeStamp"),
        ("read_sim_time.outputs:simulationTime", "publish_clock.inputs:timeStamp"),
        
        # 关节命令
        ("subscribe_joint_cmd.outputs:jointNames", "articulation_controller.inputs:jointNames"),
        ("subscribe_joint_cmd.outputs:positionCommand", "articulation_controller.inputs:positionCommand"),
        ("subscribe_joint_cmd.outputs:velocityCommand", "articulation_controller.inputs:velocityCommand"),
        ("subscribe_joint_cmd.outputs:effortCommand", "articulation_controller.inputs:effortCommand"),
        
        # 里程计
        ("compute_odom.outputs:position", "publish_odom.inputs:position"),
        ("compute_odom.outputs:orientation", "publish_odom.inputs:orientation"),
        ("compute_odom.outputs:linearVelocity", "publish_odom.inputs:linearVelocity"),
        ("compute_odom.outputs:angularVelocity", "publish_odom.inputs:angularVelocity"),
    ]
    
    # 7. 创建 Graph
    print(f"\n[setup] 创建 Action Graph: {GRAPH_PATH}")
    
    try:
        og.Controller.edit(
            {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: node_definitions,
                og.Controller.Keys.SET_VALUES: set_values,
                og.Controller.Keys.CONNECT: connections,
            },
        )
        print("[setup] Graph 创建成功!")
    except Exception as e:
        print(f"[setup] Graph 创建失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    # 8. 设置 Relationship 属性
    print("\n[setup] 设置 target prim...")
    
    set_relationship(f"{GRAPH_PATH}/publish_joint_state", "inputs:targetPrim", [art_path])
    print(f"  publish_joint_state.targetPrim -> {art_path}")
    
    set_relationship(f"{GRAPH_PATH}/publish_tf", "inputs:parentPrim", [base_link])
    print(f"  publish_tf.parentPrim -> {base_link}")
    
    set_relationship(f"{GRAPH_PATH}/compute_odom", "inputs:chassisPrim", [base_link])
    print(f"  compute_odom.chassisPrim -> {base_link}")
    
    # TF targets - 获取子部件
    stage = get_stage()
    tf_targets = []
    root_prim = stage.GetPrimAtPath(art_path)
    if root_prim and root_prim.IsValid():
        for child in root_prim.GetChildren():
            child_path = str(child.GetPath())
            if child_path != base_link:
                tf_targets.append(child_path)
    
    if tf_targets:
        set_relationship(f"{GRAPH_PATH}/publish_tf", "inputs:targetPrims", tf_targets[:15])
        print(f"  publish_tf.targetPrims -> [{len(tf_targets[:15])} 个子部件]")
    
    # 完成
    print("\n" + "=" * 60)
    print("Action Graph 创建完成!")
    print("=" * 60)
    print(f"\nGraph: {GRAPH_PATH}")
    print("\n话题:")
    print("  发布: /joint_states, /tf, /odom, /clock")
    print("  订阅: /joint_command")
    print("\n下一步:")
    print("  1. 点击 Play")
    print("  2. ros2 topic list")
    print("  3. python3 wheel_loader_teleop_v1.py")
    
    return True

# ============================================================
# 运行
# ============================================================
setup_wheel_loader_ros2()