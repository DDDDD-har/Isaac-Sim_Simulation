"""
Comprehensive Isaac Sim Scene Inspection Script
Save result to file and print to console
"""

import omni.usd
import omni.graph.core as og
from pxr import UsdGeom, UsdPhysics
import os
from datetime import datetime

# ============================================================
# Output file
# ============================================================
output_dir = "/home/auto/isaacsim4.5/issacsim_excavator"
output_file = os.path.join(output_dir, "scene_inspection_report.txt")
os.makedirs(output_dir, exist_ok=True)

stage = omni.usd.get_context().get_stage()

output_lines = []

def add_line(text=""):
    output_lines.append(str(text))
    print(text)

# ============================================================
# Basic Stage Info
# ============================================================
add_line("=" * 100)
add_line("ISAAC SIM SCENE INSPECTION REPORT")
add_line("=" * 100)
add_line(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

if stage:
    add_line(f"Stage file: {stage.GetRootLayer().identifier}")
else:
    add_line("ERROR: No stage loaded")
    raise RuntimeError("No stage loaded")

add_line()

# Stage metadata
try:
    root_layer = stage.GetRootLayer()
    default_prim = stage.GetDefaultPrim()
    add_line("STAGE BASIC INFO")
    add_line("-" * 60)
    add_line(f"Default Prim: {default_prim.GetPath() if default_prim else 'None'}")
    add_line(f"Start TimeCode: {stage.GetStartTimeCode()}")
    add_line(f"End TimeCode: {stage.GetEndTimeCode()}")
    add_line(f"TimeCodes Per Second: {stage.GetTimeCodesPerSecond()}")
except Exception as e:
    add_line(f"Failed to query stage metadata: {e}")

# ============================================================
# Search robots root
# ============================================================
robots_root = "/World/Robots"
robots_prim = stage.GetPrimAtPath(robots_root)

add_line()
add_line("ROBOTS ROOT INFO")
add_line("-" * 60)
add_line(f"Robots root path: {robots_root}")
add_line(f"Exists: {robots_prim.IsValid() if robots_prim else False}")

# ============================================================
# Inspect robots
# ============================================================
robot_infos = []

if robots_prim and robots_prim.IsValid():
    for child in robots_prim.GetChildren():
        robot_path = str(child.GetPath())
        robot_name = child.GetName()

        info = {
            "name": robot_name,
            "path": robot_path,
            "articulation_roots": [],
            "joints": [],
            "links": [],
            "graphs": [],
            "sensors": []
        }

        # Traverse under robot
        for prim in stage.Traverse():
            p = str(prim.GetPath())
            if not p.startswith(robot_path):
                continue

            # articulation root
            schemas = [str(s) for s in prim.GetAppliedSchemas()]
            if any("ArticulationRoot" in s for s in schemas):
                info["articulation_roots"].append(p)

            # joints
            type_name = prim.GetTypeName()
            if type_name in ["PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "RevoluteJoint", "PrismaticJoint"]:
                info["joints"].append({
                    "name": prim.GetName(),
                    "path": p,
                    "type": type_name
                })

            # links / transforms
            if type_name in ["Xform", "Scope", "Cube", "Sphere", "Cylinder"]:
                info["links"].append({
                    "name": prim.GetName(),
                    "path": p,
                    "type": type_name
                })

            # graph
            if "ROS2" in p or "Graph" in p or type_name == "OmniGraph":
                info["graphs"].append(p)

            # sensors
            if type_name == "Camera":
                info["sensors"].append({
                    "name": prim.GetName(),
                    "path": p,
                    "type": "Camera"
                })

        robot_infos.append(info)

add_line()
add_line("ROBOTS SUMMARY")
add_line("-" * 60)
add_line(f"Found robots: {len(robot_infos)}")

for idx, robot in enumerate(robot_infos, 1):
    add_line(f"\n[{idx}] Robot: {robot['name']}")
    add_line(f"  Path: {robot['path']}")
    add_line(f"  Articulation Roots: {len(robot['articulation_roots'])}")
    for ar in robot["articulation_roots"]:
        add_line(f"    - {ar}")
    add_line(f"  Joints: {len(robot['joints'])}")
    add_line(f"  Links: {len(robot['links'])}")
    add_line(f"  Graphs: {len(robot['graphs'])}")
    add_line(f"  Sensors: {len(robot['sensors'])}")

# ============================================================
# Detailed joint inspection for WheelLoaderURDF_full
# ============================================================
target_robot_path = "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF"
target_prim = stage.GetPrimAtPath(target_robot_path)

add_line()
add_line("TARGET ROBOT DETAILED INSPECTION")
add_line("-" * 60)
add_line(f"Target robot path: {target_robot_path}")
add_line(f"Exists: {target_prim.IsValid() if target_prim else False}")

joint_list = []

if target_prim and target_prim.IsValid():
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if not p.startswith(target_robot_path):
            continue

        type_name = prim.GetTypeName()
        if type_name in ["PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "RevoluteJoint", "PrismaticJoint"]:
            joint_info = {
                "name": prim.GetName(),
                "path": p,
                "type": type_name,
                "attrs": {}
            }

            # collect drive attrs if exist
            candidate_attrs = [
                "drive:angular:physics:stiffness",
                "drive:angular:physics:damping",
                "drive:angular:physics:maxForce",
                "drive:angular:physics:targetPosition",
                "drive:angular:physics:targetVelocity",
                "drive:linear:physics:stiffness",
                "drive:linear:physics:damping",
                "drive:linear:physics:maxForce",
                "drive:linear:physics:targetPosition",
                "drive:linear:physics:targetVelocity",
                "physics:axis",
            ]

            for attr_name in candidate_attrs:
                if prim.HasAttribute(attr_name):
                    try:
                        joint_info["attrs"][attr_name] = prim.GetAttribute(attr_name).Get()
                    except Exception as e:
                        joint_info["attrs"][attr_name] = f"<read failed: {e}>"

            joint_list.append(joint_info)

add_line(f"\nFound joints under target robot: {len(joint_list)}")
for i, joint in enumerate(joint_list):
    add_line(f"\nJoint {i}: {joint['name']}")
    add_line(f"  Path: {joint['path']}")
    add_line(f"  Type: {joint['type']}")
    if joint["attrs"]:
        add_line("  Attributes:")
        for k, v in joint["attrs"].items():
            add_line(f"    {k}: {v}")
    else:
        add_line("  Attributes: None")

# ============================================================
# Action Graph inspection
# ============================================================
add_line()
add_line("ACTION GRAPH INSPECTION")
add_line("-" * 60)

all_graphs = og.get_all_graphs()
add_line(f"Total graphs found: {len(all_graphs)}")

for graph in all_graphs:
    try:
        graph_path = graph.get_path_to_graph()
        add_line(f"\nGraph: {graph_path}")
        nodes = graph.get_nodes()
        add_line(f"  Node count: {len(nodes)}")
        for node in nodes:
            try:
                node_path = node.get_prim_path()
                node_type = node.get_node_type().get_node_type()
                add_line(f"    - {node_path} | type: {node_type}")
            except Exception as e:
                add_line(f"    - node read failed: {e}")
    except Exception as e:
        add_line(f"Graph read failed: {e}")

# ============================================================
# Key prim existence check
# ============================================================
key_paths = [
    "/World",
    "/World/Robots",
    "/World/Robots/WheelLoaderURDF_full",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/base_link",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/wheel_joint/FRwheel_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/wheel_joint/FLwheel_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/wheel_joint/RLwheel_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/wheel_joint/RRwheel_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/chassis_joint/chassis_body_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/boom_joint/boom_pivot_revolute",
    "/World/Robots/WheelLoaderURDF_full/WheelLoaderURDF/bucket_joint/rocker_bucket_revolute",
    "/World/Robots/WheelLoaderURDF_full/ROS2_Control",
    "/World/DebugPath",
]

add_line()
add_line("KEY PATH CHECK")
add_line("-" * 60)

for p in key_paths:
    prim = stage.GetPrimAtPath(p)
    add_line(f"{p}: {'EXISTS' if prim and prim.IsValid() else 'MISSING'}")

# ============================================================
# Save report
# ============================================================
add_line()
add_line("=" * 100)
add_line("END OF REPORT")
add_line("=" * 100)

with open(output_file, "w", encoding="utf-8") as f:
    f.write("\n".join(output_lines))

print()
print("=" * 100)
print(f"Report saved to: {output_file}")
print("=" * 100)
