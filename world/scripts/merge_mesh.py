# This script is executed the first time the script node computes, or the next time
# it computes after this script is modified or the 'Reset' button is pressed.
#
# The following callback functions may be defined in this script:
#     setup(db): Called immediately after this script is executed
#     compute(db): Called every time the node computes (should always be defined)
#     cleanup(db): Called when the node is deleted or the reset button is pressed
# Available variables:
#    db: og.Database The node interface - attributes are exposed in a namespace like db.inputs.foo and db.outputs.bar.
#                    Use db.log_error, db.log_warning to report problems in the compute function.
#    og: The omni.graph.core module

import omni
import warp as wp
import warp.sim
import numpy as np
from pxr import UsdGeom

@wp.kernel(enable_backward=False)
def transform_points_kernel(
    points: wp.array(dtype=wp.vec3),
    xform: wp.mat44,
    out_points: wp.array(dtype=wp.vec3),
):
    tid = wp.tid()
    out_points[tid] = wp.transform_point(xform, points[tid])

stage = omni.usd.get_context().get_stage()

def setup(db):

    # xform
    xform_path = "/World/Walls"
    robot_prim = stage.GetPrimAtPath(xform_path)    # Usd.Prim(</World/wheelloader>)

    # Xforms
    xform_prims = []
    for child_prim in robot_prim.GetChildren():
        if child_prim.GetTypeName() == "Xform":
            xform_prims.append(child_prim)
    
    # 获取mesh及其点和面
    collider_points = []
    collider_indices = []
    collider_counts = []
    current_vertex_offset = 0
    for prim in xform_prims:
        # get collision(type:mesh)
        collision_prim = prim.GetChild("geometry").GetChild("mesh")
        mesh = UsdGeom.Mesh(collision_prim)
        xform = UsdGeom.Xformable(prim)
        xform_trans = xform.GetLocalTransformation()
        xform_mat = wp.mat44(np.array(xform_trans.GetTranspose()))
        points = wp.array(mesh.GetPointsAttr().Get(),dtype=wp.vec3)
        world_points = wp.empty(len(points), dtype=wp.vec3)
        wp.launch(
            kernel=transform_points_kernel,
            dim=len(points),
            inputs=[
                points,
                xform_mat,
            ],
            outputs=[
                world_points
            ],
        )
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        face_vertex_indices = [idx + current_vertex_offset for idx in indices]
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        collider_points.extend(world_points.numpy())
        collider_indices.extend(face_vertex_indices)
        collider_counts.extend(face_vertex_counts)
        current_vertex_offset += len(points)
    
    # 创建合并后的mesh
    collider_mesh = wp.sim.Mesh(
        collider_points,
        collider_indices,
        compute_inertia=False,
    )
    merged_mesh_prim = stage.DefinePrim("/World/Walls/merged_collider", "Mesh")

    merged_mesh = UsdGeom.Mesh(merged_mesh_prim)
    merged_mesh.CreatePointsAttr().Set(collider_mesh.vertices)
    merged_mesh.CreateFaceVertexIndicesAttr().Set(collider_mesh.indices)
    merged_mesh.CreateFaceVertexCountsAttr().Set(collider_counts)
    
def cleanup(db):
    pass
 

def compute(db):
    prim = stage.GetPrimAtPath("/World/Walls/merged_collider")
    if not prim.IsValid():
        db.log_error("merged_collider prim is invalid")
    # print(prim)

    # xform
    xform_path = "/World/Walls"
    robot_prim = stage.GetPrimAtPath(xform_path)    # Usd.Prim(</World/wheelloader>)

    # Xforms
    xform_prims = []
    for child_prim in robot_prim.GetChildren():
        if child_prim.GetTypeName() == "Xform":
            xform_prims.append(child_prim)
            
    print("xform_prims", xform_prims)
    # 获取mesh及其点和面
    collider_points = []
    collider_indices = []
    collider_counts = []
    current_vertex_offset = 0
    for prim in xform_prims:
        # get collision(type:mesh)
        print("prim", prim)
        collision_prim = prim.GetChild("geometry").GetChild("mesh")    # 如果合并了collisions和visuals就改成visuals
        print("collision_prim", collision_prim)
        mesh = UsdGeom.Mesh(collision_prim)
        xform = UsdGeom.Xformable(prim)
        xform_trans = xform.GetLocalTransformation()
        xform_mat = wp.mat44(np.array(xform_trans.GetTranspose()))
        points = wp.array(mesh.GetPointsAttr().Get(),dtype=wp.vec3)
        world_points = wp.empty(len(points), dtype=wp.vec3)
        print("world_points", len(world_points))
        wp.launch(
            kernel=transform_points_kernel,
            dim=len(points),
            inputs=[
                points,
                xform_mat,
            ],
            outputs=[
                world_points
            ],
        )
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        face_vertex_indices = [idx + current_vertex_offset for idx in indices]
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        collider_points.extend(world_points.numpy())
        collider_indices.extend(face_vertex_indices)
        collider_counts.extend(face_vertex_counts)
        current_vertex_offset += len(points)
        print("collider_points", len(collider_points))

        
    
    # 创建合并后的mesh
    collider_mesh = wp.sim.Mesh(
        collider_points,
        collider_indices,
        compute_inertia=False,
    )
    merged_mesh_prim = stage.DefinePrim("/World/Walls/merged_collider", "Mesh")

    merged_mesh = UsdGeom.Mesh(merged_mesh_prim)
    merged_mesh.CreatePointsAttr().Set(collider_mesh.vertices)
    merged_mesh.CreateFaceVertexIndicesAttr().Set(collider_mesh.indices)
    merged_mesh.CreateFaceVertexCountsAttr().Set(collider_counts)

    return True