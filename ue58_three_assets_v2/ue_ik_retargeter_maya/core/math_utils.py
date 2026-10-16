# -*- coding: utf-8 -*-
"""
Math utilities for Maya animation retargeting using OpenMaya (API 2.0).
Provides matrix, quaternion, and vector transformation operations.
"""

import math
import maya.api.OpenMaya as om
import maya.cmds as cmds


def get_dag_path(node_name):
    """Get MDagPath for a given node name."""
    sel = om.MSelectionList()
    sel.add(node_name)
    return sel.getDagPath(0)


def get_world_matrix(node_name):
    """Retrieve world transformation matrix of a DAG node as MMatrix."""
    dag_path = get_dag_path(node_name)
    return dag_path.inclusiveMatrix()


def get_local_matrix(node_name):
    """Retrieve local transformation matrix of a DAG node relative to parent."""
    dag_path = get_dag_path(node_name)
    parent_path = om.MDagPath(dag_path)
    parent_path.pop()
    if parent_path.length() > 0:
        parent_inv = parent_path.inclusiveMatrix().inverse()
        return dag_path.inclusiveMatrix() * parent_inv
    return dag_path.inclusiveMatrix()


def matrix_to_trs(matrix):
    """
    Decompose MMatrix into position (MVector), rotation (MQuaternion), and scale (MVector).
    """
    transform_fn = om.MTransformationMatrix(matrix)
    pos = transform_fn.translation(om.MSpace.kWorld)
    quat = transform_fn.rotation(asQuaternion=True)
    scale = transform_fn.scale(om.MSpace.kWorld)
    return pos, quat, om.MVector(scale[0], scale[1], scale[2])


def trs_to_matrix(pos, quat, scale=om.MVector(1.0, 1.0, 1.0)):
    """
    Compose MMatrix from translation (MVector), rotation (MQuaternion), and scale (MVector).
    """
    transform_fn = om.MTransformationMatrix()
    transform_fn.setTranslation(pos, om.MSpace.kWorld)
    transform_fn.setRotation(quat)
    transform_fn.setScale([scale.x, scale.y, scale.z], om.MSpace.kWorld)
    return transform_fn.asMatrix()


def get_world_transform(node_name):
    """Get world position (MVector) and rotation (MQuaternion)."""
    mat = get_world_matrix(node_name)
    pos, quat, scale = matrix_to_trs(mat)
    return pos, quat


def set_world_transform(node_name, pos=None, quat=None):
    """
    Set world position and rotation of a Maya node.
    Maya handles internal jointOrient/rotateAxis conversions automatically via xform.
    """
    if pos is not None:
        cmds.xform(node_name, ws=True, t=(pos.x, pos.y, pos.z))
    if quat is not None:
        euler = quat.asEulerRotation()
        deg = [
            om.MAngle(euler.x).asDegrees(),
            om.MAngle(euler.y).asDegrees(),
            om.MAngle(euler.z).asDegrees()
        ]
        cmds.xform(node_name, ws=True, rotation=deg)


def euler_deg_to_quat(deg_x, deg_y, deg_z, order=om.MEulerRotation.kXYZ):
    """Convert Euler angles in degrees to MQuaternion."""
    rad_x = om.MAngle(deg_x, om.MAngle.kDegrees).asRadians()
    rad_y = om.MAngle(deg_y, om.MAngle.kDegrees).asRadians()
    rad_z = om.MAngle(deg_z, om.MAngle.kDegrees).asRadians()
    euler = om.MEulerRotation(rad_x, rad_y, rad_z, order)
    return euler.asQuaternion()


def quat_to_euler_deg(quat, order=om.MEulerRotation.kXYZ):
    """Convert MQuaternion to Euler angles in degrees."""
    euler = quat.asEulerRotation()
    euler.reorderIt(order)
    return [
        om.MAngle(euler.x).asDegrees(),
        om.MAngle(euler.y).asDegrees(),
        om.MAngle(euler.z).asDegrees()
    ]


def slerp(q1, q2, alpha):
    """Spherical linear interpolation between two quaternions."""
    return om.MQuaternion.slerp(q1, q2, alpha)


def fast_lerp(q1,q2,alpha):
    """UE FQuat::FastLerp with shortest arc and explicit normalization."""
    sign=1. if sum(a*b for a,b in zip(q1,q2))>=0. else -1.
    return om.MQuaternion(*[(1.-alpha)*a+alpha*sign*b for a,b in zip(q1,q2)]).normal()


def lerp_vector(v1, v2, alpha):
    """Linear interpolation between two vectors."""
    return v1 * (1.0 - alpha) + v2 * alpha


def quat_between_vectors(v_from, v_to):
    """Compute quaternion rotating v_from to v_to."""
    v1 = v_from.normal()
    v2 = v_to.normal()
    dot = v1 * v2

    if dot >= 1.0 - 1e-6:
        return om.MQuaternion.kIdentity
    if dot <= -1.0 + 1e-6:
        # 180 degree rotation around arbitrary orthogonal vector
        ortho = om.MVector(1.0, 0.0, 0.0) ^ v1
        if ortho.length() < 1e-3:
            ortho = om.MVector(0.0, 1.0, 0.0) ^ v1
        ortho.normalize()
        return om.MQuaternion(math.pi, ortho)

    axis = v1 ^ v2
    w = 1.0 + dot
    q = om.MQuaternion(axis.x, axis.y, axis.z, w)
    q.normalizeIt()
    return q


def rotate_vector_by_quat(v, q):
    """Rotates an MVector by an MQuaternion in OpenMaya 2.0."""
    return om.MVector(v).rotateBy(q)
