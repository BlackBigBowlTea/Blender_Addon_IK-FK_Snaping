bl_info = {
    "name": "FK/IK Toggle",
    "author": "Your Name",
    "version": (1, 0),
    "blender": (5, 0, 0),
    "location": "Properties > Bone Constraint > FK/IK Toggle",
    "description": "一键吸附 IK/FK 姿态并切换，自动重置骨骼旋转",
    "category": "Rigging",
}

import bpy
from bpy.types import Operator, Panel
from mathutils import Matrix, Quaternion


class POSE_OT_fk_ik_toggle(Operator):
    bl_idname = "pose.fk_ik_toggle"
    bl_label = "Snap and Toggle FK/IK"
    bl_description = "吸附 IK 目标和极向量，重置骨骼旋转，并切换 FK/IK 模式"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        bone = context.active_pose_bone
        if bone is None:
            return False
        for c in bone.constraints:
            if c.type == 'IK':
                return True
        return False

    def get_ik_chain(self, bone, ik):
        """返回 IK 链上所有骨骼（从根到末端，包含 bone）"""
        armature = bone.id_data
        chain = []

        # 确定起始骨骼
        start = bone
        if ik.chain_count == 0:
            # 链长为 0 表示使用整条链，向上追溯到根
            while start.parent:
                start = start.parent
        else:
            for _ in range(ik.chain_count - 1):
                if start.parent:
                    start = start.parent
                else:
                    break

        # 收集从 start 到 bone 的链
        current = bone
        while current and current != start.parent:
            chain.append(current)
            current = current.parent
        chain.reverse()
        return chain

    def get_world_matrix(self, target_obj, subtarget):
        """获取目标对象/骨骼的世界矩阵"""
        if target_obj.type == 'ARMATURE' and subtarget:
            target_bone = target_obj.pose.bones.get(subtarget)
            if target_bone:
                return target_obj.matrix_world @ target_bone.matrix
        return target_obj.matrix_world

    def set_world_matrix(self, target_obj, subtarget, matrix):
        """将目标对象/骨骼设置到指定的世界矩阵"""
        if target_obj.type == 'ARMATURE' and subtarget:
            target_bone = target_obj.pose.bones.get(subtarget)
            if target_bone:
                target_bone.matrix = target_obj.matrix_world.inverted() @ matrix
                return
        target_obj.matrix_world = matrix

    def snap_ik_to_fk(self, bone, ik):
        """吸附 IK 目标和极向量到当前 FK 姿态，并将链中所有骨骼的 FK 旋转清零"""
        if not ik.target:
            self.report({'WARNING'}, "IK 约束没有指定目标")
            return

        armature = bone.id_data
        chain = self.get_ik_chain(bone, ik)

        # 1. 将 IK 目标移动到末端骨骼的尾部位置（保持原有方向）
        end_tail_world = armature.matrix_world @ bone.tail
        target_mat = self.get_world_matrix(ik.target, ik.subtarget).copy()
        target_mat.translation = end_tail_world
        self.set_world_matrix(ik.target, ik.subtarget, target_mat)

        # 2. 移动极向量目标到第二个骨骼的头部位置（仅平移，保留旋转缩放）
        if ik.pole_target and len(chain) >= 2:
            mid_bone = chain[1]
            pole_world_pos = armature.matrix_world @ mid_bone.head
            pole_mat = self.get_world_matrix(ik.pole_target, ik.pole_subtarget).copy()
            pole_mat.translation = pole_world_pos
            self.set_world_matrix(ik.pole_target, ik.pole_subtarget, pole_mat)

        # 3. 将链上所有骨骼的 FK 旋转重置为单位四元数
        for b in chain:
            loc, rot, sca = b.matrix_basis.decompose()
            new_mat = Matrix.LocRotScale(loc, Quaternion((1, 0, 0, 0)), sca)
            b.matrix_basis = new_mat

    def snap_fk_to_ik(self, bone, ik):
        """将 IK 解算姿态精确固化到 FK 骨骼（考虑父子顺序）"""
        armature = bone.id_data
        chain = self.get_ik_chain(bone, ik)

        # 1. 记录 IK 状态下每个骨骼相对于其父骨骼的局部矩阵
        relative_matrices = {}
        for b in chain:
            if b.parent and b.parent in chain:
                parent_world = b.parent.matrix.copy()
            else:
                parent_world = armature.matrix_world @ b.parent.matrix if b.parent else Matrix.Identity(4)
            relative_matrices[b] = parent_world.inverted() @ b.matrix

        # 2. 关闭 IK，回到 FK 状态
        ik.influence = 0.0
        bpy.context.view_layer.update()

        # 3. 按链顺序（父→子）重建每个骨骼的世界矩阵
        for b in chain:
            if b.parent and b.parent in chain:
                parent_world = b.parent.matrix.copy()
            else:
                parent_world = armature.matrix_world @ b.parent.matrix if b.parent else Matrix.Identity(4)
            b.matrix = parent_world @ relative_matrices[b]

    def execute(self, context):
        bone = context.active_pose_bone
        ik = next((c for c in bone.constraints if c.type == 'IK'), None)

        # 使用容差判断浮点数，避免精度问题
        if ik.influence < 0.001:
            self.snap_ik_to_fk(bone, ik)   # FK → IK：吸附目标并重置旋转
            ik.influence = 1.0
            self.report({'INFO'}, "已切换到 IK，FK 旋转已清零")
        elif ik.influence > 0.999:
            self.snap_fk_to_ik(bone, ik)   # IK → FK：固化骨骼姿态
            ik.influence = 0.0
            self.report({'INFO'}, "已切换到 FK")
        else:
            self.report({'WARNING'}, "请先将 Influence 设为 0 或 1（或接近 0/1）")
            return {'CANCELLED'}

        context.view_layer.update()
        return {'FINISHED'}


class BONE_PT_fk_ik_toggle(Panel):
    bl_label = "FK/IK Toggle"
    bl_idname = "BONE_PT_fk_ik_toggle"
    bl_space_type = 'PROPERTIES'
    bl_region_type = 'WINDOW'
    bl_context = "bone_constraint"

    @classmethod
    def poll(cls, context):
        bone = context.active_pose_bone
        return bone is not None and any(c.type == 'IK' for c in bone.constraints)

    def draw(self, context):
        layout = self.layout
        bone = context.active_pose_bone
        ik = next((c for c in bone.constraints if c.type == 'IK'), None)
        if ik is None:
            return

        row = layout.row(align=True)
        row.scale_y = 1.5

        # 根据 Influence 值显示对应按钮（使用容差判断）
        if ik.influence < 0.001:
            row.operator("pose.fk_ik_toggle", text="Snap to IK", icon='SNAP_ON')
        elif ik.influence > 0.999:
            row.operator("pose.fk_ik_toggle", text="Snap to FK", icon='SNAP_ON')
        else:
            row.label(text="← 将 Influence 设为 0 或 1 后可切换")


def register():
    bpy.utils.register_class(POSE_OT_fk_ik_toggle)
    bpy.utils.register_class(BONE_PT_fk_ik_toggle)


def unregister():
    bpy.utils.unregister_class(POSE_OT_fk_ik_toggle)
    bpy.utils.unregister_class(BONE_PT_fk_ik_toggle)


if __name__ == "__main__":
    register()