# Blender_Addon_IK-FK_Snaping

<img width="1987" height="784" alt="image" src="https://github.com/user-attachments/assets/f4a46516-9f23-4d03-9211-39f8944c4ba3" />
安装插件后，在姿态模式下选中挂了 IK 约束的骨骼，约束属性面板中会多出一个切换按钮。

- **Influence 为 0 时**，点击按钮会将 IK 目标和极向量目标吸附到当前骨骼的对应位置，然后将 Influence 设为 1，完成 IK 吸附。
- **Influence 为 1 时**，点击按钮会将 IK 解算出的骨骼链变换烘焙到骨骼上，然后将 Influence 设为 0，完成 IK 到 FK 的烘焙。

> 使用前需将 Pole Angle 调整到关节正好朝向极向量目标，否则计算结果会出错。
