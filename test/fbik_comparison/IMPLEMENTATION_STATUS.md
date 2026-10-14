# PBIK 实现进展

PelvisRoot 已移除：实时场景只保留八个手脚 Goal（每个角色四个）。骨盆继续作为 PBIK 求解根；没有额外拖点输入预偏移或同步。当前场景迁移、四 Goal 构建测试、完整实时 PBIK 验收及输入骨架下蹲的膝盖朝向验证均已通过，场景未保存。

最新补充：已修正当前 Maya Mixamo 膝盖向后弯曲的问题，加入直腿 / 接近伸直的稳定极向量判定与实际骨轴的 Preferred Angle 符号校准。判定方法及本次验证见 [KNEE_POLE_DIRECTION.md](KNEE_POLE_DIRECTION.md)。以下数值描述最初未开启 Target 对齐时的测试，当前实机数据包含 Target 对齐。

已替换旧全身启发式求解器，使用 UE 5.8 PBIK 源码生成的独立 x64 库与 Maya DG 节点。核心回归测试 7 项已通过。

已实现：Free / PinToInput / PrePull、20/10 迭代、质量与刚度、Preferred Angles、关节轴锁定与角度限位、PullChainAlpha、StrengthAlpha、PinRotation、单次 Goal 权重混合。Source 的脚趾链已补齐；CC4 / Mixamo 锁骨刚度与肘膝设置已按实际 UE 资产更新。

矩阵节点读取当前帧的局部骨骼矩阵、Goal 世界矩阵和场景参数，输出 offsetParentMatrix；全身路径不再使用 expression / 属性回调写关节动画通道。

Maya 2024 实机装配和验收已完成：两套 PBIK 节点、八个手脚 Goal 均已在当前 test_maya 中工作。修复了 API 2.0 数组句柄的方法名，并验证了节点的真实计算与控制器到输出骨骼的 DG 更新。测试保持原当前帧和控制器姿态；159 条动画曲线的键值、时间、连接，以及骨骼 translate / rotate / scale 均保持不变。场景没有保存。

核心测试和 Maya 实机测量一致：左手前移 20 cm 时右手位置残差为 CC4 1.7557 cm、Mixamo 1.7371 cm，旧版分别为 17.03 cm 和 15.85 cm。移动的左手残差为 1.8302 cm / 1.4182 cm，脚位置残差最大为 0.0108 cm。有限迭代和多 Goal 约束仍有位置残差；大幅前移 60 cm 时左手残差为 12.41 cm / 9.58 cm，尚不能宣称所有目标都能精确到达。

旋转权重 0.5 的 60° 手脚旋转输出正确的 30°；位置权重 0 不再关闭旋转。骨骼刚度从 0 改为 1 时，最大骨骼位置变化为 CC4 12.91 cm / Mixamo 29.89 cm。PinToInput 根固定、Absolute Goal 和骨盆下移 10 cm 也已测量。单次 Maya 节点求解约 2.5–4.5 ms（本机本次采样，并非性能基准）。

独立的三关节动画夹具验证了当前帧输入和外部父级移动；未发现依赖环。热重载保留活动求解器注册表并刷新已有节点的 compute 方法，不再因模块缓存清理丢失矩阵驱动的管理信息。

本次实现和验证范围是 IK Rig 的实时全身 IK。Step 2 的动画重定向 / 烘焙流程仍有独立的 Two-Bone IK 和 Pole Vector 实现，未在本次测试中验证为完整 UE Retargeter 的逐帧等价结果。UE 源码核心使用独立容器和标量数学适配，尚未取得引擎逐骨骼输出作为基准，因此不宣称逐骨骼或位级完全一致。

构建说明：[native/README.md](../../ue58_three_assets_v2/ue_ik_retargeter_maya/native/README.md)。核心测试：[test_pbik_native.py](../test_pbik_native.py)。实机验收：[validate_maya_pbik.py](validate_maya_pbik.py)，数据：[maya_pbik_results.json](maya_pbik_results.json)。修改前的诊断保留在 [COMPARISON.md](COMPARISON.md)。
