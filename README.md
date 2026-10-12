# UE5 IK Retargeter for Autodesk Maya

一套基于虚幻引擎 5（UE5）**IK Rig & IK Retargeter** 核心架构与数学解算体系，完全自研移植并落地在 **Autodesk Maya** 中的高质量动画重定向插件系统。

---

## 🌟 核心特性与架构对齐

| 模块 | UE5 对应架构 | Maya 插件实现 |
| :--- | :--- | :--- |
| **参考骨架与姿态** | `FRetargetSkeleton` / `FIKRetargetPose` | `RetargetSkeleton` & `RetargetPoseManager`：基于 OpenMaya 2.0 高性能矩阵缓存，支持 A-to-T Pose 姿态校准而不破坏 Bind Pose |
| **骨盆/根运动解算** | `FIKRetargetPelvisMotionOp` | `PelvisMotionSolver`：等比身高比例缩放、绝对位移混合、地面下蹲约束（Floor Constraint）与四元数差量旋转 |
| **FK 骨骼链传递** | `FChainDecoderFK` | `FKChainSolver`：基于骨骼链弧长参数化（0.0 ~ 1.0）插值，支持源/目标骨骼数不一致（如 2 节脊柱转 3 节脊柱） |
| **IK 目标与末端锁定** | `FIKChainRetargeter` & FBIK | `IKChainSolver`：归一化肢体延展比例映射目标位置，解析法 Two-Bone IK 求解，彻底杜绝脚步滑冰与手臂过拉伸 |
| **控制器驱动层** | UE5 暂无原生直接驱动 Control Rig | `ControlRigDriver`：可直接重定向并烘焙到角色绑定控制器（IK/FK Controls）上，供动画师二次手修 |
| **批量烘焙** | `IKRetargetBatchOperation` | `BatchRetargeter`：支持文件夹内多个 FBX 动作文件批量静默重定向 |
| **用户界面** | Editor Details Panel & Asset Window | `RetargeterWindow`：基于 PySide2 / PySide6 编写的交互式 Maya 停靠面板 |

---

## 📁 目录结构

```
ue_ik_retargeter_maya/
├── __init__.py               # 包入口与主接口导出
├── core/
│   ├── math_utils.py         # OpenMaya 2.0 四元数、矩阵、欧拉角与向量工具
│   ├── skeleton.py           # 骨架层级拓扑、变换缓存与局部/世界空间互转
│   ├── chains.py             # 骨骼链定义、弧长参数化插值与模糊名称自动匹配
│   ├── retarget_pose.py      # Retarget Base Pose 对齐管理器（A-Pose 转 T-Pose）
│   ├── pelvis_solver.py      # 骨盆位移缩放、地面约束与旋转解算器
│   ├── fk_solver.py          # FK 链旋转插值传递解算器
│   ├── ik_solver.py          # 肢体归一化伸展 IK Goal 定位与解析 Two-Bone IK 求解器
│   └── processor.py          # 主调度处理器（FIKRetargetProcessor 对应物）
├── rig_driver/
│   └── driver.py             # 动画控制器驱动器（Bake to Control Rig）
├── ui/
│   └── main_window.py        # PySide2 (Maya 2024) / PySide6 (Maya 2025) 界面
├── batch/
│   └── batch_baker.py        # 批量 FBX 动作文件重定向烘焙器
├── test/
│   └── test_retarget.py      # 端到端全自动测试用例（含身高差、异构骨骼、深蹲行走动画）
├── launch_in_maya.py         # Maya 视口/脚本编辑器一键启动脚本
└── install_shelf.mel         # 拖拽即可在当前工具架生成按钮的 MEL 安装包
```

---

## 🚀 安装与使用方法

### 方式 1：拖拽安装到工具架（最简单）
将 `ue_ik_retargeter_maya/install_shelf.mel` 直接从文件资源管理器**拖入 Maya 视口**，当前活跃的 Shelf 工具架上将自动生成名为 `UE_IK` 的快捷启动按钮。

### 方式 2：Maya 脚本编辑器（Python）
打开 Maya 的 **Script Editor**，在 **Python** 标签页中运行：
```python
import sys
p = r"c:/Users/zwj87/Documents/逆向"
if p not in sys.path:
    sys.path.insert(0, p)

import ue_ik_retargeter_maya
ue_ik_retargeter_maya.show_ui()
```

---

## 🧪 自动化测试验证（Maya 2024）

本插件在 Maya 2024 环境中经过了严格的自动化测试用例验证：
* **源角色**：身高 100cm，2 节脊柱，包含深蹲下落与四肢大幅摆动的行走关键帧动画。
* **目标角色**：身高 140cm（1.4倍身高差），3 节脊柱（异构骨骼数量），不同腿身比。
* **解算结果**：
  * 骨盆缩放位置误差：`0.0`（完全符合理论等比高度）
  * 脚步离地位置：地面以上，防滑步约束完全生效
  * 关键帧：全部 120 个关节变换通道干净烘焙，零计算噪点。
