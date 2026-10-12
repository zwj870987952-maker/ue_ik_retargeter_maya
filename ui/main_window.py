# -*- coding: utf-8 -*-
"""
PySide2 / PySide6 UI for Maya IK Retargeter.
Compatible with Maya 2024 (PySide2) and Maya 2025+ (PySide6).
Delivers the authentic UE5 One-Click automatic retargeting experience.
"""

import json
import maya.cmds as cmds

try:
    from PySide2 import QtWidgets, QtCore, QtGui
except ImportError:
    from PySide6 import QtWidgets, QtCore, QtGui

from ..core.processor import IKRetargetProcessor
from ..rig_driver.driver import ControlRigDriver


class RetargeterWindow(QtWidgets.QDialog):
    """
    Main user interface for UE5-style animation retargeting inside Maya.
    Provides one-click automatic retargeting out of the box with zero manual alignment.
    """

    WINDOW_TITLE = "UE5 IK Retargeter for Maya"

    def __init__(self, parent=None):
        super(RetargeterWindow, self).__init__(parent)
        self.setWindowTitle(self.WINDOW_TITLE)
        self.resize(700, 620)

        self.processor = None
        self.rig_driver = None

        self._build_ui()
        self._init_defaults()

    def _build_ui(self):
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setSpacing(12)

        # Title Banner
        banner = QtWidgets.QLabel("Unreal Engine 5 IK Retargeter for Maya")
        banner.setStyleSheet("font-size: 16px; font-weight: bold; color: #5dade2; margin-bottom: 4px;")
        main_layout.addWidget(banner)

        subtitle = QtWidgets.QLabel("上传源角色与目标角色后，一键自动匹配骨骼链、自动对齐姿态并烘焙动画。")
        subtitle.setStyleSheet("color: #aaaaaa; margin-bottom: 8px;")
        main_layout.addWidget(subtitle)

        # --- 1. Character Selection ---
        char_box = QtWidgets.QGroupBox("角色选择 (Character Setup)")
        char_layout = QtWidgets.QGridLayout(char_box)
        char_layout.setSpacing(8)

        char_layout.addWidget(QtWidgets.QLabel("源角色根骨骼 (Source Root):"), 0, 0)
        self.src_root_edit = QtWidgets.QLineEdit()
        self.src_root_edit.setPlaceholderText("选择包含动画的源角色 Root/Hips 骨骼...")
        char_layout.addWidget(self.src_root_edit, 0, 1)
        src_pick_btn = QtWidgets.QPushButton("拾取选中 (Pick)")
        src_pick_btn.clicked.connect(lambda: self._pick_root(self.src_root_edit))
        char_layout.addWidget(src_pick_btn, 0, 2)

        char_layout.addWidget(QtWidgets.QLabel("目标角色根骨骼 (Target Root):"), 1, 0)
        self.tgt_root_edit = QtWidgets.QLineEdit()
        self.tgt_root_edit.setPlaceholderText("选择待接收动画的目标角色 Root/Hips 骨骼...")
        char_layout.addWidget(self.tgt_root_edit, 1, 1)
        tgt_pick_btn = QtWidgets.QPushButton("拾取选中 (Pick)")
        tgt_pick_btn.clicked.connect(lambda: self._pick_root(self.tgt_root_edit))
        char_layout.addWidget(tgt_pick_btn, 1, 2)

        auto_find_btn = QtWidgets.QPushButton("自动探测场景角色 (Auto-Detect Skeletons)")
        auto_find_btn.clicked.connect(self._auto_detect_skeletons)
        char_layout.addWidget(auto_find_btn, 2, 0, 1, 3)

        main_layout.addWidget(char_box)

        # --- 2. THE BIG ONE-CLICK ACTION BUTTON (UE5 Experience) ---
        one_click_box = QtWidgets.QGroupBox("⚡ 核心操作 (One-Click Execution)")
        one_click_layout = QtWidgets.QVBoxLayout(one_click_box)

        time_bar = QtWidgets.QHBoxLayout()
        time_bar.addWidget(QtWidgets.QLabel("时间线起止帧 (Frame Range):"))
        self.start_frame_spin = QtWidgets.QSpinBox()
        self.start_frame_spin.setRange(-99999, 99999)
        time_bar.addWidget(self.start_frame_spin)

        time_bar.addWidget(QtWidgets.QLabel("至"))
        self.end_frame_spin = QtWidgets.QSpinBox()
        self.end_frame_spin.setRange(-99999, 99999)
        time_bar.addWidget(self.end_frame_spin)
        one_click_layout.addLayout(time_bar)

        self.one_click_btn = QtWidgets.QPushButton("🚀 一键自动重定向并烘焙动画 (Auto-Align & Retarget)")
        self.one_click_btn.setStyleSheet(
            "background-color: #1e88e5; color: white; font-size: 15px; font-weight: bold; padding: 12px; border-radius: 6px;"
        )
        self.one_click_btn.clicked.connect(self._on_one_click_bake)
        one_click_layout.addWidget(self.one_click_btn)

        preview_btn = QtWidgets.QPushButton("👁 预览当前帧重定向效果 (Preview Current Frame)")
        preview_btn.clicked.connect(self._on_preview_frame)
        one_click_layout.addWidget(preview_btn)

        main_layout.addWidget(one_click_box)

        # --- 3. Advanced Settings (Collapsible) ---
        self.adv_group = QtWidgets.QGroupBox("高级微调与骨骼链检视 (Advanced Settings - Optional)")
        self.adv_group.setCheckable(True)
        self.adv_group.setChecked(False)
        adv_layout = QtWidgets.QVBoxLayout(self.adv_group)

        # Parameters
        param_grid = QtWidgets.QGridLayout()
        param_grid.addWidget(QtWidgets.QLabel("身高位移缩放比 (Height Scale):"), 0, 0)
        self.height_slider = QtWidgets.QDoubleSpinBox()
        self.height_slider.setRange(0.1, 5.0)
        self.height_slider.setSingleStep(0.05)
        self.height_slider.setValue(1.0)
        param_grid.addWidget(self.height_slider, 0, 1)

        param_grid.addWidget(QtWidgets.QLabel("脚底贴地约束 (Floor Constraint):"), 0, 2)
        self.floor_slider = QtWidgets.QDoubleSpinBox()
        self.floor_slider.setRange(0.0, 1.0)
        self.floor_slider.setSingleStep(0.1)
        self.floor_slider.setValue(1.0)
        param_grid.addWidget(self.floor_slider, 0, 3)
        adv_layout.addLayout(param_grid)

        # Chains Table
        self.chain_table = QtWidgets.QTableWidget()
        self.chain_table.setColumnCount(5)
        self.chain_table.setHorizontalHeaderLabels([
            "链名称 (Chain)", "源起始", "源结束", "目标起始", "目标结束"
        ])
        self.chain_table.horizontalHeader().setStretchLastSection(True)
        adv_layout.addWidget(self.chain_table)

        main_layout.addWidget(self.adv_group)

    def _init_defaults(self):
        try:
            start_f = int(cmds.playbackOptions(q=True, minTime=True))
            end_f = int(cmds.playbackOptions(q=True, maxTime=True))
            self.start_frame_spin.setValue(start_f)
            self.end_frame_spin.setValue(end_f)
        except Exception:
            self.start_frame_spin.setValue(1)
            self.end_frame_spin.setValue(100)

        self._auto_detect_skeletons()

    def _pick_root(self, line_edit):
        sel = cmds.ls(sl=True, type='joint')
        if sel:
            line_edit.setText(sel[0])

    def _auto_detect_skeletons(self):
        """Auto-detects source and target roots from selection or scene."""
        sel = cmds.ls(sl=True, type='joint')
        if len(sel) >= 2:
            self.src_root_edit.setText(sel[0])
            self.tgt_root_edit.setText(sel[1])
            return

        all_joints = cmds.ls(type='joint')
        roots = [j for j in all_joints if not cmds.listRelatives(j, p=True, type='joint')]
        if len(roots) >= 2:
            self.src_root_edit.setText(roots[0])
            self.tgt_root_edit.setText(roots[1])

    def _ensure_processor(self):
        src = self.src_root_edit.text().strip()
        tgt = self.tgt_root_edit.text().strip()

        if not (cmds.objExists(src) and cmds.objExists(tgt)):
            QtWidgets.QMessageBox.warning(
                self, "缺少角色", "请在上方输入或在场景中拾取源角色（Source）和目标角色（Target）的根骨骼。"
            )
            return None

        if not self.processor or self.processor.source_root != src or self.processor.target_root != tgt:
            self.processor = IKRetargetProcessor(src, tgt)
            self.rig_driver = ControlRigDriver(self.processor)
            self._refresh_table()

        # Update solver parameters
        self.processor.pelvis_solver.scale_vertical = self.height_slider.value()
        self.processor.pelvis_solver.floor_constraint_weight = self.floor_slider.value()
        return self.processor

    def _refresh_table(self):
        if not self.processor:
            return
        self.chain_table.setRowCount(len(self.processor.chain_mappings))
        for row, mapping in enumerate(self.processor.chain_mappings):
            self.chain_table.setItem(row, 0, QtWidgets.QTableWidgetItem(mapping.name))
            self.chain_table.setItem(row, 1, QtWidgets.QTableWidgetItem(mapping.source_chain.start_bone))
            self.chain_table.setItem(row, 2, QtWidgets.QTableWidgetItem(mapping.source_chain.end_bone))
            self.chain_table.setItem(row, 3, QtWidgets.QTableWidgetItem(mapping.target_chain.start_bone))
            self.chain_table.setItem(row, 4, QtWidgets.QTableWidgetItem(mapping.target_chain.end_bone))

    def _on_preview_frame(self):
        processor = self._ensure_processor()
        if not processor:
            return
        processor.retarget_frame()

    def _on_one_click_bake(self):
        """Pure UE5 one-click automatic retargeting action."""
        processor = self._ensure_processor()
        if not processor:
            return

        s_f = self.start_frame_spin.value()
        e_f = self.end_frame_spin.value()

        progress = QtWidgets.QProgressDialog(
            "正在自动执行 UE5 姿态对齐与动画重定向烘焙...", "取消", s_f, e_f, self
        )
        progress.setWindowModality(QtCore.Qt.WindowModal)

        for f in range(s_f, e_f + 1):
            if progress.wasCanceled():
                break
            cmds.currentTime(f, e=True)
            processor.retarget_frame()
            for j in processor.target_skeleton.joint_names:
                cmds.setKeyframe(j, time=f, attribute=['translate', 'rotate'])
            progress.setValue(f)

        progress.setValue(e_f)
        QtWidgets.QMessageBox.information(
            self, "重定向完成", "动画已全自动对齐、求解并成功烘焙至目标角色骨骼！"
        )


def show_ui():
    """Opens or raises the Retargeter UI window in Maya."""
    global _retargeter_win_instance
    try:
        _retargeter_win_instance.close()
        _retargeter_win_instance.deleteLater()
    except Exception:
        pass
    _retargeter_win_instance = RetargeterWindow()
    _retargeter_win_instance.show()
    return _retargeter_win_instance
