# -*- coding: utf-8 -*-
"""
PySide2 / PySide6 UI for Maya IK Retargeter.
Compatible with Maya 2024 (PySide2) and Maya 2025+ (PySide6).
Features:
- Parented to Maya's main window so it never gets hidden behind Maya when clicking viewport.
- Built-in One-Click Hot Reload button for instant code updates without restarting Maya.
- Supports the Two-Step Bind-Pose Cache workflow (100% immune to animation pollution).
"""

import os
import maya.cmds as cmds

try:
    from PySide2 import QtWidgets, QtCore, QtGui
except ImportError:
    from PySide6 import QtWidgets, QtCore, QtGui

from ..core.processor import IKRetargetProcessor
from ..core.retarget_cache import RetargetCache
from ..rig_driver.driver import ControlRigDriver

WINDOW_OBJECT_NAME = "UE5IKRetargeterMayaWindow"


def get_maya_main_window():
    """
    Retrieves Maya's main QMainWindow widget instance.
    Parenting the tool dialog to Maya's main window ensures it always stays
    on top of Maya even when clicking in the 3D Viewport or Outliner.
    """
    try:
        import maya.OpenMayaUI as omui
        main_window_ptr = omui.MQtUtil.mainWindow()
        if main_window_ptr is not None:
            try:
                from shiboken2 import wrapInstance
            except ImportError:
                from shiboken6 import wrapInstance
            return wrapInstance(int(main_window_ptr), QtWidgets.QWidget)
    except Exception:
        pass

    app = QtWidgets.QApplication.instance()
    if app:
        for widget in app.topLevelWidgets():
            if widget.objectName() == "MayaWindow":
                return widget
    return None


class RetargeterWindow(QtWidgets.QDialog):
    """
    Main user interface for UE5-style animation retargeting inside Maya.
    """

    WINDOW_TITLE = "UE5 IK Retargeter for Maya"

    def __init__(self, parent=None):
        if parent is None:
            parent = get_maya_main_window()
        super(RetargeterWindow, self).__init__(parent)

        self.setObjectName(WINDOW_OBJECT_NAME)
        self.setWindowTitle(self.WINDOW_TITLE)
        # Ensure it behaves as a floating tool window owned by Maya (never hidden behind Maya)
        self.setWindowFlags(self.windowFlags() | QtCore.Qt.Window)
        self.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        self.resize(730, 710)

        self.processor = None
        self.rig_driver = None
        self.cached_bind_data = None

        self._build_ui()
        self._init_defaults()

    def _build_ui(self):
        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setSpacing(10)

        # Header Row with Title + Hot Reload Button
        header_bar = QtWidgets.QHBoxLayout()
        banner = QtWidgets.QLabel("Unreal Engine 5 IK Retargeter for Maya")
        banner.setStyleSheet("font-size: 16px; font-weight: bold; color: #5dade2;")
        header_bar.addWidget(banner)
        header_bar.addStretch()

        hot_reload_btn = QtWidgets.QPushButton("🔄 热更新插件 (Hot Reload)")
        hot_reload_btn.setToolTip("重新加载所有插件 Python 模块并刷新界面，无需重启 Maya")
        hot_reload_btn.setStyleSheet(
            "background-color: #34495e; color: #ecf0f1; font-weight: bold; padding: 5px 12px; border-radius: 4px;"
        )
        hot_reload_btn.clicked.connect(self._on_hot_reload)
        header_bar.addWidget(hot_reload_btn)
        main_layout.addLayout(header_bar)

        subtitle = QtWidgets.QLabel(
            "推荐流程：① 导入无动画的 A 和 B 骨骼生成绑定基准缓存 ➔ ② 为 A 挂载动画 ➔ ③ 一键将 A 的动画重定向给 B"
        )
        subtitle.setStyleSheet("color: #aaaaaa; margin-bottom: 4px;")
        main_layout.addWidget(subtitle)

        # --- 1. Character Selection ---
        char_box = QtWidgets.QGroupBox("角色选择 (Character Setup)")
        char_layout = QtWidgets.QGridLayout(char_box)
        char_layout.setSpacing(8)

        char_layout.addWidget(QtWidgets.QLabel("源角色根骨骼 A (Source Root):"), 0, 0)
        self.src_root_edit = QtWidgets.QLineEdit()
        self.src_root_edit.setPlaceholderText("选择源角色 A 的 Root/Hips 骨骼...")
        char_layout.addWidget(self.src_root_edit, 0, 1)
        src_pick_btn = QtWidgets.QPushButton("拾取选中 (Pick)")
        src_pick_btn.clicked.connect(lambda: self._pick_root(self.src_root_edit))
        char_layout.addWidget(src_pick_btn, 0, 2)

        char_layout.addWidget(QtWidgets.QLabel("目标角色根骨骼 B (Target Root):"), 1, 0)
        self.tgt_root_edit = QtWidgets.QLineEdit()
        self.tgt_root_edit.setPlaceholderText("选择目标角色 B 的 Root/Hips 骨骼...")
        char_layout.addWidget(self.tgt_root_edit, 1, 1)
        tgt_pick_btn = QtWidgets.QPushButton("拾取选中 (Pick)")
        tgt_pick_btn.clicked.connect(lambda: self._pick_root(self.tgt_root_edit))
        char_layout.addWidget(tgt_pick_btn, 1, 2)

        auto_find_btn = QtWidgets.QPushButton("🔍 自动探测场景角色 A & B (Auto-Detect Skeletons)")
        auto_find_btn.clicked.connect(self._auto_detect_skeletons)
        char_layout.addWidget(auto_find_btn, 2, 0, 1, 3)

        main_layout.addWidget(char_box)

        # --- 2. Step 1: Bind Pose Retarget Cache ---
        cache_box = QtWidgets.QGroupBox("第一步：A & B 绑定姿态基准缓存 (Step 1: Bind-Pose Retarget Cache)")
        cache_layout = QtWidgets.QVBoxLayout(cache_box)
        cache_layout.setSpacing(6)

        path_bar = QtWidgets.QHBoxLayout()
        path_bar.addWidget(QtWidgets.QLabel("缓存文件路径 (.retarget.json):"))
        self.cache_path_edit = QtWidgets.QLineEdit()
        default_cache = os.path.join(os.path.expanduser("~"), "maya_retarget_bind_cache.retarget.json").replace("\\", "/")
        self.cache_path_edit.setText(default_cache)
        path_bar.addWidget(self.cache_path_edit)

        browse_btn = QtWidgets.QPushButton("浏览...")
        browse_btn.clicked.connect(self._browse_cache_file)
        path_bar.addWidget(browse_btn)
        cache_layout.addLayout(path_bar)

        btn_row = QtWidgets.QHBoxLayout()
        self.build_cache_btn = QtWidgets.QPushButton("📌 1. 生成并保存 A/B 绑定姿态缓存 (Build & Save Bind Cache)")
        self.build_cache_btn.setStyleSheet(
            "background-color: #2e7d32; color: white; font-weight: bold; padding: 8px; border-radius: 5px;"
        )
        self.build_cache_btn.clicked.connect(self._on_build_and_save_cache)
        btn_row.addWidget(self.build_cache_btn)

        self.load_cache_btn = QtWidgets.QPushButton("📂 加载已有缓存文件 (Load Cache)")
        self.load_cache_btn.setStyleSheet("padding: 8px;")
        self.load_cache_btn.clicked.connect(self._on_load_cache)
        btn_row.addWidget(self.load_cache_btn)
        cache_layout.addLayout(btn_row)

        self.cache_status_label = QtWidgets.QLabel("状态：尚未生成绑定姿态缓存（建议在 A 和 B 处于默认绑定姿态时点击生成）")
        self.cache_status_label.setStyleSheet("color: #f39c12; font-size: 12px;")
        cache_layout.addWidget(self.cache_status_label)

        main_layout.addWidget(cache_box)

        # --- 3. Step 2: Animation Retarget Execution ---
        one_click_box = QtWidgets.QGroupBox("第二步：根据基准数据将 A 的动画重定向给 B (Step 2: Retarget Animation)")
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

        sync_range_btn = QtWidgets.QPushButton("同步当前时间滑块范围")
        sync_range_btn.clicked.connect(self._sync_timeline_range)
        time_bar.addWidget(sync_range_btn)
        one_click_layout.addLayout(time_bar)

        self.one_click_btn = QtWidgets.QPushButton("🚀 2. 将当前 A 的动画重定向并烘焙给 B (Retarget & Bake A ➔ B)")
        self.one_click_btn.setStyleSheet(
            "background-color: #1e88e5; color: white; font-size: 15px; font-weight: bold; padding: 12px; border-radius: 6px;"
        )
        self.one_click_btn.clicked.connect(self._on_one_click_bake)
        one_click_layout.addWidget(self.one_click_btn)

        preview_btn = QtWidgets.QPushButton("👁 预览当前帧重定向效果 (Preview Current Frame)")
        preview_btn.clicked.connect(self._on_preview_frame)
        one_click_layout.addWidget(preview_btn)

        main_layout.addWidget(one_click_box)

        # --- 4. Advanced Settings (Collapsible) ---
        self.adv_group = QtWidgets.QGroupBox("高级微调与骨骼链检视 (Advanced Settings - Optional)")
        self.adv_group.setCheckable(True)
        self.adv_group.setChecked(False)
        adv_layout = QtWidgets.QVBoxLayout(self.adv_group)

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

        self.chain_table = QtWidgets.QTableWidget()
        self.chain_table.setColumnCount(5)
        self.chain_table.setHorizontalHeaderLabels([
            "链名称 (Chain)", "源起始", "源结束", "目标起始", "目标结束"
        ])
        self.chain_table.horizontalHeader().setStretchLastSection(True)
        adv_layout.addWidget(self.chain_table)

        main_layout.addWidget(self.adv_group)

    def _on_hot_reload(self):
        """Hot-reloads all plugin modules and re-opens the updated UI."""
        import ue_ik_retargeter_maya
        ue_ik_retargeter_maya.launch(hot_reload=True)

    def _init_defaults(self):
        self._sync_timeline_range()
        self._auto_detect_skeletons()

    def _sync_timeline_range(self):
        try:
            start_f = int(cmds.playbackOptions(q=True, minTime=True))
            end_f = int(cmds.playbackOptions(q=True, maxTime=True))
            self.start_frame_spin.setValue(start_f)
            self.end_frame_spin.setValue(end_f)
        except Exception:
            self.start_frame_spin.setValue(1)
            self.end_frame_spin.setValue(100)

    def _browse_cache_file(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "选择或创建重定向基准缓存文件", self.cache_path_edit.text(), "Retarget Cache (*.retarget.json *.json)"
        )
        if path:
            self.cache_path_edit.setText(path)

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

    def _on_build_and_save_cache(self):
        """Step 1: Captures clean bind pose of A and B, auto-aligns, and saves cache to disk & memory."""
        src = self.src_root_edit.text().strip()
        tgt = self.tgt_root_edit.text().strip()
        if not (cmds.objExists(src) and cmds.objExists(tgt)):
            QtWidgets.QMessageBox.warning(self, "缺少角色", "请先选择场景中处于绑定姿态的 A 和 B 根骨骼。")
            return

        self.processor = IKRetargetProcessor(src, tgt)
        self.processor.auto_align_base_pose()
        self.processor.update_retarget_poses()
        self.rig_driver = ControlRigDriver(self.processor)
        self._refresh_table()

        cache_path = self.cache_path_edit.text().strip()
        self.cached_bind_data = RetargetCache.save_to_dict(self.processor)

        if cache_path:
            self.processor.save_retarget_cache(cache_path)
            self.cache_status_label.setText("✅ 已锁定并保存 A/B 绑定姿态基准缓存：{}".format(cache_path))
        else:
            self.cache_status_label.setText("✅ 已在内存中锁定 A/B 绑定姿态基准缓存")
        self.cache_status_label.setStyleSheet("color: #2ecc71; font-weight: bold; font-size: 12px;")

        QtWidgets.QMessageBox.information(
            self,
            "绑定姿态缓存生成成功",
            "已成功提取并保存 A 与 B 的纯净绑定姿态与骨骼链对齐数据！\n\n现在您可以为源角色 A 导入或挂载任意动画，然后点击下方按钮将动画重定向给 B。"
        )

    def _on_load_cache(self):
        """Loads an existing .retarget.json bind-pose cache file."""
        cache_path = self.cache_path_edit.text().strip()
        if not cache_path or not os.path.exists(cache_path):
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "打开重定向基准缓存文件", "", "Retarget Cache (*.retarget.json *.json)"
            )
            if not path:
                return
            cache_path = path
            self.cache_path_edit.setText(cache_path)

        processor = self._ensure_processor(apply_cache=False)
        if not processor:
            return

        self.cached_bind_data = processor.load_retarget_cache(cache_path)
        self._refresh_table()
        self.cache_status_label.setText("✅ 已从文件加载 A/B 绑定姿态基准缓存：{}".format(cache_path))
        self.cache_status_label.setStyleSheet("color: #2ecc71; font-weight: bold; font-size: 12px;")

    def _ensure_processor(self, apply_cache=True):
        src = self.src_root_edit.text().strip()
        tgt = self.tgt_root_edit.text().strip()

        if not (cmds.objExists(src) and cmds.objExists(tgt)):
            QtWidgets.QMessageBox.warning(
                self, "缺少角色", "请在上方输入或在场景中拾取源角色 A 和目标角色 B 的根骨骼。"
            )
            return None

        if not self.processor or self.processor.source_root != src or self.processor.target_root != tgt:
            self.processor = IKRetargetProcessor(src, tgt)
            self.rig_driver = ControlRigDriver(self.processor)
            self._refresh_table()

        if apply_cache:
            cache_path = self.cache_path_edit.text().strip()
            if self.cached_bind_data is not None:
                RetargetCache.apply_dict_to_processor(self.processor, self.cached_bind_data)
            elif cache_path and os.path.exists(cache_path):
                self.cached_bind_data = self.processor.load_retarget_cache(cache_path)
                self.cache_status_label.setText("✅ 已自动应用绑定姿态缓存：{}".format(cache_path))
                self.cache_status_label.setStyleSheet("color: #2ecc71; font-weight: bold; font-size: 12px;")

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
        processor = self._ensure_processor(apply_cache=True)
        if not processor:
            return
        processor.retarget_frame()

    def _on_one_click_bake(self):
        """Retargets and bakes animation from A to B using the locked bind-pose cache."""
        processor = self._ensure_processor(apply_cache=True)
        if not processor:
            return

        s_f = self.start_frame_spin.value()
        e_f = self.end_frame_spin.value()

        progress = QtWidgets.QProgressDialog(
            "正在根据基准缓存执行 UE5 动画重定向烘焙...", "取消", s_f, e_f, self
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
            self, "重定向完成", "已根据绑定姿态基准数据，将源角色 A 的动画精确重定向并烘焙至目标角色 B！"
        )


def _close_existing_windows():
    """Closes any existing RetargeterWindow instances (including across hot-reloads)."""
    global _retargeter_win_instance
    try:
        if '_retargeter_win_instance' in globals() and _retargeter_win_instance is not None:
            _retargeter_win_instance.close()
            _retargeter_win_instance.deleteLater()
            _retargeter_win_instance = None
    except Exception:
        pass

    app = QtWidgets.QApplication.instance()
    if app:
        for widget in list(app.allWidgets()):
            try:
                if widget.objectName() == WINDOW_OBJECT_NAME:
                    widget.close()
                    widget.deleteLater()
            except Exception:
                pass


def show_ui():
    """Opens or raises the Retargeter UI window on top of Maya's main window."""
    global _retargeter_win_instance
    _close_existing_windows()
    _retargeter_win_instance = RetargeterWindow(parent=get_maya_main_window())
    _retargeter_win_instance.show()
    _retargeter_win_instance.raise_()
    _retargeter_win_instance.activateWindow()
    return _retargeter_win_instance
