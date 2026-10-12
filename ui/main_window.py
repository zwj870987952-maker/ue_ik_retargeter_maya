# -*- coding: utf-8 -*-
"""
PySide2 / PySide6 UI for Maya IK Retargeter.
Compatible with Maya 2024 (PySide2) and Maya 2025+ (PySide6).
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
    """

    WINDOW_TITLE = "UE5 IK Retargeter for Maya"

    def __init__(self, parent=None):
        super(RetargeterWindow, self).__init__(parent)
        self.setWindowTitle(self.WINDOW_TITLE)
        self.resize(720, 680)

        self.processor = None
        self.rig_driver = None

        self._build_ui()
        self._init_defaults()

    def _build_ui(self):
        main_layout = QtWidgets.QVBoxLayout(self)

        # --- 1. Character Selection ---
        char_box = QtWidgets.QGroupBox("1. Characters Selection")
        char_layout = QtWidgets.QGridLayout(char_box)

        char_layout.addWidget(QtWidgets.QLabel("Source Root:"), 0, 0)
        self.src_root_edit = QtWidgets.QLineEdit()
        char_layout.addWidget(self.src_root_edit, 0, 1)
        src_pick_btn = QtWidgets.QPushButton("Pick Selected")
        src_pick_btn.clicked.connect(lambda: self._pick_root(self.src_root_edit))
        char_layout.addWidget(src_pick_btn, 0, 2)

        char_layout.addWidget(QtWidgets.QLabel("Target Root:"), 1, 0)
        self.tgt_root_edit = QtWidgets.QLineEdit()
        char_layout.addWidget(self.tgt_root_edit, 1, 1)
        tgt_pick_btn = QtWidgets.QPushButton("Pick Selected")
        tgt_pick_btn.clicked.connect(lambda: self._pick_root(self.tgt_root_edit))
        char_layout.addWidget(tgt_pick_btn, 1, 2)

        init_btn = QtWidgets.QPushButton("Initialize Processor")
        init_btn.setStyleSheet("background-color: #2b5b84; color: white; font-weight: bold;")
        init_btn.clicked.connect(self._on_initialize)
        char_layout.addWidget(init_btn, 2, 0, 1, 3)

        main_layout.addWidget(char_box)

        # --- 2. Bone Chains Mapping ---
        chain_box = QtWidgets.QGroupBox("2. Bone Chains Mapping")
        chain_layout = QtWidgets.QVBoxLayout(chain_box)

        btn_bar = QtWidgets.QHBoxLayout()
        auto_map_btn = QtWidgets.QPushButton("Auto Map Chains")
        auto_map_btn.clicked.connect(self._on_auto_map)
        btn_bar.addWidget(auto_map_btn)

        add_chain_btn = QtWidgets.QPushButton("Add Chain")
        add_chain_btn.clicked.connect(self._on_add_chain_dialog)
        btn_bar.addWidget(add_chain_btn)

        save_preset_btn = QtWidgets.QPushButton("Export Preset (.json)")
        save_preset_btn.clicked.connect(self._on_export_preset)
        btn_bar.addWidget(save_preset_btn)

        load_preset_btn = QtWidgets.QPushButton("Load Preset (.json)")
        load_preset_btn.clicked.connect(self._on_load_preset)
        btn_bar.addWidget(load_preset_btn)

        chain_layout.addLayout(btn_bar)

        self.chain_table = QtWidgets.QTableWidget()
        self.chain_table.setColumnCount(6)
        self.chain_table.setHorizontalHeaderLabels([
            "Chain Name", "Src Start", "Src End", "Tgt Start", "Tgt End", "IK?"
        ])
        self.chain_table.horizontalHeader().setStretchLastSection(True)
        chain_layout.addWidget(self.chain_table)

        main_layout.addWidget(chain_box)

        # --- 3. Retarget Base Pose Alignment ---
        pose_box = QtWidgets.QGroupBox("3. Retarget Base Pose (A-to-T Pose Alignment)")
        pose_layout = QtWidgets.QHBoxLayout(pose_box)

        self.edit_pose_btn = QtWidgets.QPushButton("Edit Base Pose in Viewport")
        self.edit_pose_btn.clicked.connect(self._on_edit_pose)
        pose_layout.addWidget(self.edit_pose_btn)

        self.save_pose_btn = QtWidgets.QPushButton("Save Aligned Pose")
        self.save_pose_btn.clicked.connect(self._on_save_pose)
        self.save_pose_btn.setEnabled(False)
        pose_layout.addWidget(self.save_pose_btn)

        reset_pose_btn = QtWidgets.QPushButton("Reset to Bind Pose")
        reset_pose_btn.clicked.connect(self._on_reset_pose)
        pose_layout.addWidget(reset_pose_btn)

        main_layout.addWidget(pose_box)

        # --- 4. Pelvis & IK Solver Settings ---
        solver_box = QtWidgets.QGroupBox("4. Solver Parameters")
        solver_layout = QtWidgets.QGridLayout(solver_box)

        solver_layout.addWidget(QtWidgets.QLabel("Height Scale:"), 0, 0)
        self.height_slider = QtWidgets.QDoubleSpinBox()
        self.height_slider.setRange(0.1, 5.0)
        self.height_slider.setSingleStep(0.05)
        self.height_slider.setValue(1.0)
        self.height_slider.valueChanged.connect(self._on_params_changed)
        solver_layout.addWidget(self.height_slider, 0, 1)

        solver_layout.addWidget(QtWidgets.QLabel("Floor Constraint:"), 0, 2)
        self.floor_slider = QtWidgets.QDoubleSpinBox()
        self.floor_slider.setRange(0.0, 1.0)
        self.floor_slider.setSingleStep(0.1)
        self.floor_slider.setValue(1.0)
        self.floor_slider.valueChanged.connect(self._on_params_changed)
        solver_layout.addWidget(self.floor_slider, 0, 3)

        solver_layout.addWidget(QtWidgets.QLabel("Blend to Absolute:"), 1, 0)
        self.abs_slider = QtWidgets.QDoubleSpinBox()
        self.abs_slider.setRange(0.0, 1.0)
        self.abs_slider.setSingleStep(0.1)
        self.abs_slider.setValue(0.0)
        self.abs_slider.valueChanged.connect(self._on_params_changed)
        solver_layout.addWidget(self.abs_slider, 1, 1)

        main_layout.addWidget(solver_box)

        # --- 5. Timeline & Bake Actions ---
        bake_box = QtWidgets.QGroupBox("5. Timeline & Bake")
        bake_layout = QtWidgets.QHBoxLayout(bake_box)

        bake_layout.addWidget(QtWidgets.QLabel("Start:"))
        self.start_frame_spin = QtWidgets.QSpinBox()
        self.start_frame_spin.setRange(-99999, 99999)
        bake_layout.addWidget(self.start_frame_spin)

        bake_layout.addWidget(QtWidgets.QLabel("End:"))
        self.end_frame_spin = QtWidgets.QSpinBox()
        self.end_frame_spin.setRange(-99999, 99999)
        bake_layout.addWidget(self.end_frame_spin)

        preview_btn = QtWidgets.QPushButton("Preview Current Frame")
        preview_btn.clicked.connect(self._on_preview_frame)
        bake_layout.addWidget(preview_btn)

        bake_btn = QtWidgets.QPushButton("Bake to Skeleton")
        bake_btn.setStyleSheet("background-color: #2e7d32; color: white; font-weight: bold; padding: 6px;")
        bake_btn.clicked.connect(self._on_bake)
        bake_layout.addWidget(bake_btn)

        main_layout.addWidget(bake_box)

    def _init_defaults(self):
        try:
            start_f = int(cmds.playbackOptions(q=True, minTime=True))
            end_f = int(cmds.playbackOptions(q=True, maxTime=True))
            self.start_frame_spin.setValue(start_f)
            self.end_frame_spin.setValue(end_f)
        except Exception:
            self.start_frame_spin.setValue(1)
            self.end_frame_spin.setValue(100)

    def _pick_root(self, line_edit):
        sel = cmds.ls(sl=True, type='joint')
        if sel:
            line_edit.setText(sel[0])

    def _on_initialize(self):
        src = self.src_root_edit.text().strip()
        tgt = self.tgt_root_edit.text().strip()
        if not (cmds.objExists(src) and cmds.objExists(tgt)):
            QtWidgets.QMessageBox.warning(self, "Error", "Both Source and Target Root joints must exist in scene.")
            return

        self.processor = IKRetargetProcessor(src, tgt)
        self.rig_driver = ControlRigDriver(self.processor)
        self._on_auto_map()
        QtWidgets.QMessageBox.information(self, "Success", "IK Retarget Processor initialized successfully!")

    def _on_auto_map(self):
        if not self.processor:
            return
        self.processor.auto_map_chains()
        self._refresh_table()

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

            cb = QtWidgets.QCheckBox()
            cb.setChecked(mapping.enable_ik)
            self.chain_table.setCellWidget(row, 5, cb)

    def _on_add_chain_dialog(self):
        # Quick dialog to add custom chain
        pass

    def _on_edit_pose(self):
        if not self.processor:
            return
        self.processor.target_pose_mgr.start_edit_mode()
        self.edit_pose_btn.setEnabled(False)
        self.save_pose_btn.setEnabled(True)
        QtWidgets.QMessageBox.information(
            self, "Pose Editing",
            "Target skeleton is now in Edit Mode.\nRotate/translate target joints in the viewport to match Source Base Pose (e.g. A-Pose to T-Pose), then click 'Save Aligned Pose'."
        )

    def _on_save_pose(self):
        if not self.processor:
            return
        self.processor.target_pose_mgr.save_edit_mode()
        self.edit_pose_btn.setEnabled(True)
        self.save_pose_btn.setEnabled(False)
        QtWidgets.QMessageBox.information(self, "Saved", "Base retarget pose alignment offsets saved successfully.")

    def _on_reset_pose(self):
        if not self.processor:
            return
        self.processor.target_pose_mgr.reset_to_bind_pose()
        self.edit_pose_btn.setEnabled(True)
        self.save_pose_btn.setEnabled(False)

    def _on_params_changed(self):
        if not self.processor or not self.processor.pelvis_solver:
            return
        self.processor.pelvis_solver.scale_vertical = self.height_slider.value()
        self.processor.pelvis_solver.floor_constraint_weight = self.floor_slider.value()
        self.processor.pelvis_solver.blend_to_absolute = self.abs_slider.value()

    def _on_preview_frame(self):
        if not self.processor:
            return
        self._on_params_changed()
        self.processor.retarget_frame()

    def _on_bake(self):
        if not self.processor:
            return
        self._on_params_changed()
        s_f = self.start_frame_spin.value()
        e_f = self.end_frame_spin.value()

        # Show progress dialog
        progress = QtWidgets.QProgressDialog("Baking retargeted animation...", "Cancel", s_f, e_f, self)
        progress.setWindowModality(QtCore.Qt.WindowModal)

        for f in range(s_f, e_f + 1):
            if progress.wasCanceled():
                break
            cmds.currentTime(f, e=True)
            self.processor.retarget_frame()
            for j in self.processor.target_skeleton.joint_names:
                cmds.setKeyframe(j, time=f, attribute=['translate', 'rotate'])
            progress.setValue(f)

        progress.setValue(e_f)
        QtWidgets.QMessageBox.information(self, "Complete", "Animation baked onto target skeleton successfully!")

    def _on_export_preset(self):
        pass

    def _on_load_preset(self):
        pass


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
