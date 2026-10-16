"""Reusable, explicit-apply Maya UI. No per-frame callbacks or background baking."""
import traceback
import maya.cmds as cmds
import maya.OpenMayaUI as omui
try:
    from PySide6 import QtCore,QtGui,QtWidgets
    from shiboken6 import wrapInstance
except ImportError:
    from PySide2 import QtCore,QtGui,QtWidgets
    from shiboken2 import wrapInstance
from . import engine,scene,markers,rhythm_layers

_WINDOW=None
LABELS={r:label for r,label,_ in scene.ROLES}

class HoldRange(QtWidgets.QWidget):
    changed=QtCore.Signal(int,int)
    def __init__(self,parent=None):
        super().__init__(parent);self.left=2;self.right=3;self.maximum=30;self.active=None
        self.setMinimumHeight(85);self.setMinimumWidth(330)
        self.setToolTip('上方蓝色光标：向左 hold；下方橙色光标：向右 hold。两个光标均可拖到 B（0 帧）。')
    def setValues(self,left,right):
        self.left=int(left);self.right=int(right);self.maximum=max(30,self.left,self.right);self.update()
    def _x(self,value):return self.width()/2.+value*(self.width()-44)/(2.*self.maximum)
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing)
        y=43;center=self._x(0)
        p.setPen(QtGui.QPen(QtGui.QColor('#68717e'),2));p.drawLine(22,y,self.width()-22,y)
        p.setPen(QtGui.QPen(QtGui.QColor('#e6e9ef'),2));p.drawLine(int(center),y-15,int(center),y+15)
        p.drawText(QtCore.QRectF(center-22,0,44,23),QtCore.Qt.AlignCenter,'B')
        p.setPen(QtGui.QPen(QtGui.QColor('#59aafa'),5));p.drawLine(int(self._x(-self.left)),y-4,int(center),y-4)
        p.setPen(QtGui.QPen(QtGui.QColor('#ffb35c'),5));p.drawLine(int(center),y+4,int(self._x(self.right)),y+4)
        for x,dy,color,text in [(self._x(-self.left),-12,'#59aafa',str(self.left)+' 帧'),(self._x(self.right),12,'#ffb35c',str(self.right)+' 帧')]:
            p.setBrush(QtGui.QColor(color));p.setPen(QtGui.QPen(QtGui.QColor('#20242b'),1));p.drawEllipse(QtCore.QPointF(x,y+dy),7,7)
            p.setPen(QtGui.QColor(color));p.drawText(QtCore.QRectF(x-36,3 if dy<0 else 63,72,19),QtCore.Qt.AlignCenter,text)
    def mousePressEvent(self,event):
        pt=event.position() if hasattr(event,'position') else event.localPos()
        left=QtCore.QPointF(self._x(-self.left),31);right=QtCore.QPointF(self._x(self.right),55)
        self.active='left' if (pt-left).manhattanLength()<=(pt-right).manhattanLength() else 'right'
        self.mouseMoveEvent(event)
    def mouseMoveEvent(self,event):
        if self.active is None:return
        pt=event.position() if hasattr(event,'position') else event.localPos()
        delta=(pt.x()-self.width()/2.)*2.*self.maximum/(self.width()-44)
        if self.active=='left':self.left=max(0,min(self.maximum,int(round(-delta))))
        else:self.right=max(0,min(self.maximum,int(round(delta))))
        self.update();self.changed.emit(self.left,self.right)
    def mouseReleaseEvent(self,event):self.active=None

def spin(low,high,value,decimals=0,suffix=''):
    widget=QtWidgets.QDoubleSpinBox() if decimals else QtWidgets.QSpinBox()
    widget.setRange(low,high);widget.setValue(value);widget.setSuffix(suffix)
    if decimals:widget.setDecimals(decimals);widget.setSingleStep(.5)
    return widget

class LayerSlider(QtWidgets.QWidget):
    valueChanged=QtCore.Signal(float)
    dragStarted=QtCore.Signal()
    dragFinished=QtCore.Signal()
    def __init__(self,value=100.,parent=None):
        super().__init__(parent);layout=QtWidgets.QHBoxLayout(self);layout.setContentsMargins(0,0,0,0)
        self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal);self.slider.setRange(0,1000);self.slider.setTracking(True);self.slider.setMinimumWidth(105)
        self.number=spin(0,100,value,1,' %');self.number.setFixedWidth(82);self.number.setKeyboardTracking(False)
        layout.addWidget(self.slider,1);layout.addWidget(self.number);self.setValue(value)
        self.slider.valueChanged.connect(lambda v:self.setValue(v/10.));self.number.valueChanged.connect(self.setValue)
        self.slider.sliderPressed.connect(self.dragStarted.emit);self.slider.sliderReleased.connect(self.dragFinished.emit)
        self.setToolTip('直接混合 Maya 动画层；拖动实时预览，不重新采样姿态曲线。')
    def value(self):return self.number.value()
    def setValue(self,value):
        value=max(0.,min(100.,float(value)));old=self.number.value()
        self.number.blockSignals(True);self.slider.blockSignals(True)
        self.number.setValue(value);self.slider.setValue(int(round(value*10.)))
        self.number.blockSignals(False);self.slider.blockSignals(False)
        if abs(self.number.value()-old)>.00001:self.valueChanged.emit(self.number.value())

class Workbench(QtWidgets.QDialog):
    def __init__(self,parent):
        super().__init__(parent);self.setObjectName('animationRhythmWorkbench');self.setWindowTitle('全身 IK · Pose 书签 · 分段节奏')
        self.resize(960,820);self.setMinimumSize(820,720);self.root=None;self.busy=False;self.role_checks={};self.weights={};self.fields={};self._preview_context=None
        layout=QtWidgets.QVBoxLayout(self)
        bar=QtWidgets.QHBoxLayout();layout.addLayout(bar);bar.addWidget(QtWidgets.QLabel('当前绑定'))
        self.rigs=QtWidgets.QComboBox();self.rigs.setMinimumWidth(300);bar.addWidget(self.rigs,1)
        self.button(bar,'刷新 / 接入已有绑定',self.refresh)
        self.rigs.currentIndexChanged.connect(self._rig_changed)
        self.tabs=QtWidgets.QTabWidget();layout.addWidget(self.tabs,1)
        self._build_ik();self._build_bookmarks();self._build_rhythm()
        self.status=QtWidgets.QLabel('就绪');self.status.setWordWrap(True);layout.addWidget(self.status)
        self.setStyleSheet('QGroupBox {font-weight:600;margin-top:10px;padding-top:12px;} QPushButton {min-height:27px;} QTabWidget::pane {border:1px solid #555;}')
        self.refresh()
    def button(self,layout,text,action):
        b=QtWidgets.QPushButton(text);layout.addWidget(b);b.clicked.connect(lambda checked=False:self.run(action));return b
    def run(self,action):
        if self.busy:return
        self.busy=True;QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            result=action()
            if isinstance(result,str):self.status.setText(result)
        except Exception as exc:
            traceback.print_exc();self.status.setText('操作失败：'+str(exc));cmds.warning(str(exc))
        finally:QtWidgets.QApplication.restoreOverrideCursor();self.busy=False
    def _page(self,title):
        page=QtWidgets.QWidget();box=QtWidgets.QVBoxLayout(page);self.tabs.addTab(page,title);return box
    def _build_ik(self):
        box=self._page('1  全身 IK')
        note=QtWidgets.QLabel('以第 0 帧为参考 pose，保留原骨骼动画；控制器按整帧跟随原动画。生成后仍可移动任意 IK，驱动全身联动。');note.setWordWrap(True);box.addWidget(note)
        group=QtWidgets.QGroupBox('生成');form=QtWidgets.QFormLayout(group);box.addWidget(group)
        row=QtWidgets.QHBoxLayout();self.skeleton=QtWidgets.QLineEdit();row.addWidget(self.skeleton,1);self.button(row,'使用选中骨架',self.pick_skeleton);form.addRow('骨架根节点',row)
        row=QtWidgets.QHBoxLayout();self.start=spin(-100000,100000,int(cmds.playbackOptions(q=True,min=True)));self.end=spin(-100000,100000,int(cmds.playbackOptions(q=True,max=True)));self.ref=spin(-100000,100000,0)
        for label,w in [('开始',self.start),('结束',self.end),('参考',self.ref)]:row.addWidget(QtWidgets.QLabel(label));row.addWidget(w)
        form.addRow('帧范围',row)
        self.source_layer=QtWidgets.QCheckBox('将控制器源动画放在独立动画层（默认直接在控制器上）');form.addRow(self.source_layer)
        self.button(form,'一键生成全身 IK + 控制器动画',self.create)
        group=QtWidgets.QGroupBox('胸部与膝盖稳定');form=QtWidgets.QFormLayout(group);box.addWidget(group)
        self.stability=spin(0,98,85,suffix=' %');form.addRow('脊柱 / 肩部稳定强度',self.stability)
        self.button(form,'应用胸部稳定强度',self.set_stability)
        text=QtWidgets.QLabel('提高脊柱与肩部对源姿态的保持，减少双手拉扯造成的胸部抖动。数值越高，上半身跟随手部位移的幅度越小。');text.setWordWrap(True);form.addRow(text)
        self.leg_stability=spin(0,100,100,suffix=' %');form.addRow('膝盖朝向保持',self.leg_stability)
        self.button(form,'应用膝盖朝向保持',self.set_leg_stability)
        text=QtWidgets.QLabel('保持源动画的膝盖弯曲平面，减少拉手时膝盖侧摆；骨盆带动腿部的正常弯曲仍保留。');text.setWordWrap(True);form.addRow(text)
        group=QtWidgets.QGroupBox('源动画与删除');form=QtWidgets.QVBoxLayout(group);box.addWidget(group)
        self.button(form,'更新控制器源动画缓存（修改基础动画后使用）',self.capture)
        self.button(form,'删除全身 IK · 保留原骨骼源动画',self.remove_restore)
        self.button(form,'烘焙当前骨骼结果到独立动画层 · 删除全身 IK',self.remove_bake)
        note=QtWidgets.QLabel('上述操作均可 Undo。烘焙保留原动画曲线在底层；结果层设为 0 可回到原动画。场景不会自动保存。');note.setWordWrap(True);box.addWidget(note);box.addStretch()
    def _build_bookmarks(self):
        box=self._page('2  Pose 书签')
        note=QtWidgets.QLabel('书签起始帧就是重点 pose B。自动识别只添加标识，不改动画；可以在时间轴移动书签，然后重新应用节奏。');note.setWordWrap(True);box.addWidget(note)
        group=QtWidgets.QGroupBox('手动添加部位（与节奏面板同步）');grid=QtWidgets.QGridLayout(group);box.addWidget(group);self.marker_checks={}
        for index,(role,label,_) in enumerate(scene.ROLES):
            check=QtWidgets.QCheckBox(label);check.setChecked(role in scene.DEFAULT_ROLES);self.marker_checks[role]=check;grid.addWidget(check,index//4,index%4)
        row=QtWidgets.QHBoxLayout();box.addLayout(row)
        self.min_gap=spin(1,1000,30,suffix=' 帧');self.max_count=spin(1,100,8)
        row.addWidget(QtWidgets.QLabel('最小间隔'));row.addWidget(self.min_gap);row.addWidget(QtWidgets.QLabel('每部位最多'));row.addWidget(self.max_count)
        self.button(row,'自动识别全身重点 pose',self.generate)
        row=QtWidgets.QHBoxLayout();box.addLayout(row)
        self.button(row,'当前帧添加书签（勾选部位）',self.add_markers)
        self.button(row,'删除所选书签',self.delete_markers);self.button(row,'刷新书签',self.refresh_markers)
        self.tree=QtWidgets.QTreeWidget();self.tree.setHeaderLabels(['部位','B 帧','左 hold','右 hold','来源 / 节点']);self.tree.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection);box.addWidget(self.tree,1)
        self.tree.itemSelectionChanged.connect(self.marker_selected);self.tree.itemDoubleClicked.connect(lambda item,col:cmds.currentTime(int(item.text(1))))
        row=QtWidgets.QHBoxLayout();box.addLayout(row);self.button(row,'用所选书签的参数打开节奏面板',lambda:self.tabs.setCurrentIndex(2))
    def _build_rhythm(self):
        box=self._page('3  分段节奏')
        group=QtWidgets.QGroupBox('处理部位 / 总强度 · 动画层实时预览');grid=QtWidgets.QGridLayout(group);box.addWidget(group)
        for index,(role,label,_) in enumerate(scene.ROLES):
            check=QtWidgets.QCheckBox(label);check.setChecked(role in scene.DEFAULT_ROLES);self.role_checks[role]=check
            check.toggled.connect(self.marker_checks[role].setChecked);self.marker_checks[role].toggled.connect(check.setChecked)
            weight=LayerSlider();self.weights[role]=weight
            weight.dragStarted.connect(self.begin_preview);weight.dragFinished.connect(self.end_preview)
            weight.valueChanged.connect(lambda value,r=role:self.preview_weight(r,value))
            col=(index//4)*3;row=index%4;grid.addWidget(check,row,col);grid.addWidget(weight,row,col+1)
            b=QtWidgets.QPushButton('曲线');b.setFixedWidth(54);b.clicked.connect(lambda checked=False,r=role:self.run(lambda:self.graph(r)));grid.addWidget(b,row,col+2)
        group=QtWidgets.QGroupBox('Hold · 以 B 为中心，左右独立整帧');form=QtWidgets.QVBoxLayout(group);box.addWidget(group)
        self.hold=HoldRange();form.addWidget(self.hold)
        row=QtWidgets.QHBoxLayout();form.addLayout(row)
        for key,label in [('hold_left','向左'),('hold_right','向右')]:
            row.addWidget(QtWidgets.QLabel(label));self.fields[key]=spin(0,300,engine.DEFAULTS[key],suffix=' 帧');row.addWidget(self.fields[key]);self.fields[key].valueChanged.connect(self.sync_hold)
        row.addWidget(QtWidgets.QLabel('Hold 强度'));self.fields['hold_strength']=self.strength_slider('hold');row.addWidget(self.fields['hold_strength'],1)
        self.hold.changed.connect(self.hold_changed)
        group=QtWidgets.QGroupBox('反向缓冲 / 反向预备 · 均在 hold 区间之外');grid=QtWidgets.QGridLayout(group);box.addWidget(group)
        for col,text in enumerate(['效果','过渡段占比','空间幅度','独立强度 · 实时动画层']):grid.addWidget(QtWidgets.QLabel(text),0,col)
        for row,prefix,label in [(1,'settle','到达前缓冲'),(2,'prep','出发前预备')]:
            grid.addWidget(QtWidgets.QLabel(label),row,0)
            for col,part in enumerate(['duration','amplitude','strength'],1):
                key=prefix+'_'+part;w=self.strength_slider(prefix) if part=='strength' else spin(0,100,engine.DEFAULTS[key],1,' %');self.fields[key]=w;grid.addWidget(w,row,col)
                w.setToolTip('占比：本段 A→Hold 起点或 Hold 终点→C 的可用时间。幅度：原本段空间弧长的百分比。强度：独立混合比例。')
        row=QtWidgets.QHBoxLayout();box.addLayout(row)
        row.addWidget(QtWidgets.QLabel('每个 B 最大影响半径'));self.fields['support_frames']=spin(3,300,22,suffix=' 帧');row.addWidget(self.fields['support_frames'])
        self.rotate=QtWidgets.QCheckBox('旋转同步变速');self.rotate.setChecked(True);row.addWidget(self.rotate);row.addStretch()
        self.scope=QtWidgets.QLabel('未选择书签：参数应用到勾选部位的全部 B。');self.scope.setWordWrap(True);box.addWidget(self.scope)
        row=QtWidgets.QHBoxLayout();box.addLayout(row)
        self.button(row,'应用时长 / 幅度并重建节奏',self.apply)
        self.button(row,'仅按当前书签重新生成',self.rebuild)
        self.button(row,'恢复源动画预览（效果设为 0）',self.restore)
        box.addStretch()
    def require_root(self):
        if not self.root or not cmds.objExists(self.root):raise ValueError('请先生成全身 IK，或刷新并接入已有绑定')
        return self.root
    def roles(self):
        s=scene.state(self.require_root());result=[r for r,c in self.role_checks.items() if c.isChecked() and r in s['controls']]
        if not result:raise ValueError('至少勾选一个处理部位')
        return result
    def refresh(self):
        self.end_preview()
        for node in cmds.ls('*.rigManifest',objectsOnly=True) or []:
            if not cmds.attributeQuery('awState',node=node,exists=True):
                scene.adopt_legacy(node);markers.migrate_legacy(node)
        for node in scene.roots():
            if scene.state(node).get('leg_plane_version',0)<1:
                with scene.operation('Add animated knee plane stability'):scene.set_leg_stability(node,1.)
            s=scene.state(node);old_roles=[r for r in s['rhythm_layers'] if r not in s.get('rhythm_components',{})]
            if old_roles:scene.apply_rhythm(node,old_roles)
        old=scene.uid(self.root) if self.root and cmds.objExists(self.root) else None
        self.rigs.blockSignals(True);self.rigs.clear()
        for node in scene.roots():self.rigs.addItem(node.split('|')[-1],scene.uid(node))
        index=self.rigs.findData(old);self.rigs.setCurrentIndex(max(0,index));self.rigs.blockSignals(False);self._rig_changed()
    def _rig_changed(self,*unused):
        self.end_preview()
        value=self.rigs.currentData();self.root=scene.resolve(value) if value else None
        if not self.root:return
        s=scene.state(self.root);self.skeleton.setText(scene.resolve(s['root_joint']));self.start.setValue(s['start']);self.end.setValue(s['end']);self.ref.setValue(s['reference_frame'])
        self.stability.setValue(round(cmds.getAttr(self.root+'.awChestStability')*100));self.set_settings(s['settings']);self.refresh_weights();self.refresh_markers()
        self.leg_stability.setValue(round(cmds.getAttr(self.root+'.awLegPlaneStability')*100))
    def pick_skeleton(self):
        from ..core.skeleton import resolve_skeleton_root_joint
        chosen=cmds.ls(sl=True,long=True) or [];node=resolve_skeleton_root_joint(chosen[0]) if chosen else None
        if not node:raise ValueError('请在 Maya 中选择骨架根节点、任一骨骼或蒙皮网格')
        self.skeleton.setText(node)
    def create(self):
        root=scene.create(self.skeleton.text().strip(),self.start.value(),self.end.value(),self.ref.value(),self.source_layer.isChecked());self.root=root;self.refresh();return '已生成全身 IK，控制器动画与源动画同步。'
    def set_stability(self):
        with scene.operation('Adjust chest stability'):scene.set_stability(self.require_root(),self.stability.value()/100.)
        return '胸部稳定强度已应用。'
    def set_leg_stability(self):
        with scene.operation('Adjust animated knee plane stability'):scene.set_leg_stability(self.require_root(),self.leg_stability.value()/100.)
        return '膝盖朝向保持已应用，脚端与骨盆继续使用全身 IK 求解结果。'
    def capture(self):scene.recapture_controls(self.require_root());return '已更新源动画缓存；按当前书签重新生成即可使用。'
    def remove_restore(self):scene.remove_restore(self.require_root());self.root=None;self.refresh();return '全身 IK 已删除，原骨骼源动画已恢复。'
    def remove_bake(self):
        layer=scene.remove_bake(self.require_root());self.root=None;self.refresh();return '骨骼结果已烘焙到 '+layer+'，全身 IK 已删除。'
    def refresh_markers(self):
        selected=set(self.selected_nodes()) if hasattr(self,'tree') else set();self.tree.blockSignals(True);self.tree.clear()
        if self.root and cmds.objExists(self.root):
            for m in markers.list_markers(self.root):
                values=m['settings'];item=QtWidgets.QTreeWidgetItem([LABELS[m['role']],str(m['frame']),str(values['hold_left']),str(values['hold_right']),('自动' if m['automatic'] else '手动')+' · '+m['node']]);item.setData(0,QtCore.Qt.UserRole,m['node']);self.tree.addTopLevelItem(item);item.setSelected(m['node'] in selected)
        for i in range(4):
            self.tree.resizeColumnToContents(i);self.tree.setColumnWidth(i,self.tree.columnWidth(i)+12)
        self.tree.blockSignals(False);self.update_scope()
    def selected_nodes(self):return [i.data(0,QtCore.Qt.UserRole) for i in self.tree.selectedItems()]
    def marker_selected(self):
        nodes=self.selected_nodes()
        if nodes:self.set_settings(markers.settings(nodes[0]))
        self.update_scope()
    def update_scope(self):
        n=len(self.selected_nodes());self.scope.setText(('已选 '+str(n)+' 个 B：独立强度实时作用于这些 B 的动画层权重。' if n else '独立强度实时作用于勾选部位的全部 B。')+' 时长 / 幅度修改后点击应用。')
    def add_markers(self):
        nodes=markers.add(self.require_root(),self.roles(),values=self.settings());self.refresh_markers();return '已在当前帧添加 / 保留 '+str(len(nodes))+' 个重点 pose 书签。'
    def delete_markers(self):
        n=markers.delete(self.require_root(),self.selected_nodes());self.refresh_markers();return '已删除 '+str(n)+' 个书签；点击“仅按当前书签重新生成”更新动画。'
    def generate(self):
        root=self.require_root();roles=[r for r in scene.DEFAULT_ROLES if r in scene.state(root)['controls']]
        result=markers.generate(root,roles,self.min_gap.value(),self.max_count.value());self.refresh_markers();return '已自动识别 '+str(sum(map(len,result.values())))+' 个重点 pose；动画曲线未修改，手动书签已保留。'
    def sync_hold(self,*unused):self.hold.setValues(self.fields['hold_left'].value(),self.fields['hold_right'].value())
    def hold_changed(self,left,right):
        for key,value in [('hold_left',left),('hold_right',right)]:self.fields[key].blockSignals(True);self.fields[key].setValue(value);self.fields[key].blockSignals(False)
    def settings(self):
        values={key:w.value() for key,w in self.fields.items()};values['rotate_timing']=self.rotate.isChecked();return values
    def set_settings(self,values):
        for key,w in self.fields.items():
            if key in values:w.blockSignals(True);w.setValue(values[key]);w.blockSignals(False)
        self.rotate.setChecked(values.get('rotate_timing',True));self.sync_hold()
    def apply(self):
        root=self.require_root();nodes=self.selected_nodes();roles=list(dict.fromkeys(cmds.getAttr(n+'.awRole') for n in nodes)) if nodes else self.roles()
        result=scene.apply_rhythm(root,roles,self.settings(),nodes);self.refresh_markers();self.refresh_weights();return '已重建 '+str(len(result))+' 个部位；整帧采样，参考 pose 保留。'
    def rebuild(self):
        result=scene.apply_rhythm(self.require_root(),self.roles());self.refresh_weights();return '已依据现有书签重建 '+str(len(result))+' 个部位。'
    def restore(self):scene.restore_controls(self.require_root(),self.roles());self.refresh_weights();return '勾选部位的效果预览已设为 0；书签和参数保留。'
    def preview_weight(self,role,value):
        if not self.root or not cmds.objExists(self.root):return
        layer=scene.state(self.root)['rhythm_layers'].get(role)
        if layer and cmds.objExists(layer):
            self.change_preview(lambda:cmds.setAttr(layer+'.weight',value/100.))
    def strength_slider(self,part):
        widget=LayerSlider(engine.DEFAULTS[part+'_strength'])
        widget.dragStarted.connect(self.begin_preview);widget.dragFinished.connect(self.end_preview)
        widget.valueChanged.connect(lambda value,p=part:self.preview_strength(p,value))
        return widget
    def begin_preview(self):
        if self._preview_context is None:
            self._preview_context=rhythm_layers.edit_weights();self._preview_context.__enter__()
    def end_preview(self):
        if self._preview_context is not None:
            context=self._preview_context;self._preview_context=None;context.__exit__(None,None,None)
    def change_preview(self,action):
        try:
            if self._preview_context is not None:action()
            else:
                with rhythm_layers.edit_weights():action()
            cmds.refresh(force=True)
        except Exception as exc:
            if self._preview_context is not None:
                context=self._preview_context;self._preview_context=None;context.__exit__(type(exc),exc,exc.__traceback__)
            traceback.print_exc();self.status.setText('预览失败：'+str(exc));cmds.warning(str(exc))
    def preview_strength(self,part,value):
        if not self.root or not cmds.objExists(self.root):return
        def action():
            nodes=self.selected_nodes();roles=list(dict.fromkeys(cmds.getAttr(n+'.awRole') for n in nodes)) if nodes else self.roles()
            affected=rhythm_layers.set_strength(self.root,roles,part,value,nodes)
            self.status.setText('正在实时混合动画层。' if affected else '参数已记录；请先应用生成所选部位的节奏层。')
        self.change_preview(action)
    def refresh_weights(self):
        s=scene.state(self.root)
        for role,w in self.weights.items():
            layer=s['rhythm_layers'].get(role);w.blockSignals(True);w.setValue(cmds.getAttr(layer+'.weight')*100. if layer and cmds.objExists(layer) else 100.);w.blockSignals(False)
            w.setEnabled(role in s['controls'] and bool(layer and cmds.objExists(layer)));self.role_checks[role].setEnabled(role in s['controls'])
            self.marker_checks[role].setEnabled(role in s['controls'])
    def graph(self,role):
        import maya.mel as mel
        c=scene.controller(self.require_root(),role);cmds.select(c,r=True);mel.eval('GraphEditor;')
        layer=scene.state(self.root)['rhythm_layers'].get(role)
        info=scene.state(self.root).get('rhythm_components',{}).get(role)
        if info:layer=scene.resolve(info['layers']['hold'])
        if layer and cmds.objExists(layer):cmds.animLayer(layer,e=True,selected=True,preferred=True)
        return '已打开 '+LABELS[role]+' 控制器的动画曲线。'
    def closeEvent(self,event):
        self.end_preview();super().closeEvent(event)

def show():
    global _WINDOW
    if _WINDOW is not None:
        try:_WINDOW.close();_WINDOW.deleteLater()
        except RuntimeError:pass
    for old in ['leftHandRhythmCurveTool','fullBodyRhythmCurveTool']:
        if cmds.window(old,exists=True):cmds.deleteUI(old)
    parent=wrapInstance(int(omui.MQtUtil.mainWindow()),QtWidgets.QWidget)
    _WINDOW=Workbench(parent);_WINDOW.show();_WINDOW.raise_();return _WINDOW
