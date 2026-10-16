"""Drag into Maya's viewport, or run this saved file in the Python Script Editor."""
from pathlib import Path
import importlib
import sys

def launch(hot_reload=True):
    base=Path(__file__).resolve().parent
    if str(base) not in sys.path:sys.path.insert(0,str(base))
    package=sys.modules.get('ue_ik_retargeter_maya')
    if package and Path(package.__file__).resolve().parent!=base/'ue_ik_retargeter_maya':
        raise RuntimeError('已加载其他位置的同名工具。请重启 Maya 后从此入口启动，避免破坏现有求解器。')
    importlib.invalidate_caches()
    if hot_reload:
        old_ui=sys.modules.get('ue_ik_retargeter_maya.animation_workbench.ui')
        old_window=getattr(old_ui,'_WINDOW',None)
        if old_window is not None:
            try:old_window.close();old_window.deleteLater()
            except RuntimeError:pass
        for suffix in ['engine','scene','markers','rhythm_layers','ui']:
            name='ue_ik_retargeter_maya.animation_workbench.'+suffix
            if name in sys.modules:importlib.reload(sys.modules[name])
    from ue_ik_retargeter_maya.animation_workbench import show
    return show()

def install_shelf():
    """Persist module paths for scene reopening, and add one reusable shelf button."""
    import maya.cmds as cmds
    import maya.mel as mel
    base=Path(__file__).resolve().parent
    modules=Path(cmds.internalVar(userAppDir=True))/'modules';modules.mkdir(parents=True,exist_ok=True)
    core=base/'ue_ik_retargeter_maya'/'core'
    (modules/'AnimationRhythmWorkbench.mod').write_text(
        '+ AnimationRhythmWorkbench 1.0 '+base.as_posix()+'\n'
        'PYTHONPATH +:= .\nMAYA_PLUG_IN_PATH +:= ue_ik_retargeter_maya/core\n',encoding='utf-8')
    import os
    current=os.environ.get('MAYA_PLUG_IN_PATH','')
    if str(core) not in current.split(os.pathsep):os.environ['MAYA_PLUG_IN_PATH']=str(core)+os.pathsep+current
    shelf=cmds.tabLayout(mel.eval('$tmp=$gShelfTopLevel'),q=True,selectTab=True)
    for child in cmds.shelfLayout(shelf,q=True,childArray=True) or []:
        if cmds.objectTypeUI(child)=='shelfButton' and cmds.shelfButton(child,q=True,docTag=True)=='AnimationRhythmWorkbench':cmds.deleteUI(child)
    entry=str(Path(__file__).resolve())
    command="import runpy; runpy.run_path("+repr(entry)+", run_name='__main__')"
    cmds.shelfButton(parent=shelf,label='IK Rhythm',imageOverlayLabel='IK节奏',image='kinematics.png',annotation='全身 IK / Pose 书签 / 分段节奏',command=command,sourceType='python',docTag='AnimationRhythmWorkbench')
    return launch()

def onMayaDroppedPythonFile(*args,**kwargs):return install_shelf()

if __name__=='__main__':launch()
