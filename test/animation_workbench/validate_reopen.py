"""After install_shelf and validate_maya, test a fresh process with no sys.path setup."""
from pathlib import Path
import json,tempfile
import maya.standalone
maya.standalone.initialize(name='python')
import maya.cmds as cmds
try:
    cmds.file(str(Path(tempfile.gettempdir())/'animation_workbench_fixture.ma'),open=True,force=True)
    from ue_ik_retargeter_maya.animation_workbench import scene
    root=scene.roots()[0];s=scene.state(root);solver=scene.resolve(s['solver'])
    cmds.currentTime(6);matrix=cmds.getAttr(scene.joints(root)[s['role_indices']['R_Hand']]+'.worldMatrix[0]')
    assert len(matrix)==16 and len(scene.source_cache(root)['R_Hand']['positions'])==13
    print('FRESH_PROCESS_REOPEN '+json.dumps({'rig':root,'solverType':cmds.nodeType(solver),'pluginLoaded':cmds.pluginInfo('ue58_pbik_node',q=True,loaded=True),'cacheFrames':13}))
finally:maya.standalone.uninitialize()
