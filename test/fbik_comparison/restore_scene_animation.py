"""Restore missing original animCurve nodes from the saved test_maya ASCII.

Only the original curve definitions and their outgoing connections are replayed.
The mesh, skeleton, materials, and assembled FBIK nodes are not imported.
"""
import json
import re
from pathlib import Path
import maya.cmds as cmds
import maya.mel as mel
from ue_ik_retargeter_maya.core.fbik_solver import _ACTIVE_FBIK_SOLVERS

HERE=Path(__file__).resolve().parent
scene=HERE.parent/'test_maya.ma'
# Curve node names and numeric MEL statements are ASCII; unrelated localized
# metadata in this Maya ASCII file uses the system code page.
text=scene.read_text(encoding='latin-1')
baseline=json.loads(HERE.joinpath('maya_before_assembly.json').read_text(encoding='utf-8'))
expected=set(baseline['anim_curves'])
sections={}
for match in re.finditer(r'^createNode (animCurve\w+) -n "([^"]+)";\r?\n(.*?)(?=^createNode |^select |\Z)',text,re.M|re.S):
    if match.group(2) in expected:
        # Curve definitions consist only of rename -uid / setAttr statements.
        body=match.group(3)
        assert not re.search(r'^(connectAttr|disconnectAttr|delete|file)\b',body,re.M)
        sections[match.group(2)]=match.group(0)
assert set(sections)==expected, (len(sections),len(expected))
connections=[]
for match in re.finditer(r'^connectAttr "([^"]+)" "([^"]+)";',text,re.M):
    source,destination=match.groups()
    if source.split('.')[0] in expected:
        assert cmds.objExists(destination),destination
        connections.append((source,destination))
assert len(connections)==len(expected),(len(connections),len(expected))
for solver in _ACTIVE_FBIK_SOLVERS.values():solver._is_solving=True
restored=[]
selection=cmds.ls(sl=True,long=True) or []
cmds.undoInfo(openChunk=True,chunkName='Restore original test_maya animation curves')
try:
    restored=[name for name in sections if not cmds.objExists(name)]
    if restored:
        # Internal curve attributes such as .tan / .ktv are accepted by Maya's
        # ASCII file reader, not by ordinary setAttr evaluation.
        recovery=HERE/'original_animation_curves.ma'
        recovery.write_text('//Maya ASCII 2024 scene\nrequires maya "2024";\n'+
            '\n'.join(sections[name] for name in restored),encoding='ascii')
        cmds.file(str(recovery),i=True,type='mayaAscii',namespace=':',mergeNamespacesOnClash=True,options='v=0;')
    for source,destination in connections:
        if not cmds.isConnected(source,destination):
            assert not cmds.listConnections(destination,s=True,d=False,plugs=True),destination
            cmds.connectAttr(source,destination)
    result={'restored_nodes':len(restored),'original_nodes':len(expected),
            'all_nodes_exist':all(cmds.objExists(n) for n in expected),
            'all_connections_restored':all(cmds.isConnected(s,d) for s,d in connections)}
    HERE.joinpath('animation_restore_verification.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result))
finally:
    cmds.select(selection,r=True) if selection else cmds.select(clear=True)
    cmds.undoInfo(closeChunk=True)
    for solver in _ACTIVE_FBIK_SOLVERS.values():solver._is_solving=False
