"""Build the standalone PBIK adapter from an installed UE 5.8 source tree.

The generated Unreal sources stay in the ignored build directory. Only the
small portability layer / C ABI belong to this repository. The resulting DLL
has no Unreal DLL dependencies and is loaded by both Maya 2024 and 2025.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

HERE=Path(__file__).resolve().parent

def build(engine_root=None):
    root=Path(engine_root or os.environ.get('UE58_ENGINE_ROOT',r'E:\UE5\UE_5.8\Engine'))
    source=root/'Plugins/Experimental/FullBodyIK/Source/PBIK'
    paths=[source/'Public/Core'/name for name in ['PBIKSolver.h','PBIKBody.h','PBIKConstraint.h','PBIKDebug.h']]
    paths += [source/'Private/Core'/name for name in ['PBIKSolver.cpp','PBIKBody.cpp','PBIKConstraint.cpp']]
    if not all(p.is_file() for p in paths):
        raise RuntimeError('Set UE58_ENGINE_ROOT to the Engine directory of a UE 5.8 installation with PBIK source.')
    version=root/'Build/Build.version'
    if version.exists():
        v=json.loads(version.read_text())
        if (v['MajorVersion'],v['MinorVersion'])!=(5,8):raise RuntimeError('This adapter requires UE 5.8 PBIK source.')
    digest=hashlib.sha256(b'pbik-standalone-abi-1')
    for p in paths+[HERE/'pbik_compat.h',HERE/'pbik_api.cpp']:digest.update(p.read_bytes())
    tag=digest.hexdigest()[:16]
    output=HERE/'bin'/('pbik_'+tag+'.dll')
    if output.exists():return str(output)
    where=Path(os.environ.get('ProgramFiles(x86)',r'C:\Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
    install=subprocess.check_output([str(where),'-latest','-products','*','-requires','Microsoft.VisualStudio.Component.VC.Tools.x86.x64','-property','installationPath'],text=True).strip()
    if not install:raise RuntimeError('The Visual C++ x64 Build Tools are required to build the PBIK DLL.')
    work=HERE/'_build'/tag
    (work/'Core').mkdir(parents=True,exist_ok=True)
    (work/'Math').mkdir(exist_ok=True)
    output.parent.mkdir(exist_ok=True)
    for p in paths:
        content=p.read_text(encoding='utf-8-sig')
        content=re.sub(r'^#include UE_INLINE_GENERATED_CPP_BY_NAME[^\n]*','',content,flags=re.M)
        if p.name=='PBIKSolver.h':content=content.replace('#include "PBIKDebug.h"','#include "PBIKDebug.h"\n#include "PBIKBody.h"')
        (work/'Core'/p.name).write_text(content,encoding='utf-8')
    for name in ['Math/Quat.h','Math/Vector.h','PBIK.h']:(work/name).write_text('#include "pbik_compat.h"\n',encoding='utf-8')
    (work/'Core/PBIKSolver.generated.h').write_text('',encoding='utf-8')
    vcvars=Path(install)/'VC/Auxiliary/Build/vcvars64.bat'
    # Fixed compiler command, with all file paths quoted; no deletion or shell
    # interpolation of asset data or user input occurs here.
    sources=[work/'Core'/p.name for p in paths if p.suffix=='.cpp']+[HERE/'pbik_api.cpp']
    command='call "{}" >nul\ncl /nologo /std:c++17 /O2 /EHsc /MD /LD /fp:precise /I"{}" /I"{}" /FI"{}" {} /link /OUT:"{}"\n'.format(
        vcvars,HERE,work,HERE/'pbik_compat.h',' '.join('"'+str(p)+'"' for p in sources),output)
    batch=work/'compile.cmd';batch.write_text(command,encoding='utf-8')
    result=subprocess.run(['cmd.exe','/d','/c',str(batch)],cwd=str(work),capture_output=True,text=True,errors='replace')
    (work/'build.log').write_text(result.stdout+result.stderr,encoding='utf-8')
    if result.returncode:raise RuntimeError('PBIK build failed:\n'+result.stdout+result.stderr)
    (output.with_suffix('.json')).write_text(json.dumps({'source':str(source),'sha256':digest.hexdigest(),'abi':1},indent=2),encoding='utf-8')
    return str(output)

if __name__=='__main__':print(build())
