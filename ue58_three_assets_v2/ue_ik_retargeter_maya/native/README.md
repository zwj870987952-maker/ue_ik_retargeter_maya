# UE 5.8 PBIK standalone adapter

The full-body path runs the PBIK solver, body and constraint code from the
installed UE 5.8 engine. `build.py` copies those files into `_build/`, removes
only generated-file includes, supplies the small container / math compatibility
layer, and compiles an x64 DLL. Generated Epic sources and binaries are ignored
by Git. Epic's copyright notices are retained in generated files.

The DLL has no Unreal DLL or Python ABI dependencies. At runtime Maya loads it
through `ctypes`; a running Unreal Editor is not required. Rebuilding requires
UE 5.8 sources and Visual C++ x64 Build Tools. The default source location is
`E:\UE5\UE_5.8\Engine`; set `UE58_ENGINE_ROOT` to another Engine directory.

```powershell
python ue58_three_assets_v2/ue_ik_retargeter_maya/native/build.py
python test/test_pbik_native.py
```

The filename includes a hash of the engine sources and adapter. This avoids
overwriting a DLL already loaded by Maya when rebuilding. `bin/*.json` records
the source location, ABI and complete hash used for the build.

`core/ue58_pbik_node.py` registers `ue58PBIKSolver`. Its development type ID is
`0x0013B2A1`; allocate a registered Autodesk ID before distributing a compiled
production plugin that shares node IDs with other vendors.

The node reads joint local matrices and independent Goal world matrices, then
drives `offsetParentMatrix`. Translation / rotation channels, animation nodes,
keys and their connections are preserved. The settings groups drive the node's
solver, Goal and bone settings directly. A first-time Maya plugin trust prompt
may require the user to allow this project plugin.

Inputs are converted to Unreal's component axes and FBX bone frames before
solving, then converted back to Maya. This conversion is verified against the
CC4 and Mixamo fixtures in `test/fixtures/pbik_cc4_mixamo.json`. Other FBX import
conventions and arbitrary pre-existing offsetParentMatrix drivers need separate
validation; occupied / locked offsetParentMatrix inputs are rejected.

The portability layer uses standard C++ containers and scalar double precision
math. UE uses its own sort and SIMD math implementations. The algorithm and
settings are preserved, but finite-iteration results should still be compared
against engine golden poses before claiming bitwise or exact per-bone parity.
