"""Compile the local Gazebo 10 TT motor system and report its plugin directory."""
from pathlib import Path
import os
import subprocess

HERE=Path(__file__).resolve().parent
BUILD=HERE/'plugins/build'

def ensure_plugin():
    sources=[HERE/'plugins/TTMotorSystem.cc',HERE/'plugins/CMakeLists.txt']
    libraries=list(BUILD.glob('*ugv_tt_motor.*')) if BUILD.exists() else []
    libraries=[p for p in libraries if p.suffix in ('.dylib','.so','.dll')]
    newest=max(p.stat().st_mtime for p in sources)
    if not libraries or max(p.stat().st_mtime for p in libraries)<newest:
        env=dict(os.environ)
        prefixes=[p for p in ('/opt/homebrew','/usr/local') if Path(p).exists()]
        env['CMAKE_PREFIX_PATH']=os.pathsep.join(prefixes+[env.get('CMAKE_PREFIX_PATH','')])
        subprocess.run(['cmake','-S',str(HERE/'plugins'),'-B',str(BUILD),'-DCMAKE_BUILD_TYPE=Release'],check=True,env=env)
        subprocess.run(['cmake','--build',str(BUILD),'-j','4'],check=True,env=env)
    return BUILD

def plugin_environment():
    directory=ensure_plugin()
    env=dict(os.environ)
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(directory)+os.pathsep+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    return env

if __name__=='__main__':
    print(ensure_plugin())
