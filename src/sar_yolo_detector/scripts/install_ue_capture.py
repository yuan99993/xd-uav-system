#!/usr/bin/env python3
"""Copy the UE Python capture adapter into a specified project (no asset copy)."""
import argparse
from pathlib import Path
import shutil
import json
import hashlib


def validate_stream_plugin(plugin, engine=None):
    manifest=json.loads((plugin/'manifest.json').read_text(encoding='utf-8-sig'))
    for name,digest in manifest['files'].items():
        path=plugin/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise ValueError('Native plugin integrity check failed: '+name)
    if engine:
        modules=engine/'Engine/Binaries/Win64/UnrealEditor.modules'
        installed=json.loads(modules.read_text(encoding='utf-8-sig'))
        if str(installed['BuildId'])!=str(manifest['build_id']):
            raise ValueError('UE BuildId mismatch; rebuild SarCameraStream for this engine before installing')
    return manifest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--scene',type=Path)
    parser.add_argument('--install-startup',action='store_true')
    parser.add_argument('--install-stream-plugin',action='store_true')
    parser.add_argument('--engine',type=Path,help='UE root; checks the native module against this engine BuildId')
    args=parser.parse_args()
    package=Path(__file__).resolve().parents[1]
    if not (package/'package.xml').is_file():
        import rospkg
        package=Path(rospkg.RosPack().get_path('sar_yolo_detector'))
    project=args.project.expanduser().resolve()
    if not list(project.glob('*.uproject')):
        parser.error('--project must contain a .uproject file')
    if args.install_stream_plugin:
        try:validate_stream_plugin(package/'ue_plugin/SarCameraStream',args.engine)
        except (OSError,ValueError,KeyError) as error:parser.error(str(error))
    target=project/'Scripts/AirGround';target.mkdir(parents=True,exist_ok=True)
    for name in ('ue_editor_server.py','ue_create_scene.py','frame_protocol.py','virtual_gimbal.py','virtual_zoom.py','ue_stream_host.py','tcp_relay.py','ue_augment_benchmark.py','ue_benchmark_truth.py','benchmark_visibility.py','benchmark_truth_writer.py'):
        shutil.copy2(package/'tools/ue'/name,target/name)
    shutil.copy2(args.scene or package/'config/ue/airground_scene.json',target/'scene.json')
    shutil.copy2(package/'config/ue/benchmark_scene.json',target/'benchmark_scene.json')
    if args.install_stream_plugin:
        plugin=package/'ue_plugin/SarCameraStream'
        if not (plugin/'Binaries/Win64/UnrealEditor-SarCameraStream.dll').is_file():
            parser.error('Build the native stream plugin before installing it')
        destination=project/'Plugins/SarCameraStream'
        shutil.copytree(plugin,destination,dirs_exist_ok=True,
            ignore=shutil.ignore_patterns('Intermediate','*.pdb','__pycache__'))
        descriptor=next(project.glob('*.uproject'))
        shutil.copy2(descriptor,descriptor.with_suffix('.uproject.sar-stream-backup'))
        content=json.loads(descriptor.read_text(encoding='utf-8-sig'))
        plugins=content.setdefault('Plugins',[])
        existing=next((v for v in plugins if v['Name']=='SarCameraStream'),None)
        if existing is None:plugins.append(dict(Name='SarCameraStream',Enabled=True))
        else:existing['Enabled']=True
        descriptor.write_text(json.dumps(content,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if args.install_startup:
        init=project/'Content/Python/init_unreal.py';init.parent.mkdir(parents=True,exist_ok=True)
        begin,end='# BEGIN SAR UE CAPTURE','# END SAR UE CAPTURE'
        block='\n'.join([begin,'import runpy as _sar_runpy','from pathlib import Path as _SarPath',
            '_sar_runpy.run_path(str(_SarPath(__file__).resolve().parents[2]/"Scripts/AirGround/ue_editor_server.py"))',end])
        old=init.read_text() if init.exists() else ''
        if begin in old and end in old:
            before,rest=old.split(begin,1);_,after=rest.split(end,1);new=before+block+after
        else:new=old.rstrip()+'\n\n'+block+'\n'
        if init.exists():shutil.copy2(init,init.with_suffix('.py.sar-backup'))
        init.write_text(new)
    print('Capture adapter installed:',target)
    print('Enable UE Python Editor Script Plugin and Editor Scripting Utilities; restart the editor.')
    print('For a new map, run ue_create_scene.py in the editor after provisioning scene.json assets.')


if __name__=='__main__':main()
