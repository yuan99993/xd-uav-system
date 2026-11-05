#!/usr/bin/env python3
"""Copy the UE Python capture adapter into a specified project (no asset copy)."""
import argparse
from pathlib import Path
import shutil


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--scene',type=Path)
    parser.add_argument('--install-startup',action='store_true')
    args=parser.parse_args()
    package=Path(__file__).resolve().parents[1]
    if not (package/'package.xml').is_file():
        import rospkg
        package=Path(rospkg.RosPack().get_path('sar_yolo_detector'))
    project=args.project.expanduser().resolve()
    if not list(project.glob('*.uproject')):
        parser.error('--project must contain a .uproject file')
    target=project/'Scripts/AirGround';target.mkdir(parents=True,exist_ok=True)
    for name in ('ue_editor_server.py','ue_create_scene.py','frame_protocol.py'):
        shutil.copy2(package/'tools/ue'/name,target/name)
    shutil.copy2(args.scene or package/'config/ue/airground_scene.json',target/'scene.json')
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
