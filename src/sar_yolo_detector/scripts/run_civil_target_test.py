#!/usr/bin/python3
"""Run class-independent ROI locking and the existing four-mode test scenes."""
import argparse
import json
from pathlib import Path
import subprocess
from sar_yolo_detector.local_ros_transport import local_ros_environment


def scenario_profile(package, name):
    profiles=json.loads((package/'config/ue/civil_target_tests.json').read_text())
    if name not in profiles:raise ValueError('Unknown scenario: '+name)
    profile=profiles[name]
    scene=package/'config/ue/benchmark_cases'/(profile['case_id']+'.json')
    gimbal=scene.with_name(scene.stem+'_gimbal.json')
    spec=json.loads(scene.read_text())
    primary=next(e for e in spec['entities'] if e['id']==spec['benchmark']['primary_entity'])
    if profile['target_class']!='model_supported' and primary['class']!=profile['target_class']:raise ValueError('Primary class disagrees with profile')
    if not gimbal.is_file():raise ValueError('Missing paired gimbal profile')
    if any(not isinstance(e.get('class'),str) or not e['class'] for e in spec['entities']):
        raise ValueError('Test entities require a class annotation')
    return dict(profile,scenario=name,scene=str(scene),gimbal_config=str(gimbal),lock_class_policy='all_model_categories_operator_selected')


def command(package, profile, args):
    if args.benchmark:
        if args.output is None:raise ValueError('--benchmark requires a new --output directory')
        if not 10<=args.seconds<=120:raise ValueError('Benchmark seconds must be within 10..120')
        cmd=['/usr/bin/python3',str(Path(__file__).resolve().with_name('run_ue_stress_suite.py')),
            '--project',str(args.project),'--python-executable',args.python_executable,
            '--output',str(args.output),'--launch-file',profile['launch_file'],
            '--cases',profile['case_id'],'--pose-source',args.pose_source,
            '--seconds',str(args.seconds),'--roi-padding',str(args.roi_padding)]
        if not args.record:cmd.append('--no-record')
        if getattr(args,'feature_tracker_algorithm',None):cmd+=['--feature-tracker-algorithm',args.feature_tracker_algorithm]
        return cmd
    cmd=['roslaunch','sar_yolo_detector',profile['launch_file'],
        'ue_project:='+str(args.project),'scene:='+profile['scene'],
        'gimbal_config:='+profile['gimbal_config'],'pose_source:='+args.pose_source,
        'seconds:='+str(args.seconds),'python_executable:='+args.python_executable]
    if getattr(args,'feature_tracker_algorithm',None):cmd.append('feature_tracker_algorithm:='+args.feature_tracker_algorithm)
    if args.output is not None:cmd.append('output:='+str(args.output))
    return cmd


def main():
    import rospkg
    package=Path(rospkg.RosPack().get_path('sar_yolo_detector'))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list',action='store_true')
    parser.add_argument('--describe',action='store_true',help='Print scene settings without starting UE/ROS nodes')
    parser.add_argument('--scenario',default='model_targets')
    parser.add_argument('--project',type=Path)
    parser.add_argument('--python-executable',default='')
    parser.add_argument('--feature-tracker-algorithm',choices=['kcf','csrt','nano'])
    parser.add_argument('--pose-source',choices=['px4','kinematic'],default='px4')
    parser.add_argument('--seconds',type=float)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--benchmark',action='store_true')
    parser.add_argument('--roi-padding',type=int,choices=range(9),default=2)
    parser.add_argument('--record',action='store_true',help='Add raw ROS bag recording to the benchmark workload')
    args=parser.parse_args()
    if args.seconds is None:args.seconds=60 if args.benchmark else 360
    try:
        if args.list:
            names=json.loads((package/'config/ue/civil_target_tests.json').read_text())
            print(json.dumps([scenario_profile(package,name) for name in names],ensure_ascii=False,indent=2));return
        profile=scenario_profile(package,args.scenario)
        if args.describe:print(json.dumps(profile,ensure_ascii=False,indent=2));return
        if args.project is None:parser.error('--project is required to run a scene')
        if args.seconds<=0:parser.error('--seconds must be positive')
        if args.benchmark and not args.python_executable:parser.error('--benchmark requires --python-executable')
        cmd=command(package,profile,args)
    except (ValueError,OSError,KeyError) as error:parser.error(str(error))
    environment=local_ros_environment()
    if environment.get('SAR_LOCAL_WEB_HOST'):
        print('ROS local transport:',environment['ROS_IP'],'; browser: http://localhost:8787',flush=True)
    raise SystemExit(subprocess.call(cmd,env=environment))


if __name__=='__main__':main()
