"""Keep local ROS images inside Linux on WSL mirrored networking."""
import ipaddress
import json
import os
import subprocess
from urllib.parse import urlsplit,urlunsplit


def local_ros_environment(environment=None):
    env=dict(os.environ if environment is None else environment)
    # An explicit ROS network address or a remote master is authoritative.
    if env.get('ROS_IP') or env.get('ROS_HOSTNAME'):return env
    master=urlsplit(env.get('ROS_MASTER_URI','http://127.0.0.1:11311'))
    if master.hostname not in ('localhost','127.0.0.1','127.0.1.1'):return env
    try:
        route=json.loads(subprocess.check_output(['ip','-j','route','get','127.0.0.1'],text=True,stderr=subprocess.DEVNULL,timeout=2))
        if not route or route[0].get('dev')!='loopback0':return env
        addresses=json.loads(subprocess.check_output(['ip','-j','address','show','dev','lo'],text=True,stderr=subprocess.DEVNULL,timeout=2))
        for interface in addresses:
            for item in interface.get('addr_info',[]):
                if item.get('family')!='inet' or item.get('scope') not in ('host','global'):continue
                address=ipaddress.ip_address(item['local'])
                if not address.is_private or address.is_loopback or address.is_link_local:continue
                env['ROS_IP']=str(address)
                env['ROS_MASTER_URI']=urlunsplit((master.scheme,str(address)+(':'+str(master.port) if master.port else ''),master.path,master.query,master.fragment))
                env['SAR_LOCAL_WEB_HOST']=str(address)
                excluded=env.get('NO_PROXY',env.get('no_proxy',''))
                env['NO_PROXY']=excluded+','+str(address)+',127.0.0.1,localhost'
                env['no_proxy']=env['NO_PROXY']
                return env
    except (OSError,ValueError,subprocess.SubprocessError):pass
    return env
