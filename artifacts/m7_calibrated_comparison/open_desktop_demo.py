"""Open the calibrated demo in the logged-in Ubuntu desktop, initially idle."""
from pathlib import Path
import json,os,subprocess
root=Path.home()/'nav_ws';out=root/'artifacts/m7_calibrated_comparison'
environment=os.environ.copy();found=False
for path in Path('/proc').glob('[0-9]*/environ'):
 try:
  if path.stat().st_uid!=os.getuid():continue
  env=dict(x.split(b'=',1) for x in path.read_bytes().split(b'\0') if b'=' in x)
  if env.get(b'DISPLAY')==b':0' and Path(env.get(b'XAUTHORITY',b'/nonexistent').decode()).is_file():
   for key in ['DISPLAY','XAUTHORITY','DBUS_SESSION_BUS_ADDRESS','XDG_RUNTIME_DIR','WAYLAND_DISPLAY']:
    if key.encode() in env:environment[key]=env[key.encode()].decode()
   found=True;break
 except (PermissionError,FileNotFoundError,ProcessLookupError):pass
assert found,'No logged-in desktop Xwayland session found'
for path in Path('/proc').glob('[0-9]*/cmdline'):
 try:
  command=path.read_bytes().replace(b'\0',b' ').decode(errors='replace')
  assert not command.startswith('ign gazebo -s'),'Existing simulation must exit first'
 except (PermissionError,FileNotFoundError,ProcessLookupError):pass
environment['QT_QPA_PLATFORM']='xcb'
with (out/'desktop_demo.log').open('w') as log:
 p=subprocess.Popen(['bash',str(root/'src/my_robot_navigation/scripts/start_calibrated_navigation.sh'),
                     'gui:=true','rviz:=true'],env=environment,stdout=log,stderr=log,start_new_session=True)
(out/'desktop_demo.json').write_text(json.dumps({'pid':p.pid,'gui':True,'rviz':True,'navigation_goal_sent':False},indent=2))
print('Desktop demo started, process group '+str(p.pid),flush=True)
