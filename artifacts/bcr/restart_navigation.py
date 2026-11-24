"""Restart only this project's unified launch and its recorded descendants."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

root = Path.home() / 'bcr_ws' / 'validation'
rows = [line.split(None, 2) for line in subprocess.check_output(
    ['ps', '-eo', 'pid=,ppid=,args='], text=True).splitlines()]
entry = f'ros2 launch {root}/navigation_demo.launch.py'
targets = {int(pid) for pid, ppid, args in rows
           if args.startswith('/usr/bin/python3 ') and entry in args}
while True:
    children = {int(pid) for pid, ppid, args in rows if int(ppid) in targets}
    if children <= targets:
        break
    targets |= children
for pid in sorted(targets, reverse=True):
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
time.sleep(3)
for pid in targets:
    try:
        os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
log_path = root / (sys.argv[1] if len(sys.argv) > 1 else 'restart_navigation.log')
with log_path.open('w') as log:
    proc = subprocess.Popen(['bash', str(root / 'start_navigation.sh')],
                            stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                            start_new_session=True)
print('Started', proc.pid, 'log', log_path, flush=True)
