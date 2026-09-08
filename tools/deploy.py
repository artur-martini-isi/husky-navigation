#!/usr/bin/env python3
"""Deploy husky-config to the robot.
  - agrobot_husky_nav/  -> /home/robot/colcon_ws/src/agrobot_husky_nav  (then colcon build)
  - tools/*.sh, tools/*.py that run ON THE ROBOT -> /home/robot/
Usage: tools/deploy.py [--no-build]
"""
import os, sys, pexpect
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'agrobot_husky_nav')
HOST, PW = 'robot@10.0.0.60', 'clearpath'
ROBOT_SCRIPTS = ['field_up.sh', 'nav_kill.sh', 'fixposition_wired.sh', 'ntrip_tunnel_up.sh',
                 'make_forward_mission.py', 'make_relative_mission.py', 'heading_check.py']


def run(cmd, args, timeout=600):
    c = pexpect.spawn(cmd, args, encoding='utf-8', timeout=timeout)
    if c.expect(['[Pp]assword:', pexpect.EOF]) == 0:
        c.sendline(PW); c.expect(pexpect.EOF)
    out = (c.before or '').strip()
    if out: print(out)


# scp -r into an existing dir would nest a copy; wipe the remote sources first, keep build artefacts
run('ssh', ['-o', 'StrictHostKeyChecking=no', HOST, 'rm -rf /home/robot/colcon_ws/src/agrobot_husky_nav'])
run('scp', ['-q', '-o', 'StrictHostKeyChecking=no', '-r', PKG, HOST + ':/home/robot/colcon_ws/src/'])
run('scp', ['-q', '-o', 'StrictHostKeyChecking=no'] + [os.path.join(HERE, f) for f in ROBOT_SCRIPTS] + [HOST + ':/home/robot/'])
run('ssh', ['-o', 'StrictHostKeyChecking=no', HOST, 'chmod +x ' + ' '.join('/home/robot/' + f for f in ROBOT_SCRIPTS)])
# web UI (static page served by field_up.sh on port 8088)
run('ssh', ['-o', 'StrictHostKeyChecking=no', HOST, 'mkdir -p /home/robot/webui'])
run('scp', ['-q', '-o', 'StrictHostKeyChecking=no', os.path.join(HERE, '..', 'webui', 'index.html'), HOST + ':/home/robot/webui/index.html'])
if '--no-build' not in sys.argv:
    run('ssh', ['-o', 'StrictHostKeyChecking=no', HOST,
        'source /opt/ros/jazzy/setup.bash && cd ~/colcon_ws && '
        'colcon build --symlink-install --packages-select agrobot_husky_nav 2>&1 | tail -3'])
