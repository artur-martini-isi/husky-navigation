#!/usr/bin/env python3
"""Deploy husky-config to the robot.

  - agrobot_husky_nav/  -> /home/robot/colcon_ws/src/agrobot_husky_nav  (then colcon build)
  - webui/              -> /home/robot/webui

Os scripts de operação moram em `agrobot_husky_nav/scripts/` e o `colcon build` os instala em
`install/agrobot_husky_nav/lib/agrobot_husky_nav/`, onde o `ros2 run` os encontra. Para não quebrar
a memória muscular nem a documentação, este deploy cria em `/home/robot/` um **atalho** para cada
um: `~/nav_kill.sh` continua funcionando, mas agora é um link para a cópia instalada, e não mais um
arquivo solto que podia divergir do repositório.

Usage: tools/deploy.py [--no-build] [--no-links]
"""
import os, sys, pexpect
HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..', 'agrobot_husky_nav')
HOST, PW = 'robot@10.0.0.60', 'clearpath'
INSTALL = '/home/robot/colcon_ws/install/agrobot_husky_nav/lib/agrobot_husky_nav'


def run(cmd, args, timeout=600):
    c = pexpect.spawn(cmd, args, encoding='utf-8', timeout=timeout)
    if c.expect(['[Pp]assword:', pexpect.EOF]) == 0:
        c.sendline(PW); c.expect(pexpect.EOF)
    out = (c.before or '').strip()
    if out: print(out)


def ssh(cmd, timeout=600):
    run('ssh', ['-o', 'StrictHostKeyChecking=no', HOST, cmd], timeout)


scripts = sorted(f for f in os.listdir(os.path.join(PKG, 'scripts'))
                 if f.endswith(('.sh', '.py')))

# scp -r into an existing dir would nest a copy; wipe the remote sources first, keep build artefacts
ssh('rm -rf /home/robot/colcon_ws/src/agrobot_husky_nav')
run('scp', ['-q', '-o', 'StrictHostKeyChecking=no', '-r', PKG, HOST + ':/home/robot/colcon_ws/src/'])
# web UI (static page served by field_up.sh on port 8088)
ssh('mkdir -p /home/robot/webui')
run('scp', ['-q', '-o', 'StrictHostKeyChecking=no', os.path.join(HERE, '..', 'webui', 'index.html'),
            HOST + ':/home/robot/webui/index.html'])
if '--no-build' not in sys.argv:
    ssh('source /opt/ros/jazzy/setup.bash && cd ~/colcon_ws && '
        'colcon build --symlink-install --packages-select agrobot_husky_nav 2>&1 | tail -3')
if '--no-links' not in sys.argv:
    # o bit de execução se perde numa build sem --symlink-install; garante aqui
    ssh('chmod +x /home/robot/colcon_ws/src/agrobot_husky_nav/scripts/* 2>/dev/null; '
        'for f in ' + ' '.join(scripts) + '; do '
        '  [ -e "%s/$f" ] && ln -sfn "%s/$f" "/home/robot/$f"; done; '
        'echo "atalhos em /home/robot: $(ls -l /home/robot/*.sh /home/robot/*.py 2>/dev/null '
        '| grep -c \'^l\')"' % (INSTALL, INSTALL))
