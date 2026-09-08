#!/usr/bin/env python3
"""Run ON THE LAPTOP (which has internet). Keeps a reverse SSH tunnel so that the robot's
127.0.0.1:2101 reaches the IBGE NTRIP caster through the laptop. Reconnects forever."""
import time, pexpect
CASTER = 'gps-ntrip.ibge.gov.br'
while True:
    c = pexpect.spawn('ssh', ['-N', '-o', 'StrictHostKeyChecking=no', '-o', 'ServerAliveInterval=10', '-o', 'ServerAliveCountMax=3',
                              '-o', 'ExitOnForwardFailure=yes', '-R', f'2101:{CASTER}:2101', 'robot@10.0.0.60'],
                      encoding='utf-8', timeout=None)
    i = c.expect(['[Pp]assword:', pexpect.EOF, pexpect.TIMEOUT], timeout=20)
    if i == 0:
        c.sendline('clearpath')
    print(time.strftime('%H:%M:%S'), 'tunnel up', flush=True)
    c.expect(pexpect.EOF, timeout=None)
    print(time.strftime('%H:%M:%S'), 'tunnel closed, retrying in 5 s:', (c.before or '')[-200:], flush=True)
    time.sleep(5)
