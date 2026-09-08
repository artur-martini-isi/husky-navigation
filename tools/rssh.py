#!/usr/bin/env python3
import pexpect, sys
cmd = sys.argv[1]
child = pexpect.spawn('ssh', ['-t','-o','StrictHostKeyChecking=no','-o','ConnectTimeout=10','robot@10.0.0.60', cmd], encoding='utf-8', timeout=120)
i = child.expect(['[Pp]assword:', pexpect.EOF])
if i == 0:
    child.sendline('clearpath')
    child.expect(pexpect.EOF)
print(child.before)
