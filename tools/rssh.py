#!/usr/bin/env python3
"""Roda um comando no robô por SSH. Uso: rssh.py '<comando>' [timeout_s]

O timeout é do pexpect, não do comando: um comando que demora mais que ele derruba a leitura da
saída mas **deixa o processo rodando no robô**. Por isso ele é argumento, e não um valor fixo.
"""
import pexpect, sys
cmd = sys.argv[1]
espera = float(sys.argv[2]) if len(sys.argv) > 2 else 120
child = pexpect.spawn('ssh', ['-t','-o','StrictHostKeyChecking=no','-o','ConnectTimeout=10','robot@10.0.0.60', cmd], encoding='utf-8', timeout=espera)
i = child.expect(['[Pp]assword:', pexpect.EOF])
if i == 0:
    child.sendline('clearpath')
    child.expect(pexpect.EOF)
print(child.before)
