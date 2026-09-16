#!/usr/bin/env python3
"""Remove de /dev/shm apenas os arquivos que nenhum processo vivo mantem abertos.

Processos de ROS que morrem sem fechar deixam para tras os arquivos de porta e os semaforos do
FastDDS. Quando isso se acumula, novos processos falham com "Failed init_port ... open_and_lock_file
failed", e o efeito pratico e grave: o gerenciador de ciclo de vida do Nav2 nao consegue fechar o
vinculo com os servidores dentro do tempo limite e aborta a subida inteira.

A remocao so alcanca arquivo sem nenhuma referencia viva, entao nada em uso e tocado.
"""
import os, sys

SHM = '/dev/shm'

def em_uso():
    usados = set()
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        for origem in ('maps', 'fd'):
            caminho = '/proc/%s/%s' % (pid, origem)
            try:
                if origem == 'maps':
                    with open(caminho) as f:
                        for linha in f:
                            if '/dev/shm/' in linha:
                                usados.add(linha.split('/dev/shm/')[-1].strip())
                else:
                    for fd in os.listdir(caminho):
                        try:
                            alvo = os.readlink(os.path.join(caminho, fd))
                        except OSError:
                            continue
                        if alvo.startswith('/dev/shm/'):
                            usados.add(alvo[len('/dev/shm/'):])
            except (OSError, IOError):
                continue
    return usados

usados = em_uso()
todos = set(os.listdir(SHM))
orfaos = sorted(todos - usados)
print('em /dev/shm: %d arquivos | em uso: %d | orfaos: %d' % (len(todos), len(todos & usados), len(orfaos)))
if '--aplicar' not in sys.argv:
    print('(simulacao; use --aplicar para remover)')
    sys.exit(0)
removidos = falhas = 0
for nome in orfaos:
    try:
        os.unlink(os.path.join(SHM, nome)); removidos += 1
    except OSError:
        falhas += 1
print('removidos: %d | falhas: %d | restam: %d' % (removidos, falhas, len(os.listdir(SHM))))
