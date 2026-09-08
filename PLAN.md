# Husky A300 — histórico e próximos passos

Estado atual (2026-09-10): **navegação por waypoints GNSS RTK validada em campo.** Ver `README.md`
para arquitetura e operação, `agrobot_husky_nav/README.md` para o pacote.

## Histórico

### 2026-09-08 — levantamento e stack
- Levantamento do robô (Jazzy, Clearpath 2.9, namespace `a300_00096`, discovery server, `robot.yaml` da SENAI).
- Fixposition estava fora da rede; passou pelo Wi-Fi (`10.0.0.199`) e no fim do dia foi para o cabo (`192.168.131.35`).
- Decisão: **stack leve, sem Nav2**. Criado `agrobot_husky_nav` (seguidor + guarda de obstáculos + joystick).
- `pointcloud_to_laserscan` instalado; MID360 caracterizado pela nuvem: invertido, girado 90°, 0,61 m do chão.
- Correções RTK via ROS 2: `ntrip_client` integrado ao `colcon_ws`, `ntrip.launch.py`, fonte do sensor → I/O,
  provado com parada/religada do cliente. Arquitetura B escolhida (sensor no cabo, robô com Wi-Fi).
- Fusão do sensor encontrada parada (autostart habilitado; start falha). Foxglove: mesh `.STL` → `.stl`.

### 2026-09-09 — campo
- Robô fixado na Agriwing 2,4 GHz (5G removida do netplan).
- Sensor sem IP no cabo → usuário fixou `192.168.131.35/24` na interface do sensor.
- Roteador de campo sem internet → **túnel SSH reverso** do laptop (`tools/ntrip_tunnel.py` + `~/ntrip_tunnel_up.sh`).
  RTK fixed nas duas antenas (base RBMC a 486 m).
- Fusão descartada por ora (câmera sem calibração) → **`pose_source: dual_gnss`** implementado e validado
  (erro de rumo < 1° contra deslocamento RTK de 3 m).
- Testes: 5 m reto a 0,3 m/s (18 s, 0,58 m); quadrado 5 m a 0,5 m/s (68 s); a 1,0 m/s (48 s, guarda com
  reduz 4 m / para 1,8 m). Chegada sempre ~0,57 m. Guarda de obstáculos bloqueou e retomou nas duas corridas.
- Scripts de campo criados: `field_up.sh`, `nav_kill.sh`, `make_forward_mission.py`, `make_relative_mission.py`, `heading_check.py`.

### 2026-09-10 — consolidação
- Auditoria robô × diretório: pacote e scripts idênticos. `deploy.py` passou a sincronizar também os scripts do robô.
- `inventory/` atualizado (robot.yaml, ins_0.yaml, netplan, tópicos), `missions/` e `robot-home/` adicionados.
- **Web UI de teste** (`webui/index.html`, servida em http://10.0.0.60:8088): posição/estado ao vivo, waypoints por
  clique ou atalhos, envio da missão (`set_mission`) e Start/Pause/Stop via Foxglove bridge (`foxglove.sdk.v1`).

## Próximos passos
1. **Serviço systemd** (ou `platform.extras.launch`) para Livox + `bringup.launch.py` subirem com o robô, sem missão.
2. **Internet em campo sem laptop**: modem 4G no roteador ou no robô; então `ntrip.launch.py` direto no caster.
3. **Calibrar a câmera do Fixposition** e voltar a `pose_source: fusion` (rumo e pose mesmo sem RTK fixed;
   tolera perda de correção). Precisa da fusão iniciando na API (`ctrl/action {"fusion":"start"}`).
4. **Trajeto de fileiras** (ida e volta paralelas de 20 m) e teste do `obstacle_steer` (desvio lateral).
5. **Rosbag** por corrida (`gps_*/fix`, `gnsscorr`, `scan`, `cmd_vel`, `joy`, `status`) para análise.
6. Integração com o PLAAC (`agrobot-physical-layer`): expor `load_mission`/`start`/`stop` como skill de missão.
7. Curvas suaves (raio mínimo) em vez de giro parado, se a aplicação pedir.
8. Cabo do laptop: adaptador USB-ethernet sem link em 2026-09-08 (trocar); laptop precisa de IP estático em `192.168.131.x`.
