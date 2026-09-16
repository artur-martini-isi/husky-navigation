#!/bin/bash
# Roda NO ROBÔ. Salva o mapa global em /home/robot/maps: imagem .pgm + .yaml e o grafo de poses.
# Uso: save_map.sh [nome]   (padrão: mapa_<data>)
source /etc/clearpath/setup.bash; export ROS_SUPER_CLIENT=True
NS=${NS:-/a300_00096}
NOME=${1:-mapa_$(date +%Y%m%d_%H%M)}
mkdir -p /home/robot/maps
echo "salvando imagem do mapa..."
# O serviço save_map do slam_toolbox devolve result=255 sem escrever nada em algumas combinações
# (visto em 2026-09-16). Quando isso acontece, o salvador padrão do Nav2 resolve e ainda diz o
# porquê em caso de falha. Tenta o serviço primeiro, cai para o map_saver se o .pgm não aparecer.
ros2 service call $NS/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: /home/robot/maps/$NOME}}" 2>&1 | tail -1
if [ ! -f "/home/robot/maps/$NOME.pgm" ]; then
  echo "  serviço não gravou a imagem; usando o map_saver do Nav2"
  ros2 run nav2_map_server map_saver_cli -f "/home/robot/maps/$NOME" \
    --ros-args -r map:=$NS/map -p save_map_timeout:=20.0 2>&1 | tail -2
fi
echo "salvando grafo de poses (para continuar o mapeamento ou localizar depois)..."
ros2 service call $NS/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: /home/robot/maps/$NOME}" 2>&1 | tail -1
ls -la /home/robot/maps/ | grep "$NOME"
