#!/bin/bash
# Roda NO ROBÔ. Salva o mapa global em /home/robot/maps: imagem .pgm + .yaml e o grafo de poses.
# Uso: save_map.sh [nome]   (padrão: mapa_<data>)
source /etc/clearpath/setup.bash; export ROS_SUPER_CLIENT=True
NS=${NS:-/a300_00096}
NOME=${1:-mapa_$(date +%Y%m%d_%H%M)}
mkdir -p /home/robot/maps
echo "salvando imagem do mapa..."
ros2 service call $NS/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: /home/robot/maps/$NOME}}" 2>&1 | tail -1
echo "salvando grafo de poses (para continuar o mapeamento ou localizar depois)..."
ros2 service call $NS/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: /home/robot/maps/$NOME}" 2>&1 | tail -1
ls -la /home/robot/maps/ | grep "$NOME"
