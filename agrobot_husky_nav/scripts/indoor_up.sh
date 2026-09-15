#!/bin/bash
# Roda NO ROBÔ. Sobe a pilha indoor inteira e imprime um resumo. É o análogo do field_up.sh,
# que faz o mesmo para o campo.
#
#   indoor_up.sh                      nuvem + scan + mapa local + SLAM (mapa novo)
#   indoor_up.sh goto                 ... e o nó de ir-até-marcador
#   indoor_up.sh explore              ... e a exploração autônoma (não inicia sozinha)
#   MAPA=teste_terceiro_andar_isisim indoor_up.sh goto
#                                     ... continuando de um mapa salvo, em modo localização
#
# Ao contrário do field_up.sh, aqui **não** se exporta FASTRTPS_DEFAULT_PROFILES_FILE: aquele
# perfil bloqueia a recepção de tópicos transient_local, e o `map` do SLAM é um deles. Com ele
# exportado o planejador nunca recebe o mapa e o nó fica parado dizendo "sem mapa do SLAM".
# Sem `set -u`: os setup.bash do ROS e do colcon leem variáveis não definidas (AMENT_TRACE_SETUP_FILES
# entre outras) e abortariam o script na primeira linha útil.
NAV=${1:-nenhum}
MAPA=${MAPA:-}
LOGS=/home/robot/logs
MAPAS=/home/robot/maps

source /etc/clearpath/setup.bash
source /home/robot/colcon_ws/install/setup.bash
export ROS_SUPER_CLIENT=True
unset FASTRTPS_DEFAULT_PROFILES_FILE
mkdir -p "$LOGS"

sobe() {  # sobe <padrão-que-já-estaria-rodando> <rótulo> <log> <comando...>
  local pat=$1 rotulo=$2 log=$3; shift 3
  if pgrep -f "$pat" >/dev/null; then
    echo "  $rotulo: já estava no ar"
  else
    echo "  $rotulo: subindo"
    setsid nohup "$@" > "$LOGS/$log" 2>&1 < /dev/null &
    sleep 6
  fi
}

echo "pilha indoor:"
sobe "livox_ros_driver2_nod[e]" "nuvem do MID360" livox.log /home/robot/start_livox_pc2.sh
sobe "lib/agrobot_husky_nav/voxel_local_ma[p]" "scan + mapa local" indoor.log \
     ros2 launch agrobot_husky_nav indoor.launch.py

if [ -n "$MAPA" ]; then
  if [ ! -f "$MAPAS/$MAPA.posegraph" ]; then
    echo "  SLAM: mapa '$MAPA' não existe em $MAPAS" >&2; exit 1
  fi
  sobe "async_slam_toolbox_nod[e]" "SLAM (localização em $MAPA)" slam.log \
       ros2 launch agrobot_husky_nav slam.launch.py mode:=localization map_file:="$MAPAS/$MAPA"
else
  sobe "async_slam_toolbox_nod[e]" "SLAM (mapa novo)" slam.log \
       ros2 launch agrobot_husky_nav slam.launch.py
fi

case "$NAV" in
  goto)    sobe "lib/agrobot_husky_nav/goto_poin[t]" "ir-até-marcador" goto_point.log \
                ros2 launch agrobot_husky_nav goto_point.launch.py ;;
  explore) sobe "lib/agrobot_husky_nav/explor[e]" "exploração autônoma" explore.log \
                ros2 launch agrobot_husky_nav explore.launch.py ;;
  nenhum|'') ;;
  *) echo "  argumento desconhecido '$NAV' (use goto, explore ou nada)" >&2; exit 2 ;;
esac

sleep 3
echo "estado:"
echo "  nuvem: $(pgrep -cf 'livox_ros_driver2_nod[e]') | scan: $(pgrep -cf 'pointcloud_to_laserscan_nod[e]') | slam: $(pgrep -cf 'async_slam_toolbox_nod[e]')"
echo "  goto_point: $(pgrep -cf 'lib/agrobot_husky_nav/goto_poin[t]') | explore: $(pgrep -cf 'lib/agrobot_husky_nav/explor[e]')"
pub=$(timeout 25 ros2 topic info /a300_00096/cmd_vel 2>/dev/null | awk '/Publisher count/{print $3}')
echo "  publicadores em cmd_vel: ${pub:-?}  (mais de 1 são dois controladores disputando o robô)"
echo "  logs em $LOGS | encerrar com ~/nav_kill.sh all"
