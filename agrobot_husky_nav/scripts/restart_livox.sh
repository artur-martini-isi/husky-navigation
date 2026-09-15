#!/bin/bash
# Reinicia o driver Livox em modo PointCloud2, sem auto-matar o shell chamador.
SELF=$$
PAT='livox_ros_driver2'
for p in $(pgrep -f "$PAT"); do
  [ "$p" = "$SELF" ] && continue
  # ignora processos cuja cmdline seja este script
  case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in
    *restart_livox.sh*) continue ;;
  esac
  kill "$p" 2>/dev/null && echo "kill $p"
done
sleep 4
# force kill em remanescentes
for p in $(pgrep -f "$PAT"); do
  [ "$p" = "$SELF" ] && continue
  case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in
    *restart_livox.sh*) continue ;;
  esac
  kill -9 "$p" 2>/dev/null && echo "kill -9 $p"
done
sleep 2
echo "--- restantes ---"
pgrep -af "$PAT" | grep -v restart_livox || echo "nenhum (porta UDP livre)"
echo "--- subindo PointCloud2 ---"
setsid nohup /home/robot/start_livox_pc2.sh > /tmp/livox_pc2.log 2>&1 < /dev/null &
sleep 10
echo "--- processo novo ---"
pgrep -af "$PAT" | grep -v restart_livox
echo "--- log ---"
cat /tmp/livox_pc2.log
