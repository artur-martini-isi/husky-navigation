#!/bin/bash
for p in $(pgrep -f "decimate.py"); do
  case "$(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)" in *run_dec.sh*) continue;; esac
  kill -9 "$p" 2>/dev/null
done
sleep 1
setsid nohup /home/robot/start_decimator.sh 5 > /tmp/decimator.log 2>&1 < /dev/null &
sleep 12
echo "--- log ---"; cat /tmp/decimator.log
