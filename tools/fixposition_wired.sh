#!/bin/bash
# Run ON THE ROBOT after the Fixposition is back on the wired network (192.168.131.35).
# Points the driver stream at the wired IP and regenerates the Clearpath services.
set -e
IP=${1:-192.168.131.35}
Y=/home/robot/colcon_ws/src/senai02_a300/senai02_bringup/config/robot.yaml
ping -c2 -W1 "$IP" >/dev/null || { echo "Fixposition not reachable at $IP"; exit 1; }
cp "$Y" "$Y.bak-$(date +%F-%H%M)"
sed -i "s#stream: tcpcli://[0-9.]*:21000#stream: tcpcli://$IP:21000#; s#fp_output.ip: \"[0-9.]*\"#fp_output.ip: \"$IP\"#" "$Y"
grep -nE "stream:|fp_output.ip" "$Y"
sudo systemctl restart clearpath-robot
echo "restarted clearpath-robot; check: journalctl -u clearpath-sensors -f | grep fixposition"
