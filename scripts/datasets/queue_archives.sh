#!/bin/sh
# archives, smallest first; each unit is one archive read as a stream
D=/mnt/data/datasets; cd /root
export DS_SCAN_WORKERS=20
s() { echo "### $(date +%T) start $1 $2"; nice -n 15 ionice -c3 python3 ds_scan.py "$@"; echo "### $(date +%T) end $2"; }
s archive LIBERO-plus "$D/LIBERO/LIBERO-plus/*.zip"
s archive RoboCasa "$D/RoboCasa/*.zip"
s archive VITRA-1M "$D/VITRA-1M/*.tar.gz"
s archive VLAC-Cut-Benchmark "$D/InternRobotics/VLAC-Cut-Benchmark/**/*.tar"
s archive lingbot-GM-100 "$D/lingbot-GM-100/**/*.tar.gz"
s archive VLAC-Cut-FullData "$D/InternRobotics/VLAC-Cut-FullData/**/*.tar.zst"
s archive RDT "$D/RDT/rdt_data.tar.gz.*"
s archive "RoboMIND (v1.0)" "$D/RoboMIND/**/*.tar.gz"
s archive AgiBotWorld2026 "$D/AgiBot/AgiBotWorld2026/**/*.tar.gz"
s archive MotionMillion "$D/InternRobotics/MotionMillion/**/*.tar.gz"
s archive IROS-2025-Challenge-Nav "$D/InternRobotics/IROS-2025-Challenge-Nav/**/*.tar.gz"
s archive VL-LN-Bench "$D/InternRobotics/VL-LN-Bench/**/*.tar.gz"
s archive SynthVerse-Benchmark "$D/InternRobotics/SynthVerse-Benchmark/**/*.tar.gz"
s archive SynthVerse "$D/InternRobotics/SynthVerse/**/*.tar" "$D/InternRobotics/SynthVerse/**/*.tar.gz"
echo "### QUEUE B DONE $(date)"
