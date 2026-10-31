#!/bin/sh
# dsw-1 (CPU only, nothing else runs there): archives and rescans. The OSS mount allows at most 8 concurrent reads and
# writes, so 6 workers read one file each and the rest is left for the result files. Two passes: the second retries
# units the first abandoned (stalled reads leave a .failed marker, finished units are skipped).
D=/mnt/data/datasets
cd /root
export DS_SCAN_WORKERS=6
s() { echo "### $(date +%T) start $*"; nice -n 10 python3 ds_scan.py "$@"; echo "### $(date +%T) end $2"; }
for pass in 1 2; do
    echo "### pass $pass"
    s dir RoboFAC-dataset hdf5 $D/RoboFAC-dataset
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
done
echo "### QUEUE DSW1 DONE $(date)"
