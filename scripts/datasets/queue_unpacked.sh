#!/bin/sh
# unpacked datasets: count the files as they are
D=/mnt/data/datasets; cd /root
export DS_SCAN_WORKERS=16
s() { echo "### $(date +%T) start $*"; nice -n 15 ionice -c3 python3 ds_scan.py "$@"; echo "### $(date +%T) end $1 $2 $3"; }
s dir maniskill-fail hdf5 $D/ManiSkill-fail
s dir maniskill-fail.mp4 mp4 $D/ManiSkill-fail
s dir libero_fail hdf5 $D/LIBERO/libero_fail
s dir libero_fail.mp4 mp4 $D/LIBERO/libero_fail
s dir RoboFAC-dataset hdf5 $D/RoboFAC-dataset
s dir RoboFAC-dataset.mp4 mp4 $D/RoboFAC-dataset
s dir ViFailback-Dataset hdf5 $D/ViFailback-Dataset
s dir ViFailback-Dataset.mp4 mp4 $D/ViFailback-Dataset
s dir InternData-fractal20220817_data parquet $D/InternRobotics/InternData-fractal20220817_data
s dir LET-Base-Dataset bag $D/LET-Base-Dataset
s dir "RoboMIND (v1.0).unpacked" hdf5 $D/RoboMIND
s dir BiPlay.hdf5 hdf5 $D/BiPlay
s dir tracker-pov mp4 $D/tracker-pov
s dir 10Kh-RealOmin-OpenData mcap $D/10Kh-RealOmin-OpenData
echo "### QUEUE A DONE $(date)"
