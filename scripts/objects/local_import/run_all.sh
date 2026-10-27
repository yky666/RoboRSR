#!/bin/bash
# Full build + render-only previews + contact sheets for the local asset import.
set -u
source /root/miniconda3/etc/profile.d/conda.sh && conda activate RoboDojo
export OMNI_KIT_ACCEPT_EULA=YES CUDA_VISIBLE_DEVICES=${GPU:-6}
R=/data/robodojo-runs/local-asset-import-20261027
cd $R
rm -rf ovl real2sim sheets
python build_assets.py parts keep_assets.json objects.json ovl/Rigid real2sim/objects > build.log 2>&1
echo "build_exit=$?" > status.txt
cd /root/RoboDojo
ALL=($(ls -d $R/ovl/Rigid/*/[0-9]*))
CHUNK=28
for ((i = 0; i < ${#ALL[@]}; i += CHUNK)); do
    PYTHONPATH=. timeout 1500 python $R/render_assets.py --image-dir $R/real2sim/objects "${ALL[@]:i:CHUNK}" >> $R/render.log 2>&1
    echo "render_chunk_$i=$?" >> $R/status.txt
done
cd $R
python make_sheets.py ovl/build_report.json real2sim/objects sheets > sheets.log 2>&1
echo "sheets_exit=$?" >> status.txt
echo DONE >> status.txt
