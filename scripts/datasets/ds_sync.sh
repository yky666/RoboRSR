#!/bin/sh
# From a machine that reaches the scan host and 5090-2: sum the scans, write them into the platform inventory, publish.
set -e
RUN=/data/robodojo-runs/dataset-scan-20261030
SCAN=${SCAN_HOST:-dsw-1}  # any machine with the OSS mount; summaries are read from there
ssh -o BatchMode=yes $SCAN 'cd /root && { [ ! -f dsw1_queue.log ] || cp dsw1_queue.log /mnt/data/datasets/_stats/_logs/; } && python3 ds_report.py > /mnt/data/datasets/_stats/_logs/report.txt && cd /mnt/data/datasets/_stats && tar -czf - */summary.json' > /tmp/ds_summaries.tgz
ssh 5090-2 "mkdir -p $RUN/summaries && rm -rf $RUN/summaries/*" && ssh 5090-2 "tar -xzf - -C $RUN/summaries" < /tmp/ds_summaries.tgz
scp -q "$(dirname "$0")/ds_apply.py" 5090-2:$RUN/ds_apply.py
ssh 5090-2 "cd $RUN && cp /data/r2s-platform/state/output/dataset_inventory.json before.json && python3 ds_apply.py before.json summaries dataset_inventory.json"
scp -q 5090-2:$RUN/dataset_inventory.json /tmp/ds_inventory_new.json
node "$(dirname "$0")/../../src/real2sim_platform/test_dataset_catalog.cjs" /tmp/ds_inventory_new.json
ssh 5090-2 "r2s publish $RUN/dataset_inventory.json output/dataset_inventory.json | tail -1"
ssh -o BatchMode=yes $SCAN 'cat /mnt/data/datasets/_stats/_logs/report.txt; grep -h "###" /mnt/data/datasets/_stats/_logs/*.log | tail -2'
