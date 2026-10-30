#!/bin/sh
# From a machine that reaches both dsw-4 and 5090-2: sum the dsw-4 scans, write them into the platform inventory, publish.
set -e
RUN=/data/robodojo-runs/dataset-scan-20261030
ssh -o BatchMode=yes dsw-4 'cd /root && python3 ds_report.py > /root/ds_report.txt && cd /mnt/data/datasets/_stats && tar -czf - */summary.json' > /tmp/ds_summaries.tgz
ssh 5090-2 "mkdir -p $RUN/summaries && rm -rf $RUN/summaries/*" && ssh 5090-2 "tar -xzf - -C $RUN/summaries" < /tmp/ds_summaries.tgz
scp -q "$(dirname "$0")/ds_apply.py" 5090-2:$RUN/ds_apply.py
ssh 5090-2 "cd $RUN && cp /data/r2s-platform/state/output/dataset_inventory.json before.json && python3 ds_apply.py before.json summaries dataset_inventory.json"
scp -q 5090-2:$RUN/dataset_inventory.json /tmp/ds_inventory_new.json
node "$(dirname "$0")/../../src/real2sim_platform/test_dataset_catalog.cjs" /tmp/ds_inventory_new.json
ssh 5090-2 "r2s publish $RUN/dataset_inventory.json output/dataset_inventory.json | tail -1"
ssh -o BatchMode=yes dsw-4 'cat /root/ds_report.txt; grep "###" /root/scan_queue_a.log /root/scan_queue_b.log | tail -2'
