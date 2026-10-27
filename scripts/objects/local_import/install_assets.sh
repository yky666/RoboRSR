#!/bin/bash
# Install the verified local-asset-import products (README「改服务器共享资产库」steps 3-4) and expose them on the platform.
# All categories are new: nothing in the shared library is overwritten, so the rollback record is a list of added dirs.
set -euo pipefail
R=/data/robodojo-runs/local-asset-import-20261027
LIB=/data/robodojo-assets/Assets/Object/RoboDojo/Rigid
MIRROR=/root/RoboDojo_assets/Assets/Object/RoboDojo/Rigid
REPO_OBJ=/root/RoboDojo/real2sim/objects/Rigid
TS=$(date +%Y%m%d-%H%M%S)
MANIFEST=/data/robodojo-assets/backups/added-local-asset-import-$TS.txt
cd $R/ovl/Rigid
CATS=$(ls)
for c in $CATS; do
    if [ -e "$LIB/$c" ] || [ -e "$REPO_OBJ/$c" ]; then echo "conflict: $c"; exit 1; fi
done
{
    echo "# added by local-asset-import ($TS); rollback: rm -rf each path below"
    for c in $CATS; do echo "$LIB/$c"; echo "$MIRROR/$c"; echo "$REPO_OBJ/$c"; done
} > "$MANIFEST"
cp "$MANIFEST" $R/

fail=0
for c in $CATS; do
    cp -a "$c" "$LIB/$c"
    diff -r "$c" "$LIB/$c" > /dev/null || { echo "DIFF FAIL $c"; fail=1; }
done
echo "shared library: $(find $(printf "$LIB/%s " $CATS) -name object.usdz | wc -l) object.usdz, diff_fail=$fail"

if [ -d "$MIRROR" ]; then
    for d in */[0-9]*; do
        mkdir -p "$MIRROR/$d"
        cp "$d/metadata.json" "$d/description.json" "$MIRROR/$d/"
    done
    echo "mirror: $(find $(printf "$MIRROR/%s " $CATS) -name metadata.json | wc -l) metadata.json"
else
    echo "mirror missing: $MIRROR"
fi

cd $R/real2sim/objects/Rigid
for c in $CATS; do cp -a "$c" "$REPO_OBJ/$c"; done
echo "platform objects: $(find $(printf "$REPO_OBJ/%s " $CATS) -name spec.json | wc -l) spec.json, $(find $(printf "$REPO_OBJ/%s " $CATS) -name sim.png | wc -l) sim.png"
echo "manifest: $MANIFEST"
exit $fail
