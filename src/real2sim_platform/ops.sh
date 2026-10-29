#!/usr/bin/env bash
# Runs the Real2Sim platform on the GPU server apart from the work checkouts there.
#
#   $R2S_HOME/releases/<sha>/  code: `git archive` of one commit, immutable (chattr +i), never anyone's workspace
#   $R2S_HOME/current          -> the release the service runs; $R2S_HOME/previous: the one before
#   $R2S_DATA/state/           everything the platform writes: platform.sqlite3, platform.log, real2sim/
#                              (registrations, photos, and what agents publish), output/, cache/ (thumbnails), backups/
#   $R2S_DATA/platform.env     R2S_PORT, R2S_ASSETS (the shared library, read live), R2S_EVAL_ARGS (--eval-root ...)
#   $R2S_DATA/tunnel.env       optional, not in git: R2S_TUNNEL_KEY, R2S_TUNNEL_LISTEN (addr:port), R2S_TUNNEL_HOST
# A release's real2sim/, output/ and .cache/ are links into state/, so the scripts the platform imports (build_asset,
# appearance, rsi_profile) read and write there as well. The service may write nothing but state/ (systemd
# ProtectSystem=strict); the asset library, eval results and the release are read-only to it.
# The platform's registrations (spec.json, registration.json, ...) live only in state/: deploys and publishes never
# overwrite them, and `export` copies them into a checkout to commit.
#
# Usage, as root on the server:
#   ops.sh init [WORKSPACE]       once, old platform stopped: copy real2sim/, output/, the database out of WORKSPACE
#   ops.sh install                write and enable the systemd units (and the tunnel's if tunnel.env exists)
#   ops.sh deploy SHA             release SHA from $R2S_GIT; real2sim/ files git changed since the running release go
#                                 into state (backed up); switch, restart; switches back if it does not come up
#   ops.sh rollback               back to the previous release (code only; state/ stays as it is)
#   ops.sh publish SRC DEST       an agent's output (preview/, check.json, result.json, ...) into state/DEST, DEST under
#                                 real2sim/ or output/; replaced files go to state/backups/; then rescan
#   ops.sh export WORKTREE [PATH] registrations and reference photos under PATH (default real2sim) into WORKTREE
#   ops.sh scan                   rescan, as 同步项目数据 in the page
#   ops.sh status
set -euo pipefail

R2S_HOME=${R2S_HOME:-/opt/r2s-platform}
R2S_DATA=${R2S_DATA:-/data/r2s-platform}
R2S_GIT=${R2S_GIT:-/root/RoboDojo}  # any clone holding the commits; only read by sha
R2S_SERVICE=${R2S_SERVICE:-r2s-platform}
STATE=$R2S_DATA/state
OWNED=(spec.json modeling_prompt.md registration.json appearance.json randomization.json request.json)
export PYTHONDONTWRITEBYTECODE=1  # releases are immutable; their bytecode is compiled at deploy

die() { echo "ops.sh: $*" >&2; exit 1; }
port() { sed -n 's/^R2S_PORT=//p' "$R2S_DATA/platform.env"; }  # a systemd EnvironmentFile, not shell
excludes() { for f in "${OWNED[@]}"; do printf -- '--exclude=%s\n' "$f"; done; }

init() {
  local ws=${1:-/root/RoboDojo}
  [ -e "$STATE/platform.sqlite3" ] && die "$STATE already holds a platform"
  mkdir -p "$R2S_HOME/releases" "$STATE"/{real2sim,output,cache,backups}
  rsync -a "$ws/real2sim/" "$STATE/real2sim/"
  [ -d "$ws/output" ] && rsync -a "$ws/output/" "$STATE/output/"
  if [ -f "$ws/.cache/real2sim_platform.sqlite3" ]; then
    python3 -c 'import sqlite3, sys; d = sqlite3.connect(sys.argv[2]); sqlite3.connect(sys.argv[1]).backup(d); d.close()' \
      "$ws/.cache/real2sim_platform.sqlite3" "$STATE/platform.sqlite3"
  fi
  [ -f "$R2S_DATA/platform.env" ] || printf '%s\n' R2S_PORT=8765 R2S_ASSETS=/data/robodojo-assets/Assets \
    "R2S_EVAL_ARGS=--eval-root $ws/eval_result" > "$R2S_DATA/platform.env"
  echo "state in $STATE; settings in $R2S_DATA/platform.env"
}

install_units() {
  cat > "/etc/systemd/system/$R2S_SERVICE.service" <<EOF
[Unit]
Description=Real2Sim platform ($R2S_HOME/current, state in $STATE)
After=network-online.target

[Service]
EnvironmentFile=$R2S_DATA/platform.env
Environment=PYTHONDONTWRITEBYTECODE=1
ExecStart=/usr/bin/python3 -u $R2S_HOME/current/src/real2sim_platform/server.py --port \${R2S_PORT} --db $STATE/platform.sqlite3 --assets \${R2S_ASSETS} \$R2S_EVAL_ARGS
Restart=always
RestartSec=3
ProtectSystem=strict
ProtectHome=read-only
ReadWritePaths=$STATE
PrivateTmp=yes
NoNewPrivileges=yes
StandardOutput=append:$STATE/platform.log
StandardError=inherit

[Install]
WantedBy=multi-user.target
EOF
  if [ -f "$R2S_DATA/tunnel.env" ]; then
    cat > "/etc/systemd/system/$R2S_SERVICE-tunnel.service" <<EOF
[Unit]
Description=Reverse tunnel to the Real2Sim platform's public entry
After=network-online.target $R2S_SERVICE.service

[Service]
EnvironmentFile=$R2S_DATA/platform.env
EnvironmentFile=$R2S_DATA/tunnel.env
ExecStart=/usr/bin/ssh -i \${R2S_TUNNEL_KEY} -o BatchMode=yes -o IdentitiesOnly=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -N -R \${R2S_TUNNEL_LISTEN}:127.0.0.1:\${R2S_PORT} \${R2S_TUNNEL_HOST}
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
  fi
  local units=("$R2S_SERVICE")
  [ -f "$R2S_DATA/tunnel.env" ] && units+=("$R2S_SERVICE-tunnel")
  systemctl daemon-reload
  systemctl enable "${units[@]}"
  [ "$R2S_HOME" = /opt/r2s-platform ] && ln -sfn "$R2S_HOME/current/src/real2sim_platform/ops.sh" /usr/local/bin/r2s
  echo "enabled ${units[*]}; start the platform with: ops.sh deploy SHA"
}

switch() { ln -sfn "$1" "$R2S_HOME/.current.new" && mv -T "$R2S_HOME/.current.new" "$R2S_HOME/current"; }

restart() {
  local p n
  p=$(port)
  systemctl restart "$R2S_SERVICE"
  n=$(systemctl show -p NRestarts --value "$R2S_SERVICE")  # a manual restart resets it; RestartSec=3 leaves time to read
  for _ in $(seq 180); do  # the first scan runs before the server listens
    [ "$(curl -s -o /dev/null -w '%{http_code}' "http://127.0.0.1:$p/api/info")" = 401 ] && return 0
    [ "$(systemctl show -p NRestarts --value "$R2S_SERVICE")" = "$n" ] || return 1  # it exited: Restart=always
    sleep 1
  done
  return 1
}

merge_repo_data() {  # real2sim/ files changed in git from release $1 to $2 into state, but registrations and LFS files
  [ -n "$1" ] || return 0
  local tmp files
  mapfile -t files < <(git -C "$R2S_GIT" diff --name-only --no-renames --diff-filter=AM "$1" "$2" -- real2sim)
  [ ${#files[@]} -gt 0 ] || return 0
  tmp=$(mktemp -d)
  git -C "$R2S_GIT" archive "$2" -- "${files[@]}" | tar -x -C "$tmp"
  { grep -rlZ '^version https://git-lfs.github.com/spec/v1' "$tmp" || true; } | xargs -0 -r rm -f  # LFS is skipped here
  mapfile -t ex < <(excludes)
  rsync -a --backup --backup-dir="$STATE/backups/deploy-$(date +%Y%m%d-%H%M%S)/real2sim" "${ex[@]}" \
    "$tmp/real2sim/" "$STATE/real2sim/"
  rm -rf "$tmp"
}

deploy() {
  local sha rel tmp prev
  sha=$(git -C "$R2S_GIT" rev-parse --verify "${1:?usage: ops.sh deploy SHA}^{commit}")
  rel=$R2S_HOME/releases/$sha
  if [ ! -d "$rel" ]; then
    tmp=$R2S_HOME/releases/.tmp-$sha
    rm -rf "$tmp" && mkdir -p "$tmp"
    git -C "$R2S_GIT" archive "$sha" | tar -x -C "$tmp" --anchored --exclude=real2sim --exclude=output --exclude=.cache
    ln -s "$STATE/real2sim" "$tmp/real2sim"
    ln -s "$STATE/output" "$tmp/output"
    ln -s "$STATE/cache" "$tmp/.cache"
    python3 -m compileall -q "$tmp/src" "$tmp/scripts" "$tmp/env" >/dev/null 2>&1 || true
    mv "$tmp" "$rel"
  fi
  find "$rel" ! -type l -exec chattr +i {} +  # links cannot carry the flag; the directories holding them do
  prev=$(readlink "$R2S_HOME/current" || true)
  merge_repo_data "${prev:+$(basename "$prev")}" "$sha"
  switch "$rel"
  if ! restart; then
    [ -n "$prev" ] && switch "$prev" && restart || true
    die "release $sha did not come up; running ${prev:-nothing}; see $STATE/platform.log"
  fi
  [ -n "$prev" ] && [ "$prev" != "$rel" ] && echo "$prev" > "$R2S_HOME/previous"
  echo "running $sha"
}

rollback() {
  local prev cur
  prev=$(cat "$R2S_HOME/previous" 2>/dev/null) || die "no previous release"
  cur=$(readlink "$R2S_HOME/current")
  switch "$prev"
  restart || die "$prev did not come up either; see $STATE/platform.log"
  echo "$cur" > "$R2S_HOME/previous"
  echo "running $(basename "$prev")"
}

scan() {  # a short-lived session of the first admin, like the page's button
  python3 - "$R2S_HOME/current/src/real2sim_platform" "$STATE/platform.sqlite3" "$(port)" <<'PY'
import sys
import urllib.request

sys.path.insert(0, sys.argv[1])
import server

store = server.Store(sys.argv[2])
token = store.open_session(next(u["username"] for u in store.users() if u["admin"]))
try:
    req = urllib.request.Request(
        f"http://127.0.0.1:{sys.argv[3]}/api/scan", method="POST", headers={"Cookie": f"{server.COOKIE}={token}"}
    )
    print(urllib.request.urlopen(req, timeout=600).read().decode())
finally:
    store.end_session(token)
PY
}

publish() {
  local src=${1:?usage: ops.sh publish SRC DEST} dest=${2:?usage: ops.sh publish SRC DEST} backup
  case $dest in real2sim/?*|output/?*) ;; *) die "DEST must be under real2sim/ or output/" ;; esac
  case /$dest/ in */../*|*/./*) die "DEST must be a plain relative path" ;; esac
  [ -e "$src" ] || die "no $src"
  backup=$STATE/backups/publish-$(date +%Y%m%d-%H%M%S)
  mapfile -t ex < <(excludes)
  if [ -d "$src" ]; then
    mkdir -p "$STATE/$dest"
    rsync -a --backup --backup-dir="$backup/$dest" "${ex[@]}" "$src/" "$STATE/$dest/"
  else
    for f in "${OWNED[@]}"; do [ "$(basename "$dest")" = "$f" ] && die "$f is written by the platform"; done
    mkdir -p "$(dirname "$STATE/$dest")"
    rsync -a --backup --backup-dir="$backup/$(dirname "$dest")" "$src" "$STATE/$dest"
  fi
  scan
}

export_registrations() {
  local wt=${1:?usage: ops.sh export WORKTREE [PATH]} sub=${2:-real2sim}
  case $sub in real2sim|real2sim/?*) ;; *) die "PATH must be under real2sim/" ;; esac
  git -C "$wt" rev-parse --git-dir >/dev/null 2>&1 || die "$wt is not a git checkout"
  local inc=(--include='*/' --include='reference/***')
  for f in "${OWNED[@]}"; do inc+=(--include="$f"); done
  mkdir -p "$wt/$sub"
  rsync -a -m "${inc[@]}" --exclude='*' "$STATE/$sub/" "$wt/$sub/"
}

status() {
  echo "release:  $(readlink "$R2S_HOME/current" || echo none)"
  echo "previous: $(cat "$R2S_HOME/previous" 2>/dev/null || echo none)"
  systemctl --no-pager --lines=0 status "$R2S_SERVICE" | sed -n '1,3p'
}

cmd=${1:-}
shift || true
case $cmd in
  init) init "$@" ;;
  install) install_units ;;
  deploy) deploy "$@" ;;
  rollback) rollback ;;
  publish) publish "$@" ;;
  export) export_registrations "$@" ;;
  scan) scan ;;
  status) status ;;
  *) sed -n '2,/^set -e/{/^set -e/d;s/^# \{0,1\}//;p;}' "$0"; exit 2 ;;
esac
