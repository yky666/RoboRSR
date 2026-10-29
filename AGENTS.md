# RoboDojo — Agent Guide

Project conventions, layout, eval commands, and validation loops live in `CLAUDE.md`; follow it as well.

## Session Handoff (required)

`HANDOFF.md` at the repo root carries context across sessions.

- **Before** editing: read `HANDOFF.md` for current branch, goal, decisions, and open issues.
- **After** finishing any task that changes files: update `HANDOFF.md` — date, current state, what was done, key decisions, validation actually run (or why not), remaining issues/risks, and the entry point for the next session. Keep its existing sections; trim stale detail instead of only appending.
- Include the updated `HANDOFF.md` in the same commit as the change. If it conflicts with the code, Git, or test results, those win — fix the file.

## Data Platform on `5090-2` (`r2s`)

The team's data-collection platform runs on `5090-2`, apart from everyone's checkouts. `/root/RoboDojo` is a workspace, not the platform: files put there are not shown, and editing it cannot break the platform.

| What | Where | Who writes it |
| :-- | :-- | :-- |
| Platform code | `/opt/r2s-platform/current` → `releases/<sha>/` (a `git archive`, `chattr +i`) | only `r2s deploy` |
| Platform state: database, registrations (`spec.json`, `registration.json`, `appearance.json`, `randomization.json`, `request.json`, `modeling_prompt.md`), photos, published renders and results | `/data/r2s-platform/state/` (`real2sim/`, `output/`, `platform.sqlite3`, `backups/`) | the platform, and `r2s publish` |
| Shared asset library, eval results | `/data/robodojo-assets/Assets`, `/root/RoboDojo/eval_result` | read live by the platform, never written |

Every `r2s` command runs **on `5090-2`, as root** (`r2s` is `/usr/local/bin/r2s` → `src/real2sim_platform/ops.sh` of the running release; usage: `r2s help`). Paths given to it are paths on `5090-2`.

- **Read a registration** (input for modeling a scene, object, task or evaluation): read-only from the state dir, e.g. `rsync -az --exclude preview/ 5090-2:/data/r2s-platform/state/real2sim/objects/<id>/ real2sim/objects/<id>/` on your machine, or `r2s export <worktree on 5090-2> real2sim/objects/<id>` on the server. Commit registrations and photos from your branch; never copy them back into the state dir.
- **Show your output on the platform** (`preview/`, `check.json`, `instructions.json`, `result.json`, `output/dataset_inventory.json`, ...): `r2s publish <SRC on 5090-2> <DEST under real2sim/ or output/>`. Replaced files go to `state/backups/publish-<time>/`, registration files are never overwritten, and the platform rescans. Work produced on your machine goes to your run dir first:

  ```bash
  rsync -az real2sim/objects/<id>/preview/ 5090-2:/data/robodojo-runs/<run>/real2sim/objects/<id>/preview/
  ssh 5090-2 r2s publish /data/robodojo-runs/<run>/real2sim/objects/<id>/preview real2sim/objects/<id>/preview
  ```

- **After installing into the shared asset library**: `ssh 5090-2 r2s scan`.
- **Deploy platform code** (only after it is merged into `main`): push the commit to `5090-2:/root/RoboDojo` (`GIT_LFS_SKIP_PUSH=1 git push 5090-2:/root/RoboDojo <sha>:refs/heads/incoming-main`), then `ssh 5090-2 r2s deploy <sha>`; it switches back by itself if the release does not come up. `r2s rollback` restores the previous code (not the state dir), `r2s status` shows what runs. Do not start the platform any other way, and do not stash/fast-forward `/root/RoboDojo` to deploy it.
- Never edit `/opt/r2s-platform` or `/data/r2s-platform/state` by hand, and never restart the platform with `pkill`/tmux; it is the systemd unit `r2s-platform` (public tunnel: `r2s-platform-tunnel`).
