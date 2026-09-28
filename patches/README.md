# Local patches to Arena

Our changes to the two upstream Arena repos, exported with `git diff`. Every edit in the source is also marked with
the comment `initial-arena-testing`, except the hospital fixes and the two startup fixes from the first chat session.

| Patch | Upstream repo @ commit | Contents |
|---|---|---|
| `arena-rosnav.patch` | `~/arena_ws/src/arena/arena-rosnav` @ `c2ff4a87` | map-callback deadlock fix, HuNav `_wall_segments` init, `localization` + `scenario` launch args, ground-truth mode, episode-reset fixes (stale goal status, spawn-height creep), `timeout: 600`, HuNav phantom-wall fix (wall geometry names + `<ignore_models>`) |
| `hunav_gz_plugin.patch` | `~/arena_ws/src/gazebo/hunav_gz_plugin` @ `9492688` | debug-loop removal, `update_rate` honoured (C++: rebuild `hunav_gz_plugin` after applying) |
| `simulation-setup.patch` | `~/arena_ws/src/arena/simulation-setup` @ `3f142b25` | hospital world/scenario fixes, Jackal ground-truth pose plugin + `{odom_source}` bridge switch, `bt_navigator.default_server_timeout: 200` |

Not in the patches (untracked files): the `*.before-*` backups, and `worlds/school_hallway/`, which contains per-file
symlinks created by `scripts/generate_hallway.py --link`.

## Regenerate after changing Arena

```bash
cd ~/Documents/Lyu_Lab/initial-arena-testing
git -C ~/arena_ws/src/arena/arena-rosnav diff > patches/arena-rosnav.patch
git -C ~/arena_ws/src/arena/simulation-setup diff > patches/simulation-setup.patch
git -C ~/arena_ws/src/gazebo/hunav_gz_plugin diff > patches/hunav_gz_plugin.patch
```

## Re-apply on a fresh checkout / after an Arena update

```bash
cd ~/arena_ws/src/arena/arena-rosnav && git apply --3way ~/Documents/Lyu_Lab/initial-arena-testing/patches/arena-rosnav.patch
cd ~/arena_ws/src/arena/simulation-setup && git apply --3way ~/Documents/Lyu_Lab/initial-arena-testing/patches/simulation-setup.patch
cd ~/arena_ws/src/gazebo/hunav_gz_plugin && git apply --3way ~/Documents/Lyu_Lab/initial-arena-testing/patches/hunav_gz_plugin.patch
```

`--3way` falls back to a merge when upstream changed nearby lines. Resolve any conflicts, then rebuild the affected packages.
