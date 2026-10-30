# Project Handoff

更新时间：2026-10-30
本会话分支：`chongzheng/platform-isolation`（worktree `../RoboDojo-platform-isolation`，基于 `origin/main@a23b9c2`）：数采平台和 `5090-2` 上的工作区隔离，PR #50 已合并，10-30 00:23 线上已切换（「最近任务」第一节）。之前的会话（场景 3D 预览、场景审核、办公室布局、物体指令名称、BanzhiFlywheel 改版等）都已进 `main` 并部署，见「最近任务」。下行保留主线历史。
主线历史：G1-D 桌面版 / 移动版（[PR #17](https://github.com/Zheng-Chong/RoboRSR/pull/17)）已于 2026-10-21 合并进 `main`。深圳实验室场景（[PR #22](https://github.com/Zheng-Chong/RoboRSR/pull/22)）已于 2026-10-21 合并（`2975b4a`）并部署到数采平台。更早：流水线规范化（[PR #1](https://github.com/Zheng-Chong/RoboRSR/pull/1)）和学校场景接入（[PR #2](https://github.com/Zheng-Chong/RoboRSR/pull/2)）都已于 2026-10-20 合并，短分支已删除。
当前目标：
- **榜单冲分（2026-10-22 起，20 天）**：用本平台做多轮 Coding Agent 仿真数据 → 评测 → 对抗式（找失败、补难例和恢复数据）迭代，提高 OpenWAM、pi0.5 在上游 RoboDojo 榜单（54 个 dual X5 任务）上的成功率。用户确认：榜单允许用自生成数据训练；训练在 dsw-4（一轮 ≤ 1 天）。见「最近任务」第一节。
- 把 G1-D 的三条流水线规范化：Real2Sim 采集（物体、场景）、仿真数据生成，以及本仓库内的仿真评估。训练在仓库外。

本文件只记录当前状态。逐次实验的细节（相机拟合、抓取调参、倒豆参数扫描、各次服务器验证）见 `git show 4fb9290:HANDOFF.md`。数采平台的运维细节（端口、账号、隧道、重启方法）已移到本机 `.agent/ops.local.md`，不入库，因为仓库是公开的。

## 项目状态

- **机器人** G1-D + Dex1-1（带相机版夹爪）：两条 7 自由度手臂加两指夹爪，关节角控制，30 Hz。分两个版本（对应官方标准版 / 旗舰版）：桌面版 `g1d` 底盘固定，腰（`waist_joint_state`）和升降（`lift_joint_state`）是可选动作键，不发就保持；移动版 `g1d_mobile` 再加差速底盘 `base_velocity` [vx, vyaw]，不发就停车。已有数据和策略都只用 16 维手臂 + 夹爪，不受影响。细节见 README「机器人本体」和 CONTRACT「观测与动作」。
  - 头部用双目的左眼 `cam_head`（棋盘格标定 fx≈279.5，宇树合同外参），两个腕部相机 fx≈314；投影到真机画面核对过（误差 0.4–3.6 px）。
  - 关节驱动参数（刚度 4400、阻尼 40）和夹爪力是拍定的，没做系统辨识。夹爪从 10-21 起改为每指刚度 3.6e4 N/m、上限 55 N，是按真机 kp 估算的（见「剩余事项」），之前是 1e4 和 20 N；官方规格（零售商转载）是 5–120 N 可调。开合速度从 10-23 起限到每指 0.064 m/s（真机遥操作数据的合爪速度；之前是 URDF 的 0.2 m/s，即 Dex1-1 最快档）。两项在 10-27 一起合进 `main`（PR #24、#26）。仿真里能读到夹持力：`obs["gripper_force"]`（指垫 ContactSensor，单位 N，不进数据集）。同事把"相机+动作标定工具包"作为 `g1d_calib_toolkit_20261019.zip` 传到了仓库根目录，还没接入。
  - 构建：`python scripts/g1d/build_asset.py`（读 `real2sim/robot/g1d_v4`）。下载官方资产后先跑一次 `utils/update_embodiment_config_path.py`。
- **场景**（`real2sim/scenes/`）：env 配置 `g1d` 是办公室，`g1d_counter` 是吧台（移动版 `g1d_counter_mobile` 加了左侧客区；10-28 起过道里有高吧台和绿色岛台，见「最近任务」第一节），`g1d_lab` 是公司实验室，`g1d_school` 是学校实验室绿幕桌，`g1d_shenzhen_lab` 是深圳实验室办公桌。前三个从照片拟合，尺度靠先验；学校的来自同事用真机相机画面拟合的 MuJoCo 孪生；深圳实验室是第一个按平台登记、用卷尺定尺度的场景（卷尺三项通过，站位和叠图待做），10-27 起按房间全景补成整层房间。其余都没有卷尺实测，都没有叠图验收。状态汇总：`python scripts/scenes/acceptance.py`（目前全部 pending）。新场景先在数采平台登记（≥ 5 张照片 + 卷尺实测，写 `registration.json`），再按 `real2sim/scenes/SKILL.md` 交给 coding agent 建模。新场景先在数采平台「登记场景」（至少 5 张照片 + 卷尺实测，写 `real2sim/scenes/<场景>/registration.json`），再按 `real2sim/scenes/SKILL.md` 建模。
- **物体**：每件资产的来源和匹配程度看 `real2sim/ASSETS.md`。按数采平台的登记（`real2sim/objects/<id>/spec.json` + 照片）给实物建模，交给 coding agent 时按 `scripts/objects/SKILL.md`；平台物体页生成的「建模任务」文本第一行就指向它。
  - 量杯、塑料马克杯、红豆有实测尺寸和质量。量杯 `00002–00005`、马克杯 `00001–00005` 是估计的泛化变体。
  - 共享资产库缺 `measuring_cup`、`plastic_mug`，以及量杯任务的布局。
  - 学校的纸杯 `paper_cup/0000{0,1}`、纸盘 `paper_plate/0000{0,1,2}` 是孪生的拟合几何，没在平台登记，没有实测，未装共享库。平台上能看到它们（10-20 部署，见下面「数采平台」）。
  - 耳机充电盒 `airpods/00000`：第一件按 `scripts/objects/SKILL.md` 从平台登记建的物体，实测尺寸和质量，已装共享库。
  - 农夫山泉塑料瓶 `Clutter/plastic_bottle/00000` 和 5 个泛化变体：按平台登记建，带盖、贴图标签；另有能拧盖的 `Articulation/plastic_bottle/0000{0..5}`。都已装共享库（2026-10-23，见「最近任务」）。
  - 自建物体写的 `friction` 不生效：运行时只读 `physics.static_friction` 和 `dynamic_friction`。
  - 指令名称：`scripts/objects/object_names.json` 给平台上 54 件完备物体（含变体）和 13 个能拧盖的瓶子写了中英文各 5–6 个从简到详的名称，`object_names.py apply` 写进 `description.json` 的 `names`，指令模板的 `<标签>` 从中取词（10-29，见「最近任务」第一节）。共享库里已装的 38 件已写入。
- **任务和脚本专家**（`scripts/g1d/scripted_demo.py`）：
  - 拾取、叠块：三个场景 seed 0 都成功。
  - 杯对杯倒豆、量杯对倒：部分 seed 成功。
  - 壶倒杯：5.6 mm 的豆子会堵住 13 mm 的壶嘴。
  - 移动送杯 `g1d_deliver_cup_to_lounge`（`g1d_counter_mobile`）：10-24 seed 0 前 4 个布局 4/4；10-28 现在的 `main` 上只剩 2/4，布局 0、3 抓杯时杯子没被抬起，旧场景下也一样（见「最近任务」第一节）。
  - 冷水壶倒水再送杯 `g1d_pour_water_deliver_to_lounge`（`g1d_counter_mobile`）：seed 0 布局 0，10-28 带岛台的场景下重跑仍成功。
- **数据**：所有数据集登记在 `scripts/datagen/registry.json`。目前有 2 份办公室量杯全量倒豆数据，各 100 条，LeRobot v3.0，都是按旧的 legacy 格式导出的：
  - 固定朝向的一份，已上传 ModelScope 私有仓库；
  - 随机左右手、随机朝向、大范围位置的一份，也已上传 ModelScope 私有仓库（单独的仓库）。
- **数采平台** `src/real2sim_platform/`（页面名 BanzhiFlywheel，分「仿真平台」和「数据集」两个板块）：部署在 `5090-2`，经阿里云反向隧道对外访问。10-30 起由 `r2s`（`src/real2sim_platform/ops.sh`）管理：代码是只读 release `/opt/r2s-platform/current`，平台只写状态目录 `/data/r2s-platform/state/`，systemd `r2s-platform` 和 `r2s-platform-tunnel`，不再从 `/root/RoboDojo` 起、不再用 tmux。agent 怎么用见 `AGENTS.md`「Data Platform on `5090-2`」（「最近任务」第一节）。场景卡片封面是 RTX 渲染图，抽屉里有 3D 预览（见「最近历史」10-21）。
- **服务器**：只用 `5090-2`（8×5090）。
  - 共享资产库：`/data/robodojo-assets/Assets`（10-20 整体迁来，39 GB）。改它要先征得同意，备份放 `/data/robodojo-assets/backups/`。物体分两个库：`Object/RoboDojo/` 是官方的，`Object/Real2Sim/` 是自建孪生（10-28 起；RoboDojo 下留有指向它的软链接，见「最近任务」第一节）。运行目录的 `Assets/` 覆盖目录里，不改的部分直接从它软链接或拷贝。数采平台 10-30 起直接只读这个库；旧的元数据镜像 `/root/RoboDojo_assets/Assets` 不再用，可以删。
  - 仿真环境 `/root/miniconda3/envs/RoboDojo`；导出环境 `robodojo-lerobot`（lerobot 0.4.4，10-19 补装了 h5py 3.16，`pip -i` 用清华镜像）；上传环境 `/data/robodojo-runs/ms-venv`；运行目录在 `/data/robodojo-runs/`。

## 最近任务

### 裁纸刀本体和三个颜色变体（2026-10-30，codex/objects-paper-knife-00000）

- 隔离目录 `/data/robodojo-runs/paper-knife-20261029`，worktree基于origin/main；生成器 `make_paper_knife.py`，原生工厂接触验证 `check_paper_knife.py`。输入、四张原图、照片派生贴图、独立名称条目及REPORT/delivery_report已归档。
- 用户补测厚17.14 mm并指示继续；登记新revision `e249245a730f5980ca6d5b91285c3dad45f476e8a4ded6581e561aa9bbf0fdd4`。长度152.88、最大宽39.44、中间宽32.83 mm、58.9 g保持原实测。对话补测来源署名与原登记人区别记录，旧输入有备份。局部几何是照片拟合，非额外实测。
- 本体00000和颜色变体00001–00003；开放导轨、分段刀片、尾孔/压印保留，压印来自照片，独立label_blade_stamp；rt_color仅model，固定尾框颜色用于名称区分。各语言各5条，apply/check通过。
- 标准check_object四件OK；validation06原生部署项目工厂加载、质量、落地、尾孔/实体通道、推挡、夹持代理抬升/释放、复位均通过。夹持代理49.94 mm/50 mm，非机器人场景验证。相机为同侧近似、背/底面缺照片，摩擦/材料未标定。保持研究资产，未装共享库。
- scripts/check.sh 91 passed、1 skipped、82 subtests passed；首轮LFS指针问题通过git-lfs materialize解决，失败日志保留。没有把部署工作树其他会话的物理加载器改动纳入本分支；报告用源码SHA识别实际测试实现。
- 下一步：用户审核预览并补背面/底面照片；完整机器人场景抓取验证后再申请基础资产验收。共享库部署需要用户授权，届时只复制验证过的这4个ID并备份，不在共享库重新生成。


### 数采平台和 `5090-2` 工作区隔离（2026-10-29 23:xx 至 10-30 00:2x，`chongzheng/platform-isolation`，[PR #50](https://github.com/Zheng-Chong/RoboRSR/pull/50) 已合并，线上已切换）

- **起因**：平台从 `/root/RoboDojo` 起，那里同时是大家的工作区（218 个未提交改动），数据库、登记、照片也写在里面；部署靠 stash → ff → pop，10-29 11:42 的重启把别的会话未提交的 `server.py` 带上了线。所有人都是 root，靠权限拦不住。用户选定：登记以平台状态目录为准，定稿时写回 Git；先在隔离端口验证，再约时间切线上。
- **做法**（新增 `src/real2sim_platform/ops.sh`，装好后 `/usr/local/bin/r2s`；`server.py` 只在 `/api/info` 加 `data_root`）：
  - 代码：`/opt/r2s-platform/releases/<sha>/` 是 `git archive` 的副本，`chattr +i`，`current` 软链接指向在跑的那个；release 里的 `real2sim`、`output`、`.cache` 是指向状态目录的软链接，所以 `build_asset`、`appearance`、`rsi_profile` 这些按自身位置找 `real2sim/` 的模块也读写状态目录，平台代码不用改路径。
  - 状态：`/data/r2s-platform/state/`（`platform.sqlite3`、`platform.log`、`real2sim/`、`output/`、`cache/thumbs`、`backups/`）；设置 `/data/r2s-platform/platform.env`（端口、`R2S_ASSETS=/data/robodojo-assets/Assets`、`R2S_EVAL_ARGS=--eval-root /root/RoboDojo/eval_result`），隧道 `tunnel.env`（不入库）。
  - systemd `r2s-platform`（和可选的 `r2s-platform-tunnel`）取代 tmux：`ProtectSystem=strict`、`ProtectHome=read-only`、`ReadWritePaths=` 只有状态目录，`Restart=always`，开机自启。
  - `deploy SHA`：建 release，把上一个 release 到这个 release 之间 git 改过的 `real2sim/` 文件拷进状态目录（不含平台写的登记文件和 LFS 指针，替换的进 `backups/`），切换、重启；进程退出（NRestarts 变了）或 180 s 内 `/api/info` 不回 401 就切回去。`rollback` 只换代码。
  - `publish SRC DEST`：agent 的渲染、`check.json`、`result.json` 等进状态目录（DEST 限 `real2sim/`、`output/`；`spec.json`、`registration.json`、`appearance.json`、`randomization.json`、`request.json`、`modeling_prompt.md` 永远不被覆盖），替换的旧文件进 `backups/`，然后自动扫描。`export WORKTREE [PATH]` 把登记和 `reference/` 照片拷进 checkout 入库。`scan` 用第一个管理员的临时会话触发扫描。
  - 文档：`CLAUDE.md`、README 平台一节、`scripts/objects/SKILL.md`、`real2sim/scenes/SKILL.md`、`scripts/datagen/SKILL.md`、页面里给 coding agent 的任务文本，"同步到部署机"都改成 `r2s publish`，登记从状态目录读；共享库装好后不再刷新镜像，`r2s scan` 即可。
- **验证**（`5090-2` 隔离实例 `r2s-isotest`，端口 8766，目录 `/data/robodojo-runs/platform-isolation-20261029-225907/`，验证完已停用删除；线上 8765 没动）：
  - `init` 从 `/root/RoboDojo` 拷出 `real2sim/`（4541 个文件，`diff -rq` 无差异）、`output/`、数据库在线备份（物体 2220、13 个账号）。部署 6 s；扫描 2.6 s，直接读共享库：场景 10、任务 68、物体 2220、回合 507。
  - 服务进程里只有状态目录可写；共享库、`/root/RoboDojo`、`eval_result`、`/data/robodojo-runs`、`/etc`、release 只读（`nsenter` 进它的挂载命名空间测 `W_OK`）。root 在外面改 release：追加、`sed -i`、新建、删软链接都是 Operation not permitted。
  - 网页上传照片落在状态目录，删除正常。`publish`：`spec.json` 被跳过、单独发布被拒，第二次发布时旧 `check.json` 进了备份，越界 DEST 被拒。`export` 只带出登记和 `reference/`，没有 `preview/`。
  - `d346685` ↔ 新提交来回部署：112 个登记文件校验和不变，LFS 指针数不变，替换的 25 / 39 个文件进备份；`rollback` 和再前进各 3 s。故意坏掉的提交 6 s 内判定失败并切回，页面一直 200。
  - 本机：`test_server.py`（直接运行的部分）、`test_dataset_catalog.cjs`、页面脚本 `node --check`、`ruff check/format`（0.11.7）、`git diff --check`、`bash -n` 通过（本机没有 shellcheck）。`5090-2` 隔离目录在 RoboDojo 环境里跑 `scripts/check.sh` 的 pytest 部分 92 passed / 82 subtests、任务清单 `--check` 通过（同步时没带 `.git`，lint 那段在本机跑）。测试实例和测试分支已删除，线上进程（22:25 起）没动。
- **切换**（10-30 00:23，`62a9571` = PR #50 合并提交）：`tunnel.env` 从旧隧道脚本生成；`tmux kill-session -t r2s-platform` → `init /root/RoboDojo` → `install` → `deploy`，停机 00:23:24–00:23:35 共 11 s；旧 tmux 隧道先转到新服务（公网首页 200、未登录 401），再换成 systemd 隧道（4 s 内恢复）。没有 tmux 会话了，两个 unit 开机自启。
  - 切换后：扫描场景 10、任务 68、物体 2228（共享库比旧镜像多 8 个）、回合 507、评测 17；账号 13、材质 990；`data_root` = 状态目录，`assets_ok`；服务进程里只有状态目录可写，root 改 release 被拒。
  - 本体页从 8 个变 5 个：`aurora_lite`、`g1d_aurora_upper`、`ur5_official` 的配置文件只在部署机工作区（10-23 起未提交，`env/robot_manager/robot_config/`），release 里没有，所以不再算已接入；登记还在，研究资产从 15 变 17。要恢复就把那些文件提交进 `main` 再 `r2s deploy`。
  - 回退到旧方式：`systemctl disable --now r2s-platform r2s-platform-tunnel`，按 `.agent/ops.local.md` 重建两个 tmux 会话；旧数据库 `/root/RoboDojo/.cache/real2sim_platform.sqlite3` 停在 00:23，之后的新登记先 `r2s export /root/RoboDojo` 拷回，数据库用 `/data/r2s-platform/state/platform.sqlite3`。
- **AGENTS.md** 加了「Data Platform on `5090-2` (`r2s`)」：各目录谁写、`r2s` 都在 `5090-2` 上以 root 跑、读登记 / 发布产出 / 装库后扫描 / 部署的命令和例子。
- **风险**：切换后直接往 `/root/RoboDojo/real2sim` 放的渲染平台看不到，要 `r2s publish`；`/root/RoboDojo_assets` 镜像不再用，可以删；状态目录本身 root 仍能手改，靠文档约束；`rollback` 不回退状态目录，要回退数据用 `backups/`。

### 已建模物体可「隐藏」；批量操作栏改成浮动工具栏（2026-10-30 09:3x）

- **隐藏**（用户选的：已建模物体不删，只从平台拿掉，资产和代码不耦合也不动）：已建模（有生成脚本或已在资产库）、没有任务在用、没有变体的物体，管理员看到的按钮从禁用的「删除物体」变成「隐藏」；隐藏后详情顶部有说明、按钮变「取消隐藏」。服务端 `PUT /api/items` 带 `{"hidden": bool}` 走单独分支：只有管理员（403）、只能带这一个字段（400）、物体要存在（404），只写数据库，不重写 `spec.json`。前端 `live()` 排除隐藏的，所以导航计数、总览、待审核数、任务向导的物体选择都不含它；`variantsOf` 不含隐藏的变体；物体页有「已隐藏 N」开关，打开时只列隐藏的物体（变体不收进本体卡片）。为什么不做真删除：平台进程对共享资产库只读（PR #50 的沙箱），资产库改动要审批；用户考虑后先只做隐藏。
- **浮动工具栏**（用户反馈黑色操作栏难看、没贴底）：物体页勾选后的批量栏改成底部居中的浅色毛玻璃胶囊（`--glass` + 模糊 + `--shadow-3`，离底 28px，出现时上滑），左边数字徽标「3 已选」、待审核数，按钮去边框分组（通过 / 退回… | 负责人 + 指派 | ×）。原来 `.bulkbar` 写在 `.obj-bulk` 后面、同级优先级，把圆角、背景、内边距盖掉了，现在浮动规则用 `.bulkbar.obj-bulk`。质检页的批量栏（列表顶部）同样从黑底改成浅灰底。之后按用户反馈：「退回…」去掉省略号（被当成没显示全）；管理员多一个红色「删除」，只删选中里能删的（与单件删除同一套 `objDeleteBlocker`；一起选中的变体不算阻碍，先删变体再删本体），已建模、任务在用的跳过并在确认框里说明，提示可以「隐藏」；没有能删的时按钮禁用。浏览器里用两个临时登记加一个已建模物体核对过：删 2 跳 1，服务端也没了，已建模的留在选择里。
- 本机：`test_server.py` 补了隐藏接口的 403/400/404/往返；浏览器里用管理员走了一遍隐藏 → 列表和计数少一件（69 → 68）→「已隐藏 1」里能看到 → 取消隐藏恢复；勾选 3 件截图核对浮动栏位置（距底 28px）和样式。

### 柔性物体补模型图、登记种类；非孪生保存不再删手写 spec.json（2026-10-30 00:5x）

- **用户要求**：补上可变形纸杯的模型图；毛绒卡皮巴拉和 FEM 软块登记成实心软体。
- **新脚本 `scripts/objects/render_model.py`**：柔性资产不能按刚体加载，`check_object.py` 跑不了。它按 `check_object.py` 单件视图的地面、灯光、机位和 640×480 渲染，把 USD 引用进来按包围盒放到地面上，不步进仿真（静止形状），写 `preview/00_model.png`。在 `5090-2` 隔离目录 `/data/robodojo-runs/flex-model-render-20261030/` 用 GPU 0 渲染了 `Dynamic/paper_cup/00000`、`Dynamic/soft_block/00000`（包围盒 87.9×87.9×136、100³ mm，与 metadata 一致；目测与刚体纸杯 `sim.png` 风格一致），`r2s publish` 发布（只增不删，测试图都还在）。卡皮巴拉已有 `00_sim.png`，没重渲。`scripts/objects/SKILL.md` 和建模 prompt 的柔性段指向这个脚本。
- **登记种类**：照 `r2s scan` 的做法用首个管理员的临时会话调平台 `PUT /api/items`（只改 `deformable.family`，其余字段原样带回）：纸杯 `bag → shell`，卡皮巴拉、软块 `null → volume`；署名是「管理员」。扫描后纸杯、卡皮巴拉的柔性验收按新种类重算为待验收（多了「压缩与回弹」）。
- **发现的平台 bug（已修）**：`write_spec` 对非孪生物体一律删 `spec.json`，所以保存软块（`twin: false`，`asset_origin: simulation_reference`）时把杨开元手写的 `state/real2sim/objects/Dynamic/soft_block/00000/spec.json` 删了。数据库记录完整，页面不受影响；原文件还在 `/root/RoboDojo/real2sim/objects/Dynamic/soft_block/00000/spec.json`（工作区未入库）。修复：只删自己写的（带 `"twin": true`），手写的留着；`test_server.py` 补了这一条。状态目录里那份不恢复（用户 10-30 09:11 决定；数据库记录完整，平台不依赖它）。

### 物体资产：刚体 / 柔性筛选、柔性封面与形变测试分开（2026-10-30 00:4x）

- **用户问题**：实物孪生里的柔性物体要不要单独分出来；「可变形纸杯」的渲染图和其他展示图完全不一样。结论（用户选定）：不单独分库（库按来源分，柔性是物理属性），在物体页加「全部 / 刚体 / 柔性」筛选；柔性物体的封面只用模型图，测试图单独放；补两种柔性种类。可变形纸杯 `Dynamic/paper_cup/00000` 仍单独一张卡，种类由用户在平台上从误标的「布袋」改成「薄壳」（状态目录不手改）。
- **原因**：卡片封面取 `preview/` 第一张图。可变形纸杯 `preview/` 里只有挤压三联、挤压到回弹 8 格和 G1-D 抓取序列三张验证图，没有单件模型图；其他柔性条目的落地、悬垂、压缩图也和模型图混在一起。
- **判定柔性**：`server.py` 物体多了 `auto.sim_backend`（资产 `metadata.json` 的 `backend`，只有柔性生成脚本写：`isaac_physx_fem`、`isaac_physx_particle_cloth`、`isaac_physx_d6_rod`；在共享库里核对过，恰好是 8 个柔性条目，传送带和刚体没有）。前端 `isFlex` = 登记了柔性种类，或有 `sim_backend`，或类型是 `Garment`；`flexFamily` 没登记时按后端推断（FEM → 实心软体、粒子布料 → 布料、D6 → 绳）。`real_material_calibrated` 不能当判据：锤子、空气清新剂等刚体也有。
- **封面与测试图**：柔性物体文件名匹配 `FLEX_TEST`（drop、suspend、squeeze、compress、recover、grasp、reset、rest、sheet、bend、twist、hang、press、release）的图归「形变测试」图库（抽屉里「实物参考与模型」下方），其余才做封面、渲染大图和变体缩略图；只有测试图时封面退回实拍并标「缺模型渲染」。刚体不受影响。卡片和列表多一个「柔性 · 种类」标签。
- **柔性种类**：`deformables.FAMILIES` 加 `shell`（薄壳 / 纸杯纸盒）和 `volume`（实心软体 / 毛绒海绵），两者验收多一项 `compression_recovery`（压缩与回弹）；建模 prompt 的柔性段补了薄壳、实心软体的要求，并要求另出一张文件名含 model 的单件模型图、测试图文件名写明测试项。`test_deformables.py` 补了这两种。
- 本机浏览器用一个物体模拟可变形纸杯（FEM 后端、薄壳、只有三张测试图）核对：筛选「刚体 67 / 柔性 1」切换正确，卡片封面退回实拍并显示「缺模型渲染」「柔性 · 薄壳」，抽屉渲染区提示缺单件模型图、测试图进「形变测试」，种类下拉有薄壳和实心软体。

### 代码块改成浅色、prompt 高亮（2026-10-29 22:5x）

- **用户反馈**：物体的统一 prompt 是一大块黑底，很难看。`.code`（统一 prompt 和各处命令共用）改成 `--surface-2` 浅底 + 细边框 + `--text-2` 字色，复制按钮跟着改成白底小按钮；最高 360px，超出在块内滚动。夜间主题跟着 token 走。
- **建模 prompt 加了 Markdown 高亮**（用户要求）：`code(text, label, fmt)` 多了第三个参数，建模 prompt 传 `mdHighlight`，命令块仍用 `esc`。`mdHighlight` 逐行处理：`#` 标题、`-`/`1.` 列表符号、行内反引号代码，独占一行的 `{`…`}` 之间按 JSON 给键、字符串、数字、true/false/null 上色（GitHub 配色，`light-dark()` 适配夜间）。先在原文上匹配再逐段转义，所以不会切坏实体；去掉标签后的文本与原文一致，复制内容不变（浏览器里用含 `<b>&amp;` 的样例核对过）。
- **确认按钮挪位**（用户反馈位置不对）：`modelingPanel` 的确认按钮和确认状态放进表单下方同一行 `.toolbar`；已确认后按钮变成次级「重新确认」，prompt 前加小标题 `.sub-h`「统一建模 prompt」。浏览器里核对了待确认和已确认两种状态。

### 场景详情 3D 预览置顶、预览调节控件整理（2026-10-29 22:5x）

- **用户要求**：3D 预览放在抽屉最上面；调节场景（材质、灯光）的 UI 优化。
- **改动**（只有 `index.html`）：`sceneDrawerView` 把 3D 段放进 `top`，`drawerScene` 先放它，再放名称、所属场景 / 工况、审核和其余内容。`v3dControls` 改成卡片里两组（材质：三个表面的材质选择；灯光与桌高：亮度、色温、主光方向、桌高滑块，数值右对齐），滑块用主题色 `accent-color` 并去掉全局输入框的边框和最小高度；底部一行「设为固定设置」（主按钮）、「载入固定设置」、「复原」，说明文字放在按钮下面。本机用临时假模型路径核对了顺序和控件（本机没有 `scene.glb`）。

### 3D 预览打不开时带上浏览器给出的原因（2026-10-29 22:4x）

- **用户问**：同一个浏览器以前能看 3D，现在提示没有 WebGL，没 GPU 不也能渲染吗。浏览器已不再自动退回软件 WebGL，显卡路径一关（设置、黑名单，或本次会话里显卡进程崩溃过）three.js 就起不来；之前看到的「Error creating WebGL context」是同一个失败。`webglProblem()` 现在收集 `webglcontextcreationerror` 的 `statusMessage` 并显示在提示里，提示改为先建议完全退出浏览器重开、看 chrome://gpu。

### 不能删的物体，点「删除物体」会说明原因（2026-10-29 22:3x）

- **用户问**：删除物体为什么点不了。规则是 10-24 定的（`object_delete_blocker`）：任务在用、有泛化变体、已建模（有生成脚本或已在共享资产库）的物体都不能在平台删，要在仓库和共享库里删；线上 2216 件里只有 188 件（多为 geniesim）能删。原因以前只在悬停提示里，按钮是 `disabled`，点了没反应。现在按钮用 `aria-disabled` 显示为不可用，点击弹出不能删的原因。规则本身没改。

### 场景也有「待重建 → 待审核 → 完备」审核（2026-10-29 22:xx，`chongzheng/platform-scene-review`）

- **用户要求**：场景重建加一个和物体资产一样的审核机制。
- **服务端**：扫描给场景加 `auto.model_version`（孪生场景取 `usd/scene.usda` 的内容哈希，部署机 stash/pop 只改 mtime 不算重建；导入的场景取入口文件的大小 + mtime），`model_signature` 优先用它，所以 `review_state` 物体、场景共用。物体那段审核抽成 `Handler.verdict()`，场景复用：管理员才能审，退回要写原因，可撤回；工况变体（`source_scene` 指向已有场景）跟随源场景、单独审返回 409；场景列表和保存返回带 `review_state`。**场景审核只写平台数据库，不调用 `write_registration`**（它会在没标登记时删掉 `registration.json`），审核请求只能带审核字段。`test_scene_review` 覆盖上面各条和「只改 mtime 不算重建」。
- **前端**：`reviewPanel(d, kind)`、`reviewRecord(kind, id, action)` 物体和场景共用（文案按 `REVIEW_TEXT`）；场景页加「全部 / 待重建 / 待审核 / 完备」分段筛选和卡片状态，详情里有审核区（变体只写跟随源场景），管理员的「场景重建」导航有待审核角标。RoboDojo 自带基准场景不参与。
- **上线后**：已建好的孪生场景第一次都会是「待审核」，要管理员逐个审一遍。
- **验证**：`bash scripts/check.sh` 通过；本机浏览器走通退回（无原因被拦）→ 撤回 → 通过，角标随之减少，仓库文件没被改动。


### 场景工况变体并入源场景；3D 预览的 WebGL 提示（2026-10-29 22:xx）

- **起因**（用户）：场景页多了「茶水间吧台 / 办公室升降桌 · Benchmark桌面工况」两张卡；3D 预览报「Error creating WebGL context」却提示网络。
- **来源**：两张卡来自 10-29 16:06 的 benchmark 同步（从 `/root/robodojo-pro_benchmark` 同步 4 个任务、iPhone 和手机盒，记录在部署机 `real2sim/benchmark_sync/20261029T160621/`，署名「刘小凡授权同步」），是部署机上未提交的 `real2sim/scenes/g1d_benchmark_tabletop_{counter,office}/platform.json`，`source_scene` 指向吧台和办公室。平台把任何带 `platform.json` 的目录都当独立场景。
- **改动**（只有 `index.html`）：`source_scene` 指向另一个已有场景的条目算工况变体（`sceneSource`/`sceneVariants`），不单独出卡、不计入孪生场景数；源场景卡片显示「+N 工况」，详情里列出工况并可点开，工况详情写明所属场景。3D 预览先 `webglWorks()` 检测（临时上下文用完即释放），没有 WebGL 时说明是硬件加速 / GPU 问题、不再提示网络，也不去下载 three.js；加载中抛出的 WebGL 错误同样如此。关闭抽屉时本来就会 `dispose` + `forceContextLoss`，不是泄漏。
- **验证**：本机临时造一个带 `source_scene` 的场景目录（测完删除）核对卡片、+1 工况、双向链接和计数；把 `getContext` 换成返回 null 核对提示。

### 部署机上的平台未提交工作收进 `main`（2026-10-29 17:0x，`chongzheng/platform-inventory-materials`）

- **起因**（用户）：另一个 Codex 会话在 `5090-2` 部署目录里直接改平台（VS Code 远程，16:35 停），一直没提交；用户让我接管并整理进 `main`。
- **收进来的**（平台代码以部署机当时的文件为准，它最新）：数据集本地盘点（`index.html` 的 `applyDatasetInventory`，运行时经 `/file` 读不入库的 `output/dataset_inventory.json`，说明见 `src/real2sim_platform/DATASET_INVENTORY.md`）；材质按表面和场景限定（`scripts/scenes/appearance.py`、新文件 `real2sim/materials/surface_pools.json`、`server.py`）；柔性物体登记与验收契约（新文件 `src/real2sim_platform/deformables.py`、`object_registration.py` 的柔性专用验收段、建模协议 `real2sim/prototypes/softmimicgen/DEFORMABLE_MODELING.md`）；以及本会话加的数值单位换算（`fmtCount` 万/亿、`fmtHours`、`fmtGB` MB–PB，三位有效数字，精确值在悬停提示和抽屉里）。
- **从分支补的**：`codex/surface-material-pools` 的测试改动（`scripts/scenes/test_appearance.py`、`test_server.py`，部署机缺这两份才有 5 项失败）和 `real2sim/scenes/SKILL.md` 说明；`codex/sync-dataset-inventory` 的 `DATASET_INVENTORY.md`、`test_dataset_catalog.cjs`（加了单位换算断言，接进 `check.sh`，没有 node 时跳过）。新增 `test_deformables.py`（校验、失败即关闭的验收）。这两个分支的内容都已在这里，可以删；`z/platform-deformable-objects` 改的是仿真运行时（`env/scene_manager` 的 IsaacLab FEM 柔性物体），**没收**，等作者确认。
- **验证**：`bash scripts/check.sh` 90 passed、1 skipped（退出码 0）；`node test_dataset_catalog.cjs output/dataset_inventory.json` 用线上完整盘点（99 组、281 条）通过；ruff 只改了格式和两行过长的 prompt 文本。


### 办公室场景布局精简（2026-10-29 13:xx–14:3x，`chongzheng/scene-office-layout`，`f3da0c7` 已进 `main`、已部署、已装共享库）

- **用户要求**（看平台 3D 预览的俯视图）：不要窗外背景；东侧那张橙色办公桌（俯视图上方）靠墙；西南角两张桌子只留一张。
- **第二轮**（15:5x，用户："放置物体的桌子要挨着墙边的办公桌"）：任务桌是场景原点，机器人站位和所有布局都以它为准，所以不挪它，改成把整个房间（墙、家具、窗户补光的顶点和位置）在场景里平移 `ROOM_SHIFT` = (−0.84, 0, 0)（`build_scene.py` 的 `Layer(offset=...)`；平台工具都读网格原始顶点，不认父节点平移，所以直接烘焙进顶点）。`LAYOUT` 仍是照片测得的房间坐标，`build_params.json` 新增 `room_shift_unscaled`。任务桌到墙边办公桌 5.5 cm（和照片里两张桌的相对位置一样），任务桌在房间里等于往东挪了 0.84 m；`reference/cameras_scene.json` 的照片相机仍在房间坐标，叠图前要先平移。`check_scene.py` 31/31。 Isaac（`5090-2:/data/robodojo-runs/office-layout2-20261029-1602/`，GPU 2）：RTX 封面重渲，`g1d` seed 0 拾取、叠块 `success_rate` 1.0、`eval_time` 1，第三人称画面里机器人身后柜子和矮木架不碰；Open3D 预览同目录重渲。`0f985c7` 进 `main`，部署机 stash → ff → pop（只有 `HANDOFF.md` 重叠），备份 `.cache/ff-main-20261029-160710/`，`preview/` 旧版备份 `.cache/office-layout2-preview-before-20261029-160710/`，扫描后办公室 31/0；共享库 `Room/real_scene1` 再次整目录替换，上一版备份 `backups/backup-shared-room1-20261029-160726.tgz`（最初的旧版仍是 `-142552.tgz`）。
- **改动**（`real2sim/scenes/real_scene1_office/build_scene.py`）：删掉北窗、东窗外两块视频帧贴图背景板（`Exterior/*`、`exterior` 材质、`window_exterior.png`，也去掉了只给它用的 `no_shadow`），窗洞里现在是 `AmbientDome` 的颜色；`DeskEast` 整体 +0.84 m 贴东墙（x 1.43–2.99，离墙 1 cm，窗台在桌面上方，不冲突），桌上筐、墙角皮椅、女同事椅跟着平移；删掉粗摆的 `DeskWest2`，留下挨着西侧 a 椅的 `DeskWest`。任务桌、桌面物体、机器人站位都没动。依据写在 `LAYOUT` 的 `src` 和 `PLAN.md` 对照表「布局精简」一行；这两处从此和照片对不上。
- **验证**（只在本机）：`build_scene.py` 重建（`usd/objects` 只有面序变化，已还原）；`check_scene.py` 31/31，任务桌到最近背景碰撞体从 5.5 cm 变成 47.6 cm；`scene_parts.py`（`parts.json` 不变）、`make_manifest.py`；`export_glb.py` 生成 `preview/scene.glb`，用平台同款 three.js 渲染俯视、斜视、头部视角和旧版对比，GLB 里已没有 `Exterior`、`DeskWest2`；`bash scripts/check.sh`（uv 临时环境）87 passed、1 skipped。没跑 Isaac RTX，也没跑任务。
- **Isaac 验证**（`5090-2:/data/robodojo-runs/office-layout-20261029-1415/`，GPU 2；`Assets/` 除 `Room/real_scene1` 外都软链接到共享库）：`render_scenes.py` 出新的 RTX 封面；`g1d` seed 0 拾取、叠块 `success_rate` 1.0、`eval_time` 1，头部相机画面正常（看的是桌面，拍不到窗）。量杯倒豆没跑起来：共享库里本来就没有 `g1d_pour_beans_measuring_cups` 的布局（老缺口，和本次无关），脚本 16 s 退出、不写结果。Open3D 预览（5 个自由视角、15 个照片 / 视频帧对比）在同一目录重渲，服务器的 RoboDojo 环境没有 `pxr`，用的是 `PYTHONPATH=/data/robodojo-runs/usdcore-py310`（pip 装的 usd-core）。
- **部署**（用户同意）：部署机 `d60f1ba` → `f3da0c7`（stash → ff → pop，只有 `HANDOFF.md` 重叠，另外 161 个别人的改动逐字节不变），备份 `.cache/ff-main-20261029-141830/`；`preview/` 覆盖同名文件（`scene.glb`、`render/sim.jpg`、`cmp_/ov_/render_/view_*.jpg`，旧版整个备份在 `.cache/office-layout-preview-before-20261029-142520/`）；没重启平台（部署机 `server.py` 有别的会话未提交的改动），临时会话 POST `/api/scan` 后办公室检查 31/0、`scene.glb` 为新版，未登录 `/api/info` 401。
- **共享库**（用户同意）：`/data/robodojo-assets/Assets/Room/real_scene1` 整个换成验证过的那份（`window_exterior.png` 不再残留，和仓库 `usd/` 逐字节一致，root 644/755）。旧版备份 `/data/robodojo-assets/backups/backup-shared-room1-20261029-142552.tgz`，回滚就在 `Assets/Room` 下删掉 `real_scene1` 再解开它。`g1d`、`g1d_mobile`、`g1d_aurora_lite_ws` 从此用新房间，新采的办公室数据背景和已发布的两份 100 条不同。

### 物体指令名称 + 建模 prompt v3（2026-10-29，`chongzheng/objects-instruction-names`）

- **起因**（用户）：每个物体和变体要有多条从简到详的名称，填进训练指令做泛化、提高指令遵循；应在建模 prompt 里就设计好，并给现有完备物体补上。
- **发现**：指令模板的 `<label>` 原来由 `layout_manager.get_label_descriptions` 从 `description.json` 的 `description`（5 句英文长描述）里随机取一句填入，填进去的是整句而不是名称。现有任务只有 `general_pickup` 用了占位符（单个 `<target>`），G1-D 任务都是固定句子。
- **数据格式**：`description.json` 新增 `names: {"en": [...], "zh": [...]}`，每种至少 5 个，顺序是大类泛称 → 类别名 → 加一个显著属性 → 加两三个区分属性 → 在同族里唯一的完整短语。名称必须是名词短语（不带冠词、数量词、位置、比较级、任务角色）；登记开了 `rt_color` 的族（金属杯、陶瓷杯、马克杯整件换色，农夫山泉族只换瓶盖）不写会被换掉的颜色。
- **来源与写入**：名称表是入库的 `scripts/objects/object_names.json`（键是资产 id，Articulation 瓶子写成 Clutter id 的别名），由 `object_names.py apply <物体根目录>` 写进资产，`check` 校验条数、顺序（英文词数和中文字数不减）、措辞，以及最详细的名称全表唯一。没有改生成脚本，因为金属杯、陶瓷杯、垃圾盒、新版塑料杯的脚本还在同事的 `z/objects-*` 分支上没合进 `main`。代价是重新生成后必须再跑一次 `apply`（已写进 SKILL 的第 6 步和「常见坑」）。
- **运行时**（向后兼容）：`load_desc_info` 读 `names`；`get_label_descriptions(..., lang=None)` 有对应语言的名称就返回名称，否则仍返回 `description`；`desc_manager` 按模板里有没有汉字选 `zh`/`en`，同一模板里两个物体共有的名称（如两只量杯共有的 "cup"、"measuring cup"、"plastic measuring cup"）跳过，只剩能区分的几级。官方资产没有 `names`，行为不变。
- **prompt**：`object_registration.py` 升到 `real-object-v3`，新增「指令名称」一节，交付清单加上名称表条目和 `description.json` 的 `names`。旧版确认的记录需要管理员重新点确认才会拿到新 prompt（和 v2 一样）。同步改了 `ASSET_WORKFLOW.md`、`scripts/objects/SKILL.md`、`scripts/datagen/SKILL.md`，平台任务抽屉的实现文本也要求候选物体写成 `<角色标签>`。
- **覆盖范围**：平台 `review_state == done` 的 54 件，即金属杯 7、农夫山泉和东方树叶两族瓶子共 13、陶瓷杯 7、量杯 6、桌面垃圾盒 7、塑料杯 `00001–00007`、马克杯 6、红豆 1，另加 13 个 Articulation 瓶子。名称依据各资产 `description.json` 的 caption 和描述（共享库里有的取共享库，否则取最新的隔离运行目录），并与同事分支生成脚本里的变体参数核对过。
- **验证**：本机 ruff 0.11.7、`git diff --check`，`test_object_names.py` 5 项（名称表合规、校验器能拦住错误、`apply` 只加 `names`、别名、真实 `DescManager` 在中英文模板下两只量杯都只用能区分的名称），`test_server.py` 13 项。`5090-2` 隔离目录 `/data/robodojo-runs/object-names-20261029-010544/`：把 67 件真实的 `description.json` 拷进 `obj/`（原件留在 `obj.before/`），在 `RoboDojo` 环境里跑 `apply` 和 `load_desc_info` 回读，67/67 写入，除 `names` 外字段不变，回读一致。没跑 Isaac 任务（没有任务用到这些物体的占位符）。
- **共享库**（09:10，用户同意）：对 `5090-2:/data/robodojo-assets/Assets/Object/Real2Sim` 跑 `apply`，写了已装的 38 件（瓶子 Clutter、Articulation 各 13 个，量杯 `00002–00005`，塑料杯 `00001–00007`，红豆）。逐个和备份对比：除 `names` 外不变，属主（uid 1008）和权限 644 不变。备份 `/data/robodojo-assets/backups/description-before-names-20261029-091001.tar.gz`，回滚就在 `Object/Real2Sim` 下解开它。
- **剩余**：
  - 金属杯、陶瓷杯、垃圾盒、马克杯、量杯 `00000/00001` 不在共享库，装库时要跑 `apply`。
  - 平台读的元数据镜像 `/root/RoboDojo_assets` 没动，平台也不显示名称；物体页的建模阶段不检查 `names`。
  - Notion 记录待补录：授权后工具没加载出来，草稿在 worktree 的 `.cache/notion_draft_object_names.md`（不入库）。

### 数采平台加载提速 + 筛选栏统一（2026-10-29 00:3x，`chongzheng/platform-banzhi-flywheel`）

- **起因**（用户）：平台加载太慢；物体页筛选栏（原生下拉 + 复选框）不协调。
- **测量**：部署机本机每个接口 ≤ 20 ms，慢在公网隧道，实测只有 30–90 KB/s；登录后一次加载 1.85 MB 且都没压缩（回合 634 KB、物体 373 KB、页面 368 KB、材质 270 KB），卡片图是每张约 200 KB 的 PNG 或照片原图。
- **服务端**（`server.py`）：JSON 和 `.html/.json/.md` 文件在客户端支持时 gzip（1.85 MB → 约 230 KB；页面 368 → 114 KB）；GET 的 JSON 带弱 ETag、`Cache-Control: private, no-cache`，数据没变时返回 304；`/file?path=…&w=320|640|1280` 返回 ffmpeg 生成的 WebP 缩略图（缓存在 `.cache/thumbs/`，按路径 + mtime + 大小命名，失败或没有 ffmpeg 时回退原图；场景照片 157 → 25 KB，`image/webp` 类型要手动登记，Python 3.10 的表里没有）。只用标准库加系统 ffmpeg，部署机已有。
- **前端**：卡片、缩略图、抽屉大图改用 `small(p, w)`；灯箱和「原图」链接仍是原文件。下拉组件改成选项按序号回调（`DD`/`ddPick`），物体页负责人、任务页能力维度、评测页状态、质检页 5 个筛选都换成胶囊下拉；「只看有照片参考」改成胶囊开关 `.toggle`，统计文字靠右。
- **验证**：`bash scripts/check.sh` 82 passed、1 skipped（`test_http` 新增 gzip、304、缩略图回退断言）；本机浏览器 10 个页面无报错，场景卡片加载 640 px WebP。
- **下拉修正**（00:5x，用户反馈）：菜单改为和按钮左对齐向右展开，右侧放不下时（`placeMenus()`）才向左，最大宽 420 px、长选项省略号；质检页倍速也换成胶囊下拉并记住倍速（`UI.qaRate`，换回合后沿用）。原先倍速按钮在 `#qa-videos` 里，而这个容器为保住播放进度会在重渲染时整块换回旧节点，导致菜单打不开；现在 `#qa-videos` 只包视频网格。本机用 `make_run` 造的假回合验证：倍速菜单能开、两路视频都是 2×、视频节点和进度保留、任务菜单左缘与按钮对齐。
- **成员管理上移**（10-29 13:1x，用户提出）：账号在仿真平台和数据集通用，所以「成员管理」从仿真平台二级导航移到产品栏用户区（管理员才有的成员图标，`#users`），打开时和数据集页一样隐藏二级导航、两个产品标签都不高亮。
- **事故，已修复**（10-29 12:1x）：`d14a164` 用 `git commit -a` 把工作区里被编辑器旧缓存写回的 `server.py`（停在访客改动做到一半时：有 `guest` 列，没有入口拦截和 `role` 处理）一起提交了，`check.sh` 失败却因 `| tail -1` 没拦住推送；部署机快进到了它，但运行中的进程是 11:42 用完整代码起的，线上访客限制没失效。已从 `6104232` 恢复 `server.py`，部署机文件同步回来。`bcf8cc1` 里 `.ds-side` 的 `thin` 也是同一原因。之后只用显式路径提交，检查用真实退出码判断。
- **数据集页示意说明**（10-29 12:0x，用户要求）：标题旁加琥珀色标签「仅作示意，未支撑实际业务」，页底说明同步。顺带修回 `.ds-side` 的 `scrollbar-width: thin`：`bcf8cc1` 把这一行意外改回了旧版本（同一提交里其余改动都核对过是有意的），筛选栏滚动条又露了出来。
- **访客角色**（10-29 11:3x，用户要求「只能查看，不能进行任何编辑」）：`users` 表加 `guest` 列（旧库启动时自动补，原账号都不是访客；管理员不会同时是访客）。服务端在 `parse_request` 统一拒绝访客的非 GET/HEAD 请求（403，只放行 `/api/logout`、`/api/password`）；`/api/users` 接受 `role`（admin / member / guest），旧的 `admin` 字段仍可用，管理员不能把自己改成别的角色。前端：成员管理表格三种角色、新建账号用分段选择；访客顶栏显示「访客 · 只读」，`guestLock()` 在每次渲染后隐藏新建、同步、审核、上传、删除、批量、质检结论等按钮和勾选框，把 `data-f` 字段设为只读/禁用（只读字段灰底），`api()` 对访客的写请求先拦截提示。`test_server.py` 新增访客断言（读 200，PUT/POST/DELETE 403 且断开连接，可改自己的密码，角色互转）；浏览器里以访客登录核对了物体页、场景抽屉、质检页，并用管理员在成员管理建访客、改角色。
- **访客上线**（11:42，`6104232`）：部署机 stash → ff → pop 无冲突后重启了平台进程（用户确认）。**部署机上另一会话未提交的材质白名单改动（`server.py`、`scripts/scenes/appearance.py`、`index.html`，按场景限定可选材质）随这次重启一起生效**，它让 `test_server.py` 的 `test_randomization_profile`、`test_appearance_settings` 两项失败（测试里的材质被判为不适合该表面），等那位同事提交时一起修。线上核对：临时访客读 200、写 403，已删除；物体 1319（重启前备份里已是 1317，是别人更新了资产镜像，不是这次部署）。
- **批量审核**（10-29 11:1x，用户要求）：物体页批量栏对管理员加「批量通过」「批量退回…」（退回先填一条原因，写进每件的审核记录），只作用于选中件里待审核的本体（`bulkPending()`），变体和其他状态跳过并在确认框里说明；逐件走现有 `PUT review_action`，服务端权限和校验不变。负责人改用胶囊下拉，菜单在屏幕底部时向上展开（`placeMenus` 的 `.up`）。本机用桩掉的 `save` 走过流程：无原因时不发请求，有原因时 3/4 件提交、1 件跳过。
- **滚动顿挫**（同上）：去掉卡片角标 `.pill` 和吸顶二级导航的 `backdrop-filter`（每帧重算模糊，物体页有几百个），抽屉遮罩只在打开时模糊；`.mcard`/`.ds-card` 加 `content-visibility: auto`。页面滚轮改为阻尼滑动（每帧走剩余距离的 14%，至少 1 px，1.5 px 内直接到位）；内层还能滚的面板、横向/缩放手势、打开抽屉或灯箱、`prefers-reduced-motion` 时用原生滚动，键盘/滚动条/`scrollTo` 改动时缓动让位。模拟 3 格滚轮：0 → 300 px 约 32 帧，先快后慢。
- **隐藏内部滚动条**（10-29 11:0x，用户要求）：数据集筛选栏、缩略图条、物体选择列表、杂物类别、抽屉内容、质检列表用 `scrollbar-width: none` + `::-webkit-scrollbar` 隐藏滚动条，仍可滚动；宽表格和代码块保留横向滚动条。
- **分段筛选滑动**（10-29 10:5x，用户要求）：`.segctl` 选中项改成一个滑块 `.seg-thumb`，`render()` 前 `segSnapshot()` 记下每组滑块位置（按 aria-label + 序号），渲染后 `segSlide()` 先放回原位再过渡到新选中项（0.28 s，`prefers-reduced-motion` 下不动画；窗口缩放时重新定位；折叠区里看不到的组让按钮自己上色）。`.slid` 要在第一次测量前加上，否则新选中的按钮会先画黑底再淡出。用 `getAnimations()` 核对过：滑块有 transform/width 过渡，按钮没有背景动画。
- **样式冲突修正**（10-29 10:4x，用户反馈）：新加的胶囊开关原先叫 `.toggle`，和表单里已有的 `label.toggle > input + span` 撞名，评测申请的图像输入 / 泛化等级、登记向导的允许变化属性 / 指令语言 / 杂物类别等 7 处多出一圈外框；改名为 `.pill-toggle`。
- **部署**（00:39，`4f504c2`，00:4x 补 webp 类型）：部署机 stash → ff → pop，别人的改动（`index.html` 评测默认筛选一行、`HANDOFF.md` 追加）保留，只重启平台进程。公网实测：页面 4.4 → 1.1 s，回合 13.3 → 0.3 s，物体 4.2 → 0.6 s，材质 8.8 → 0.5 s，渲染卡片图 136 KB / 1.9 s → 3.6 KB / 0.6 s，未变数据 304；资产数量不变（场景 7、物体 315、任务 65、本体 5、评测 17、研究资产 9、回合 507）。

### BanzhiFlywheel 产品顶栏 + 公开数据集页（2026-10-28 至 10-29，`chongzheng/platform-banzhi-flywheel`，已进 `main` 并部署）

- **起因**（用户）：整个平台叫 BanzhiFlywheel（纯文字 logo），现有平台作为「仿真平台」，并列「数据集」板块；先出设计稿，不替换现在的平台。后续要求：不要大块黑色、顶栏扁平、背景更白、日间 / 夜间 / 跟随系统切换、下拉框学 RoboFlywheel。**数据集板块收录的是外部公开发布的数据集（人工录入），不是我们自产的**；只做商业零售、家居服务两类室内场景。
- **改动**（只动 `src/real2sim_platform/index.html`，服务端和接口不变）：
  - 产品栏 `.gbar`（56 px，白底细线）：字标 `.wordmark`（23 px，粗体 Banzhi + 细体 Flywheel + 蓝点）、「仿真平台 / 数据集」下划线标签、主题切换、一行的用户区。原横向导航降为仿真平台的二级导航 `.side`，数据集页隐藏（`.app.no-sub`）；`--top` = 两栏高度之和。
  - 主题：所有颜色变量改为 `light-dark(日, 夜)`，`<html data-theme>` 取 light / dark / system（`color-scheme` 切换），存 `localStorage['bzf-theme']`，`<head>` 里的一行脚本在首帧前设好。写死的浅色（callout、表头、胶囊、玻璃底、按钮文字等）都换成了变量，新增 `--on-primary`、`--glass`；3D 预览背景读 `.m3d` 的计算颜色。日间整体调白（surface-2 #F8F8F9、边框更浅）。
  - 通用下拉 `dropdown(id, label, opts, cur, pick)`：胶囊按钮 + 浮层菜单，点外面或 Esc 关闭（`UI.dd`）。
  - 数据集页 `#datasets`：页头 4 个统计；左侧分面筛选 `DS_FACETS`（数据类型 真机 / 仿真 / Ego、Episode 对数双滑块、应用领域、室内空间、机器人形态、操作类型、本体、技能、格式、许可、发布平台、本地副本），组内或、组间且，计数动态；右侧顶栏：数量、重置、说明、搜索、排序下拉（推荐 / 规模 / 体量 / 名称）、卡片 / 列表视图切换。卡片仿 RoboFlywheel（图标 + 类型角标、名称、机构、分类行、简介、Episode / 时长、许可和体量）；抽屉有分类、来源链接和本地副本。重新渲染时保留筛选栏滚动位置，选项有固定 id 以保住焦点。
  - **数据**：常量 `DATASETS` 30 条，从用户的 Notion「OSS /mnt/data 数据清单」（公开页面，浏览器读取）挑出与室内服务相关的公开数据集，排除自采 G1-D、转换中间产物和团队上传的 failure 数据；宇树按 G1 WBT 全身遥操作（59 个任务）和 G1 Dex1 桌面（94 个任务）两条汇总。Episode / 时长 / 许可只填了核对过的：AgiBot World Beta 和宇树取自 HF 卡片，RoboMIND / RoboMIND 2.0 / DROID / RoboCOIN / InternData-M1 取自 RoboFlywheel 页面，其余来自 Notion 备注或留空。体量和本地副本来自 Notion。**仓库是公开的，所以没写 Notion 里的本地路径、服务器名和运维备注。** 领域 / 空间 / 技能等分类是初稿。
- **验证**：本机临时库（`/tmp/bzf/`，账号 `preview`）起 `server.py --port 8799`，浏览器 1440 px 下看了数据集页（日间、夜间、下拉展开、筛选后侧栏滚动保持）、仿真平台场景页夜间；早先在 390 px 看过顶栏。`git diff --check` 通过。没跑 `check.sh` 和 `test_server.py`（没动服务端）。Notion 连接器授权后没加载出工具，改用公开页面读取。
- **合并与部署**（10-29 00:0x，用户要求合并到 `5090-2` 的部署并推 `main`，不影响仿真平台现有资产）：五个提交 rebase 到 `origin/main@806f4e6`，冲突只有 `index.html` 的大预览样式（保留 #36 的居中裁剪、用新圆角）和 `HANDOFF.md`（两边的节都留）；main 上的变体跟随本体、场景中文名等函数都在。`bash scripts/check.sh`（uv 临时环境）82 passed、1 skipped；页面脚本 `node --check`；本机浏览器 10 个页面无 JS 报错。部署机 `806f4e6` → `65abd16`（stash → ff → pop，无冲突）：另一个会话 00:01 在部署机改的 `index.html` 一行（评测页默认筛选「全部」）和 `HANDOFF.md` 追加都保留，其余 30 个别人的改动逐字节不变；只重启了平台进程（00:05:13 起），tmux 会话和隧道没动；数据库、资产库镜像、预览图都没碰（数据库在线备份在部署机 `.cache/ff-main-20261029-000442/`，回滚方法见本机 `.agent/ops.local.md`）。部署验证：`test_server.py` 13 passed；本机和公网首页 200、未登录 `/api/info` 401；管理员临时会话：场景 7、物体 315、任务 65、本体 5、评测 17、研究资产 9、回合 507，`assets_ok`。部署机停在 `65abd16`，没为本文件再快进。
- **剩余**：(1) 数据集目录要转正：改成服务端读的数据文件（如 `real2sim/datasets.json`）或接 Notion，并补齐空缺的 Episode / 时长 / 许可、确认分类；(2) 手机宽度下筛选栏堆在列表上方，可改成抽屉；(3) 登录页没有主题切换（跟随上次的选择）。

### 数采平台界面改为 RoboFlywheel 风格（2026-10-28，`chongzheng/platform-ui-refresh`，随上一节一起进 `main` 并部署）

- **起因**（用户）：喜欢 [roboflywheel.com/datasets](https://roboflywheel.com/datasets) 的风格，平台以后也要做类似的数据集页；用 UI UX Pro Max 技能优化现在的数采平台。
- **改动**（只动 `src/real2sim_platform/index.html`，功能和接口不变）：
  - 配色换成黑白极简：主色近黑 `#111113`，中性灰表面，链接和「已登记」这类状态标签用蓝色（`--link`、`.tag.info`）；通过/不通过/待办的语义色不变。圆角 16 px，按钮、搜索框、分段筛选改成胶囊形，分段筛选选中项黑底白字。
  - 左侧边栏改成顶部横向导航（品牌 · 文字导航带计数 · 数据源状态点 · 黑色「同步项目数据」· 用户）。顶栏内边距和 1440 px 内容区对齐；≤ 1240 px 时导航换到第二行横向滚动。数据源信息收进状态点的悬停提示（读屏可读）。QA 面板的 sticky 偏移改为让出顶栏高度 `--top`。
  - 卡片：图片内嵌在卡片里带圆角，角标改成白色半透明胶囊；场景卡片和本体卡片加了「标签在上、数字在下」的灰底统计块（新 helper `stats()`）：场景是照片 / 预览 / env 配置数，本体是站位配置 / env 配置数。KPI 数字改为无衬线粗体。
  - 没有引入外部字体（Google Fonts 在服务器所在网络不稳定），沿用系统字体栈。
- **验证**：本机用临时库起 `server.py`（没有资产库，物体没有渲染图），浏览器在 1440 / 1024 / 390 px 下逐页切换 9 个页面无 JS 报错、无横向溢出，打开过场景详情抽屉；`test_server.py` 13 项通过（本机默认 Python 没有 pytest，用的是另一个 conda 环境）；`git diff --check` 通过。没跑整套 `check.sh`，没在 `5090-2` 上看真实数据。
- **剩余**：要上线需推送、开 PR、合并后按 `.agent/ops.local.md` 重启 `5090-2` 上的平台。手机宽度下同步按钮会换到第二行。下一步可以按参考站做「数据集」页：读 `scripts/datagen/registry.json`，卡片显示回合数、时长、格式、许可证（需要服务端新增接口）。

### 数采平台：变体不审核、跟随本体，物体统计只数本体（2026-10-28 22:xx，`chongzheng/platform-variants-no-review`）

- **用户要求**：变体不需要审核，跟随本体；物体资产的统计只统计本体。取代 #43 的「本体 N + 变体 M」写法。
- **服务端**：`public_object(doc, base)` 的 `review_state` 对变体取本体的（`variant_base()` 按 `auto.variant_of` 找本体；本体不存在时仍按自己算）；列表和保存后的返回都这样。对变体提交 `review_action` 返回 409「变体跟随本体 … 审核」。
- **前端**：`isBase`、`stateOf`（变体取本体的状态，审核完本体后页面上变体立即跟着变）。只数本体：侧栏「物体资产」、总览 KPI 和流程卡、分区标题计数和状态标签、状态分段按钮、工具栏「N 件」及照片/渲染数、待审核角标、资产库抽屉的物体数。按状态筛选只出状态相符的本体卡片。变体详情的审核区只写「泛化变体不单独审核，跟随本体 …」，没有表单。列表视图仍一件一行（变体行显示本体的状态）。
- **实时数据**：侧栏物体 118（记录 315）；Real2Sim 本体 63 = 待重建 6 + 待审核 48 + 完备 9，各筛选下没有状态不符或变体单独出的卡；待审核角标 48；量杯 00000、00002–00005 跟随 00001 显示完备。
- **验证**：`bash scripts/check.sh`（uv 临时环境）82 passed、1 skipped，`test_object_review_and_libraries` 新增变体断言（对变体审核 409、列表里变体取本体状态）；页面脚本 `node --check`。没开浏览器看。

### 量杯变体 00002–00005 装共享库；平台变体始终跟着本体卡片（2026-10-28 21:1x，`chongzheng/platform-variants-follow-base`）

- **装库**（用户要求）：把隔离目录 `measuring-cup-variants-20261028-150013/ovl/Rigid/measuring_cup/0000{2..5}`（生成脚本与 `main` 逐字节一致、`check_object.py` 已通过）原样拷进 `5090-2:/data/robodojo-assets/Assets/Object/Real2Sim/Rigid/measuring_cup/`，`diff -r` 一致，属主 1008。目标目录原本不存在，没有覆盖；装库前清单 `backups/listing-before-measuring-cup-variants-20261028-211322.txt`。按 10-28 搬库之后的做法只放 Real2Sim，不在 RoboDojo 留软链接（`object_dir` 先找 Real2Sim）。本体 00000/00001 仍未装库。装后对共享库副本跑 `check_object.py`（GPU 2）：4 只 OK，倾角 0°，高度等于半高。平台镜像 `/root/RoboDojo_assets` 手动放了这 4 份 `metadata.json`/`description.json`（整体重刷镜像会删掉）。回滚：删掉这 4 个目录。
- **界面**（用户纠正 #43）：#43 让本体不在筛选里的变体单独出卡，用户要变体始终跟着本体。现在 `cardDocs()` 取每件的本体去重：只要本体或任一变体落在当前筛选里就出本体卡，按审核状态筛选时卡片上写「N 个变体待审核」等。实时数据：待审核 57 张卡（48 个待审核本体 + 9 个本身完备/待重建但有变体待审核的本体，如量杯 00001「4 个变体待审核」），没有变体单独出卡，244/244 可见；完备 9 张、待重建 6 张。列表视图不变（变体一行一件）。
- **验证**：`bash scripts/check.sh`（uv 临时环境）82 passed、1 skipped；页面脚本 `node --check`；上面的卡片计数用页面里的函数跑实时数据得到。没开浏览器看。

### 合并开着的 PR（#36、#38–#43）并部署数采平台（2026-10-28 晚，`chongzheng/merge-open-prs-1028b`）

- **用户要求**：合并处理所有 PR，然后部署。
- **合并顺序**（合进 `chongzheng/merge-open-prs-1028b`，再快进推 `main`）：#36 大预览居中裁剪；#38 量杯 00000 作 00001 的变体；#39 三个冷水壶合成变体；#40 场景可编辑中文名（带了一份和 #36 相同的裁剪改动，自动合并后无重复）；#41 建模 prompt v2（照片贴图标签）；#42 量杯变体 00002–00005（包含 #38）；#43 审核计数注明含变体。
- **冲突**：只有 `HANDOFF.md`（开头的会话行和「最近任务」第一节），各 PR 的节都保留，会话行改写成上面这行。
- **验证**：合并结果 `bash scripts/check.sh` 通过（uv 临时环境：ruff 0.11.7，82 passed、1 skipped），页面内联脚本 `node --check` 通过。
- **部署**（21:04）：部署机 `/root/RoboDojo` 从 `65d7d5b` 快进到 `a67584f`（stash → ff → stash pop）。部署前 57 个未提交改动的文件里：21 个和新 `main` 一致（#35/#37/#36/#40 等部署时拷过去的），快进后变干净；`make_pitcher.py`、`make_plastic_cups.py`、`object_registration.py` 正好是上一版 `main`（`224b43f`），取新版本；`index.html`、`test_server.py` 是新 `main` 的子集，也变干净；其余 31 个是别的会话（杨开元等）真正的未提交改动，全部保留。唯一手工处理的是 `real2sim/ASSETS.md`：共享库路径那行取新的 `Object/Real2Sim`，保留部署机追加的 10-23–10-27 各节。119 个未跟踪文件没动。备份（`tracked.patch`、状态、改动文件原样、数据库在线备份）和回滚方法在本机 `.agent/ops.local.md`。
- **部署验证**：RoboDojo 环境 `test_server.py` 13 passed；重启后本机和公网未登录 `/api/info` 401、公网首页 200、页面带新代码；管理员临时会话：物体 315 件（完备 10、待重建 6），场景 7 个，`assets_ok`，冷水壶 00001/00002 挂在 00000 下。没在浏览器里看。
- **没做**：共享资产库没动（量杯变体 00002–00005 仍未装库）；#41 的 prompt v2 会让 v1 确认过的登记显示为「未确认」，要管理员重新确认。

### 量杯 4 个泛化变体 `measuring_cup/00002–00005`（2026-10-28 15:2x，`chongzheng/objects-measuring-cup-variants`）

- 用户要求：量杯在平台上只有一个变体（`00000`），多加几个。
- `make_plastic_cups.py` 新增 `MEASURING_VARIANTS`，`variant_of` 都指向本体 `00001`：透明 170 mm / 500 ml 刻度、烟灰半透明 200 mm / 600 ml、不透明橙色 120 mm / 350 ml、浅蓝半透明 155 mm / 450 ml。截面不变（`measuring_cup_shape.CUP`），只改高度、颜色和透明度，所以 `measuring_cup_shape.in_cavity(p, H)` 对它们同样精确；`shape.MEASURING` 没加项（`collect_office_beans` 按两项解包）。刻度取满口容积约 80%（两只实测量杯都是这个比例），质量按两只实测量杯随高度线性插值（27–80 g），全部是估计值。`clear_pp` 加了 `milky` 参数，`measuring_description` 改为按词表生成；`00000`/`00001` 生成的 `metadata.json`、`description.json` 和上午那版逐字节一致。
- 检查：`5090-2:/data/robodojo-runs/measuring-cup-variants-20261028-150013`（GPU 0），`check_object.py` 6 只量杯全部 OK（倾角 0°，z 等于半高）。渲染看过后调了一次材质（烟灰太不透、橙色偏黄）再重跑。本机 ruff 0.11.7 通过。没跑 `check_cup_beans.py`（截面和杯底与实测量杯相同）。Kit 要设 `OMNI_KIT_ACCEPT_EULA=YES`，否则后台运行卡在 EULA 提示上。
- 平台（部署机，没改平台代码）：只把 `00002–00005` 的 `preview/{sim.png,check.json}` 放进 `real2sim/objects/Rigid/measuring_cup/`，`00000`/`00001` 的预览没动（换 `check.json` 会让它们的审核退回待审核）。数据库在线备份 `.cache/mcup-variants-20261028-151854/`，用种政临时会话扫描。结果：4 件都挂在 `00001` 卡片下，有渲染图，`review_state` 为待审核（要管理员审核）；没填负责人、名称等登记字段。
- 没做：没装共享库，也没进任何任务布局（任务配置写死 `index: [0]`/`[1]`）。

### 量杯合成本体 + 变体并标为完备（2026-10-28 11:3x，`chongzheng/objects-measuring-cup-variant`）

- 用户要求：两只 `measuring_cup` 已按实测尺寸建好，高的作本体、矮的作变体，改成完备，负责人种政。
- 本体 `Rigid/measuring_cup/00001`（500 ml，高 170 mm，60 g），变体 `00000`（400 ml，高 140 mm，40 g）。`make_plastic_cups.py` 给 00000 的 metadata 写 `variant_of: Rigid/measuring_cup/00001`；几何、质量没改，已有模型没有重新生成（共享库仍未装量杯，`ASSETS.md` 不变）。
- 检查：`5090-2:/data/robodojo-runs/measuring-cup-check-20261028`（GPU 4），模型取自 `office-beans-random-20261019` 的 10-19 版本（数据集用的同一份；00000 的副本补了 `variant_of`），`check_object.py` 两只 OK：z=0.070/0.085 m（等于期望）、倾角 0°。在同一目录用新生成器重跑到 `gen/`：00000 带 `variant_of`，质量、尺寸和 10-19 版一致（旧版多一个不生效的 `friction` 键）。本机 `check.sh` 的 ruff 通过；pytest 在隔离目录因没拷 LFS 网格大量失败，与本改动无关，没在完整仓库补跑。
- 平台（部署机，没改平台代码）：`check.json` 和新 `sim.png` 放进 `real2sim/objects/Rigid/measuring_cup/0000{0,1}/preview/`，旧文件和数据库在线备份在 `.cache/mcup-20261028/`。用种政临时会话经 API：扫描；两只都写负责人 `chongzheng`、名称、材质颜色、实测尺寸和质量（尺寸、质量依据都选「实测」）；审核通过。结果两只 `built`、`review_state=done`，00000 的 `variant_of` 指向 00001。没有点「确认建模 prompt」。

### 数采平台：三个 water pitcher 合成一张「冷水壶」卡片（2026-10-28，`chongzheng/platform-pitcher-variants`）

- `Rigid/pitcher/00000–00002` 是 `make_pitcher.py` 同一把壶的烟灰/琥珀/透明三色，之前在物体页是三张都叫 "water pitcher" 的卡片。现在 00001、00002 的 `metadata.json` 写 `"variant_of": "Rigid/pitcher/00000"`，收进 00000 的卡片。
- 改动：`make_pitcher.py` 给非 0 号写 `variant_of`；5090-2 共享库 `/data/robodojo-assets/Assets` 和平台镜像 `/root/RoboDojo_assets/Assets` 里这两份 `metadata.json` 直接补了这个键（仿真运行时不读它）。备份在 `5090-2:/data/robodojo-runs/pitcher-variants-20261028-113722/`（两边原文件 + 平台数据库）。
- 平台录入的中文名（数据库 `name` 字段，记在「种政」名下）：00000「冷水壶」，00001「冷水壶（琥珀色）」，00002「冷水壶（透明）」。另一组 `water_pitcher/00000–00005`（已被驳回、待补实物照片重建）没动。
- 验证：平台重扫后三条的 `auto.variant_of` 符合预期；部署机页面的卡片视图会过滤掉变体。没重启服务，也没在浏览器里看。

### 物体建模 prompt v2：实物标签必须用照片贴图（2026-10-28，`chongzheng/platform-prompt-labels`，未部署）

- **起因**（用户）：平台按 prompt 建的东方树叶、农夫山泉"很假"，而且没有贴纸。原因：`real-object-v1` 的外观只有一句"标签与参考核对"，没说标签怎么做，于是农夫山泉 00000 用了程序画的通用文字和插图，东方树叶第一版标签包满整瓶、瓶型也套了圆瓶模板（`z/objects-plastic-bottle-00006` 下午已改成从照片裁剪，那个分支还没合并）。
- **改动**：`object_registration.py` 模板升到 `real-object-v2`，新增「标签、贴纸与印刷图案」：标签、logo、文字必须从参考照片裁剪并做透视校正或柱面展开、去掉明暗后作纹理，不得省略或用程序绘制的代替；只贴在实物有标签的位置，每块单独网格，≥ 10 px/mm，不透明材质，透明主体透出的背面用纸背色；照片缺的面要求补拍、不编内容；变体可用虚构品牌但位置沿用本体；交付同视角照片对照图，标签不符不算完成。几何一句加了"截面、肩颈比例、盖子逐张对照照片，不套现成模板"。`scripts/objects/SKILL.md` 第 4 步加同样的标签规则（指向 `bottle_label.py` 的 `photo_panel`/`photo_band`/`even_light`），第 8 步改成按每张照片的视角渲染并拼对照图；`ASSET_WORKFLOW.md` 同步版本说明。
- **兼容性**：版本号进了 `revision`，所以 v1 确认过的登记在平台上都会变回"未确认"，管理员重新点确认才会生成新 prompt 和 `modeling_prompt.md`。这是有意的：旧 prompt 正是出问题的那份。
- **验证**：本机 `test_server.py` 13 passed（新增断言：prompt 含标签规则、v1 的确认失效），ruff 0.11.7 check/format、`git diff --check` 通过。本机缺 ruff/pytest 环境，用 `uvx`/`uv run` 临时跑；没跑完整 `scripts/check.sh`（其余单测需要 h5py、opencv 等）。没部署到 `5090-2`。

### 数采平台：Real2Sim 审核计数注明含变体，筛选时变体不再漏显示（2026-10-28，`chongzheng/platform-variant-counts`）

- **用户问题**：Real2Sim 分区显示「全部 260 · 待重建 6 · 待审核 244 · 完备 10」，点开待审核只有 48 张卡、完备 9 张。部署机实时数据核对：计数本身对（6 + 244 + 10 = 260），差额都是泛化变体——计数含变体，卡片视图只给本体出卡、变体挂在本体卡上（待审核 196 个变体、完备 1 个，即 `measuring_cup/00000`，它的 `variant_of` 是 `00001`）。真 bug：按状态筛选时，本体不在筛选结果里的变体完全看不到（待审核里 49 个，本体是完备的 metal_cup、plastic_bottle 00000/00006、ceramic_cup、plastic_box、plastic_cup/00001、plastic_mug、measuring_cup/00001，和待重建的 water_pitcher）。
- **改动**（只有 `index.html`）：分段筛选按钮显示「待审核 244（本体 48 + 变体 196）」（`tally()`）；卡片视图改用 `cardDocs()`：变体只在本体也在同一列表里时才挂到本体卡上，否则单独出卡并标「变体，本体 …」；「全选本组」用同一规则。分区标题上的状态标签仍只显示总数。
- **验证**：`node --check` 页面脚本；用部署机实时物体数据（管理员临时会话拉取，用完 `end_session`）跑页面里的 `libOf`/`cardDocs`/`tally`：待审核 97 张卡（48 本体 + 49 单独的变体），加上本体卡上的变体共 244/244 可见，完备 10/10、待重建 6/6；`test_server.py` 13 passed（uvx pytest），`git diff --check`、任务清单检查通过。本机没 pytest 等依赖，`check.sh` 其余单测没跑（没改 Python）；没开浏览器看。
- **未部署**：部署机 `/root/RoboDojo` 的 `index.html` 有别的会话未提交的改动，没动。

### 自建孪生从 `Object/RoboDojo` 搬到独立的 `Object/Real2Sim` 物体库（2026-10-28，`chongzheng/platform-variant-twins`）

- **起因**（用户）：建模时把 Real2Sim 实物孪生错存进了官方 RoboDojo 物体库，应该单独放 Real2Sim 库。范围（用户选）：全部自建孪生。
- **代码**：
  - `env/global_configs.py` 新增 `OBJECT_LIBRARIES = ("Real2Sim", "RoboDojo")` 和 `object_dir(rel, assets)`：按 `<类型>/<类别>/<编号>` 先在 Real2Sim 库找，再找 RoboDojo。布局和任务配置只写类别和编号，不写库名，所以不用改；编号在两个库之间不重复。
  - `layout_manager.py` 加载物体时用它；`build_asset.py`、`rsi_profile.py`、`rsi_assets.library`（两个库都扫）、`x5_layouts.py`、`run_batch.asset_hashes`、`instance_fit.py`、`prepare_bean_pour_layouts.py`（干扰物从两个库的 Clutter 里选）、`manitwin.py` 的尺寸查重也都改成了两个库。
  - `scripts/objects/make_*.py` 默认写到 `Assets/Object/Real2Sim/...`。不要对 Real2Sim 库跑官方库的 `Rigid/format.py`，它会重排编号。ManiTwin 和 GSO 导入的仍然进 RoboDojo 库（外部数据集，不是孪生）。
  - 平台：扫描时按 `object_dir` 定位，`auto.library` 记录物体在哪个磁盘库（`Real2Sim` / `RoboDojo`）。和 #32 的页面分组（字段 `library`：`real2sim` / `robodojo` / 公开库）合并后，`ownObj` 也认 Real2Sim 磁盘库，所以搬过去的物体默认进「Real2Sim 实物孪生」分区；落地检查命令按物体所在的库生成路径。
- **共享库**（用户同意，11:01）：`5090-2:/data/robodojo-assets/Assets/Object/Real2Sim/` 下共 195 件，126 MB。搬的依据：
  - `metadata.json` 有 `source_asset`（真实场景资产包 scene1/4/5/6/7、real_scene1、pour_scene_v2、scene3_pour_pack），或有 `variant_of`、`measurements_mm`；
  - `ASSETS.md` 里生成脚本的产出（pitcher、water_cup、plastic_cup、red_bean、airpods、plastic_bottle 等）；
  - 平台登记了 `spec.json` 的（cable、microwave、sorting_bin、capybara_plush、soft_block、cloth_patch、long_zip_bag、plastic_mailer、air_freshener、aurora_flange、hammer/00009、mango_prop、wooden_arch_block）。
  - 每件原路径留了相对软链接 `../../../Real2Sim/<id>`，旧代码和老运行目录照常能加载；属主改成和库一致的 1008。搬前目录清单、清单和脚本在 `backups/*-20261028-110056*`，脚本加参数 `undo` 可以原样搬回。平台元数据镜像 `/root/RoboDojo_assets/Assets` 也做了同样的搬迁。
- **验证**：
  - 本机：`check.sh` 里的单测 78 项通过、1 项跳过，包括 `test_server.py` 新增的 Real2Sim 库用例；ruff 0.11.7 通过。
  - `5090-2` 搬前对照：隔离目录 `real2sim-lib-20261028-104917`（本分支，pitcher、water_cup、plastic_cup 只放在 Real2Sim 下）对比 `real2sim-lib-ctrl-…`（`origin/main`），`g1d` seed 0：
    - 倒水两边都是 1.0；
    - 杯对杯两边都是 0.0：初始状态逐毫米相同，失败是这个 seed 本来就不稳（已知）；
    - 拾取 1.0。
  - 搬后：新代码直接用真实共享库跑倒水 1.0、杯对杯 0.0；旧代码经软链接跑倒水 1.0，两边都没有报错。
  - 新平台代码对镜像扫描：195 件归 Real2Sim，官方物体归 RoboDojo。
- **平台修正**：生成脚本按类别名模糊匹配，`bottle/00061`（官方瓶子）因为 `make_plastic_bottle.py` 里有 `"bottle"` 被当成自建、进了 Real2Sim 分区。现在磁盘上在 RoboDojo 库的物体不再匹配生成脚本；`auto.library` 对还没建模型的物体为空。镜像里另把学校纸杯、纸盘（5 件，只在镜像、不在共享库）也搬到了 Real2Sim。
- **部署**（PR #35 合并为 `82bf33c` 后，11:22）：部署机工作区有其他会话的 149 个未提交改动，按上次的做法只替换本 PR 改到的 26 个代码文件，文档不动。其中 `index.html` 部署机上有另一会话 11:12 的照片框 CSS 热修复，用 `git merge-file`（基准 `65d7d5b`）三方合并，无冲突，两边都保留。替换前的文件备份在 `.cache/backup-before-real2sim-lib-20261028-112218.tgz`。部署机上 `test_server.py` 全部通过，按 `.cache/run-platform.sh` 的方式重启，隧道会话没动。
- **跟进**（`chongzheng/platform-variant-section`）：已经建好但还没装库的泛化变体（`metal_cup`、`ceramic_cup`、`iphone_17_pro_max` 的变体，只有 `check.json` 里的 `variant_of`），以及它们的本体，原来落在 RoboDojo 分区，现在也归 Real2Sim。用页面自己的分组代码对线上 279 件跑了一遍：Real2Sim 224 件，RoboDojo 55 件，后者全是磁盘上 RoboDojo 库里的官方物体。
- **遗留**：
  - 软链接是过渡用的：等旧运行目录不再用（它们的 `Assets/Object/RoboDojo/Rigid/pitcher` 等链到共享库），并且主线合并了本分支，再删掉。删之前先确认没有在跑的作业。
  - 平台没部署：`5090-2` 的工作区里有其他会话未提交的改动，等合并后按 `.agent/ops.local.md` 更新。
  - 本机 `Assets/` 是空的，`init_assets.sh` 下载的官方资产没有 Real2Sim 库，自建物体要用生成脚本自己生成。

### 合并全部开着的 PR（#31–#34）并部署数采平台（2026-10-28，`chongzheng/merge-open-prs-1028`）

- **用户要求**：处理当时所有开着的 PR，合并后部署。
- **合并顺序**：按此顺序合进 `chongzheng/merge-open-prs-1028`，再快进推 `main`（GitHub 自动标为已合并）：
  - #31 泛化维度 L1–L3 + 综合等级 G0–G3；
  - #32 物体按资产库分组、Real2Sim 审核、重建负责人、照片排序；
  - #33 `plastic_mug` 实测尺寸 + 5 个变体；
  - #34 吧台岛台（本会话上一节）。
- **冲突**：只有 `HANDOFF.md`。各 PR 的「最近任务」节都保留，「最近历史」两条都留，开头的会话行改写成这一行。#31、#32 都改了 `index.html`、`server.py`、`test_server.py`，自动合并无冲突。
- **验证**：本机合并结果 `bash scripts/check.sh` 通过（82 passed、1 skipped），页面内联脚本 `node --check` 通过。
- **部署**（11:13）：部署机 `/root/RoboDojo` 从 `a21b1ac` 快进到 `65d7d5b`，别的会话的 148 处未提交改动全部保留（前后清单核对过）。平台文件在部署前就已和 `main` 一致（#32 的会话 11:03 部署过）。唯一冲突是 `plastic_mug/00000/spec.json`：平台上的登记字段和 `main` 的 `generalization` 按字段合并。吧台场景的新 LFS 文件、预览（含新 `scene.glb`、RTX 渲染）和马克杯 6 件的预览图从本机同步。重启后 `test_server.py` 13 passed，公网未登录 401、首页 200，平台上吧台场景 11 张照片、马克杯 6 件都有渲染。细节和回滚方法在本机 `.agent/ops.local.md`。
- **没做**：共享资产库没动（吧台岛台版、马克杯变体都没装）；`g1d_counter_mobile` 布局没重新生成。

### 数采平台：物体按资产库分组，Real2Sim 物体管理员审核（2026-10-28，`chongzheng/platform-object-libraries`，已部署）

- **用户要求**：物体资产分类：Real2Sim 单独一组（像场景页的 RoboDojo 基准那样分区，但能收起），RoboDojo 资产库一组，以后的其他公开资产库也能加进来；Real2Sim 物体加管理员审核，按待重建 / 待审核 / 完备查看。按 UI UX Pro Max 做。
- **分组**：物体页每个库是一个可折叠分区（原生 `<details>`，收起状态存浏览器 `localStorage.objLibClosed`）。内置两个库按来源自动归：有生成脚本或实物来源（`ownObj`）→ Real2Sim，其余 → RoboDojo。其他公开库是新的 kind `library`（`id`/`name`/`source`/`description`），只有管理员能在「管理资产库」抽屉里加、删（库里还有物体时不能删）。物体详情「基本信息」里的「资产库」下拉把物体移到某个库（字段 `library`，服务端校验库存在，空 = 按来源自动）。原来的「来源」筛选去掉了（分组已经覆盖）。
- **审核**：状态由服务端 `review_state()` 算，随物体返回：没有建好的模型 → 待重建；建好了、没有对这一版模型的结论 → 待审核；管理员通过 → 完备；退回 → 待重建（必须写原因）。结论记在 `review`（`state/by/at/note/model` + 最近 20 条 `history`），`model` 是 `preview/check.json` 的 `checked_at`（入库无检查记录时为 `library`），模型重建后自动回到待审核。只有管理员能写结论（服务端 403，前端传的 `review` 一律丢弃）。Real2Sim 分区里有按状态的分段筛选，卡片 / 列表带状态标签，管理员侧栏「物体资产」显示待审核角标。
- **重建负责人**（同日追加，用户问每个物体是谁重建的）：物体加 `owner` 字段（平台账号，和任务、本体的负责人同一套），登记向导默认填登记人，抽屉「基本信息」可改；卡片、列表、审核区显示，Real2Sim 物体没填时显示「未指派负责人」；物体页加负责人筛选（我负责的 / 未指派 / 各人）。已有物体都没填，需要补：物体页卡片 / 列表每件左边有勾选框，每组有「全选本组」，勾选后底部浮出批量栏，选人后「批量指派」逐件保存（同一个 PUT，谁都能指派，和单件一致）。批量栏是悬浮的，不占页面流，勾第一件时卡片不会下移。
- **审核区只在待审核时给表单**（同日，用户反馈通过后不该还有意见框）：已通过 / 已退回的物体只显示一个「撤回通过（退回），改回待审核」，服务端动作 `reopen`（管理员、模型已建好、当前不是待审核，记一条 `reopened` 历史）。
- **抽屉里切换变体预览**（同日）：「仿真渲染」下方一排小缩略图（和左侧实拍缩略图同尺寸，角标「本体」或变体编号，悬停看全名），点一下就地换成那件的渲染图，可再点「打开详情」（按钮始终占位，标题高度不变）；原来抽屉底部的「泛化变体」区去掉了。

- **场景中文名可改**（同日，`chongzheng/platform-scene-names`）：每个场景抽屉顶部「名称 → 中文名」，存场景记录的 `name`（只在平台数据库，不改 `scene_spec.json` / `registration.json`），优先于登记名和 `scene_spec.json` 的名字；留空恢复原名。服务端对只改名的保存不重写 `registration.json`（普通保存会重写，未登记场景还会删掉它）。
- **大图改成中心裁剪**（同日，`chongzheng/platform-hero-crop`）：抽屉里实拍 / 渲染大图的竖拍照片原来撑高网格行、底部被裁（看起来是顶部裁剪），现在图片钉在 4:3 框里，`object-fit: cover` 从中心裁，和缩略图一致。
- **实物照片拖动排序**（同日）：抽屉里实拍缩略图可拖动（或聚焦后 Alt + ← / →），第一张是卡片封面。顺序存物体字段 `photo_order`（文件名列表），服务端 `ordered_photos()` 在返回物体时按它排 `auto.photos`，没列进去的新照片排在后面；文件本身不动，`spec.json` 和建模确认不受影响。
- **部署前提**：部署机上有另一会话（10-27 20:48）未提交的平台改动（`catalog_policy.py`、`object_registration.py`、`ASSET_WORKFLOW.md` 和 `server.py`/`index.html`/`test_server.py` 的修改）。本分支以部署机当时的这几个文件为基础改，所以这些改动单独放在分支第一个提交里（原样取自部署机）；部署只替换了 `server.py`、`index.html`、`test_server.py` 三个文件。
- **验证**：本机 `test_server.py` 13 passed（新增 `test_object_review_and_libraries`：没模型不能通过、成员 403、退回必须写原因、重建后回到待审核、库的增删与校验）；`ruff 0.11.7` 检查 `server.py`、`node --check` 页面脚本。浏览器（本机，生产库副本 + 临时管理员）：分区折叠和记忆、加 YCB 库、空原因被拦、退回、通过、移入 YCB、手机宽度无横向滚动。`5090-2` 上同样 13 passed，重启后未登录 401，管理员会话下 Real2Sim 48 件：待审核 41、待重建 7，自定义库 0。
- **剩余**：已有 41 件建好的 Real2Sim 物体全部是待审核，要管理员逐个过。公开库的物体目前只能先登记、再手动移进库；按库目录自动扫描（如 `Assets/Object/<库>/`）没做，等真有第二个库的文件再加。第一个提交顺手按 ruff 格式化了 `test_server.py`（原有一处 E501）。

### 塑料马克杯 `plastic_mug` 完善 + 5 个泛化变体（2026-10-28，`chongzheng/objects-plastic-mug`）

- **用户要求**：完善 `plastic_mug`，增加泛化，然后跑 `check_object`。
- **完善**（`scripts/objects/make_plastic_cups.py`，几何未改）：`metadata.json` 补 `measurements_mm`（键与平台登记一致：总长（含把手）/ 外径 / 高，从网格量，外径只取杯身）= 119.0 / 84.0 / 100.0，与实测误差 0；去掉不生效的 `physics.friction`（量杯同样去掉；运行时本来就用默认 0.6 / 1.5，行为不变）；`metadata.json` 改为 `ensure_ascii=False`。
- **泛化**：登记 `spec.json` 补 `generalization`（count 5，颜色 / 材质 / 尺寸 / 形状可变，`rt_color`、`rt_scale` ±5%），由 agent 代写。`MUG_VARIANTS` 生成 `plastic_mug/00001–00005`（`variant_of` 00000，全部估计）：天蓝儿童杯 80 mm、粉色高身锥形杯 120 mm、藏青陶瓷咖啡杯（壁厚 4 mm、圆环把手、289 g）、烟棕半透明杯（着色 OmniSurface）、薄荷绿圆角方杯（超椭圆截面 SQ=4）。质量 = 网格体积 × 密度（PP 0.9、陶瓷 2.3）。杯身宽 72–84 mm，276–490 ml。`build_mug(g)` 参数化，`plastic_mug_shape.py` 仍只描述 00000（任务杯腔判据不变）。
- **验证**（`5090-2:/data/robodojo-runs/plastic-mug-20261028-102627`，GPU 4）：生成 + 自检（凸包不侵入杯腔）通过；`check_object.py` 6 件马克杯 + 2 件量杯全部 OK（倾角 0°、z 误差 0）；渲染图在本机 `real2sim/objects/Rigid/plastic_mug/0000{0..5}/preview/`。ruff 0.11.7 通过；单测 78 passed，1 failed 是运行目录没同步场景照片（`test_scene_specs_are_complete_and_read_the_model`），与本改动无关。没跑 `check_cup_beans.py`（只覆盖 00000，00000 几何没变）。
- **没做**：预览没同步到部署机（写部署机被权限拦下，等用户确认）；没装共享库（00000 本来也没装）；`RANDOMIZE` 没在任何任务打开；变体没进任何任务布局。

### 吧台场景：过道岛台、天花板、右侧木柜墙、垃圾桶（2026-10-28，`chongzheng/scene-counter-island`）

- **用户要求**：给了 6 张茶水间照片（天花板、垃圾桶细节，以及吧台前多出来的岛台），按照片优化吧台场景 `real_scene2_counter`。照片入库为 `reference/island_{front,right,left,end}.jpg`、`ceiling.jpg`、`bins.jpg`。
- **拟合**：新增 `fit_island.py` → `fit_island.json`。四张岛台照片联合拟合相机、共享焦距和岛台尺寸，相机锚在已拟合的吧台上（海报、架子、台面前沿、凹龛木柱），尺度基准仍是「微波炉宽 0.485 m」。118 个像素约束，中位 3.3 px，最大 25 px。岛台轮廓标注要指定落在跑道形的哪一段，否则会被吸到看不见的一侧（第一次就这样拟歪了）。
- **结果**（细节和置信度表见该目录 `PLAN.md`「岛台」）：
  - 高吧台：顶高 1.091 m，长 4.49 m、宽 0.84 m，左端立在铜色跑道形立柱上，右端由铜鼓座架在绿色岛台上。
  - 绿色岛台：顶高 0.867 m，长 4.83 m、宽 1.15 m，绿色竖向半圆波纹板、铜踢脚。
  - 6 把吧凳：过道侧、靠墙侧各 3 把，座高按 0.75 m 假设。
  - 2 盏线形吊灯：黑杆、3 个铜灯盒，灯盒底面自发光。
  - 吧台靠墙边到吧台台面前沿 1.60 m；靠墙侧吧凳把过道收窄到 1.22 m。
- **顺带改的**：
  - 天花板灯槽原来建成平行于墙，实际是垂直于墙、每 2.6 m 一条（lounge1 反投影 x = −4.97，新照片 0.2 / 2.8），每条配双头筒灯；白色石膏带延伸到整面墙；x > 6.8 改为白色石膏顶。
  - 右侧木柜墙按 island_right 正面照片重排：板缝、消防栓门、饮水机凹槽（原来两槽只隔 5 cm，实际约 17 cm）；木墙在 x = 5.4 结束，之后是刷漆墙。
  - 房间扩到 x = 9.0、y = −7.0（拍照位置在原来的墙外）。
  - 垃圾桶改成圆角不锈钢桶、不锈钢盖、黑底边和脚踏，「可回收物 / 其他垃圾」印刷从照片校正成镂空贴花。
  - `preview.py` 加 `compare_island.jpg`；`plan.jpg` 原来整张都是天花板，现在去掉了 2.5 m 以上的东西。
  - `check_scene.py` 加了岛台碰撞体顶点数、两座台面高度、过道宽 ≥ 1.0 m 三项（25/26，唯一 FAIL 仍是原有的站位提醒）。
- **脚本专家** `scripted_demo.BACK_OFF` 从 0.7 改为 0.5 m。按 `AGV_link` 底盘矩形（0.54 × 0.525 m）扫过 `carry_to_lounge` 的路线，0.7 m 会压到靠墙侧的吧凳，0.6 m 擦边，0.5 m 最小间隙约 9 cm；杯子倒退后仍离台面前沿 0.33 m。
- **用户确认后的两处修改（11:05）**：去掉微波炉（照片里已撤；提示卡改为立在台面上；尺度仍按 `fit_photo.py` 里它的假设尺寸）；客区白色圆桌的扶手椅从 2 把改为 4 把，十字摆放（`CHAIRS`），椅子是布局刚体，所以 `g1d_counter_mobile` 的布局要重新生成才会有 8 把椅子。改后 `check_scene.py` 25/26、RTX 渲染核对过；两个移动任务没重跑（微波炉不在送杯路线上，白桌离蓝桌 2.6 m，送杯只看蓝桌 1.2 m 内的椅子）。
- **验证**：
  - 本机：`fit_island.py`、`build_scene.py`、`check_scene.py`（25/26）、`preview.py`；`scene_parts.py` 重新生成的 `parts.json` 和原来一致；`bash scripts/check.sh` 通过（78 passed、1 skipped）。
  - `5090-2` 隔离目录 `/data/robodojo-runs/counter-island-20261028`（GPU 4/5；资产硬链接自 `pour-deliver-jug-c`，只换 `Room/real_scene2`，共享库没动）：
    - `render_scenes.py` 的 RTX 画面正常，岛台、吧凳、吊灯、灯槽都在，本机 `preview/render/sim.jpg`；
    - `g1d_pour_water_deliver_to_lounge` seed 0 布局 0：success_rate 1.0、eval_time 1；
    - `g1d_deliver_cup_to_lounge` seed 0 前 4 个布局：2/4（eval_time 4）。成功的两个底盘都穿过过道到了圆桌。失败的布局 0、3 都是侧抓时杯子没被抬起（布局 0 被推到 y = −0.16），还没开到岛台。
  - 对照 `/data/robodojo-runs/counter-island-ctrl-20261028`（GPU 6）：同一份代码配旧场景，逐布局结果一样，布局 0 的杯子位置都一样。所以抓杯失败不是岛台造成的，最可能是 10-27 合进 `main` 的夹爪 55 N / 限速 0.064 m/s（这个任务在那之后没人重跑过）。
- **没做**：
  - 抓杯失败没修；
  - 共享库没装：共享库里的 `real_scene2` 还是只有吧台的旧版，客区版也没装；
  - `cam_third`（移动版，房间右前角）现在前景里多了吧台，台面和客区圆桌仍在画面里，没调；
  - 吧凳是静态的（客区扶手椅是可推动的刚体）；
  - 天花板高度 3.20 m 没改：灯槽反投影更像 3.0 m，但木柜墙顶和灯带顶指向 3.0–3.1 m，互相矛盾，留待卷尺。

### 深圳实验室装共享库 + 合并全部开着的 PR（2026-10-27，`chongzheng/merge-open-prs`）

- **用户要求**：深圳实验室装进共享资产库；处理本仓库（`origin` Zheng-Chong/RoboRSR）所有 PR 并合并。`gh` 没设默认仓库时会列出 upstream（RoboDojo 官方）的 10 个外部 PR，那些不是本仓库的，没动。
- **共享库**（经用户同意，15:42）：`5090-2:/data/robodojo-assets/Assets` 新增 `Room/real_scene5`（10-27 整层房间版）和 `Eval_Layout/RoboDojo/g1d_shenzhen_lab`（574 个：拾取 34、杯对杯 55、倒水 48、叠块 55 × seed 0/1/2，和 `g1d_lab` 同一套；量杯任务因共享库没有 `measuring_cup` 不生成，同 10-21）。两者原本都不存在，没有覆盖，装前目录清单在 `backups/listing-before-shenzhen-20261027-154209.txt`。布局在隔离目录 `/data/robodojo-runs/shenzhen-install-20261027`（`Assets/` 除 `Room/real_scene5` 外全部软链接共享库）用当前代码的 `build_asset.build_layouts` 只生成这个配置；同样生成的 `g1d_lab` 和共享库比只差倒豆 / 倒水的豆子粒子参数（共享库仍是旧 4 mm，已知），所以生成输入一致。验证：同目录 `g1d_stack_blocks`、`g1d_general_pickup` seed 0 都是 success_rate 1.0、eval_time 1；拷入后 `diff -r` 一致。平台元数据镜像里本来就有同样的 574 个布局（逐文件一致）。
- **PR**：#30（L2 机器人位姿等级）、#29（RSI v0.6）、#28（倒水 + 送到客区）、#26（夹爪限速 0.064 m/s）、#24（夹爪力传感 + 55 N / 3.6e4）、#25（带盖塑料瓶）按此顺序合进 `chongzheng/merge-open-prs`，再快进推 `main`（GitHub 自动标为已合并）。冲突：
  - `env/robot_manager/robot_config/g1d.py` 夹爪执行器：#24 的 55 N / 3.6e4（10-21 用户拍板的参数）和 #26 的限速都保留（#26 记录里写明两边都留）。**G1-D 夹爪默认物理因此变了**：已有数据都是 20 N / 1e4、不限速时采的（见「剩余事项」）。
  - `HANDOFF.md` 各节都保留（「项目状态」里两条夹爪说明合成一条）；`README.md`、`real2sim/ASSETS.md` 两边新增的行都保留。
  - #29 的 CI 失败是 `rsi_start.py`、`test_rsi_start.py` 两处 import 排序（ruff I001），已修。
- **验证**：本机 `check.sh`（78 passed、1 skipped）。合并后代码在 `5090-2:/data/robodojo-runs/merge-prs-20261027`（GPU 6）跑 `g1d_shenzhen_lab` 量杯 seed 0：success_rate 1.0、eval_time 1（55 N 和限速同时生效的第一次运行）。#28 的移动任务没重跑（要 `g1d_counter_mobile` 布局，隔离目录里没有；作者已验证）。
- **没做**：部署机 `/root/RoboDojo` 没更新到这次的 `main`（上面还有别的会话 150 个未提交文件，要的话照 10-27 15:28 那次做）。
- **平台物体卡片打不开（16:45 修）**：部署机上另一会话 14:03 未提交的 `index.html` 改动（物体抽屉改成「登记确认与统一建模 prompt」）删掉了 `objDeleteBlocker` / `objDeleteButton`，抽屉仍调用后者，点任何物体卡片都报 `ReferenceError`（从 14:04 那次重启起就坏了，和本会话部署无关）。在部署机工作区里把这两个函数按 `HEAD` 原样补回（`modelingBrief` 之后），其余改动不动，仍是未提交状态；改前备份 `.cache/index.html.before-objfix-*`。页面每次加载都重读文件，没重启服务。用临时实例（端口 8799、数据库副本、临时管理员，验证后都删了）在浏览器里复现并确认抽屉能打开、删除按钮在。**那个会话提交时要带上这两个函数。**
- **深圳实验室椅子按照片重建（17:00，用户：椅背实际都是黑的，建模要仔细）**：原来人体工学椅背后贴了一整块浅灰板，看起来像白椅背。现在两种都按 IMG_4948/4949/4950/4954 建：黑色弧形网背（下窄上宽 0.38→0.50 m，半透明网面）加黑色边框和腰托、黑色座框和网面（座面在照片里是实心黑，不透明）、L 形扶手、底盘机构、锥形气杆套、渐细的抛光铝五星脚和双轮脚轮；黑网椅是带立杆的宽头枕，人体工学椅是弯杆上的环形头枕，背后多一块黑色连接壳和一根银灰色 Y 形背撑（只在背面看得到）。椅子离桌面前沿改为 ≥ 0.40 m（脚轮伸得更远）。`check_scene.py` 28/28，三角面 1.4 万；`5090-2` 上 RTX 特写 5 张核对过外观。共享库 `Room/real_scene5` 换成这一版（旧版备份 `backups/backup-shared-room5-20261027-165823.tgz`，`diff -r` 一致；布局不变）；平台的 `scene.glb`、RTX 封面、叠图已换。

### 吧台冷水壶倒水 + 端到客区圆桌：第二个移动操作任务（2026-10-26，`chongzheng/task-pour-deliver-lounge`）

- **用户要求**：吧台场景里"在吧台倒完水，再拿到桌子上"的移动操作，要拼接后的 4 视角视频。用户明确要**用冷水壶倒水**（一开始做成杯对杯倒，被否了）。
- **任务** `g1d_pour_water_deliver_to_lounge`（`g1d_counter_mobile`）：台面上放冷水壶（约 300 ml 水）和水杯。一只手抓壶把往**立在台面上的**杯子里倒水，壶放回原位，另一只手侧抓杯口下方把杯子拿起，底盘开到客区蓝色圆桌放下。判据：`delivered_checks()`（杯 `cup`）加 `is_A_fluid_in_B`（倒出 ≥ 20%，倒出的 ≥ 90% 在杯里；水花碎成 3 mm 小水滴，按洒出算，不像豆子任务那样忽略小团）。2100 步。
- **布局** `build_asset.pour_deliver_layout`（`LOUNGE_TASKS`，seed 0–2 各 10 个）：取 `g1d_pour_water_into_cup` 同一条流水线（`pour_liquid_into_cup` → `swap_bottle_for_pitcher`），再做四件事：随底盘后移 5.3 cm（`MOBILE_BACK`）；去掉台面杂物和 `Geometry`（X5 的相机支架正好立在机器人站的位置，会把移动版底盘推走 8 cm；杂物总让布局稳定性检查不过）；壶限制在 |x| ≤ 0.12、y ≤ -0.44（`JUG_MAX_X/Y`）；豆子换成水。
- **水**：stock 液体（PBD fluid + isosurface，`Linen_Blue.mdl`）改了四处：`particle_mass` 2.7e-5（3 mm 粒子按 1 g/ml，`FluidObject` 默认 0.5 g 重 20 倍，杯子直接从手里滑掉）；`cohesion` 0、去掉 `surface_tension`（真实质量下水抱成一个球整团甩出去）；`damping` 0（0.99 时倾到 75° 还倒不出）；壶里装量 `WATER_FILL`（1.2, 1.2, 1.6），约 300 ml（豆子的 510 ml 换成水对壶把抓握太重）。
- **脚本专家** `scripted_demo.pour_deliver_actions`：`pour_actions` 新增 `fluid_label`、`dry`、`pour_target`、`hold_cup` 参数（默认值不变，壶倒豆行为不变）。水用 `WATER_DRY` 40° 快倾、倒出 `WATER_POUR` 30% 停；`hold_cup=False` 时杯子留在台面、另一只手不动（原来空中托杯那步在这个场景里够不到）。倒完把壶移回原位放下、手沿进来的方向退出回原位；再用 `lift_cup`（从 `deliver_cup_actions` 抽出来，可指定手臂；这里固定用不拿壶的那只手，`arm_for` 按 x 会选到壶那只手，从壶那边伸过去会撞）拿杯，再调 `carry_to_lounge`（同样从 `deliver_cup_actions` 抽出来：后退、绕椅子开到圆桌、降升降柱、放杯、退手）。
- **移动版底盘出生时卡在地柜里**：`g1d_counter_mobile` 原来的根节点 y=-0.80 抄自桌面版；移动版底盘碰撞体和地柜重叠，reset 时被推回 5.3 cm（`base_x_joint` = -0.0527，之后不再变），于是按桌面版调好的物体位置对它都远了 5 cm。`env_cfg/robot/g1d_counter_mobile.yml` 根节点改为 -0.853（就是它实际站的位置，送杯任务行为不变）。`DeliverCupToLoungeG1dCommon` 加了类属性 `CUP`。
- **`cam_third`**：吧台场景原来的第三视角在微波炉后面被挡住。新增 `env_cfg/camera/g1d_counter_mobile_camera.yml`（只给 `g1d_counter_mobile` 用）：相机在房间右前角 (0.7, -3.8, 2.3)，能同时看到吧台和客区圆桌。
- **视频**：`5090-2:/data/robodojo-runs/pour-deliver-20261026/stitch4.sh <回合目录> episode_0000000 <输出>` 把 `cam_head`、`cam_third`、左右腕拼成 2×2（1280×960，带标签）。
- **10-26 下午复查（用户：夹爪和关节没限速、杯子最后没放到桌上）**：两点都属实。① 判据漏洞：握着 72–73 mm 的杯子时夹爪开度约 0.75，旧判据「张开 ≥ 0.6」直接通过，杯子悬在桌面上方时就判了成功、回合结束（送杯任务同样受影响）。`DeliverCupToLoungeG1dCommon` 改成杯子必须落在桌面高度（相对台面初始高度在 `drop ± REST_TOL` 内，`drop` 从 `build_params.json` 算）且夹爪开度 ≥ `OPEN` 0.85。② 限速：仿真里手臂执行器上限 5 rad/s、夹爪没有上限，专家指令也没限幅。新增 `scripted_demo.Limiter`（只对 `LIMITED` 里的本任务生效，其他专家和已有数据不变）：关节离软限位 `REAL_JOINT_MARGIN` 0.05 rad（同真机 `calib_arm_server.py`），每步不超过 `REAL_ARM_VMAX` 0.8 rad/s（标定工具包 docs/05 记录的真机回放最快速度），夹爪指令每步不超过 `REAL_GRIP_RATE`（Dex1-1 的 0.2 m/s 换算，0.15/步，`assemble.json`）。限幅后倒水的快倾（4°/步≈2 rad/s）跟不上、洒了 25%，所以 `pour_actions` 加了 `rate`，本任务用 `WATER_RATE` 0.4。执行器层面（`robot_config/g1d.py` 的 `velocity_limit_sim`）没改，所有 G1-D 任务共用。
- **验证**：本机 `bash scripts/check.sh` 通过（77 个单测）。`5090-2` 隔离目录 `/data/robodojo-runs/pour-deliver-jug-c/`（7 号卡）跑 seed 0 布局 0：`success_rate` 1.0、`eval_time` 1，壶倒出 34%、杯里 33%，底盘全程停在 (0, -0.853)，开到圆桌后放杯、松手才判成功（50 s，1511 帧）；限幅日志：专家原本最大要求单步 0.447 rad（≈13 rad/s）、夹爪单步 0→1，都被截到上限。4 视角视频 `keep/v19_4view.mp4`，本机 `output/pour_deliver/g1d_pour_water_deliver_4view.mp4`（不入库）。对照：同布局不降倾倒速度（v18）倒出 44%、杯里只有 33%。上一版 v17（没限速、旧判据）的成功是假的：杯子还在手里。
- **没做**：回合在松手瞬间结束，视频里看不到手退出；只验证了 seed 0 布局 0（壶 (0.10, -0.45)、杯 (-0.04, -0.43)）。`JUG_MAX_X/Y` 是在相机支架还没去掉时定的（当时底盘被推走，远处的壶够不到），去掉支架后可能可以放宽。`g1d_pour_water_into_cup` 自己的布局（`build_layouts`）没改。没跑完整 `build_asset.py`（服务器缺 `g1d_v4`/`unitree_ros`），布局用 `build_lounge_layouts` 单独生成；共享资产库没动。

### 深圳实验室 `real_scene5_shenzhen_lab` 补成整层房间（2026-10-27，`chongzheng/scene-shenzhen-lab-room`）

- **用户要求**：按 8 张深圳实验室全景照片优化现有场景，包括天花板；物体只要桌子、椅子、架子、隔断，不要细小物品。
- **测量**（`fit_room.py` → `fit_room.json`，照片入库 `reference/IMG_4948–4955.jpg`）：7 张照片是同一站位约 (−1.2, −4.4) 转身拍的，拟合出的机位互相印证。拍照时桌子已被拉离墙、往柜子挪了约 0.3 m，所以尺度取自柜子、墙上三个插座（旧 IMG_4914 相机反投影）和 IMG_4913 的墙角；孪生里桌子仍在登记位置。分 5 步拟合（联合 → 工位腿 → 吊顶设备 → 西墙 → 西墙设备），再把像素反投影到已知平面量家具和灯位。
- **结果**（桌子坐标系，单位 m）：后墙吊顶下沿 2.83、深 0.75（和两根角柱齐平），穿孔铝板天花 2.93；角柱 x ∈ [2.34, 3.28] 和 [−8.19, −7.32]；东墙 x 3.28、西墙 −8.19、电梯墙 y −8.94（西端 x −5.55，往西南凹进到 y −10.60）；3 条灯条 x 0.09 / −3.31 / −5.90，后吊顶筒灯 5 个；两部电梯 KT3 / KT4 关于 x = 0 对称。家具：东墙 5 个工位模块（1.33 m，蓝挡板、活动柜）、西北升降桌、西墙蓝色货架、5 把椅子、电梯厅玻璃栏杆和闸机。数值和依据表在该目录 `PLAN.md`「房间（v2）」。
- **代码**：`build_scene.py` 重写房间部分（10 段墙的轮廓、角柱、两条吊顶、穿孔板贴图、灯条和筒灯、电梯饰面），家具放在 `/Background/Furniture`（同上海实验室，Room 入口和 `scene_parts.py`、`render_scenes.py` 不用改）；灯光改成 3 条灯条 RectLight + 5 个锥形 DiskLight，强度按数值积分让桌面 / 后墙 / 近地面照度和旧标定相当。`check_scene.py` 桌子检查只看 `/World/Table`，新增墙段数、天花封闭、家具落地 / 在轮廓内 / 互不重叠 / 离桌子和站位 ≥ 1 m（28/28）。`preview.py` 加新机位叠图 `compare_room.jpg` 和俯视 `room.jpg`。`parts.json` 重生成（墙面 4 块、桌面、地板不变）。
- **验证**：本机 `check_scene.py` 28/28、`preview.py` 7 个机位叠图（墙角、柱、工位、栏杆、货架都对得上，目测 ≲ 10 px）、ruff、`check.sh`。`5090-2` 隔离目录 `/data/robodojo-runs/shenzhen-room-20261027`（GPU 7；资产硬链接自 `shenzhen-beans-20261021`，只换 `Room/real_scene5`）：`render_scenes.py` 正常；`scripted_demo --env_cfg_type g1d_shenzhen_lab --task_name g1d_pour_beans_measuring_cups --seed 0` success_rate 1.0、eval_time 1，四路相机正常（之后只改了两把远处椅子的朝向，没重跑）。
- **部署**（15:28，用户同意）：部署机 `/root/RoboDojo` 原在 `8838b1c`（另一会话的 `chongzheng/docs-eval-levels-robot-pose`，未进 `main`），在部署机上合并 `2d95c63` 得到 `5b45b39`（只在部署机，没推 GitHub；`HANDOFF.md` 两节都留）后快进。部署机别的会话 150 个未提交文件不动，只把重叠的 `HANDOFF.md`、`real2sim/ASSETS.md` stash 后放回（`ASSETS.md` 学校行取部署机版本、深圳行取新版本），仍是未提交状态；备份 `.cache/ff-deploy-20261027-152647/`。新照片和穿孔板贴图从本机补实体；预览整个换新（`scene.glb` 254 网格、RTX 封面、`compare_room.jpg` 等），旧的在 `.cache/shenzhen-preview-before-room-20261027`。重启后部署机 `test_server.py` 9 项通过，扫描到 14 张照片、新模型和封面，本机和公网首页 200、未登录 `/api/info` 401。
- **没做**：RTX 下按 IMG_4916 重新对色（灯光换了，颜色可能和 10-21 的标定有偏差）；房间尺寸没有卷尺；没装共享库（5090-2 共享库本来就缺 `real_scene5`）。

### G1-D 分级泛化改为「五个维度 × L1–L3 + 综合等级 G0–G3」（2026-10-27，`chongzheng/platform-generalization-axes`）

- **用户要求**：上一版（[PR #30](https://github.com/Zheng-Chong/RoboRSR/pull/30) 的 L0–L5，每级只改一个因素）没有体现「分级」，分级应层层递进、难度递增；按用户给的五个维度重组：空间关系（机器人高度、朝向、离桌距离，桌高，物体位置）、场景视觉（光照、纹理、不同场景）、物体属性（纹理、颜色、同类实例、缩放）、视角成像（相机偏移、图像扰动）、语言对齐（简短 / 详细描述，靠指令区分物体）。
- **方案**（README「三、分级泛化评测集」）：每个维度分 L1 轻度、L2 训练范围全开、L3 分布外（不进训练）；训练布局到 min(最高档, L2)。综合等级 Gk = 所有维度同时取 min(k, 该维度最高档)，G0 孪生 25、G1–G3 各 50；另有单维度诊断（只升一个维度，其余 L0），每维度每档 10，不计分级成绩。每任务约 325 个布局；专家过滤 G0–G2 ≥ 80%。干扰物拆到两处：一般杂物算场景视觉，同类干扰物算语言对齐（不然测不出指令遵循）。各档默认范围见 README 和平台 `AXES`，空间关系的底座范围仍是暂定，要按真机停靠实测定。
- **平台**：`LEVELS` 换成 `AXES`（维度、各档默认范围）和 `GRADES`；登记向导第 4 步每个维度选最高档并改各档范围，下面按选择列出综合等级和诊断的布局数；物体属性 L3 要有角色的保留物体、场景视觉 L3 要有保留场景，没有时默认只开到 L2，手动开 L3 会被拦下；第 3 步去掉「外观随机」，第 4 步去掉「位置随机」「机器人位姿随机」。实现任务文本按维度和综合等级写，并提示三处缺口（见下）。任务抽屉的分级表按综合等级和诊断显示，「已生成」改存 `grades.<G>`（旧的 `levels.<L>` 不再显示）。评测申请的等级改为 G0–G3 和 D（默认 G0–G2），`result.json` 的 `level` 写 "G2" 或 "spatial:L3"。
- **`registration.json`**：去掉 `randomize`、`robot_pose`、`appearance`、`levels`，新增 `generalization`（{维度: {max, ranges}}）、`grades`、`diagnostic`。`g1d_pour_beans_any_cups` 已迁移：原「位置朝向随机 + L0/L1」→ 空间关系最高 L2（L2 保留原登记的范围文字），其余维度 L0。它有 8 个候选杯子，按新方案其实属于物体属性 L2，但原登记没选，没自动加。部署机数据库里这条登记要同样迁移。
- **还没实现**（实现任务文本里都已提示）：底座位姿不能按布局设（`env_cfg/robot/g1d.yml` 的 `default_root_pos` 固定）；评测客户端没有观测图像扰动的接口；`instructions.json` 还没有 `detailed` / `holdout` 的约定和读取。
- **和 RSI 的关系**：`rsi/subjects.yaml` 也是「维度 × L0–L3」，但轴是实例、杂物、表面、灯光、相机；以后可以统一成这五个维度，RSI 补空间关系和语言对齐。
- **验证**：本机 `test_server.py` 9 passed（新字段写入、`max: 0` 保留）、ruff 0.11.7、任务清单检查、`git diff --check`、内联脚本 `node --check`。本机浏览器（临时库）：第 4 步五个维度和默认档位（无保留资产时物体、场景为 L2）、物体开 L3 被拦、降回 L2 通过、语言设 L0 后范围收起；提交测试任务，`registration.json` 和实现任务文本正确（G3 中只开到 L2 的维度停在 L2），已删除；任务抽屉分级表、评测向导等级选项。
- **部署**（[PR #31](https://github.com/Zheng-Chong/RoboRSR/pull/31) 未合并）：数采平台已部署 `a21b1ac`，部署机上另一会话的未提交平台改动三方合并保留（`5090-2` 上 `test_server.py` 12 passed）；数据库里三条任务登记已迁移（保留署名）。中途推送命令出错导致部署机工作区短暂退回旧提交、9 个 LFS 文件变成指针、仓库根目录属主被改，均已恢复并核对。细节和以后的注意事项见本机 `.agent/ops.local.md`。

### RSI 协议 v0.6：基模定为 OpenWAM-α 开源权重，起始评测 = 官方评测集 0 的简化版（2026-10-27，`chongzheng/rsi-openwam-start`）

- **用户要求**：用 OpenWAM 在 RoboDojo 上微调的开源权重作后训练基模，以 RoboDojo 官方评测为起点，但评测要简化。
- **改动**：
  - `rsi/AGENTS.md` v0.6：基模 = HF `OpenWAM/OpenWAM-Alpha-Sim-RoboDojo` `checkpoint_step_60000`（ee；榜单视频里的 ckpt 名是 `New_OpenWAM_RoboDojo_SFT_60k`，就是这份）。R0.5 原来是"5 任务校准 + 24 任务起点"两件事，合成一件：官方评测集 0 上 `rsi/start_eval_tasks.txt` 的 20 个任务各考布局 0–9（`--eval-num 10` 按布局编号取前 10 个，固定），共 200 个回合，同时当起点和校准。去掉的 4 个是能力缺口任务的 `_random`（`push_T`、`sweep_blocks`、`sort_nesting_dolls_by_size`、`arrange_largest_number`，榜单 0–2 分）。封存探针等 R0.4 建好后单独补考。
  - `rsi/rsi.example.yaml`：`models` 只留 OpenWAM（Pi-05 拿掉），`budget.start_eval` 改成任务文件 + 官方 seed 0 + 判定参数，删掉 `calibration`。
  - `rsi/baseline/openwam_alpha_leaderboard.json`：从榜单页面的前端代码（`assets/index-CjJl8yvH.js` 里内嵌的逐任务表）抓出的 OpenWAM-α 54 个任务的 Score 和成功率，加各维度汇总。之前记录"榜单是 JS 渲染、抓不到分数"，其实数据就嵌在 JS 里。
  - `scripts/datagen/rsi_start.py`（+ `test_rsi_start.py`）：读 `<run_id>_<任务>/_result.json` 的逐回合分，和榜单逐任务比较：榜单分落在我们 10 回合均值的 bootstrap 95% 区间 ±5 分内算通过，最多允许 2 个任务不通过；输出 JSON 作 ledger 第 0 行的输入。
- **验证**：本机 `pytest scripts/datagen/test_rsi_start.py` 通过，ruff 0.11.7 检查和格式化通过，YAML 能解析。没跑完整的 `check.sh`（本机 base 环境没有 pytest 和依赖），也没上 GPU 跑起始评测。
- **起始评测 10-27 11:02 已在 `5090-3` 启动**（`5090-2` 当时只有 1 张空卡）：经内网（代理转发 rsync）把 `5090-2:/data/robodojo-runs/leaderboard-baseline-20261022/`（不含 eval_result、logs）和 `/root/miniconda3/envs/openwam-bench` 拷到 `5090-3` 的同一路径，权重 sha256 `78ca4e…0abb`；仿真用 `/data/lfwj/envs/RoboDojo`（另一会话的环境，IsaacLab/cuRobo 在 `/data/lfwj/RoboRSR`）。tmux `rsi-start` 跑运行目录里的 `rsi_start.sh`：三路，策略 GPU 1/2/3、仿真 GPU 5/6/7，run id `rsi_start_openwam_20261027`，跑完自动调用 `rsi_start.py`，结果写 `smoke_results/rsi_start_openwam_20261027.start.json`，日志 `logs/rsi_start_openwam_20261027.log`。预计 2–3 小时。`5090-4` 上也拷了一份权重到同一运行目录，没用上。

### RSI 协议 v0.5：起始评测和每轮考卷压缩（2026-10-26）

- **用户要求**：用 RoboDojo 完整评测作起点太慢，要简化；选了"官方起点用榜单数 + 校准、起始评测 24 任务各 10 回合 + 封存探针、每轮考卷约 400 回合、成绩单按维度汇总"。
- **依据**（10-22 基线，`5090-2:/data/robodojo-runs/leaderboard-baseline-20261022/`）：已经是每卡 10 个并行环境、OpenWAM 批推理，慢在 OpenWAM 推理本身，一路（策略卡 + 仿真卡）每小时约 30 个回合；54 任务原生回合数约 1900 回合，3 路约 21 小时（脚本估每路 15.6 小时）。OpenWAM 25 GB 权重放不下和 Isaac 同卡，`5090-2` 最多 4 路。
- **改动**：`rsi/AGENTS.md` v0.5：R0.5 官方起点不重跑（用榜单逐任务成绩，抽 5 任务 × 10 回合校准），RSI 起点 = 24 任务 × 10 回合（固定布局）+ 封存探针，约 4–5 小时；封存探针改成每任务 10 个布局（L0 2、单维 L2 各 1、全 L2 3），每轮同一组成对比较；成绩单按维度 × 级别汇总（各任务族合并，保留率先在任务内按 L0 归一再合并），考卷要求改成每个维度前沿 ≥ 30 回合且各族大致平均；能力缺口任务每轮 10 个 L0 回合；考试一轮约 430 回合、3.5–4 小时。`rsi.example.yaml` 预算随之改（考卷 250、封存 120、起始评测和校准字段）；`subjects.yaml` 注释改为成绩单合并各族。
- **补充**：第 0 轮加 R0.6，按 R0.5 的作业记录实测每个模型的吞吐 v 和路数 w，考试时长 ≈ 回合数 ÷（v × w），每轮考试控制在 4 小时内（`exam_hours_max`），超出就减考卷，但每个维度前沿不少于 30 回合；两个模型并行时按慢的定节奏。
- **验证**：纯文档，`check.sh`。

### RSI 协议 v0.4：裁判拆成判定层和运维 Agent（2026-10-26）

- **用户要求**：问裁判能不能用 Agent；同意"判定用脚本、运维用 Agent"的方案后要求写进协议。
- **`rsi/AGENTS.md` v0.4**：规则 3 说明裁判分两层和判定不用 agent 的四个原因（看得到一切会漏信息、会被提交里的文字操纵、不可复现、可能和两方同源）；角色表加"裁判运维 Agent"一列（只派发和盯作业、处理故障、两次运行之间维护裁判脚本；不写成绩单、`levels_r`、审计结论、不挑回合、不读两方提交的自由文本、不和两方通信；同一作业最多重跑 3 次；`referee/ops/` 唯一可写，`ops.jsonl` 记每次动作；判定问题交给人）；隔离加第三个 Unix 用户；第五节目录、七步表、G2 门和现状表跟着改；第六节审计加第 7 条（运维日志核对没有挑选回合）；第八节待办加运维 Agent 的作业清单格式、日志格式和运维手册（这几轮手工排查过的故障）。
- **验证**：纯文档改动，`check.sh`。

### RSI 协议 v0.3：标准化的递归迭代流程（2026-10-26）

- **用户要求**：更新 RSI 的 md，定一个标准化的 Recursive 迭代流程。
- **`rsi/AGENTS.md` v0.3**：第五节重写成标准流程：固定的运行目录结构（`rsi-<名>/{config.yaml, referee/, exam/, data/}`）；第 0 轮五步（冻结配置、分池、专家就绪统计（验收率 ≥ 50% 才能给该任务造演示）、封存探针每任务 25 个、基线）；每轮七步（出卷 → 考试 → 发成绩 → 定计划 → 造数据 → 审计 → 训练），每步写明谁做、输入、输出、工具；四道检查门（考卷、考试、数据、退步回退）；停止条件（跑满轮数、连续两轮封存探针没有显著提升、预算用完）；资产、级别、专家的准入只在下一轮开始生效；每轮约 1.5–2 天；每一步现在能不能跑的对照表。第八节待办按现状更新。
- **`rsi/subjects.yaml`**（新）：7 个任务族（12 个 `_random` 基础任务）、5 个维度（实例、干扰物、表面、灯光、相机）各 L0–L3，每一级是一段随机方案；`rsi_profile.py` 加 `compose()` 和命令行（`--pool --levels axis=Lk,... --out`），写作 `pool` 的字段展开成本池全部可用资产。
- **修正**：`x5_layouts.SEED_RANGES` 的考场段原来是 5000–9999，会写进裁判的封存探针号段，改成 5000–8999（协议和 `rsi.example.yaml` 本来就是这样写的）；平台页面同步。`rsi.example.yaml` 加训练起点（`init: previous`）、数据累积、考卷预算 1800（7 族 × 5 维度 × 30 回合是前沿的 60%）、检查门和早停字段。
- **验证**：本机 `check.sh`（`test_subject_levels_compose`：每一级都能合成并通过 `check()`）；`5090-2:/data/robodojo-runs/rsi-gso-20261026/` 上五个维度全 L3 的训练池、考场池方案各生成 3 个 `stack_bowls_random` 布局，`audit` 都通过；考场方案写 seed 9001 被拒。
- **没做**：5.8 表里的裁判脚本（成绩单、批量出卷 / 出数据、ledger、位姿复用审计）、混合训练配方、两方隔离。

### RSI 新实例：GSO 鞋 77 只（装箱），11 个马克杯验证能装液体（倒水）（2026-10-26，`chongzheng/rsi-rooms`）

- **用户要求**：随机化下一步做装箱、倒水的物体实例。
- **约定**：官方 18 只鞋全是鞋跟（高的一端）在 +x，装箱判定要鞋的 +x 朝世界 +x；官方鞋长 0.151–0.189 m。车、牙刷、锤子要逐个定义 `checkpoint`（−x 指车头 / 刷头 / 锤头），白酒瓶的液体初始位置写死在布局里相对瓶子，这两类没做。
- **代码**：`scripts/objects/gso.py`（GSO 模型 zip → RoboDojo 物体：水平主轴转到 x、鞋跟转到 +x、长度缩到 0.15–0.175 m、高度 0.045–0.095 m 之外的不收（拖鞋、靴子），带贴图的 USD + 减面凸分解碰撞体，元数据记 CC-BY 署名和 `source.tasks`）；`test_gso.py`。`instance_fit.py` 加装箱（鞋丢进箱子中间，逐项算任务的 `_shoe_checks`）和倒水（任务那团 0.06 × 0.07 m 的液体直接放在杯底，算任务的 `is_A_fluid_in_B` 97%）。`rsi/assets/imported.json` 加 `gso_shoes`、11 个杯子的任务加 `pour_liquid_into_cup`；`pools.json` 重扫（只多了鞋：训练池 53、考场池 24）。
- **结果**：GSO 1033 个模型里按名字 99 个鞋类，下完 99 个（服务器到 Fuel 约 50 KB/s，8 路并行约 2 小时）→ 转出 77（19 个高度不合格、2 个不是 Shoe 类别、1 个面数碰巧等于一件衣服）。鞋长 0.19 m 时扫描件会卡在箱壁里把箱子弹下桌，改成上限 0.175 m 后 77/77 过装箱检查（官方 3 号鞋作对照）。落定倾角 13 只在 1.8–3.7°（鞋底前翘），放宽到 4°。去重：没有和官方鞋面数相同的；贴图缩略图和 18 只官方鞋的相关最高 0.27（复制件应接近 1）；渲染出来鞋口都朝 +x。杯子：19 个里 11 个装得下（官方 15、16 号作对照都过），细的装不下。
- **验证**（`5090-2:/data/robodojo-runs/rsi-gso-20261026/`，资产 = `rsi-merged-20261025/Assets` + `rsi-assets-20261024/extra_gso`；杯子元数据改在 `extra_inst`）：训练池方案生成 `pack_objects_into_box_random`、`pour_liquid_into_cup_random` 各 12 个布局，新鞋占 10/12、新杯占 7/9 个马克杯，`audit` 通过；保持姿态跑完退出码 0，倒水 `eval_time` 12、无不稳定；装箱 1 个布局判为不稳定，是官方电动牙刷（站着的，被重摆位后倒了），不是新鞋。液体场景并行 21 个环境会 CUDA 非法内存访问，3 个可以。
- **没做**：车、牙刷、锤子、白酒瓶的新实例；新鞋没进共享库；装箱随机布局里站立牙刷不稳的问题。

### RSI 房间变体：换 `Simple_Room` 墙面材质（2026-10-26，短分支 `chongzheng/rsi-rooms`）

- **用户要求**：继续解决随机化（X5 专家暂停，进度在 `chongzheng/x5-experts`）。
- **做法**：不造新房间目录。布局的 `Room` 条目本来就支持 `appearance`（`env/scene_manager/objects/room_overrides.py`，给 Real2Sim 场景换面用的：运行时把指定 mesh 重新绑定到 MDL，世界坐标投影）。`Simple_Room` 的面分组：墙面（`WhitePlastic`：背墙、侧墙、墙角）、木墙板（`WoodWall`）；榜单 12 种彩色房间也只是改了这几种材质的颜色。
- **代码**：随机方案加 `walls`（材质目录列表，只能选本池的，`rsi_profile.check`）；`x5_layouts.py` 对 `Simple_Room*` 每个布局给墙面、木墙板各抽一种写进 `Room.appearance`（`SIMPLE_ROOM_SURFACES`，每米一个贴图重复）；`rsi_assets.layout_assets` 把 `appearance` 里的材质也算进资产，审计覆盖到墙面。平台随机方案抽屉加「墙面材质池」。`real2sim/materials/library.json` 用合并资产重建（`scene_parts.py --library`）：多了 469 种 Poly Haven 材质（原条目一个没变），之前平台上选不到新材质。
- **验证**：本机 `check.sh` 通过（`test_x5_layouts` 加了墙面断言），页面脚本 `node --check`。`5090-2:/data/robodojo-runs/rsi-merged-20261025/`（材质 + 新实例合并的资产）：训练池方案（`walls` = 336 种 Poly Haven 材质）生成 `stack_bowls_random` seed 1201 的 6 个布局，`audit` 通过，保持姿态跑完退出码 0、无不稳定、无 MDL 报错；头部相机最后一帧上角能看到不同的墙面（砖墙等）。
- **没做**：平台没部署、没在浏览器点过；地面踢脚线（`WhitePlastic2`）、窗户、灯没换；新材质的预览缩略图只在服务器的 `real2sim/materials/preview/`（不入库）。

### Aurora 分支本机复测（2026-10-25，尚未提交）

以下为用户提供的 Linux 工作站复测记录，标题及“尚未提交”等措辞保留该次复测结束时的状态；不是本 Windows 会话执行的仿真结果。用户随后已授权本会话将测试修复与记录提交、推送到本分支并提出 PR，提交状态见下方补记。

- 按用户要求检出并更新 `komari/feat-g1d-aurora-tactile@f5a86f7`，独立 worktree 为 `/mnt/data2/Codex-RoboRSR-grippers-20261022/branch_validation_20261025`。原 `integration` 的 `codex/g1d-grippers-local`、未提交文件与本地服务完整保留；等用户提出 PR 请求后再返回原工作区。本次不提交、不推送、不创建 PR。
- 首次 CPU 检查实际为 75 passed / 1 failed：`test_live_bridge_waits_and_rejects_stale` 的模拟生产者直接写最终 NPZ，读线程偶尔读到未写完的文件。正式 `tactile_worker.py` 已原子发布；仅修正测试夹具为临时文件写完再重命名，不放宽消费者完整性检查。修复后 `bash scripts/check.sh 961a9f5`：76 tests / 82 subtests、36 个 Python 文件 lint/format、任务清单、diff 检查通过；该竞态用例另连续 100 次通过。此测试修复与本条记录仍为本地未提交改动，远端原提交不能称为已含修复。
- 冻结模型审计及两套模型三个开度的真实指垫表面对齐通过；47 links、46 joints、75 meshes。远端 LFS 完整 ZIP SHA256 与原交付一致，1,345 个清单成员逐项校验通过。从 ZIP 取出六组原始轨迹，用本分支重新审计，均与记录一致；07/08 仍失败，06 仍不接受为完整触觉任务。
- 复用已有隔离 Isaac Sim 5.1 / IsaacLab 运行环境，在私有 Assets 副本执行本分支默认 30 mm / 20 g 的完整双手任务；154 帧、双侧约 120 mm 实际抬升、搬运、竖直放稳、释放与撤离的严格审计通过。原生轨迹 SHA256 为 `392b653760191e9e20c557bc5cf072030a028510e8e56837671b256968980dc6`。现有厂家 Python 3.10 仿真进程完成四路 Simple/FEM 154 帧，源摘要一致，9,351 个私有运行资源校验通过。没有连接真机、DDS、真实相机，没有修改共享环境或共享资产库。
- 本次重新导出无损 HDF5 与 LeRobot v3，独立读回通过：154 帧、16 维状态/动作、四路视频完整解码，抽取 5 帧的实际 Dataset 输入核对通过。另生成三路原始视角与四触觉同帧合成视频。私有数据库的本体登记/扫描通过，未动旧平台数据库和服务。
- 复测证据、失败原始日志、运行快照、媒体和分支保留凭据位于同级 `validation_20261025/`，不入 Git。当前工作站有非致命文件监听及相机插值告警，原始日志保留；回合完成凭据、严格状态审计和数据/媒体解码均通过。真机安装/TCP、当前硬件标定、双向软体耦合和触觉策略权重仍未验收。本次只新增默认回合，不把它扩大为宽块重跑或 54 项榜单全测。

提交补记（本 Windows 会话，2026-10-25）：按用户后续授权，将上述测试夹具修复与记录提交到 `komari/feat-g1d-aurora-tactile`，随后推送并向 `main` 提出 PR。本会话仅更改此测试的生产者写法，不修改消费者、业务逻辑或断言；Linux 工作站的原工作区、服务与未提交文件不由本会话操作。

本会话实际检查：`bash scripts/check.sh 961a9f5` 通过（76 tests / 82 subtests，36 个 Python 文件 lint/format、任务清单、diff 均通过）。额外在 Windows 系统临时目录连续调用该用例时出现一次读 NPZ 的 `PermissionError`，因此不能称本 Windows 会话的 100 次压力复跑通过；未扩大修改消费者或放宽断言。上文 Linux 100 次通过及仿真/导出结果均来自用户提供的工作站记录。

### Aurora 分支同步主线与完整资料上传（2026-10-25）

- 用户要求先同步远端更新，再上传本分支；明确确认包括完整原始 v3 ZIP（907,275,550 bytes）。已合并 `origin/main@961a9f5` 的 28 个新增提交，三处文本冲突保留双方内容。
- 原 ZIP 放在 `real2sim/embodiments/g1d_aurora_lite_ws/delivery/g1d_aurora_fusion_v3_20261024.zip`，通过专用 Git LFS 规则上传；SHA256 `4193982fd25d8ed307213938f6b8b8de09c4bc4b418fb09efc43f3813c1dd339`，含全部原始视频、轨迹、HDF5/LeRobot 样例、预览和原覆盖文件。此为用户明确授权的归档例外，解压输出和私有厂家运行包仍不直接入库。恢复命令见 `scripts/g1d/xense_gripper/INTEGRATION.md`。
- 合并发现主线新增 Dex1 `finger_coupling` 会被 Aurora 继承，但其开闭方向相反；在 Aurora mixin 中显式设为 None，保持交付控制目标。新增回归用例先复现继承 0.002 的失败，再验证修复，Dex1 原逻辑保留。
- 只做 CPU 合并检查，不安装或复测 Isaac/XENSE。`bash scripts/check.sh origin/main` 通过：76 tests / 82 subtests，lint、format、任务清单及 diff 均通过；结果见本体的 `delivery/sync_validation_20261025.json`。下一步沿用该分支，不修改主 worktree 的 `main`。

### G1-D + Aurora Lite / 四路 G1-WS v3 交付接入（2026-10-24）

- 用户确认采用 `g1d_aurora_fusion_v3_20261024.zip`，新增本体 ID 沿用 `g1d_aurora_lite_ws`。模型、CAD/夹爪参考、登记与校准边界、控制适配、完整任务/触觉/数据工具按覆盖包的目录归位。
- 对已有文件只合并 Aurora 增量；保留当前 `build_asset.py` 的场景外观设置，以及 README、数据契约、平台测试与本文件的后续更新。旧 `g1d` / `g1d_mobile` 保留。
- 新增 CPU 接入检查并纳入 `scripts/check.sh`；CI 只拉取新模型所需 LFS。补全 STEP/STP 大小写规则及冻结文件的 LF 规则，避免 Windows 检出改变 SHA。平台相对路径统一 POSIX，修复 Windows 下物体索引与生成器路径序列化失败。
- 来源、文件取舍和交付检查记录见 `scripts/g1d/xense_gripper/INTEGRATION.md` 及 `real2sim/embodiments/g1d_aurora_lite_ws/delivery/`。完整 evidence、HDF5/LeRobot 样例在本 worktree 的 `.cache/aurora-v3-delivery/g1d_aurora_fusion_v3_20261024/`，不入库；预览按项目规则忽略。
- 用户明确要求不再安装/复测 Isaac、XENSE 和整机仿真，按交付方已有证据交付。本机只做代码接入与交付检查；本地隔离 CPU 依赖在收到该指示前已启动准备，未部署共享资产、平台数据库或远程服务。
- 实际检查：`bash scripts/check.sh main` 通过（62 tests / 82 subtests，ruff lint/format、任务清单及 diff 检查通过）；215 文件输入摘要匹配，5 项新增接入检查通过。依赖审计发现隔离 venv 自带 pip 25.0.1 的已知问题，其他扫描包无已知问题；遵用户不再安装要求未升级 pip，原报告保存在 `.cache/aurora-dependency-audit.json`。
- 下一步入口为 `scripts/g1d/xense_gripper/INTEGRATION.md`；本体仍是候选装配，安装/TCP、当前机器零位/相机、材料/动力学和真实 SN 标定保持 pending。厂家专有运行包未包含，未将其加入 Git。

### RSI 任务物体新实例：ManiTwin 碗 7 个、带把马克杯 19 个（2026-10-25，`chongzheng/rsi-instances`，已合并进 `main`）

- **用户要求**：物体实例还没解决，和材质并行做（材质交给另一个会话，分支 `chongzheng/rsi-materials`）。
- **去重改了判据**：10-24 按尺寸去重，把 ManiTwin 的碗、杯几乎全当成"疑似原物"丢了。碗、杯彼此太像，尺寸和形状距离都分不开（同一模型两次采样的倒角距离 0.011–0.019，官方不同碗之间 0.013–0.024），改成按网格面数：和任何 RoboDojo 物体相同就不收。已下载的 938 个 ManiTwin 模型没有一个面数相同，GLB 里的贴图字节也没有相同的；新旧实例并排渲染能区分（红色马克杯放大比过）。
- **代码**：
  - `manitwin.py instances`：碗要求圆、敞口、碗心低，缩放到官方尺寸（直径 0.11–0.18 m、高 ≤ 0.08 m）；马克杯按把手外伸 / 杯口半径 1.4–2.3 认把手，转到把手朝 +x，缩放到高 0.069–0.082 m，在把手外缘往里 3 mm 处写官方同款的两个 `handle` 坐标系；元数据 `source.tasks` 记验证过的任务。`convert` 支持缩放和转向。
  - `x5_layouts.py`：新实例写了 `source.tasks` 的，只换进这些任务（马克杯也用于倒水，没验证装液体）。
  - `scripts/objects/instance_fit.py`：把实例摆成已完成状态（碗叠好、杯挂上），再作为 `--policy_module` 保持 60 步，用任务自己的判定逐项打印 `TASKFIT`。
  - 测试：`test_manitwin.py` 加碗、杯的合成点云；`test_x5_layouts.py` 加 `test_instance_tasks`。
  - `rsi/assets/imported.json` 加 `instances`（来源、缩放、转角、任务、池子、9 个剔除及原因）；`pools.json` 重扫。
- **结果**：碗 15 → 筛出 8 → 7 个（剔除带勺子的）；杯 85 → 筛出 27 → 19 个（剔除落桌倾斜 2.6° 的 1 个、挂不住的 2 个、只在部分挂钩上挂得住的 5 个）。分池：训练池碗 6、杯 15，考场池碗 1、杯 4。
- **验证**（`5090-2:/data/robodojo-runs/rsi-inst-20261025/`，资产 = `rsi-pool-20261024/Assets` 加 `rsi-assets-20261024/extra_inst`，剔除的在 `extra_rejected_inst/`）：落桌检查 35 个（底部原点，34 个 z≈0、倾斜 ≤ 1.5°）；叠碗 `stacked` 和竖直全为 1（含官方对照）；挂杯先用官方杯扫抬高 6–18 mm、6 个挂钩，8 组全 3/3（不抬高时官方杯也只挂住 1/3：挂钩插进了把手），再按 10 mm 让每个杯子轮流上 6 个挂钩，官方杯 0/4/8 都 6/6，新杯 19 个 6/6；用训练池方案生成 `stack_bowls_random`、`hang_mugs_random` 各 12 个布局，新实例占 12/36、23/36，`audit` 通过，保持姿态策略跑完退出码 0、`eval_time` 12、无不稳定。本机 `check.sh` 通过。
- **合并**：和材质分支一起合进 `main`；两边的新资产合在 `5090-2:/data/robodojo-runs/rsi-merged-20261025/Assets`（`rsi-inst-20261025/Assets` 加 `extra_mat`），`pools.json` 在这里重扫，结果正好是材质分支的材质加本分支的实例。之后生成 RSI 数据用这个目录。
- **坑**：评测客户端的回合数取 `EVAL_NUM` 和任务自带数的较小值，一个 seed 文件夹只会用到前 25 个布局，多出的环境重复跑 0 号；同一个 `ROBODOJO_RUN_ID` 会接着旧结果续跑。
- **没做**：装箱的车、牙刷、锤子、鞋（ManiTwin 没有锤子、牙刷，鞋只有 1 只拖鞋，要从 GSO 导；车的 `checkpoint` 要分出车头）；倒水的杯和白酒瓶（要验证装液体）；新实例没进共享库，也没部署平台。

### RSI 新资产：Poly Haven 材质 469 种（2026-10-25，`chongzheng/rsi-materials`，已合并进 `main`）

- **用户要求**：三套评测集用掉官方 504 种材质里的 494 种，训练池只剩 6 种、考场池 2 种；把 Poly Haven 的 CC0 PBR 贴图做成 MDL 材质补上。
- **代码**：`scripts/scenes/polyhaven_materials.py`（标准库，复用 `polyhaven_hdris.get`）：选 `surface_use` 为 floor / wall / object、非航拍、不属于 terrain / road / asphalt / bark / roofing 的贴图，去掉名字（去掉末尾编号后）和官方材质同名的（Stone_Wall、Plywood、Marble、Fabric_Pattern 共 9 个；官方是 NVIDIA vMaterials，145 种没有颜色贴图，没法按图像去重）；下载 1k jpg 的 Diffuse、arm（= ORM 通道顺序）、nor_gl，按官方 `material_0001/Ceiling_Tiles.mdl` 的 OmniPBR 参数写 `material_ph_<id>/ph_<id>.mdl`，写 `polyhaven_materials.json` 记来源；已有的目录跳过，可续跑。单测 `scripts/scenes/test_polyhaven_materials.py`（`check.sh` 跑整个 `scripts/scenes`）。`rsi/assets/imported.json` 加 `materials`，`pools.json` 重新生成。
- **结果**：候选 474 → 下载 469（5 个缺贴图：`book_pattern`、`church_bricks_02`、`leather_red_02`、`leather_red_03` 缺 Diffuse，`gravel_concrete` 缺 arm）→ 渲染检查剔除 0。分池：材质训练池 342 种、考场池 135 种（原来 6 / 2）；T0 仍 937 项，其余池子不变。
- **服务器**：材质在 `5090-2:/data/robodojo-runs/rsi-assets-20261024/extra_mat/Material/`（921 MB），下载缺图的记录在 `extra_rejected_mat/rejected.json`，脚本副本在 `mat_tools/`。共享库没动。
- **验证**：`5090-2` 隔离目录 `rsi-mat-20261025/`（`Assets/` = `rsi-pool-20261024/Assets` 加 `extra_mat` 的软链接合并；GPU 2）：一个 Isaac 进程里依次把每种材质绑到默认桌面和一块 4 m 地板上渲染（`render_materials.py`，只在运行目录），再按 Kit 日志里的 MDL 编译 / 贴图错误和画面亮度筛（`check_renders.py`）。另放两个故意做坏的对照（MDL 语法错、贴图路径错），日志都抓到了（画面分别是红色报错材质和纯白）；469 种都没有报错，联系表 `mat_check/sheet_*.jpg` 看过，纹理正常。偏白的 4 种（`polystyrene`、`white_stucco` 等）原图本来就是浅色。本机 `check.sh` 56 passed 1 skipped。
- **没做**：没用新材质生成 X5 布局、没跑任务；贴图尺度没按 Poly Haven 的实际尺寸（`dimensions_mm`）调 `texture_scale`，和官方材质一样用 1；房间变体（换墙面）。

### RSI 新资产：ManiTwin 干扰物 390 个、Poly Haven HDR 40 张（2026-10-24，直接推 `main`）

- **用户要求**：新的干扰物和实例最重要，去网上找现成的模型库；按"先 ManiTwin 补干扰物、Poly Haven 补 HDR、GSO 第二批"的顺序做。
- **调研**（细节见 `rsi/AGENTS.md` 第八节）：ManiTwin-100K（Apache-2.0，已放出 4259 个、484 类，GLB / USDZ 加抓取、功能点标注）；GSO（CC-BY 4.0，1030 个扫描件，鞋 254 个、玩具 147 个）；Poly Haven（CC0，521 个模型、997 张 HDR）；OmniObject3D、Objaverse-LVIS、Infinigen 可以补充。BEHAVIOR-1K、Lightwheel、ABO 的许可证不能用。**RoboDojo 的杂物库是在 ManiTwin 的工作目录里做的**（`Clutter/clutter.yml` 的 `source_root: /home/pc/ManiTwin/...`），所以导入 ManiTwin 必须去重。
- **代码**：
  - `scripts/objects/manitwin.py`：`select` 按尺寸筛、尺寸去重、规范类别名；`convert` 引用 ManiTwin 模型、关掉它自带的穹顶灯、落到 z = 0、生成减面的凸分解碰撞体、写元数据和描述。
  - `scripts/scenes/polyhaven_hdris.py`：按下载量取室内 HDR。
  - `rsi_assets.py`：`link` 用软链接合并资产目录；`scan` 改为统计**三套**官方评测集（`arx_x5/{0,1,2}`，各 54 个任务、2690 个布局；之前只算了 seed 0）。
  - `layout_manager.py`：不稳定时多打印一行 `Unstable actors`。
  - 测试：`scripts/objects/test_manitwin.py`（已加进 `check.sh`）、`test_link`。
  - `rsi/assets/imported.json`：记录 390 个物体、40 张 HDR 的来源和许可证。
- **结果**：ManiTwin 选出 765 个（尺寸不合适 1032 个、疑似 RoboDojo 原物 647 个，主要是球、瓶、杯、酒瓶、花瓶、眼镜、笔、碗）→ 转换 762 个 → 地面检查通过 450 个（多半失败是生成资产的"朝上"方向不是自然摆放姿态）→ 剔掉倾斜超过 1.5° 的剩 390 个。分池（官方资产加新资产）：T0 937 项；干扰物训练池 280 个、考场池 115 个；HDR 训练池 24 张、考场池 16 张。**材质成了新缺口**：三套评测集用掉了官方 504 种里的 494 种，训练池只剩 6 种。
- **服务器**：资产在 `5090-2:/data/robodojo-runs/rsi-assets-20261024/`（`extra_obj/`、`extra_hdr/`、`extra_rejected/`，ManiTwin 标注的 git 克隆 `mt_git/`）；运行目录 `rsi-pool-20261024/` 的 `Assets/` 是官方资产加这两个目录的软链接合并。共享库没动。
- **验证**：本机 `check.sh` 51 passed 1 skipped。`5090-2`：在桌面上做密集筛查（`stack_bowls_random` 80 个布局，每个 18–22 个干扰物，共 1597 次摆放），不稳定的只有 `correction_tape/00901`（6 次）和 `cutting_board/00900`（2 次），两者地面检查都有 1.7–2° 倾斜，据此加上 1.5° 门槛；剔除后重新做最终验证（25 个布局）：`eval_time` 25，没有不稳定的，没有报错，画面里新物体的贴图正常。
- **没做**：GSO（鞋、玩具）；材质（10-25 已补，见上一节）；房间变体；任务物体的新实例（还要补任务需要的元数据）；生成资产的稳定姿态自动修正（能救回一部分不稳定物体）。

### RSI 资产分池（规则 1 严格口径）+ 干扰物池、新实例、换色缩放接入 X5 布局生成器（2026-10-24，直接推 `main`）

- **用户要求**：先确定干扰物能用多少；房间不单独做，换桌面和墙面；HDR 和物体实例怎么补；然后按计划实现（先做分池和 T0 过滤，再接干扰物池、新实例、换色缩放；房间变体之后再做）。规则 1 的口径由用户选了**严格**：评测布局里用过的，加上没有官方数据的任务（`_random`、开放任务）配置里能抽到的编号，都算榜单保留资产。
- **代码**：
  - 新增 `scripts/datagen/rsi_assets.py`：`scan` 写 `rsi/assets/pools.json`（入库），包含 T0、官方训练见过的、训练池 / 考场池（按资产 id 哈希 7:3 切分，新资产加入不会改变已有资产的归属）；`audit` 检查一个布局目录有没有用到不该用的资产。只有 `scan` 用到 PyYAML，平台只读 json。
  - `rsi_profile.py`：方案加 `pool`（train / exam）、`color`、`scale`；可选项只列出所选池子里的资产和官方训练见过的。
  - `x5_layouts.py`：换实例时把池子里的新实例加进候选，并跳过所有 T0 编号；干扰物来自池子，杂物库的写成 cluttered 条目，其他类别的物体写成不带标签的普通 `Rigid` 条目；`RESTYLE` 白名单按任务、按物体类别打开换色和缩放（缩放后修正落桌高度、质量按 s³ 换算；有东西压在上面的、或压在别的物体上的不缩放）；训练池只能写 seed 1000–4999、考场池只能写 5000–9999；写每个布局前用 `rsi_assets` 检查一遍。
  - 平台：随机方案加池子切换、换色、缩放范围、本池新实例展示；`/api/rsi-options?pool=`。
- **严格口径下现成能用的**（官方资产 `5090-2` 扫描，T0 813 项）：房间只有基础的 2 个；材质两方都能用的 2 种，另有训练池 88 种、考场池 43 种；干扰物训练池 3 个、考场池 2 个（全是字母牌 b / d / j / o / r）；新实例训练池 4 个、考场池 1 个（全是 `Geometry` 的照片、垫块、显示器）；HDR 1 张。宽松口径（只算布局）时原本有 33 个干扰物、16 个新实例，几乎都被 `general_pickup` 配置的 85 个候选（另有 `pack_objects_into_box_random` 的锤子 07、鞋 12）占住。
- **查明的**：14 个房间是同一个 `Simple_Room` 的 12 种纯色加 2 个基础款，所以房间变体可以靠换墙面生成；HDR 在画面里基本看不到，引擎只支持换贴图和调亮度；刚体的 `visual` 只支持纯色（`visual_mdl_path` 只在流体里生效），纯色会盖掉物体原来的贴图；Hunyuan3D 试点（另一个人的运行目录）只做出 3 个没有纹理的形状，10-22 被叫停。
- **验证**：本机 `check.sh` 48 passed 1 skipped（新增 `test_rsi_assets.py`、`test_restyle_scale`，`test_x5_layouts` 和 `test_server` 改成合成分池），页面脚本 `node --check`。`5090-2` 隔离目录 `rsi-layouts-20261023`：严格分池、训练池方案生成 `hang_mugs_random` seed 1001 的 10 个布局（换实例、换色、缩放 0.85–1.15、3 个字母牌干扰物、88 种材质里抽的桌面和地面、灯光、相机角度）→ `rsi_assets.py audit` 通过 → 保持姿态的策略在 Isaac 里跑 10 个布局：退出码 0、`eval_time` 10、没有 PhysX 报错，最后一帧黑色像素占 4–10%（机器人底座）。画面里杯子是纯色、大小不一，字母牌在桌边。
- **没做**：房间变体（换墙面）；新干扰物和新实例；HDR 旋转和颜色；页面没在浏览器里点过；平台没部署。

### 办公室 `real_scene1_office`：墙面、柜子、椅子按照片返工（2026-10-24，`a13ddea` 快进进 `main`，短分支已删除）

- **用户要求**：把墙壁、各种柜子、椅子的建模和材质和真实拍摄对齐。
- **做法**：在 `reference/` 的照片上读像素，用 `cameras_scene.json` 的 SfM 相机反投影到已知平面或多视角三角化；颜色取照片 sRGB 中值当反照率（同实验室 10-23 的墙面做法）。逐项数值和依据在 `build_scene.py` 的 `LAYOUT[...]["src"]`，改前改后对照表在 `PLAN.md`「墙面、柜子、椅子」。
- **改了什么**（只动 `build_scene.py` 的背景部分，桌子、物体、灯光位置不变）：
  - 南墙：墙脚 y=−2.13；电视下深胡桃色腰线 0.83–0.935 m（原 0.95–1.01）、上方玻璃白板、下方带竖缝的墙板和踢脚线，x > −0.12 是通高墙板。
  - 窗台顶 1.09 m、窗槛墙 1.09–1.49 m（原 1.00 / 1.45，投影到 v0260、p4 落在窗台下方），北墙窗下墙板竖缝；东北角包柱改成浅米色。
  - 柜子：门面 y=−1.775（深 0.355，原 0.40）；x 0.30–1.28 玻璃展示柜（白框玻璃门从 0.835 m 起、隔板、下木门），x 1.28–2.00 木门高柜（原整柜止于 1.60），竖条拉手、内缩踢脚、程序生成的木纹贴图 `usd/textures/cabinet_wood.png`。机器人身后矮柜改成敞口木架；新增白色活动柜（x 2.03–2.53，p1、v0340）。
  - 椅子：两种参数化模型，网背办公椅 4 把、墙角的高背皮椅 1 把；墙角椅按三角化的五星脚移到 (1.25, 0.84)（原 (0.90, 0.65)），门边、女同事、西侧 b 三把按脚中心反投影微调。
  - 南墙主墙面厚度 0.12 → 0.10 m，`scene_parts.py` 现在把它识别为墙面（原来因浮点误差被排除）；`parts.json` 墙面 2 块，`library.json` 里办公室 `wall_light` 的颜色和 `tile_m` 跟着更新。
- **验证**：本机 `check_scene.py` 31/31；15 个真实相机用 `render_all.py`（Open3D）重渲染并和照片叠加核对，六个视角的对比图在本机 `real2sim/scenes/real_scene1_office/preview/rework_furniture_sheet.jpg`（不入库）；`scene_parts.py` 重新生成 `parts.json`；`make_manifest.py`；`check.sh`。本机环境：`uv` + Python 3.11，Open3D 需要 arm64 的 libusb（命令见 `REPORT.md`「复现」）。
- **Isaac 验证和部署**（10-24 下午，用户要求"提交更新并部署"）：`5090-2` 隔离目录 `/data/robodojo-runs/office-furniture-20261024-1605/`（GPU 0；`Assets/` 硬链接自 `lab-walls-20261024-1314`，只换 `Room/real_scene1/`）：`render_scenes.py` 正常（RTX 下网背半透明、木纹正常）；`g1d` seed 0 的拾取、叠块、量杯倒豆都是 `success_rate` 1.0、`eval_time` 1，头部相机画面看过。经用户同意装进共享库 `5090-2:/data/robodojo-assets/Assets/Room/real_scene1/`（和隔离目录 `diff -r` 一致；布局没动，`g1d`、`g1d_mobile` 的布局不引用背景路径），旧版备份 `/data/robodojo-assets/backups/backup-shared-room1-20261024-161130.tgz`。平台：部署机检出**没快进**（`server.py`、`index.html`、`HANDOFF.md`、`ASSETS.md` 上有别的会话没提交的改动，和要快进的提交重叠），只把 `main` 版本的 `real2sim/scenes/real_scene1_office/` 改动文件和 `real2sim/materials/library.json` 拷了过去，`preview/` 整个换成新版（15 视角对比图、5 个自由视角、返工对比表、RTX 封面、`scene.glb`）；旧的备份在 `/root/RoboDojo/.cache/office-before-furniture-20261024-161217/`。重启平台后扫描：办公室检查 31/0、51 张预览、封面和 3D 模型都是新的，未登录 `/api/info` 401。
- **部署机更新到 `main`**（16:58，用户同意）：`/root/RoboDojo` 从 `db77167` 快进到 `c1b601c`。别的会话的 28 个未提交文件先 `git stash`（备份 `.cache/ff-main-20261024-165752/local.patch`、`status.txt`），快进后 `stash pop` 无冲突（`server.py`、`index.html` 各一行本地改动保留）；办公室那 12 个文件先还原再快进，木纹 LFS 文件补实际内容。部署机 `test_server.py` 9 项通过，重启后扫描正常。
- **沙发、盆栽返工**（10-24 下午，用户："沙发和盆栽还是有点假"）：沙发外轮廓不变，改成圆角部件、一体靠背垫、带拉扣和滚边的长坐垫、细金属腿、斜靠的圆角抱枕，皮革贴图 `usd/textures/leather.png`；盆栽改成金钱树（34 根直立茎、成对双面小叶）和釉面锥形花盆。依据和数值见 `PLAN.md` 对照表最后两行。验证：`check_scene.py` 31/31；Open3D 叠图（v0160、v0260、p4）轮廓重合；隔离目录里 RTX 封面渲染两轮（第一轮皮革偏红、斑驳，调了色相、斑驳度和粗糙度），`g1d` 拾取 seed 0 `success_rate` 1.0、`eval_time` 1，头部相机画面看过。
- **西墙改磨砂玻璃**（2026-10-25，用户："办公室有门的那个墙面其实是磨砂玻璃"，短分支 `chongzheng/scene-office-furniture`，已快进进 `main`，`a6bb5b5`）：西墙隔断、门上亮窗、门芯原来是几乎全透明的 `glass`，渲染里西墙隐形；改成磨砂膜玻璃（`frosted`，不透明度 0.8）。门改成白框（边梃 5 cm、上下横档）、框内磨砂玻璃、下横档上方 14 cm 透明带、北边梃上不锈钢执手和刷卡器；门北侧的玻璃条改成米色墙板。代码在 `build_scene.py` 的 `add_west_wall`，数值和依据在 `LAYOUT["door"]`、`PLAN.md` 对照表「西墙和门」。验证：`check_scene.py` 31/31；`scene_parts.py` 重新生成 `parts.json`（墙面 2 → 3 块，多了门北侧米色墙板）；`make_manifest.py`；Open3D 重渲染 v0040、v0100、v0160、p0 和照片叠图，v0160 里门框、执手、刷卡器、透明带位置重合。`5090-2` 隔离目录 `/data/robodojo-runs/office-frosted-20261025-1018`（GPU 0）：`render_scenes.py` 封面渲染正常，另用临时改机位的副本（不入库）正对门和隔断拍了两张，RTX 下乳白半透明、白框、执手、刷卡器、透明带都对；没重跑 `g1d` 脚本演示（碰撞体没变，只改了可见网格和材质）。**装库和部署**（10:18，用户同意）：共享库 `Room/real_scene1` 换成这版（含 10-24 的沙发、盆栽，之前库里还是 16:12 的版本），旧版备份 `/data/robodojo-assets/backups/backup-shared-room1-20261025-101800.tgz`，与隔离目录 `diff -rq` 一致；部署机 `/root/RoboDojo` 从 `c3571f3` 快进到 `a6bb5b5`（别的会话 112 个未提交文件先 `git stash`，备份 `.cache/ff-main-20261025-101828/`，`stash pop` 无冲突），平台预览（`scene.glb`、RTX 封面 `render/sim.jpg`、15 张叠图和 5 张自由视角）已换，旧预览备份 `.cache/office-preview-before-frosted-20261025`，平台重启后 `test_server.py` 通过。不确定：p0 里门和隔断之间那一段能看到走廊地面，可能是反光或透明窗，按用户说法整面都做了磨砂；执手、刷卡器、透明带高度和门宽是估计值。`build_scene.py` 在本机重建时 `usd/objects/*.usda` 会因 trimesh 版本不同而面序变化（几何不变），提交前用 `git checkout -- usd/objects` 还原。
- **没做**：柜高 2.05 m、白板顶 2.40 m 在所有照片之外，是估计值；没对机器人头部相机调色。

### 吧台客区 + 第一个移动操作任务（2026-10-23/24，`chongzheng/scene-counter-lounge-mobile`）

- **用户要求**：把吧台旁边的客区（3 张照片：墙纸凹龛、圆桌、蓝色扶手椅）加进吧台场景，做成支持移动操作的场景。用户选定：从移动版分支开（已在 `main` 里）、加一个跨区任务并带脚本专家、椅子做成可推动的刚体、暂时没有卷尺，尺度按先验。
- **场景**（`real2sim/scenes/real_scene2_counter/`，细节见其 `PLAN.md`「左侧客区」）：新增 `fit_lounge.py`，三张照片 `reference/lounge{1,2,3}.jpg` 联合拟合三台相机、凹龛和圆桌（31 个像素约束，最大 12.7 px），尺度只来自「圆桌高 0.75 m」。凹龛宽 5.07 m，在吧台左木柱左侧；吧台左木柱宽从假设的 0.45 m 改为 1.0 m（两组照片的下限）。新增 `usd/lounge.usda`（墙纸和灯带贴图从照片透视校正、两张圆桌、垃圾桶、静态碰撞体），房间扩到 x = −9.84、y = −5.5，客区上方加两块顶灯（未标定）。椅子不在场景 USD 里，摆位写在 `usd/build_params.json` 的 `lounge.chairs`。
- **椅子** `Rigid/lounge_chair/00000`（蓝）、`00001`（白，只换颜色）：`scripts/objects/make_lounge_chair.py`，尺寸按照片估计，7 kg。`RigidObject` 新增布局键 `physics.mass_cap`（默认仍截到 0.5 kg，只有椅子放开），否则椅子 0.5 kg 一碰就飞。
- **env 配置** `g1d_counter_mobile`（`env_cfg/g1d_counter_mobile.yml`、`env_cfg/robot/g1d_counter_mobile.yml`）：移动版 G1-D，起点和升降柱同 `g1d_counter`，`body_states` 打开。`build_asset.py` 的 `SCENES` 加了它，`LOUNGE_TASKS` 给它生成客区任务布局（seed 0–2 各 10 个，按 `任务/seed/序号` 定种子）。
- **任务** `g1d_deliver_cup_to_lounge`：从台面拿塑料杯（`plastic_cup/00000`，\(|x|\) 0.06–0.16 m 随机左右、y −0.46 ~ −0.38），送到客区圆桌放下。判据（`delivered_checks()`）：杯心在桌心半径 −4 cm 内、高度没掉到地上、杯口朝上 15° 内、两个夹爪都张开。1500 步。
- **脚本专家**（`scripted_demo.deliver_cup_actions`）：侧抓（同倒豆的 `cup_grasp`；俯抓会把 30 g 塑料杯夹滑，第一次 GPU 运行就是空手开过去的），抬 8 cm，后退 0.7 m，挑椅子之间最朝向出发点的空档，开过去时把升降柱降到 0.07 m，转向圆桌，按到桌后实测的手–杯偏移放下，张开后沿手指退出再抬手。`Robot.drive` 是 `base_velocity` 的转向 + 直行控制器。
- **框架修复**：`func_parser.is_all_gripper_open` 归一化时没看 `gripper_move.sign`，G1-D 的 Dex1-1（sign −1，正向闭合）全开时算成 0，永远判不过；改成和 `obs_manager._to_gripper_scale` 一致。X5、Franka 的 sign 都是 1，结果不变。之前没有 G1-D 任务用到它。
- **验证**：
  - 本机：`check.sh`（ruff 0.11.7、41 passed 1 skipped、任务清单）；`build_scene.py` + `check_scene.py` 19/20（唯一 FAIL 是吧台站位的固定提醒）；`scene_parts.py` 重新生成 `parts.json`。
  - `5090-2` 隔离目录 `/data/robodojo-runs/counter-lounge-20261023-163001`（1 号卡，和 `owam-server` 共用；其余 7 张卡被另一位同事的 `gpu_hold.py` 占着）：两把椅子 `check_object.py` OK；脚本专家 `--env_cfg_type g1d_counter_mobile --seed 0` 前 4 个布局 4/4 成功（左手 1 个、右手 3 个），`_result.json` `success_rate` 1.0、`eval_time` 4。头部相机截图在本机 `output/deliver_head.jpg`（不入库）。
  - 没跑：完整 `build_asset.py`（服务器缺 `g1d_v4`/`unitree_ros`，布局用 `build_lounge_layouts` 单独生成），seed 1、2，`body_probe.py`，椅子被推开的专门测试。
- **10-24 下午补**（用户：顶部太假、3D 预览没更新）：房间加 3.20 m 穿孔木天花板、客区木吊顶做实并用自发光底面、白色石膏带和筒灯灯槽，木柱和木柜墙顶到天花板，顶灯挪到天花板（吧台画面会略暗一点）；改完隔离目录里送杯 seed 0 布局 0 仍然成功。平台 `/file` 对 `.glb` 和 `preview/` 下的渲染图改为 `no-cache`（原来的 `stale-while-revalidate` 让重建后的第一次打开总是旧模型），实拍照片和视频不变。
- **椅子进预览**：椅子在仿真里是布局刚体，不在 Room 里，所以场景渲染和平台 3D 预览原来没有椅子。`build_scene.py` 另写 `usd/chairs.usda`（和资产同一份几何，`make_lounge_chair.build()`，按 `build_params.json` 的标称摆位）和 `usd/preview.usda`（= `scene.usda` + 椅子）；`export_glb.py`、`render_scenes.py` 有 `preview.usda` 时用它。仿真用的 `real_scene2.usd`、`scene.usda` 不变。
- **椅子按照片重建结构**（10-24 傍晚，用户：结构和照片不一样）：原来是一条后背带加两段分开的矮扶手、下面一块平座板；改成平面 U 形、一圈软包底座托内嵌坐垫、一片连续弧形壳（顶边后高前低，两侧前段落到底座，后面一道圆角镂空）。各面组顶点分开，硬边不再被平滑法线抹掉；`build_scene.py` 的 `mesh()` 加 `normals=` 让预览椅子用资产的法线。隔离目录里重新生成资产，`check_object.py` 两把 OK。
- **共享资产库没动**：`lounge_chair`、新的 `Room/real_scene2`（带客区）、`Eval_Layout/RoboDojo/g1d_counter_mobile/` 都只在隔离目录里。装库要先征得用户同意；新 `real_scene2` 也会改变 `g1d_counter` 的画面左侧（原来是一面刷漆墙加挂画），吧台本身几何不变。
### RSI 协议初版 `rsi/AGENTS.md`（2026-10-24，直接推 `main`，只有文档）

- **用户要求**：① RoboDojo 评测用到的随机资产，生产数据时必须保留（第一规则；用户原话里的"拼车"按语音误识别理解为"评测"）；② 给 coding agent 能做泛化的操作（探索边际），避免押题、抄答案；③ 多 agent：一个改进数据、一个改进评测，二者对抗、不能互相通信；④ 写成类似 AGENT.md 的东西，给定模型、训练资源、评测平台就能跑 RSI。
- **内容**：三条硬规则；资产分三层（T0 官方保留、T1 考场池、T2 训练池，剩余资产 7:3 分 T2 / T1）；数据方、评测方、裁判（确定性脚本 + 人工）的权限和 seed 段；泛化算子表；成绩单只给分档的科目成绩（信息流唯一通道）；每轮流程、封存探针（画"轮数 → 泛化"曲线）、固定训练配方；裁判审计清单；`rsi/rsi.example.yaml` 输入配置；v0.1 缺口清单。
- **规则 1 的实测依据**（`5090-2` 上统计官方 `arx_x5/0` 全部 54 个任务）：房间 14 个全被评测用掉（基础任务 2 个，`_random` 12 个）；材质 504 种里评测用了 374 种，没用过的 130 种；杂物 193 个实例里评测用了 192 个（`_random` 191，`general_pickup` 145），官方训练数据里没有杂物；`_random` 独有物体实例 95 个；HDR 只有 1 张（基础任务也用）。所以房间和干扰物只能靠新资产做随机。
- **和现有代码的冲突**：10-23 提交的 `x5_layouts.py` 和平台随机方案能选到 T0 资产，违反规则 1，要先做裁判的 T0 统计和池子过滤（协议第八节 P0）。
- **验证**：只有文档；`check.sh`。
- **v0.2（同日，用户觉得 80% / 20% 不合适）**：看了官方仿真榜单（2026-10-24 版，61 个模型）后，难度调整改为用 Score（0–100 部分完成分）和相对本任务 L0 的保留率 ρ：ρ 的置信下界 ≥ 0.7 升级、上界 < 0.3 降级，每个科目、每个级别至少 30 回合，15% 回合留作 L0 锚点；L0 Score < 10 的任务算能力缺口，不调难度。依据（Gen-Std → Gen-Rand，按 Score）：OpenWAM-α 33.16 → 8.26（ρ 0.25，成功率 25.56% → 4.11%），Pi-05 20.93 → 5.82（0.28，14.89% → 1.44%），VPP2 0.40，Simate-beta 0.73，WorldScape-R²SI 0.84，HexaShell-Preview 0.81。总分：OpenWAM-α 17.18 / 11.92%（第 14），Pi-05 11.44 / 6.93%（第 22）。

### 上海实验室 `real_scene3_lab`：墙面去掉照片贴图，角落放一台 G1-D（2026-10-23，`chongzheng/scene-lab-walls`）

- **用户要求**：墙面不要用贴图；角落里那台 G1-D 直接放一个 G1-D 模型过去。
- **墙面**（`build_scene.py` 的 `WALL_SPANS`、`MULLIONS`、`DOOR`、`BOARDS`、`TV`）：四张墙面照片贴图和抠图补色的代码都删了。墙改成纯色饰面，颜色取照片中值：南墙是玻璃隔断（不透明绿灰色，带竖框）、白墙和一扇白门；北墙是白墙，挂一块黑边白板和一块黑屏；东西两面是深灰吸音板加踢脚线，挂黑边白板；西墙 y > 2.5 是白墙。位置由照片像素反投影得到，表格见 `PLAN.md`「墙面（v3）」。墙面网格带米制 UV，平台换材质可以用；材质库里实验室的 4 个照片墙条目换成 `paint`、`acoustic` 两个（`library.json` 只动了这几条，`test_server.py` 跟着改）。
- **G1-D**：用 `real2sim/robot/g1d_v4/g1_d_full_dex1_v4.urdf` 做正运动学（两肘 1.2 rad，手臂下垂和 p3 一致；其余关节为 0），材质和仿真里的 G1-D 一致（机身、手臂、底盘、升降柱哑光金属，夹爪、头、腰、轮子和前臂一圈黑色，规则抄自 `env/robot_manager/robot_config/g1d.py`；URDF 里的蓝白色是 CAD 默认色，仿真不用），每个网格单独抽稀，共约 5.5 万面，写成二进制层 `usd/g1d.usdc`（1.2 MB，走 LFS），由 `/Background/Furniture/g1d_0` 引用，位置 (−2.22, 2.92)、面朝 −Y。碰撞体是底盘、上身两个盒子；它不是 articulation，不会动。位置取 p3（p2 偏 0.4–0.7 m）。构建要多装 `fast-simplification`（`PLAN.md` 里的命令已更新）。
- **验证**：`check_scene.py` 24/24；`preview.py` 的四张照片对比里墙面分段和照片对得上，p3 视角下 G1-D 和照片里的机器人重合；`parts.json` 的墙面识别为 6 块（玻璃、门、竖框按名字排除）；本机 `check.sh` 46 passed、1 skipped。10-24 在 `5090-2` 隔离目录 `/data/robodojo-runs/lab-walls-20261024-1314/`（GPU 0；`Assets/` 拷自 `lab-furniture-install-20261022-085858`，只换了 `Room/real_scene3/`）验证：`render_scenes.py` 正常；`g1d_lab` seed 0 的拾取、叠块、量杯倒豆都是 1.0、`eval_time` 1。之后（用户指出夹爪不是黑的）改了 G1-D 的材质，重跑 `render_scenes.py` 和拾取 seed 0（1.0），头部相机里角落那台和工作的那台外观一致。10-24 经用户同意装进共享库 `5090-2:/data/robodojo-assets/Assets/Room/real_scene3/`（和隔离目录 `diff -r` 一致，布局没动、也没有布局引用旧墙面路径），旧版备份 `/data/robodojo-assets/backups/backup-shared-20261024-135419.tgz`；平台部署机只替换了这个场景 `preview/` 下的五个展示文件（旧的在 `/root/RoboDojo/.cache/lab-preview-before-walls-20261024-135438/`），部署机检出没快进（上面有别的会话没提交的改动），所以平台抽屉里的检查数和材质库条目还是旧的。
- **坑**：其他卡满载时，只给 `--device cuda:N` 不够，RTX 会用所有卡渲染，Vulkan 报显存不足后进程卡死（要 `kill -9`）。要加 Kit 参数 `--/renderer/multiGpu/enabled=false --/renderer/multiGpu/maxGpuCount=1 --/renderer/activeGpu=N`；`scripted_demo` 通过 `main(extra_kit_args=...)` 传。

### 数采平台：上游榜单任务的泛化随机方案 + X5 布局生成器（2026-10-23，直接推 `main`）

- **用户要求**：把上游引擎已经支持的随机项都在数采平台上实现（RSI 的"课程"要覆盖 RoboDojo `_random` 测的泛化）。实现是另一个会话（不在 Cursor 里，可能是 Codex / Claude 桌面端）在 `../RoboDojo-wt-leaderboard` 里写好、没提交的；用户让本会话接手，本会话审查、修了相机扰动、补测试和 GPU 验证后提交。
- **RoboDojo `_random` 测的**（统计官方 `arx_x5/0` 全部 12 对布局）：没见过的实例（编号和训练分开）、7–20 个干扰物、11–12 种房间、每 30 个布局 22–30 种桌面 / 地面材质；所有任务都随机位置，部分随机朝向。官方训练数据（34 个基础任务）全是同一房间、同一桌面、无干扰物，所以只用官方数据就是 clean-to-random。榜单不测的：灯光、HDR、相机扰动、指令改写、尺寸、颜色。
- **代码**：
  - `scripts/datagen/rsi_profile.py`：方案格式和校验（只用标准库，平台服务端直接导入）。方案写 `real2sim/benchmark_tasks/<任务>/randomization.json`，入库。
  - `scripts/datagen/x5_layouts.py`：上游引擎只回放布局、没有生成器，这个脚本以官方评测布局为结构模板重抽：位置 / 朝向只在任务配置自己的范围里（配置写死位置的物体不动，从不照抄评测布局）、实例只抽基础任务的训练侧编号（`_random` 独有编号不抽）、干扰物（杂物库、稳定面、±30°）、房间、桌面 / 地面 MDL 材质、HDR 和亮度、额外灯光（类型、数量、亮度、色温）、头部相机角度扰动（生成 git 忽略的 `env_cfg/arx_x5_rsi_<任务>.yml`、`env_cfg/camera/rsi_<任务>.yml`）。写 `split.json`（默认 train）和 `report_<任务>.json`；seed 0 拒绝。
  - 平台：任务抽屉新增「泛化随机方案」（只对上游任务显示）：物体（位置、朝向、实例，列出训练侧和留出编号）、干扰物、房间与表面（房间、MDL 材质搜索选择）、背景 HDR、灯光、相机扰动；`PUT /api/items?kind=task` 的 `randomization` 字段经校验后写文件；`GET /api/rsi-options` 给选项。`datagen/SKILL.md` 新增「上游榜单任务（dual X5）的随机布局」。CI / `check.sh` 加 `shapely rtree transforms3d`。
- **本会话改的**：相机扰动去掉平移，只留角度；方案里写 `pos_m` 会被校验拒绝。原因：`stack_bowls_random` 10 个布局开 ±2 cm / ±2° 时第 2、4、6 号并行环境头部相机最后一帧 45–98% 全黑（两次都是这三个）；同样布局不开相机扰动、或只开角度，10/10 正常。没查清平移撞到了什么（相机在 z = 1.308，`camera_stand` 顶部约同高，"只往上抬"也照样黑）。另外：ruff 格式、`.gitignore` 两条、`test_camera_configs`、页面文案。
- **验证**：本机 `check.sh` 46 passed 1 skipped（含 `test_x5_layouts.py` 4 项、`test_randomization_profile`），页面脚本 `node --check`。`5090-2` 隔离目录 `/data/robodojo-runs/rsi-layouts-20261023/`（另一会话 13:55 建的，本会话沿用；`Assets` 软链接到 `leaderboard-baseline-20261022` 的官方资产，`Eval_Layout` 单独）：
  - 另一会话已在 seed 901 给 54 个任务各生成 5 个布局；本会话用它的 `rsi_check.py` 复查：留出实例 0 次，干扰物 8–15 个；"和评测布局同位置"的都是配置里写死位置的物体（相机支架、棋盘棋子、麻将槽位等），"超出范围"都是锚点不在原点的槽位物体（检查脚本判得粗，官方布局本身也报）。`play_stacking_toy` 有 2 处足迹重叠没细查。
  - 端到端：全部随机项（14 个房间中的 Simple_Room、20 种桌面 + 20 种地面材质、HDR 亮度 600–1400、1–3 盏灯含平行光、8–15 个干扰物、相机 ±2°）→ `x5_layouts.py --seed 905 --count 10 --split val` → 保持姿态的策略（`rsi_hold.py`，不带 `--allow_train_layouts`）跑 10 个布局：退出码 0、`eval_time` 10、10/10 无黑块，画面各不相同。
- **没做**：页面没在浏览器里点过（只有服务端单测和语法检查）；没部署到 `5090-2:/root/RoboDojo`。
### 真机 Motus eps380 倒豆模型：按真机数采对齐后的仿真评测，夹爪两指联动（2026-10-23/24）

- **用户要求**：只测 ModelScope `Vincia/WanVlmMask-G1DPourBeansAbsQpos64Eps380-38500`（64 步动作块），并把仿真对齐到真机数采：杯子放正、按真机摆放、高度 / 站位 / 手臂起始对齐、左杯换成真机的杯子、豆子数和指令都和真机一致；夹爪闭合要和真机一样居中。
- **模型和数据**：权重在 `rtx5090-180:/home/work/wenjj/ckpt/`（sha256 与 ModelScope 一致 `6570e7f0…`），训练配置 `/home/work/wenjj/MotusV2/configs/g1d_pour_beans_wan_vlm_mask.yaml`（stitch aspect、qpos、gdr 1、ratio 8）。训练数据 `dsw-4:/mnt/data/datasets/G1D-自采数据/G1D-PourBeansEps380`（OSS 挂载，380 条，5 种中文指令，原文在 `meta/tasks_zh_backup.jsonl`，`t5_cache/` 里有每句的嵌入；`tasks.parquet` 已被改成纯编号，缓存里另有 "0"–"4" 的嵌入是后来生成的，不是训练用的）。右手拿方口高杯倒，左手拿目标杯；160 条灰色粗口杯、160 条蓝色杯子。
- **推理和仿真都在 `5090-6`**（`rtx5090-180`、`5090-2` 的卡被 `wuruihan` 的 `gpu_hold` 占满）：模型、`wallx` 环境、`/root/miniconda3`（只有 base + `RoboDojo`）、Isaac 缓存（`~/.cache/ov`、`~/.local/share/ov`、`~/.nv`）、可编辑安装的 `office-beans-100-20261019/third_party` 及其 `.git/modules` 都经 `rtx5090-180` 走内网拷到同路径（`/home/work`、`/root/miniconda3` 是指向 `/data/motus-eval-20261023/` 的软链接）；运行目录 `5090-6:/data/robodojo-runs/motus-lab-beans-20261023/`。服务端 tmux `motus380-8111`（GPU 2，`127.0.0.1:8111`，`run_server380.sh`），一个服务端可供多路仿真（锁只包推理），请求里的 `prompt` 切换指令；仿真 `run_policy3.sh`（GPU 3、GPU 2）。
- **对齐**（脚本和中间结果在本机 `output/motus_lab_eval_20261023/eps380_align/`，不入库）：
  - 站位：头部画面桌子轮廓拟合（`fit_stand.py`，1.3 px）→ 机器人比 `g1d_lab` 后退 20.1 cm、左移 5.8 cm，升降柱 0（真机读数约 0）；新机器人配置 `g1d_lab_eps380`，env 配置 `g1d_lab_beans_eps380`。380 条里只有 37 条是这个站位（`stand_cluster.py` 按首帧桌子轮廓分），只从这些里取布局。
  - 桌高：头部拟合说 0.865 m，但真机手最低到 0.845 m、腕部相机桌沿说 0.775 m、左手抓灰杯要落在杯身上约 0.80–0.85 m，互相矛盾（单视角尺度不可测；头部相机外参和标定包 `g1d_calib_toolkit` 一致，关节零位偏差 < 0.6°）。取 0.83 m（布局里 `table_lift`）；0.865 时仿真手在 2.5 s 被桌面挡住（`compare_tcp.py`）。**要准需要现场卷尺量桌高、底座到桌沿距离、桌子尺寸。**
  - 杯位：各真机回合左右夹爪开始闭合那一帧的 FK 夹持点，两杯偏航 0；起始姿态：各回合自己的首帧（`remote_policy.py --init-joints`，`--settle-steps 45` 让豆子落稳）。
  - 杯子：目标杯按对应回合换成灰杯 `plastic_cup/00001`（Ø66×89 mm，烟灰透明）或蓝杯 `plastic_cup/00002`（杯口 72、杯底约 48、高 99 mm，不透明锥形），都按真机头部画面量（`measure_cups.py`），`make_water_cup.py --gray/--blue` 生成，只装在 5090-6 运行目录的资产覆盖层，**未装共享库**。高杯豆子 1740 颗（真机 ep0 装到 131 mm、85%；672 颗只到 50 mm），椭球顶部必须低于杯口，否则会弹出几颗、开局检查不过。
  - 判据：目标杯是圆杯时，插件按圆柱 / 锥形杯腔计数（`collect_office_beans.in_round_cavity`，布局可写 `cavity`）。
- **夹爪两指联动（G1-D 默认行为改变）**：真机 Dex1 两指同一电机对称开合，仿真里 URDF 的 mimic 导入时丢了（两指独立 PD），夹偏的杯子会停在先接触的一侧。试过 `PhysxMimicJointAPI`（API 挂上了但在 GPU 管线不生效，已撤回）；改为控制层联动：`robot_class/g1d.py` 的 `finger_coupling = 2 mm`，`robot_manager.control_robot` 每个物理步把闭合方向的两指目标限制在较慢那根 + 2 mm 以内（20 N 推力）。验证：空夹两指差 < 0.001 mm、闭合 10 步；回合中夹杯两指差中位数 < 0.1 mm（`remote_policy` 轨迹新增 `fingers`）。**已有的 G1-D 仿真数据是两指独立时采的。**
- **结果**（seed 14：12 条灰杯回合 + 8 条蓝杯回合，各用自己的指令，平滑 σ2 / 块间 4，64 步块）：任务判据、严格判据都是 0/20；3 个回合开局豆子已洒（起始手离高杯太近）。灰杯 10 个有效回合里 6 个有豆子进杯，共 1523 颗（最多 575），洒 6924；蓝杯 7 个有效回合只有 1 个进杯（53 颗），其余基本不倒。两句指令都是"倒半杯"，而任务判据和严格判据要求全倒，判据和指令不一致。之前几轮（标准布局、eps222、未对齐）都是 0 颗进杯。
- **验证**：本机 `check.sh` 52 passed 1 skipped（新增圆杯 / 锥形杯腔单测）；GPU 上只在 `5090-6` 跑了夹爪空夹探针（`scripts/g1d/gripper_probe.py` 只在运行目录）和上面的评测，没有重跑脚本专家或采集。
- **腕部相机"穿模"（10-25）**：原因是 `DEX1_RGB` 的近裁剪 3 cm，拿着的杯壁离镜头约 2 cm 被裁掉，看起来像镜头钻进杯里（物理上没有：镜头在杯口外上方，外壳有 70 个碰撞网格）。3 cm 是为了挡住镜头前 10–12 mm 的实心镜头罩（`housing_visual`，真机上是开孔的；1 cm 近裁剪时整块黑盘挡在画面中央）。改为：`robot_config/g1d.py` 生成时把两手 `housing_visual` 设为不渲染（碰撞保留），近裁剪 1 cm；画面底边仍露出一点相机机身（19–22 mm），没隐藏，因为头部相机要看到它。中间试过把灰杯换成不折射材质（v2），用户觉得不如原来真实，已恢复原来的折射玻璃。
- **三轮 seed 14**（灰杯回合，进杯 / 洒出合计）：v1 玻璃 + 近裁剪 3 cm：1523 / 6924；v2 不折射：958 / 9877；v3 玻璃 + 近裁剪 1 cm（`eps380_seed14_v3`）：1119 / 6067（9 个有效回合）。蓝杯三轮都基本不倒。同一布局轮间差别很大（单次运行）。目前最干净的一回合是 v3 布局 7（真机 ep134）：进杯 410、洒 23。
- **没做**：每布局只跑 1 次（方差大，比较要重复）；"倒半杯"的判据还没写进插件；共享库没装新杯子和布局；`remote_policy.py` 的初始化路径没有单测；夹爪联动后没回归其他 G1-D 任务。

### G1-D 夹爪开合速度按真机限速（2026-10-23，`chongzheng/robot-g1d-gripper-speed`）

- **起因**（OpenWAM 深圳倒豆子评测时用户发现）：仿真夹爪合得比真机快。统计 30 Hz 数据里开度从离开张开位到停住的帧数：深圳仿真 500 条中位数 2 帧（0.07 s），峰值每帧 0.149 个行程；真机 G1D 遥操作的硬杯任务（PourBeans、PourBeansPlus、PickKettle）6–8 帧（0.20–0.27 s），峰值约每帧 0.048 个行程，软物体（毛绒、瓶子）30–46 帧。
- **原因**：仿真只受 URDF 的 0.2 m/s（每指）限速，两指合拢约 400 mm/s，等于 Dex1-1 官方最快档（全行程最短 0.24 s，速度可调 50–375 mm/s）；真机控制器每步把目标裁到当前位置附近（遥操作 ±0.18 rad 再加平滑，见「剩余事项」的真机设定），实际约每指 0.064 m/s（0.048 × 44.5 mm × 30）。
- **改动**：`env/robot_manager/robot_config/g1d.py` 夹爪执行器加 `velocity_limit_sim=0.064`（桌面版、移动版共用）；README 控制对齐一节注明。刚度、力上限没动（PR #24 另改刚度 3.6e4 / 55 N，与本改动相邻，合并时只需保留两边）。
- **验证**：本机 `scripts/check.sh` 通过（41 passed，1 skipped）。`5090-2` 隔离目录 `/data/robodojo-runs/gripper-speed-20261023/{old,new}`（代码取自生成深圳数据的 `1d39c9d`，new 只多这一行），`g1d_pour_beans_cup_or_mug` / `g1d_shenzhen_lab_beans_collect` 种子 500 布局 0–7 的专家采集：通过 6/8 对 6/8（同一批布局通过），合爪中位数 2 帧 → 5 帧（0.17 s），比真机 6–8 帧仍略快（真机还有平滑滤波）。
- **影响**：已有数据集（办公室 100 条 ×2、深圳 500 条）都是旧速度采的，不改；新采的数据用新速度。用旧数据训练的策略在新速度下评测会更早抬手，需要时再测。

### 农夫山泉塑料瓶 `plastic_bottle`（2026-10-23，分支 `z/objects-plastic-bottle` → `chongzheng/objects-plastic-bottle-cap`）

- 平台建模任务 `real2sim/objects/Clutter/plastic_bottle/00000/spec.json`（徐鹤鸣 10-23 登记）。脚本 `scripts/objects/make_plastic_bottle.py` 生成本体和 5 个泛化变体 `00001–00005`（`metadata.json` 写 `variant_of`）。Clutter 物体经同一个 `RigidObject` 加载（`layout_manager.py` 的 `type: cluttered`，干扰物也是这样放的），所以 Clutter 版必须是单刚体。
- 用户 10-23 补充：总高 227 mm（不含盖）；登记的 328 g 含水，按空瓶 18 g 建模（算作含盖）；开口直径指外径。**平台登记里质量还是 328 g**，要在平台改成 18 g 并加「总高」，否则平台的质量核对不过；加了「总高」后在 `save()` 的 `measurements_mm` 里补同名键。
- **加盖、贴图标签（用户 10-23 下午：瓶身没贴图太假，要盖子，最好能拧）**：
  - 标签：`scripts/objects/bottle_label.py`（只依赖 numpy + PIL）程序绘制环绕贴图：照实拍的红白版式画品牌字、山水图、成分表、条码、细字、二维码；变体是虚构品牌（清泉、山涧、橙动、汽泡、果园）。贴图打进 usdz（`textures/label.png`），标签网格带 faceVarying UV，OmniPBR `diffuse_texture`。需要中文字体：`5090-2` 系统里没有，用 `apt-get download fonts-noto-cjk && dpkg -x` 解压到运行目录，`CJK_FONT_DIR` 指过去（本次在 `/data/robodojo-runs/plastic-bottle-cap-20261023/fonts/x/usr/share/fonts/opentype/noto`）；Mac 上自动用系统黑体 / 宋体。
  - 盖子：所有瓶子加竖纹螺旋盖（防盗环、圆角顶），尺寸照片估计（本体外径 33 mm、高 16.4 mm，带盖总高 228.5 mm）。Clutter 版盖子并进瓶身刚体，碰撞体多一个 24 边圆柱；`visual.color_parts` 改成 `["cap"]`，`rt_color` 只换盖子颜色（标签是贴图，换成纯色就白画了）。
  - 能拧：同一次运行另写 `Assets/Object/RoboDojo/Articulation/plastic_bottle/0000{0..5}`，经 `ArticulationObject` 加载。结构：`/root/body`（挂 ArticulationRootAPI，`fix_root_link` 要求它在第一个连杆上）→ `cap_twist` 旋转关节（0–720°，正向 = 从上看逆时针 = 拧开，阻尼驱动 3e-4 N·m·s/°）→ `nut` 连杆 → `cap_lift` 棱柱关节（PhysxMimicJointAPI 跟随 twist，每圈升 2.7 mm 螺距）→ `/root/cap`。盖子和瓶身碰撞用 FilteredPairs 关掉。拧到 720° 停住，盖子不会脱离瓶身（关节物体里做不到真的拿下来）。`metadata.json` 的 `articulation` 段写了两个关节开 / 关的位置（rad、m）。
- 验证（`5090-2` 隔离目录 `/data/robodojo-runs/plastic-bottle-cap-20261023/`，`run.sh` 一键重跑；本机 `bash scripts/check.sh` 通过，ruff 0.11.7 三个脚本全过）：
  - `measurements_mm` 开口直径 29.8、底部直径 64.7，误差 0；
  - `check_object.py` Clutter 六件全部 OK（倾角 0°）；
  - 新脚本 `scripts/objects/check_bottle_cap.py`（CPU 物理，和 TaskEnv 默认一致；GPU 管线下 `SingleArticulation.dof_properties` 会报 cuda tensor 转 numpy 的错）：六件自由落地都直立、关节保持 0；固定底座、给 `cap_twist` 速度目标 π rad/s，六件都转到 12.566 rad、`cap_lift` 5.40 mm，反转回到 0；
  - 新渲染图和 `check.json` 已同步到部署机 `real2sim/objects/Clutter/plastic_bottle/0000{0..5}/preview/`，平台上能看到带盖带标签的样子。
- **已装共享资产库**（用户 10-23 同意）：把隔离目录 `ovl/` 里验证过的产物拷到 `/data/robodojo-assets/Assets/Object/RoboDojo/{Clutter,Articulation}/plastic_bottle/`，`diff -r` 一致。两处都是新目录，没覆盖任何东西，所以没做备份。12 件的 `metadata.json`、`description.json` 也手动放进了平台镜像 `/root/RoboDojo_assets/Assets` 的同一路径（整体重刷镜像时会被删，重刷要从 `5090-2` 共享库拉，不再从 21196）。没改 `Clutter/clutter.yml`；`prepare_bean_pour_layouts.py` 扫目录选干扰物，以后重新生成布局时这 6 个瓶子会进候选，已有布局不受影响。
- 未做：`rt_scale`、`rt_color` 要在任务层 `build_asset.py` 的 `RANDOMIZE` 打开；没有拧盖任务，也没验证 Dex1-1 夹爪真能拧动（盖子外径 33 mm，夹爪能夹住，但要靠转腕，阻尼和是否需要摩擦得在任务里调）；登记里「去盖、中空」的要求已被用户新要求取代，平台登记文本没改。

### 真机 Motus 倒豆模型在上海实验室的仿真评测（2026-10-23，`chongzheng/eval-motus-lab-beans`）

- **用户要求**：测 ModelScope `Vincia/WanVlmMask-G1DPourBeansGenEps222-AbsQpos16-30000` 在上海实验室（`g1d_lab`）仿真里倒豆子的成功率，只用量杯，不加干扰物。推理代码在 `rtx5090-180`（端口 34134，内网 `192.168.0.180`）的 `/home/work/wenjj/InferenceMotusV2`。
- **模型**：Motus（Wan2.2-TI2V-5B + Qwen3-VL-2B），从 380 条的 checkpoint 出发，用 222 条**真机**"倒豆子"数据后训练 3 万步；16 维绝对关节位置 qpos（内部顺序 [L7, LG, R7, RG]），每块 16 步、30 Hz；头部左眼加两个腕部相机拼成 T 形 384×320；指令只有"倒豆子"（带 T5 缓存）。权重已在服务器 `/home/work/wenjj/ckpt/` 下，预训练模型在 `/home/work/cache/wx1469573/pretrained_models/`；eps222 训练集在服务器上已经不存在。`/home/work` 盘 100% 满，自己的文件放根分区。
- **做法**（不改 InferenceMotusV2）：
  - 服务端 `rtx5090-180:/root/motus-sim-eval-20261023/serve_motus_sim.py`（本机副本在 `output/motus_lab_eval_20261023/`）：包装 `MotusV2PolicyAdapter`，按真机 `inference_motusv2_PourBeans.sh` 的串行路径调用（5 步去噪、插值 1、不开 RTC），协议同 OpenWAM 的 `serve_g1d.py`；Python 用 `/home/work/wenjj/envs/wallx`，要加 `PYTHONNOUSERSITE=1`（否则 `/root/.local` 的 torchvision 冲突）。每个实例 20 GB，加载约 2 分钟，单次推理约 0.63 s。`run_server.sh <gpu> <port>`，tmux `motus-sim-<port>`，已全部停掉。
  - 仿真端 `5090-2:/data/robodojo-runs/motus-lab-beans-20261023/`：入库 `scripts/g1d/remote_policy.py`（原来只在 OpenWAM 运行目录，原样收入）和 `env_cfg/g1d_lab_beans_collect.yml`（实验室 + 1/600 s）；`Assets/` 是 `lab-furniture-install-20261022-085858` 的硬链接副本（那份机器人 USD 按当前代码重建、在 `g1d_lab` 用脚本专家验证过；共享库的 `Robots/g1d` 和它不同），`Eval_Layout` 单独复制。两台机器内网直连。
  - 核对过的对齐：真机图像是 RGB（`process_images_and_observations` 做了 BGR→RGB），仿真也是；真机夹爪 0 闭合到 5.4 全开，夹住量杯读约 4.1–4.5，仿真夹住量杯实测开度约 0.75（≈4.05），线性换算一致；真机数据里腕部相机没有左右互换。没对齐的：真机起始姿态是双臂下垂（例如肘部约 −0.3），仿真默认肘部 −1.0、夹爪贴近头部相机。
- **结果**（seed 0 的 20 个布局各 1 次；`build_asset.py` 的标准布局：左手倒、两杯 45° 偏航、672 颗豆子、无干扰物；步数上限 1200）：任务判据 0/20，严格判据 0/20，矮杯里 0 颗。7 个回合碰洒了豆子（共 178 颗，最多 98 颗，是高杯被碰倒）。失败模式每个回合都一样：左手伸向高杯后夹爪闭合在空处（开度 < 0.5 持续 537–1098 帧），从来没有夹住过杯子；右手从不闭合。`_result.json` 三份，`eval_time` 合计 20。逐回合 JSON、轨迹 npz 和截图在本机 `output/motus_lab_eval_20261023/`。
- **验证**：本机 `check.sh` 41 passed 1 skipped（`test_server.py` 的 `g1d_lab` 现在对应两个 env 配置）；单回合探针后再跑全量，三路并行（服务端 GPU 1/4/5，仿真 5090-2 的 GPU 2 两路、GPU 7 一路），约 85 分钟。
- **没做**：每个布局只跑了 1 次（SKILL 要求 3 次；结果是 0 颗入杯，重复意义不大）；没试真机起始姿态、10 步去噪、夹爪阈值等变体；没有和脚本专家在同一批 20 个布局上对照（10-22 只验证过布局 0）。

### 场景中文名（2026-10-23，直接推 `main`）

- 用户要求所有场景都显示中文名。五个重建场景的 `scene_spec.json` 加 `name`：办公室升降桌、茶水间吧台、上海实验室（10-23 由公司实验室改名）、学校实验室绿幕桌、深圳实验室（同登记名）；平台扫描读它，卡片标题用中文名、下面一行小字是目录名。显示优先级：登记名 > `scene_spec.json` / 导入场景 `platform.json` / RoboDojo 自带场景的名字 > 目录名。给 coding agent 的建模任务文本和 SKILL 都要求新场景在 `scene_spec.json` 写 `name`。验证：`check.sh` 42 passed（`test_repo_scan` 断言五个场景都有名字）。

### 研究资产入库 + 界面按 UI UX Pro Max 优化（2026-10-22，直接推 `main`）

- **用户要求**：用 UI UX Pro Max skill 优化「研究资产」页；用户选定先把这块入库再改（它原来只在部署机上，是杨开元会话未提交的改动，见下一节）。
- **入库**：部署机工作区的平台改动（`prototype` 类型和扫描、研究资产页，以及同批的"导入的仿真场景"卡片和任务 `implementation` 进展显示，数据不存在时不显示）、`real2sim/prototypes/` 的元数据 / 文档 / 校验 JSON、`scripts/prototypes/`；`preview/`（图、视频、15 MB GLB）照 `.gitignore` 不入库，部署机上原样保留。
- **界面**（skill 推荐的模式：画面优先 + 按类别筛选，避免"层级弱、无筛选"；它推荐的衬线字体和学术配色没用，保持平台统一风格）：顶部搜索 + 分类分段按钮（带数量）；按「官方外形参考 / 柔性数值原型 / 待建模实物」分段，每段一句说明这类是什么、缺什么；卡片统一封面底色，封面角标 3D / 视频数，状态用图标 + 短标签（不只靠颜色）再附完整状态句，摘要限三行，底部「已核对 N · 待完成 N · 记录 N」；没图的卡片封面压成一条"还没有模型或图片"。抽屉：顶部状态、摘要、3D、预览图、视频网格（带文件名）、能力核对拆成「已核对」和「未完成 / 待核对」两组（按结论里有没有"未 / 待 / 无 / 候选 / 不同 / 非实测"分）、接下来要做、来源、检查记录按钮。导航图标改成和物体资产不同的 `layers`。
- **部署**（21:57）：入库时发现部署机上 5 个研究资产文件 21:51 又被那边的会话更新（软块 Isaac FEM 检查等），另补提交 `e02c2e6` 收进最新版再部署。部署机快进到 `e02c2e6`：已入库且内容逐字节一致的未跟踪文件先删再快进，`preview/` 不动；平台两文件不再是未提交改动；HANDOFF 合并时保留了部署机新增的两节绿桌任务笔记（刘小凡），删掉了重复的研究资产原文。备份 `.cache/deploy-ra-20261022-215710/`（含未提交改动 patch 和研究资产目录打包）。部署机上还有别的会话未入库的 `real2sim/scenes/g1d_green_table*`、`real2sim/tasks/g1d_green_table_five_stage`、`real2sim/tasks/g1d_behavior_pick_place_20261022`，没动。重启后部署机 `test_server.py` 8 项通过。
- **验证**：本机 `check.sh` 42 passed（新增 `test_prototypes`：七项都在、都不进任务、检查记录不能越出资产目录）；`scripts/prototypes/test_assets.py` 3 passed 2 skipped；本机浏览器看了列表、筛选、搜索、无结果、抽屉。

### 极光 Lite、通用法兰与柔性研究资产同步（2026-10-22，杨开元会话；原文取自部署机 HANDOFF，10-22 晚入库）

- 用户授权先核对官方模型、传感器及法兰，再同步到平台。定向部署本机独立分支提交 `ac31ff5`：平台新增 `prototype` / "研究资产"页，7项：极光Lite、通用法兰、布片、软块、长条拉链布袋、塑料包装袋、线缆；不进入任务对象/机器人候选。最近操作署名"杨开元（agent 代更新）"。
- 当前官方Aurora STEP与本地原件逐字节同源；USD/MJCF为静态整体几何，无运动/碰撞指垫或视触觉。CAD内G1-WS与当前单独WS模型版本名不同；无实机序列/SDK/逐设备标定。小面缺失和CAD包围盒/官网标称差异保留待核。
- 用户kaiyuan2提供的通用法兰与官网字节不同，但BRep双向差体积均为0，外径68、厚22.5mm与图纸一致。未取得G1-D腕端接口图/孔位与固定变换，没有套用旧Dex1安装或替换机器人。
- 布片和软块为已跑过独立数值测试的原型；袋/线缆仅reference登记、无原图文件及几何模型，明确待建模。厂商原件/派生USD、STL与MJCF放 `/data/robodojo-assets/research/yangkaiyuan-20261022/`，25文件约141MB，逐文件hash核对，目录只供root读取。没有安装共享Assets或修改默认夹爪/训练。
- 批次 `/data/robodojo-runs/research-assets-sync-20261022/`：`deployment.json`、`server_runtime_check.json`、`private_manifest.json`；备份 `backup-20261022-122100/`。
- 下一步：取得G1-D腕端安装接口和实机传感器序列/SDK标定，验证安装、运动、指垫接触/触觉后再考虑任务接入；三个实物参考仍需原图与尺寸/材料采集。

### 场景换材质 + 升降桌（2026-10-22，直接推 `main`）

- **用户要求**：场景建好后提取地板、墙面、桌面材质，每个场景可选别的材质替换；桌子都是升降桌，可调高度。用户选定：平台固定组合 + 数据生成时随机两层都做；材质库 = 场景提取 + 资产库 504 个 MDL；**调桌高时机器人不动**（站位和升降柱不变，测高度泛化）。
- **提取** `scripts/scenes/scene_parts.py`（usd-core，本机跑）→ 各场景 `usd/parts.json`：按形状找地板（顶面 z≈0 的水平网格）、墙面（沿房间轮廓高于 1.5 m，排除玻璃 / 门 / 框）、桌面（桌子分组里顶面 = top_z 的薄网格），记材质和每个网格的 UV 尺度；升降：伸缩接缝（build_params `legs.split_z`，没有就取半高）以上的网格上移，跨接缝或止于接缝的拉伸；桌上的场景物体跟着上移。五个场景都提取到三类表面；吧台 `real_scene2_counter` 是落地柜，标为不升降。办公室墙面只剩一面实墙（其余是玻璃隔断和窗）。
- **材质库** `real2sim/materials/library.json`（入库）：523 个 = 19 个场景材质 + 504 个 MDL，按名字分 12 类；按表面过滤（用户 18:53 指出地板和墙面能选到同样的材质）：`appearance.ROLE_CATEGORIES` 规定地板（木材 / 瓷砖 / 石材 / 织物 / 塑料）、墙面（涂料 / 瓷砖 / 石材 / 木材 / 纸 / 织物 / 塑料）、桌面（木材 / 石材 / 塑料 / 金属 / 玻璃 / 瓷砖）可用的类别，场景提取的材质只给原来的表面，室外和"其他"（轮胎、反光片、屏幕、水）都不给；`/api/materials` 每项带 `roles`，选择器按它过滤，保存和 `appearance.py` 都按同一规则拒绝；可选数量地板 242、墙面 279、桌面 354；缩略图 `real2sim/materials/preview/`（375 张，不入库，部署机要同步）。在 5090-2 用共享库 `Material/` 生成（读图要 4–5 分钟）。
- **运行时** `env/scene_manager/objects/room_overrides.py`，`Room` 加载后调用（只有布局 Room 带 `appearance` / `table_lift` 时）：建 UsdPreviewSurface（贴图按原真实重复尺寸缩放 UV）或 MDL（OmniPBR 世界坐标投影，每 1 m 重复）并换绑；桌子网格加最外层变换（上移或按底部拉伸），在 PhysX 解析前完成。不改 Room 资产，所以不用动共享库。
- **布局** `scripts/scenes/appearance.py`：读 `parts.json` + 平台 `appearance.json` + 材质库，把选择写进布局 Room（`appearance`、`table_lift`、`appearance_choice`、`table_height`），物体随桌面平移（按当前 `table_height` 算差，重复应用不会累加）；`--randomize --seed` 时有随机池的部分每个布局按文件名可复现地抽。`build_asset.build_layouts` 在场景有 `appearance.json` 时自动套固定设置（没有就和以前完全一样）。
- **平台**：`/api/materials`；场景扫描带 `parts`、`appearance`；建好的场景抽屉加「外观与桌高」：每类表面一个固定替换（单选）和随机池（多选，可含原材质），复用任务向导的搜索选择器（缩略图、悬停预览、按类别中文名搜）；固定桌高、随机桌高范围（限行程 0.62–1.27 m）。保存时服务端校验材质 id、行程、不升降的场景，再写 `appearance.json`（全空则删除）。
- **3D 预览**（用户随后要求：Web 3D 预览加换地板、墙面、灯光、桌板）：抽屉 3D 预览下方加控制面板：地板 / 墙面 / 桌面各一个材质选择器（同一个搜索选择器），桌高滑块（按 `parts.json` 上移、拉伸同一批网格；预览模型里桌上的摆件按位置识别后一起上移，仿真 Room 里没有这些摆件），灯光亮度、色温、主光方向。MDL 用底色缩略图近似（1 m 重复），没有 UV 的网格（办公室桌面）现场按法线投影生成米制 UV。「设为固定设置」把预览选择填进「外观与桌高」（再保存才写 `appearance.json`）；「载入固定设置」反向。灯光只影响预览，没写进场景（仿真灯光在 USD 里）。
- **验证**：本机 `check.sh` 40 passed（`test_appearance.py` 4 项、`test_server.py` 新增外观设置校验）。5090-2 隔离目录 `/data/robodojo-runs/scene-appearance-20261022-1807/`（资产覆盖层：只有 `Eval_Layout/RoboDojo/g1d_lab` 是新目录，其余软链共享库，未改共享库），`g1d_lab` 布局目录 7（地板 MDL 橡木、墙深圳墙色、桌面 MDL 大理石、桌高 0.90）和 8（地板吧台水磨石贴图、墙实验室北墙照片、桌面 MDL 桃花心木、桌高 0.70），拾取、叠块各 2 回合：画面里材质都换上、桌子升降对、物体停在新桌面上、无 PhysX 报错；成功率 0.90：叠块 0/2、拾取 0/2；0.70：叠块 1/2、拾取 0/2；原布局对照：叠块 2/2、拾取 1/2（机器人不随桌高变，脚本专家按原桌高调）。第一次在 GPU 4 上跑报 PhysX GPU 内核启动失败，GPU 4 当时被别人占到 30/32 GB，换 GPU 5 后同样布局无报错。平台本机起服务用浏览器走了一遍：选材质、保存、刷新后保留、3D 预览换材质 / 灯光 / 桌高（实验室、办公室）。
- **没做 / 风险**：材质缩略图有 172 个 MDL 没有（只有打包通道图或没有底色图），选择器里显示图标、预览里用灰色；场景提取的贴图材质 UV 尺度换到别的网格上是按"每米 UV"换算的，照片墙贴到别的墙上会被拉伸重复，只适合做测试（实验室四面墙 10-23 起已是纯色）；`appearance.json` 的随机池还没接到 `run_batch.py`，采集前要手动对布局目录跑 `appearance.py --randomize`；共享库评测布局没有重新生成（各场景都还没有 `appearance.json`）。
- **部署**（18:47）：`5090-2:/root/RoboDojo` 快进到 `ea2ea54`，做法同下面「删除场景」（另一会话未提交的平台改动和 HANDOFF 三方合并无冲突，合并前备份 `.cache/deploy-overlap-20261022-184728/`），缩略图拷进 `real2sim/materials/preview/`，重启后部署机 `test_server.py` 7 项通过。注意部署机上学校场景有另一会话未提交的几何改动（U 形绿幕），而 `parts.json` 是按仓库版本提取的，那边入库后要重跑 `scene_parts.py`。
- **文档**：`real2sim/scenes/SKILL.md`「换材质和升降桌」+ 接入步骤 4；CONTRACT「仿真配置」加一条：外观和桌高随布局记录，评估要用同样设置的布局。

### 数采平台：RoboDojo 自带场景（2026-10-22，直接推 `main`）

- **用户要求**：把 RoboDojo 自身的场景也放进场景重建页。
- **做法**：`scan_benchmark_scenes` 读 `env_cfg/scene/*.yml`（标准库逐行解析），记为 `robodojo_default`、`robodojo_conveyor`（`auto.source = "robodojo"`）：房间、桌子尺寸 / 高度 / 材质、HDR 背景，以及按 `_task.yml` 的 `scene_config` 选用它的上游任务（default 52 个，conveyor 2 个）。场景页在孪生卡片下面单列「RoboDojo 基准场景」，抽屉只读（配置、任务链接、重渲染命令），没有验收和删除（服务端 409）。总览的验收计数和 G1-D 任务向导的场景选择器不含它们。
- **封面**：`render_scenes.py` 现在也接受 `env_cfg/scene/<name>.yml`，用环境自己的 `Room` / `Table` / `Background` 类从 `Assets/` 生成，输出 `real2sim/benchmark_scenes/<name>/preview/render/sim.jpg`（`preview/` 不入库）。`5090-2` 隔离目录 `/data/robodojo-runs/bench-scene-render-20261022-1359/`（GPU 6，资产软链共享库只读）两个都退出码 0；default 是悬空桌板（上游桌子本来就只有桌面），conveyor 是空房间（传送带由任务生成）。
- **验证**：`test_server.py` 新增扫描断言（桌高 0.765、`random: False` 解析成布尔、conveyor 两个任务、无 G1-D 任务）和删除 409；本机 `check.sh` 35 passed；本机起服务用浏览器看了场景页和抽屉。
- **部署**（14:05）：`5090-2:/root/RoboDojo` 快进到 `ed49a76`，做法同下面「删除场景」：另一会话未提交的平台改动和 HANDOFF 三方合并无冲突（合并前备份 `.cache/deploy-overlap-20261022-140459/`）；两张封面拷进 `real2sim/benchmark_scenes/`；重启后扫描 8 个场景（含两个基准场景）、研究资产 7 项，部署机 `test_server.py` 6 项通过。

### 榜单基线：OpenWAM、pi0.5 官方 checkpoint 全量评测（2026-10-22，进行中，没改仓库代码）

- **20 天计划**：① 基线 + 失败归类；② 把 `scripts/datagen` 推广到 dual X5（CONTRACT 补 X5 语义），Coding Agent 按"差距 × 好写程度"给任务写专家（特权状态 + cuRobo，验收严于 `run_reward`）；③ 第 1 轮数据 → dsw-4 微调 → 内部验证集评测；④ 两轮对抗：训练分布内大范围采样找失败、聚类、在失败附近生成难例，加 DAgger 式恢复数据；⑤ 官方 seed 终评。官方评测布局（`Eval_Layout/RoboDojo/arx_x5/0`）只在基线和终评各用一次，迭代看内部留出集。
- **关键事实**：
  - 官方训练数据 = HF 数据集 `RoboDojo-Benchmark/RoboDojo` 的 `data/RoboDojo/<任务>/arx_x5/data/`，34 个基础任务各 100 条（单任务 28–46 GB，共约 1.3 TB）；dsw-4 已有完整副本 `/mnt/data/datasets/RoboDojo`（另有 `RoboDojo_ee_lerobot`）。
  - **54 个榜单任务里 20 个没有官方数据**：12 个 `_random` 变体，加 `align_blocks`、`classify_objects_by_language`、`general_pickup`、`pick_from_conveyor_by_image`、`pour_by_language`、`solve_equation`、`stack_blocks_by_language`、`store_tools_in_toolbox`。这是自生成数据最大的抓手。
  - 官方 checkpoint 也在同一 HF 数据集的 `ckpt/RoboDojo/<模型>/`：pi0.5 是 `Pi_05/RoboDojo-sim-arx_x5-joint-{0,1,2}/59999`（joint）；OpenWAM 是模型仓库 `OpenWAM/OpenWAM-Alpha-Sim-RoboDojo`（`checkpoint_step_60000`，24.8 GB，ee）。
  - 仓库里没有任何榜单任务的脚本专家，只有 `env/planner_manager/curobo_planner.py`。
- **运行目录** `5090-2:/data/robodojo-runs/leaderboard-baseline-20261022/`：代码是本机 `a694da5` 的 `git archive`（`REVISION`）加本机 XPolicyLab `53c5da5`（rsync），`third_party` 软链接到 `office-beans-random-20261019`。`Assets/` 是 HF 官方 Assets 的干净副本（15365 个文件，经 hf-mirror 逐文件下载，最后 521 个从 dsw-4 经本机中转），不是 `/data/robodojo-assets` 共享库（那份有 G1-D 改动）；下载后跑过 `utils/update_embodiment_config_path.py`（不跑会缺 `Robots/x5/curobo.yml`，而 `robodojo.sh eval` 照样返回 0）。checkpoint 在 `ckpt/`，软链接进两个策略的 `checkpoints/`。
- **环境（5090-2 上新增）**：`openwam-bench`（克隆 `openwam`，再装 vendored OpenWAM；原 `openwam` 指向 G1-D 分叉，给真机服务用，没动）；pi0.5 的 uv 环境在运行目录 `XPolicyLab/policy/Pi_05/openpi/.venv`（JAX 0.5.3，uv 在 `.uvtool/`，依赖经本机 `ssh -R` SOCKS 隧道装，隧道已关）；tokenizer 预置在 `~/.cache/openpi/big_vision/`；conda base 装了 `pyyaml`（Pi_05 服务端脚本用 base python 读 yaml）。Isaac 要 `OMNI_KIT_ACCEPT_EULA=YES`（`smoke_one.sh`、`bench.sh` 已设）。
- **下载坑**：5090-2 连不上 huggingface.co、GitHub、PyPI。hf-mirror 要 `HF_HUB_DISABLE_XET=1`；`snapshot_download` 和超过 1000 个文件的 `list_repo_tree` 会卡住（分页链接指回 huggingface.co）；git LFS 经镜像全部失败。可行的是按文件列表 `hf_hub_download`（`dl_prefix.py --list`）。
- **验证**：两个模型 `stack_bowls` 各 1 回合：OpenWAM 成功（314 s），pi0.5 成功（193 s），`_result.json` `eval_time` 1。
- **全量评测（13:40 启动）**：`bench.sh` → `robodojo.sh benchmark --eval-num native --tasks-file leaderboard_tasks.txt`（54 个任务，去掉 `g1d_*`）。OpenWAM run id `baseline_openwam_20261022`，策略 GPU 0/2/3，仿真 GPU 4/5/6（策略 25 GB，不能和 Isaac 同卡），预计每组约 16 h；pi0.5 run id `baseline_pi05_20261022`，GPU 7 单卡串行，实测 `align_blocks` 50 回合约 25 min，全量约 22 h。汇总在 `smoke_results/<run id>.md`，日志 `logs/bench_*.log`。GPU 1 是 `owam-server`（另一会话）。
- **16:35 按用户要求停止**（两个 `bench.sh` 会话组和 4 个残留 Isaac 客户端都已结束，GPU 0、2–7 释放，`owam-server` 未动）。停止时完整跑完的任务（成功率 / 回合）：OpenWAM `imitate_sorting_sequence` 0/50、`pour_by_language` 0/50；pi0.5 `align_blocks` 0/50、`arrange_largest_number` 0/25、`arrange_largest_number_random` 0/25、`build_tower` 0.48/50。中途停下的（`_result.json` 只有部分回合，不能当基线）：OpenWAM `play_tic_tac_toe` 0.1/40、`classify_objects_by_language` 0/30、`play_stacking_toy` 0/20；pi0.5 `classify_objects` 0.1/10。续跑：pi0.5 是串行模式，每个任务写一次 `smoke_results/baseline_pi05_20261022.json`（已有 4 个 PASS），同一 `bench.sh` 加 `--resume` 即可跳过；OpenWAM 是并行模式，summary 只在全部结束时写，停止时没有生成，`--resume` 不起作用，要从 `leaderboard_tasks.txt` 去掉已完成的 2 个任务另做任务列表。中途停下的任务都会整任务重跑。

### 数采平台：物体删除（2026-10-24，`chongzheng/object-delete`）

- **问题**：物体抽屉只有「删除登记」，被任务用到的物体根本没按钮；而且只删 `spec.json`，`real2sim/objects/<id>/` 下的照片和渲染图还在，扫描会把物体又列出来。
- **做法**：`DELETE /api/items?kind=object` 整目录删除（登记、照片、渲染图）；被任务使用、有泛化变体、有生成脚本或已在共享资产库的物体回 409（这类物体在代码和共享库里，要在仓库删）。抽屉按钮改名「删除物体」，上述情况显示为不可点并在悬停提示里写原因。只有管理员可以删（非管理员回 403，抽屉里不显示按钮），和删除场景一致。
- **验证**：`object_delete_blocker` 五种情形单测、前端脚本 `node --check`；没有端到端点击验证，`ruff` 本机未装所以没跑 `check.sh` 的 lint 段。

### 数采平台：删除权限与失败回滚（2026-10-25，`chongzheng/object-delete`）

- **审查发现**：成员直接调 `DELETE /api/items` 就能删本体、任务、评测登记（服务端只拦了场景和物体）；输入框保存失败后界面仍显示未保存的值；`removeDoc`、`unregisterTask` 删除失败没有提示。
- **做法**：`delete_blocker(kind, doc, user)` 统一把关：已接入仿真的本体（`auto.in_code`）和代码里已实现的任务（`auto.module`）回 409；评测只有管理员或提交人能删（403）；`episode` / `prototype` 页面没有删除入口，只允许管理员（403）。前端 `onFieldChange` 在保存失败时把值放回去并重绘（若期间已有更新的编辑则不动那个字段）；`removeDoc`、`unregisterTask` 失败弹 toast，不再关抽屉。
- **验证**：`test_server.py` 新增本体 409、评测成员 403、episode 成员 403，全部通过；没有端到端点击验证回滚。
- **部署**（18:41）：`5090-2:/root/RoboDojo` 从 `a6bb5b5` 快进到 `c5bfc22` 并重启服务（端口 8765，未登录 DELETE 回 401，新页面脚本已生效）。部署机上另一会话未提交的 `HANDOFF.md`、`README.md`、`ASSETS.md`、平台 `index.html` / `server.py`（各一行的 `generation_command`、`Garment/Articulation` 改动）已三方合并回去，无冲突，仍是未提交状态；合并前的备份在 `.cache/deploy-overlap-<时间>/`，全部未提交改动的 patch 在 `.cache/deploy-dirty-<时间>.patch`。拉取时用了 `GIT_LFS_SKIP_SMUDGE=1`（LFS 下载超时），所以这次新增的 Aurora 机器人 STL / STEP / zip 在部署机上是 LFS 指针；要在那台机器上跑 Aurora 需先 `git lfs pull`。
- **未做**（审查里的其余项）：深链接（刷新丢抽屉）、抽屉焦点陷阱、照片上传/删除触发整库重扫、批量质检逐个请求、驳回不强制选问题标签、删除改软删除（回收目录）、登录锁定可被恶意触发。

### 数采平台：管理员删除场景（2026-10-22，直接推 `main`）

- **用户要求**：场景重建里加删除，只有管理员能删。
- **做法**：`DELETE /api/items?kind=scene` 非管理员回 403；未建模的场景整目录删除（`registration.json`、照片、预览一起删，原来只删 `registration.json`，照片目录留着，下次扫描又冒出来）；已建模的场景（有 `*.usd` / `usd/`，或在 `build_asset.SCENES` 里）回 409，提示在仓库里删。场景抽屉的「删除场景」按钮只对管理员、且对未建模场景显示（已登记未建模和只有照片的都有）；原来人人可见的「删除登记」去掉了。
- **验证**：`test_server.py` 新增成员 403、已建模 409、整目录删除三条；本机 `check.sh`（ruff 0.11.7 经 `uvx`）35 passed；页面脚本 `node --check`。
- **部署**（13:22）：`5090-2:/root/RoboDojo` 从 `4f8cccf` 快进到 `fafb475` 并重启服务。部署机上有另一会话（杨开元）未提交的「研究资产 / prototype」平台改动、学校场景改动和 HANDOFF、ASSETS 笔记：平台两文件三方合并无冲突；`HANDOFF.md` 两边段落都留，`ASSETS.md` 实验室、深圳行取主线、学校行取部署机版本；仍是未提交状态，等那边入库。合并前的四个文件备份在 `.cache/deploy-overlap-20261022-132146/`，全部未提交改动的 patch 在 `.cache/deploy-dirty-20261022-132106.patch`。没用 `git stash`：部署机跳过 LFS smudge，stash 回来会把 `real_scene4.usd` 写成指针。新提交里 7 张实验室贴图（LFS）从本机 rsync 了实体。部署机上用系统 python 跑 `test_server.py` 6 项通过，研究资产 7 项仍在。
- **注意**：部署机上删除的是工作区文件；已入库的场景（如 `real_scene5_shenzhen_lab` 的登记和照片）被删后部署机检出会变脏，需要入库同步。

### 深圳实验室倒豆 500 条（2026-10-22，`chongzheng/datagen-shenzhen-beans` 快进进 `main`，已上传 ModelScope）

- **用户要求**：深圳实验室场景；高量杯倒进矮量杯或白色塑料马克杯；2–5 个无关干扰物；500 条；多卡并行。看过首批视频后改为：红豆 300–600（原 100–300）、抓取要稳、两杯不能碰。
- **数据集** `g1d_shenzhen_pour_beans_cup_or_mug_500`：500 回合、199697 帧、30 Hz，XPolicyLab 官方转换器出的 LeRobot v3.0；目标矮量杯 257、马克杯 243；豆数 320–594；干扰物 2/3/4/5 个各 140/123/119/118 回合。私有仓库 `ChongZheng/g1d_shenzhen_pour_beans_cup_or_mug_500`。registry 已登记。
- **代码**：任务 `g1d_pour_beans_cup_or_mug`（指令按目标说 “…into the short measuring cup / the white mug.”）；插件 `scripts/datagen/plugins/g1d_pour_beans_cup_or_mug.py`，验收 `all_beans_retained_any_target_v2`：结尾 30 帧全部豆子在目标杯腔内、两杯 ≤ 15°；整回合两杯杯身不接触（圆柱采样 + 马克杯把手）、目标杯 ≤ 15°、干扰物位移 ≤ 2 cm，违规即结束回合；`scripts/objects/plastic_mug_shape.py`（马克杯尺寸、原点偏移、杯腔）；`scripted_demo.cup_to_cup_actions` 按杯种取夹持高度、下倾角、杯腔，源杯先在原地升到倒豆高度再横移（旧版 43% 回合在搬运时擦碰）；布局 `scripts/g1d/prepare_bean_pour_layouts.py`（复用办公室采样器；马克杯把手朝持杯手指方向 ±40°；豆数目标 350–630，因颗粒网格采样实际 320–594；干扰物从 19 类 Clutter 直立放在桌后部和两侧，离杯 ≥ 0.16 m）；env 配置 `g1d_shenzhen_lab_beans_collect`；`run_batch.py --per-gpu N`，以及进程位多于分片时空队列 pop 崩溃的修复。
- **运行** `5090-2:/data/robodojo-runs/shenzhen-beans-20261021`，`output/datagen/shenzhen-beans-v2`，代码 `1d39c9d`（rebase 前的提交号，内容同 `main` 上的对应提交）。GPU 0/6/7 每卡 3 进程，6.3 小时，751 回合 505 合格（67%；矮量杯 262/372、马克杯 243/379；左手 246/364、右手 259/387）。不合格：结尾条件 143、目标杯倾斜 87、两杯接触 15、干扰物被碰 1。100–300 豆的 v1 批次（199 回合 156 合格）已弃用。
- **验证**：导出读回 500/199697；`audit.py` 500/500 重算验收通过，布局互不相同（`final_validation.json`、`sha256.json`、`coverage.png`）；开环回放 seed500、seed509 各 4 条：手臂状态偏差 ≤ 2e-3 rad，6/8 复现合格，1 条豆子落点不同（190 颗留在源杯，颗粒物理开环发散），1 条稳定帧 29/30；上传后 `upload.py verify` 逐文件核对 sha256。本机 `check.sh`。
- **没做**：55 N 夹爪 A/B（另一会话 `c575c8e`）合格率同为 16/28，没合入；指令只有一种英文说法。

### 实验室场景 `real_scene3_lab` 的家具建成立体模型（2026-10-22）

- **用户要求**：实验室场景要把桌子、椅子、柜子也重建出来，而不是只贴图。原来只有操作台是几何体，房间里的家具都压平在四面墙的照片上。
- **做法**：相机沿用 `fit_photo.json`，在照片上手标落地点（桌脚、椅脚、柜底边），反投影到地面；p1、p4 都拍到的铁皮柜做三角化。尺寸取常见规格，再用照片核对。`build_scene.py` 新增 `FURNITURE`（15 件：北排 3 张工位桌 + 4 把网椅，西侧 2 张工位桌 + 2 把网椅，活动柜带纸箱，两门文件柜，木面边桌，圆咖啡桌）和六种参数化模型，放在 `background.usda` 的 `/Background/Furniture/`。这样 Room 入口、`render_scenes.py` 的取景（只对准 Background、Lights 以外的部分）和平台的「隐藏房间」都不用改。碰撞体都是盒体或 ≤ 50 点的凸包。墙面贴图把家具的投影区域膨胀 21 px 后抠掉，按行中值补色；操作台那块原来用 Telea 修补，也改成按行补色。
- **照片之间的矛盾**（写进了 `PLAN.md`）：p2 的相机在画面边缘外推不准，按它算西墙转了约 13°，远处高度也偏低约 5 cm，所以西侧家具按与墙平行摆放。北排最右那张桌子按 p3 摆在 y≈3.05，p4 却要求 y≈2.45，相差 0.6 m；p4 左缘这一块用 `HAND_MASK` 手标多边形抠掉。圆桌只在 p4 出现，高度按 0.75 m 假设。
- **验证**：`check_scene.py` 24/24。新增 5 项：碰撞体 ≤ 64 点、家具落地、不进墙、离 G1-D 根节点 1 m 以上且离操作台 0.3 m 以上、家具之间不重叠；原来的站位等检查改成只看操作台。`preview.py` 的叠图用绿色画家具边线：p1、p3、p4 误差约 10 px，p2 约 20 px。本机 `check.sh` 29 passed 1 skipped；`export_glb.py` 输出 205 个网格、2.9 MB。`5090-2` 隔离目录 `/data/robodojo-runs/lab-furniture-20261022-003901/`（GPU 4）只载场景跑 `render_scenes.py`：退出码 0，没有报错，家具正常渲染、有投影。带机器人的脚本演示见下面「装共享库」。
- **平台**：部署机 `/root/RoboDojo` 只替换了这个场景 `preview/` 下的五个展示文件（`compare_photo.jpg`、`overview.jpg`、`robot_head.jpg`、`scene.glb`、`render/sim.jpg`，都不入库），旧文件备份在 `.cache/lab-preview-before-legs-20261022-112855/`。部署机的检出仍停在 `4f8cccf`，没有快进：上面有别的会话没提交的 `HANDOFF.md`、`airpods` 的 `spec.json`、`real2sim/scenes/real_scene4_school/` 的 9 个文件，以及未跟踪的 `real2sim/scenes/g1d_green_table/`，而且部署机这次连不上 GitHub。所以平台抽屉里这个场景的检查数、尺寸、拟合残差还是旧的，等下次快进再更新。浏览器缓存的旧图要刷新两次（`/file` 先显示缓存，再在后台校验）。
- **桌腿改直**（10-22 用户指出：中间桌子的腿是直的，不是斜的）：v1 在 p1 上标的四条立柱线其实落在 V 形底脚上，把立柱拟合成了外倾 15°。现在 `fit_photo.py` 只用一个 `col_x`（竖直），立柱线改在 p3 的两根立柱中线上取，结果 `col_x = 0.638`（桌端往里约 5 cm），相机变化 ≤ 3 cm，焦距 786.5 → 790.6 px。试过再加一个 Y 方向倾斜，拟合出来只有 1.5 cm，所以不加。p3 立柱线残差 4–15 px（暗处取点），其他照片的点残差不变。重新验证：`check_scene.py` 24/24；隔离目录重跑 `build_asset.py`，`g1d_lab` 的叠块、拾取 seed 0 都是 1.0（两个演示同时启动时，有一个读到同时生成的机器人 USD 半成品而报错，单独重跑就好了）；RTX 渲染重出。共享库的 `Room/real_scene3` 再次替换，旧版备份为 `/data/robodojo-assets/backups/backup-shared-20261022-112705.tgz`。
- **装共享库**（用户 10-22 同意：先验证再替换）：
  - 隔离目录 `5090-2:/data/robodojo-runs/lab-furniture-install-20261022-085858/`：资产复制自 `shenzhen-lab-20261021-125246`，缺的积木、小车和三类杂物另外补进来。在里面跑 `build_asset.py`（退出码 0），再用 `g1d_lab` 跑 `scripted_demo` 的 seed 0：拾取、叠块、量杯倒豆都是 `success_rate` 1.0、`eval_time` 1，五路相机画面正常。
  - 踩坑：起初拾取、叠块的 34 个 seed 全部判为不稳定。用旧房间、办公室场景对照，结果一样，说明和家具无关：复制来的资产里 `block`、`car` 只有元数据，没有 `object.usdz`。**5090-2 上的运行目录资产不全，跑别的任务前先确认 `object.usdz` 在。**
  - 只替换了共享库 `5090-2:/data/robodojo-assets/Assets` 的 `Room/real_scene3/`（`diff -r` 和验证产物一致），布局没动。旧版备份在 `/data/robodojo-assets/backups/backup-shared-20261022-101545.tgz`。
- **没做**：家具没有卷尺实测。（墙上的照片贴图和压平在上面的杂物 10-23 去掉了，见本节上方的 v3。）

### G1-D 夹爪力反馈（2026-10-21，分支 `chongzheng/robot-g1d-gripper-force`）

- **用户要求**：仿真里要能知道夹爪用了多大力，在三个方案里选了"指垫接触传感器"。
- **实现**：`robot_config/g1d.py` 新增 `FINGER_PADS`（四个手指关节各自对应的指垫 `*_dex1_finger_link_{1,2}_tip`，即 Link{1,2}_3），并打开 `activate_contact_sensors`。`RobotManager._get_SceneCfg` 发现 `FINGER_PADS` 时，给场景加 `robot{idx}_pads` 这个 ContactSensor。`get_gripper_force()` 返回每个指垫的接触合力大小（N，顺序同 `gripper_joints_name`），`ObsManager` 写进 `obs["gripper_force"][gripper_name]`。没写 `FINGER_PADS` 的机器人（X5、Franka）不受影响。录制器只取 `state_keys`，所以 16 维 state 和数据集格式不变（`CONTRACT.md` 已注明）。
- **验证**：本机 `scripts/check.sh` 通过（22 passed，1 skipped；契约单测加了 `gripper_force` 断言）。`5090-2` 隔离目录 `/data/robodojo-runs/gripper-force-20261021-142346`（GPU 2），`g1d_beans_collect` 量杯 seed 0：`success_rate` 1.0、`eval_time` 1；张开时力为 0，夹住时每个指垫 15–22 N（最大 22.6）。同一回合把上限改为 120 N、刚度改为 6e4（目录 `…-120n`，GPU 3，只改了服务器上的副本，没提交）：也成功，夹持中位约 24 N、最大 30 N。原因是脚本专家只把夹爪关到指令 0.3，力由刚度乘以剩余行程决定，到不了上限。`g1d_general_pickup` 没测：这份资产副本里没有它的布局。
- **改参数**（同日，用户按估算拍板）：每指刚度 3.6e4 N/m、上限 55 N。回归在 `5090-2:/data/robodojo-runs/gripper-force-55n-20261021-145151`（GPU 2–4），`g1d_beans_collect` 量杯 seed 0、100、101 全部成功（`success_rate` 1.0，`eval_time` 1）。夹持时开度 0.744–0.756，和旧参数一样；每个指垫的力中位 18–25 N，最大 31 N。脚本专家只把夹爪关到指令 0.3，所以力到不了 55 N 的上限。
- **合并 `main`**（PR #17 带来移动版 `g1d_mobile`）：`g1d_mobile.py` 也导出 `FINGER_PADS`，两版都有 `obs["gripper_force"]`；夹爪参数两版共用。合并后 5090-2 上桌面版量杯 seed 0 回归通过（`success_rate` 1.0，`eval_time` 1，夹持力中位 21 N）；移动版没在 GPU 上测（运行目录没有 `g1d_mobile_dex1.urdf`）。
- **没做**：力没有进数据集，也没给策略当输入；真机 Dex1-1 能否读出对应的量没查。

### G1-D 桌面版 / 移动版（2026-10-21，[PR #17](https://github.com/Zheng-Chong/RoboRSR/pull/17) 合并进 `main`）

- **用户要求**：仿真支持腰部、升降和底盘移动，分成桌面操作版和移动操作版；底盘和升降按官方方案。
- **官方方案**（unitree_sdk2 `example/g1/g1d`、`g1_agv_client.hpp`，宇树 G1D 开发者文档）：标准版推拉底盘无电驱，17 自由度（两臂 14、腰 2、升降 1）；旗舰版两轮差速，19 自由度。腰是底层 12 WaistYaw、14 WaistPitch 位置控制；升降是 `AgvClient.HeightAdjust` 速度指令（最大 76.5 mm/s，高度读 `rt/hispeed_state`）；底盘是 `AgvClient.Move(vx, 0, vyaw)`（±1.5 m/s、±0.6 rad/s，`vy` 忽略）。官方 `g1_d.urdf` 只有两个驱动轮、没有万向轮，也没有任何仿真配置（`unitree_sim_isaaclab` 不支持 G1-D）。官方 URDF 里腰部关节名就和转轴对不上（`torso_Joint` 绕 z 是偏航，`Yaw_Joint` 绕 y 是俯仰），不是 v4 改的，CONTRACT 已更正。
- **决策**（用户选定桌面版身体自由度为可选键）：
  - 动作键：`waist_joint_state` [偏航, 俯仰] rad，`lift_joint_state` [高度] m（0–0.42，两级各一半），`base_velocity` [vx, vyaw]（仅移动版）。可选，缺省保持（底盘缺省为 0），旧 16 维策略不用改。
  - 移动版底盘用虚拟平面关节（`base_x/y/yaw_joint`，在起点坐标系里），按差速运动学把 (vx, vyaw) 积分成位置目标并下发速度前馈；目标最多领先实测 1 cm / 0.02 rad，撞上东西会停而不是蓄力。轮子锁死、去掉碰撞体（两个轮子没有万向轮，靠轮子驱动会翻；轮子碰撞会拖地），底盘外壳（离地 3.4 cm）照常碰撞。
  - 升降和腰按官方限速（0.0765 m/s 分到两级、5 rad/s）；复位时直接把身体关节写回默认值（升降柱 1 s 内开不回来）。
  - 观测要 env 配置 `observation.robot.body_states: true` 才加（只有 `g1d_mobile` 打开），所以桌面版六个配置的观测和以前完全一样。
- **改动**：`env/robot_manager/robot_class/g1d.py`（身体键 ↔ 关节的映射）、`robot_manager.py`（关节索引、复位、初始保持值、控制下发、`get_body_state`，注册 `g1d_mobile`）、`robot_config/g1d.py`（`mobile` 参数；执行器拆成 lift、waist、base）、`robot_config/g1d_mobile.py`、`x5.py`（`asset_name`，移动版读 `Assets/Robots/g1d/`）、`obs_manager.py`、`eval_env.py`（校验、身体键转控制、腰和升降在动作内插值）、`build_asset.py`（多写 `g1d_mobile_dex1.urdf`，`SCENES` 加 `g1d_mobile` 生成办公室布局）、`env_cfg/g1d_mobile.yml`、`env_cfg/robot/g1d_mobile.yml`、`_robot_info.json`、新测试 `scripts/g1d/test_g1d_body.py`（进 `check.sh`）、GPU 自检 `scripts/g1d/body_probe.py`；README、CONTRACT 同步。`test_server.py` 的期望改为办公室场景对应 `g1d` 和 `g1d_mobile` 两个配置。
- **验证**：
  - 本机：ruff 0.11.7（改动文件）、`git diff --check`、pytest 26 passed 1 skipped（XPolicyLab 空）、任务清单检查。
  - `5090-2` 隔离目录 `/data/robodojo-runs/g1d-mobile-20261021/`（资产从 `school-scene-20261020-114517` 复制，`g1d_mobile_dex1.urdf` 由 `mobile_urdf()` 从其中的 `g1d_dex1.urdf` 生成，`Eval_Layout/RoboDojo/g1d_mobile` 软链接到 `g1d`；没跑 `build_asset.py`，那边没有 `unitree_ros`）。用量杯任务 seed 100（运行目录只有量杯布局）：
    - `body_probe.py --env_cfg_type g1d_mobile`，2 回合都通过：升降 +0.1 m、腰 [0.3, 0.2]、后退 2 s 到 -0.6 m、原地转 2 s 到 0.6 rad，误差都 < 1e-5；躯干世界位姿跟着底盘走；不发底盘键 0.5 s 后速度 4e-5 m/s；第二回合开始时底盘和升降回到默认；`_result.json` success_rate 1.0、eval_time 2。截图在本机 `output/g1d_body_probe_20261021/`。
    - `body_probe.py --env_cfg_type g1d`，2 回合都通过，桌面版拒绝 `base_velocity`。
    - 回归：`scripted_demo.py --env_cfg_type g1d --task_name g1d_pour_beans_measuring_cups --seed 0` success_rate 1.0、eval_time 1。
  - 踩坑：运行目录的资产只有量杯布局，拾取任务 seed 0 没有布局时 `main.py` 不报错直接退出；Kit 退出时会吞掉缓冲的 stdout，要 `PYTHONUNBUFFERED=1` 才看得到 `[main]` 输出。

### 深圳实验室场景 `real_scene5_shenzhen_lab`（2026-10-21，[PR #22](https://github.com/Zheng-Chong/RoboRSR/pull/22) 合并进 `main`，已部署）

- **输入**：平台登记 `registration.json`（卷尺：桌面 0.75 m 高、1.20 × 0.60 m、厚 2 cm；储物柜 75 × 120 × 45 cm；后沿贴墙；顶条面光源）和 6 张照片，从 `5090-2:/root/RoboDojo/real2sim/scenes/real_scene5_shenzhen_lab/` 同步，入库。
- **建模**（从 `real_scene3_lab` 的写法复制）：`fit_photo.py` 在 5 张照片上手标桌沿、立柱、桌脚、柜角（4915 和 4914 同机位，不用），桌面尺寸、厚度、柜子尺寸、贴墙都固定成登记值，只拟合相机、共享焦距、桌腿和柜子位置。桌沿残差 ≤ 3.7 px；焦距 773.6 px，与 EXIF 26 mm 等效的 769 px 一致。`build_scene.py` 出 `usd/`（背景、桌子、柜子、灯光四层）和 `real_scene5.usd`；碰撞体全是盒体。`check_scene.py` 22/22。`acceptance.py`：桌高、长、宽误差 0。
- **和登记不一致**（按照片建，写进了 `PLAN.md` 和 `scene_spec.json`）：“四条腿”实为两条 T 形升降腿；“条纹瓷砖”实为方块地毯；桌上有 ChArUco 板但没量格子，没用。
- **站位未实测**：`env_cfg/robot/g1d_shenzhen_lab.yml` 根节点 y=-0.73（桌沿在前 0.13 m），桌子中线；`SCENES` Room 位置 (0, -0.30, 0)，物体不平移；升降柱各 0.035（桌高比办公室高 0.07 m）。
- **灯光**：4 条 1.2 × 0.15 m RectLight，6000 K，强度 54000；`lighting.status: recorded`。在 Isaac 里把 `cam_third` 放到 4916 的拟合位姿对了三轮颜色（数值表在 `PLAN.md`），最终墙 / 桌 / 地毯和照片差 12 / 10 / 1 sRGB 级。
- **顺手修的 bug**：`build_asset.py` 的 `seat_wrist_camera` 在临时目录里建 meshes 软链接时用了 `out` 的原样路径，`--assets` 传相对路径时链接断开，报 `Missing mesh: .../AGV_link.STL`；改为 `.resolve()`。默认参数（绝对路径）不受影响。
- **验证**：
  - 本机：`scripts/check.sh` 通过（22 passed，1 skipped）；新场景脚本单独跑过 ruff 0.11.7；`test_scenes.py` 加了 `g1d_shenzhen_lab` 的期望站位。
  - `5090-2` 隔离目录 `/data/robodojo-runs/shenzhen-lab-20261021-125246`（GPU 3）：资产 = 学校场景运行目录的 `Assets` + 平台镜像的 `arx_x5` 布局和物体元数据；`unitree_ros` 里唯一需要的 URDF 在本机下载后放进 `.cache/unitree_ros`（5090-2 上不了 GitHub）。`build_asset.py --assets Assets` 退出 0，`g1d_shenzhen_lab` 739 个布局。`scripted_demo --env_cfg_type g1d_shenzhen_lab --task_name g1d_pour_beans_measuring_cups --seed 0`：success_rate 1.0、eval_time 1，四路相机正常；`cam_third` 换到 4916 / 4913 位姿又各跑一次（也都 1.0），运行目录的相机配置已还原。
  - 对比图在场景目录 `preview/`（不入库）：`compare_photo.jpg`（几何叠线）、`compare_isaac.jpg`（照片 vs Isaac）、`robot_cameras_isaac.jpg`（四路相机）；已拷到 `5090-2:/root/RoboDojo/real2sim/scenes/real_scene5_shenzhen_lab/preview/` 给平台显示。
- **共享库**（用户同意后，2026-10-21 13:5x）：`Room/real_scene5`（8 个文件）和 `Eval_Layout/RoboDojo/g1d_shenzhen_lab`（574 个布局，0/1/2 三组）装进的是迁移前的旧资产库，**`5090-2:/data/robodojo-assets/Assets` 里没有**，要从上面隔离目录的验证产物补装（见「剩余事项」）。
  - **去掉了 `g1d_pour_beans_measuring_cups_*`（165 个）**：共享库没有 `measuring_cup`，其他配置也没有这个任务的布局。
  - 生成输入核对：同一次运行生成的 `g1d_lab` 和共享库的 `g1d_lab` 比，拾取、叠块逐字节相同；倒水、杯对杯只有豆子参数不同，因为**共享库里 `g1d`、`g1d_counter`、`g1d_lab` 的这两个任务布局还是旧的 4 mm 豆子参数**，当前代码是 5.6 mm。`g1d_shenzhen_lab` 按当前代码生成，所以和旧配置在这两个任务上不一致（见「剩余事项」）。
  - 平台：`5090-2` 的元数据镜像补了同样的 574 个布局 json（整体重刷镜像时会从共享库带上）。
- **部署**（14:00）：`5090-2` 从 `d5db8aa` 快进到 `2975b4a`（推临时分支再 `--ff-only`）；部署机 `HANDOFF.md` 里另一会话的绿幕桌记录照旧先存为 `.cache/handoff-local-20261021-135957.patch`，快进后重新应用；airpods `spec.json` 的平台时间戳改动保留。`real_scene5.usd` 和地毯贴图检出后只是 LFS 指针，从本机 rsync 真实文件后 `git add` 刷新 stat。重启服务后未登录 `/api/info` 401；部署机上直接调用 `scan_scenes()`：已建模、配置 `g1d_shenzhen_lab`、离线检查 22/22、拟合 5 视角（最大 8.1 px）、5 张渲染图。网页里点「同步项目数据」即可看到。
- **没做**：站位卷尺和叠图（要真机头部、腕部画面和关节角）；灯光没和头部相机比。

### 数采平台任务登记（2026-10-20，[PR #11](https://github.com/Zheng-Chong/RoboRSR/pull/11) 合并进 `main`，已部署）

- **用户要求**：任务采集也加新建 / 登记按钮，主要能选泛化等级，并选定所用物体范围、干扰物、可用场景，以及其他泛化。
- **页面**：「任务采集」新增「登记任务」四步向导，沿用场景和物体登记的向导组件：
  1. 任务定义：新建（ID `g1d_<后缀>`，不能和已有任务重名），或给已有 G1-D 任务登记泛化方案（物体角色从任务配置的 label 预填）；中英文名、语言指令（英文，原样发给策略）、成功判据（必填，用于写 `run_reward`）、用手、预计时长、合格回合目标、负责人；
  2. 物体角色：每个角色一个 label、每回合数量、候选物体（从物体资产里选，可多选）、可选 x/y 范围和朝向随机；干扰物：数量范围、指定物体、是否也从 Clutter 库取；
  3. 场景：每个场景设为训练、L4 保留或不用，未建模的场景也能选；至少一个训练场景；
  4. 泛化等级 L0–L4：勾选等级，范围和目标布局数从原 `LEVELS` 预填，可改；L3 选保留实例；选了 L4 必须有保留场景；其他泛化（预设：桌高、站位、初始姿态、指令改写、控制延迟）；脚本专家思路、其他要求、备注。
- **落盘**：服务端 `write_task_registration` 写 `real2sim/tasks/<任务>/registration.json`，场景附上加载它的 env 配置（取自 `build_asset.SCENES`），空字段省略（`compact` 现在也递归处理列表）。`scan_tasks` 把只有登记、还没实现的任务也列出来（`runnable: False`、`module: None`），流程里「任务配置」显示为待办，而不是不通过。
- **抽屉**：登记过的任务顶部是给 coding agent 的实现任务（新任务指向 CLAUDE.md 的任务约定和 `scripts/datagen/SKILL.md`，已有任务只补布局和采集设置），下面是登记方案摘要和可直接编辑的文字字段；分级评测集表格改为只列登记的等级，显示登记的范围和目标数。「删除登记」：未实现的任务删记录，已实现的只清掉登记，保留负责人、目标和采集记录。
- **验证**：`test_server.py`（登记 JSON、场景映射 env 配置、列表内空字段省略、未实现任务出现在扫描里、坏 id 400、删除）；`check.sh` 的 ruff、diff、任务清单；pytest 22 passed、1 skipped。浏览器里走完向导：缺指令、角色没有 label、角色没选物体、干扰物最少大于最多、没有训练场景、选了 L4 却没有保留场景都被拦住；提交后文件内容正确，列表显示「待实现」，抽屉文字字段写回文件，「已有任务」模式预填了量杯任务的 3 个角色。测试中发现最后一步的校验没在提交前跑，已修复。测试数据已删。
- **部署**：`5090-2` 从 `4e9567b` 快进到 `73b2c89`，部署机 `HANDOFF.md` 的另一会话记录照旧先存补丁、快进后重新应用；重启后本机和公网未登录 `/api/info` 401，页面带「登记任务」。线上登记的 `real2sim/tasks/*/registration.json` 要拷回入库。

### 数采平台场景登记（2026-10-20，[PR #10](https://github.com/Zheng-Chong/RoboRSR/pull/10) 合并进 `main`，已部署）

- **用户要求**：场景重建也按物体资产那样登记，每个场景不少于 5 张不同角度的照片，再加需要人工提供的信息（字段由 agent 设计）。
- **页面**：「场景重建」新增「登记场景」三步向导，和物体向导共用步骤条、校验和拖放组件（抽成 `wzFld`/`wzPane`/`wzNav`/`wzFoot`/`photoDrop`，`wizGo` 按当前打开的向导取步骤和校验函数）。
  1. 中英文名称，目录名 `real_scene<N>_<后缀>`（N 取已有最大号 + 1，后缀由英文名生成，可改，不能和已有场景重名），类型、地点、用途（孪生 / 训练泛化 / L4 保留），照片至少 5 张，不够不能下一步；
  2. 卷尺：桌面离地高、长、宽必填（cm），可选厚度、测量日期、支撑形式、桌面外观、桌沿形状；机器人站位（桌沿到根节点、横向偏移、偏角、升降柱读数）；ChArUco 标定板（勾上后格子实测边长必填，加板角到桌左沿、前沿的距离）；
  3. 地面、墙面、后沿到墙距离、要建模的固定物；光源、色温、照度、自然光；是否已拍机器人相机画面；计划任务（G1-D 自建任务多选）、建模要求、备注。
- **落盘**：服务端 `write_registration` 写 `real2sim/scenes/<场景>/registration.json`。卷尺值换成米，按 `scene_spec.json` 的 `measurements` 键和格式写（`table_top_z` 等，`tool: 卷尺`、`date`），空字段省略（0 保留）。照片上传 `/api/photo?kind=scene` 进 `reference/`，删除照片放行 `real2sim/scenes/`。场景 id 校验 `real_scene\d+_[a-z0-9_]+`。不新建 `scene_spec.json`：`acceptance.py` 要求场景已接入 `build_asset.py`，未建模的场景放进去会报错。
- **部署**（23:2x）：`5090-2` 从 `daf4ea0` 快进到 `4e9567b`（推临时分支再 `--ff-only`）。快进前删掉了和 `main` 内容一致的未跟踪 airpods 文件（`spec.json` 和 3 张照片，LFS oid 核对过），检出后照片只剩 LFS 指针，又从本机 rsync 回真实图片；部署机 `HANDOFF.md` 里另一会话（绿幕桌相机标定）未提交的一段先存为 `.cache/handoff-local-*.patch`，快进后重新应用。重启服务后扫描到 5 个场景（含未入库的 `g1d_green_table/`），本机和公网未登录 `/api/info` 都是 401，页面带「登记场景」。
- **显示**：「照片」阶段改为至少 5 张才算通过，旧场景里吧台（2 张）、实验室（4 张）、学校（2 张渲染）因此变成待办。「尺寸实测」阶段没有手填桌高时，用登记的桌高。已登记但还没建模的场景，抽屉顶部是给 coding agent 的建模任务文本（指向 `real2sim/scenes/SKILL.md`），下面是照片上传/删除、验收进度和可编辑的登记信息；建好后登记信息仍在抽屉里。`real2sim/scenes/SKILL.md` 加了「平台登记」一节。
- **验证**：`test_server.py`（新增：登记写出的 JSON 全量比对、坏 id 返回 400、场景照片上传/删除、扫描识别已登记、删除登记会删文件）；`check.sh` 里的 ruff 0.11.7、`git diff --check`、任务清单；pytest 22 passed、1 skipped（本机没有 pytest，用 `uv run --with pytest ...` 补）。本机起服务在浏览器里走完向导：3 张照片被拦、6 张通过、缺桌高被拦、勾标定板不填格子被拦；提交后 `registration.json` 和 6 张照片落盘，抽屉显示建模任务；抽屉里改墙面会写回文件。测试场景目录已删。没有在手机宽度下截图检查。

### 耳机充电盒 `airpods/00000` 实物孪生（2026-10-20，PR #7 合并进 `main`）

- **输入**：平台登记 `real2sim/objects/Rigid/airpods/00000/`（`spec.json` + 3 张照片，从 `5090-2:/root/RoboDojo` 同步）：PP 塑料、白色、38 g，`尺寸（高 × 宽 × 厚）：46.40 × 54.40 × 21.38 毫米`。
- **脚本** `scripts/objects/make_airpods.py`：平放的圆角盒，沿"高"放样，截面是"宽-厚"平面里的圆角矩形；俯视圆角 R15、边缘圆角 R8、盖深 16 mm 是照片估计。可见部件：壳 `model`、盖缝 `seam`（壳上 0.3 mm 深的槽加槽底灰带，不超出外形）、USB-C 口 `port`。碰撞体是一个 64 顶点凸包，比外观最多小 1.2 mm。不写摩擦（用默认）。`measurements_mm` 写 `高`/`宽`/`厚`，从壳的网格量。
- **踩坑**：第一版凸包是贪心选点，落地后歪 2.4°、下沉 0.7 mm（GPU 和 CPU 物理都一样，延长到 6 s 也一样，改成 60 顶点也一样）。原因是 PhysX 烹饪的平面容差 0.7 mm：离平底面不到 0.7 mm 的点被合并，底面就不平了。现在凸包的点要么正好在平面上，要么离开 ≥ 1.5 mm；已记进 SKILL「常见坑」。
- **验证**：
  - 本机：`scripts/check.sh` 通过（用 `uv run --with ruff==0.11.7 ...` 补依赖；22 passed，1 skipped）；pxr 打桩跑几何和 `self_check`。
  - `5090-2` 隔离目录 `/data/robodojo-runs/airpods-20261020-194222`（GPU 3）：生成到 `ovl/Rigid`，`check_object.py` OK：z=0.0107（期望 0.0107）、倾角 0.00°、速度 0。尺寸：高 46.4 / 宽 54.4 / 厚 21.4 mm，对实测误差 0 / 0 / 0.02 mm；质量 0.038 kg。
  - 渲染图 `preview/sim.png` 和 3 张实拍对比过：轮廓比例、盖缝位置、USB-C 口一致，仿真高光比实拍弱。已放到本机和部署机的同一路径。生成产物也拷到了本机 worktree 的 `Assets/`（不入库）。
- **共享库**（用户同意后，20:22）：新目录 `Object/RoboDojo/Rigid/airpods/00000`，没有覆盖任何东西，所以没做备份；从隔离目录 `ovl/Rigid/airpods/00000` 经本机中转拷入，三个文件的 sha256 和验证产物一致。
- **平台**：没有整体重刷镜像（会删掉手动放的纸杯纸盘元数据），只把这个 id 的 `metadata.json`、`description.json` 拷进镜像（和共享库一致，以后整体重刷也会保留），再重启服务重新扫描。数据库里显示：已入库、质量 0.038、`measurements` 高 46.4 / 宽 54.4 / 厚 21.4、有渲染图；「生成脚本」为空，因为部署机代码还没更新到含 `make_airpods.py` 的 `main`。
- **尺寸文本**：登记原文「尺寸（高 × 宽 × 厚）：46.40 × 54.40 × 21.38 毫米」旧版 `parseDims` 拆成乱码；PR #9 的新解析拆出 `高`/`宽`/`厚`（本机用 node 跑过），和 `measurements_mm` 对得上，登记不用改。

### 学校实验室场景和纸杯纸盘接入（2026-10-20，PR #2，已合并进 `main`）

- **来源**：同事在 sys01 上做的 MuJoCo 孪生 `g1d_food_sorting`，10-20 整包拷到了 `5090-2:/root/g1d_sim_assets_20261020`（`SHA256SUMS` 通过）。用户选的范围：场景，加上杯盘（用包里的拟合几何）；花材（40 簇）不接。
- **场景** `real2sim/scenes/real_scene4_school/`（Room `real_scene4`，env 配置 `g1d_school`）：
  - `build_scene.py` 读 `bundle/environment_manifest.json` 里的桌面四角（初始躯干坐标系），按 `reference.npz` 里记录的躯干偏航（12 号电机 0.067 rad）换到底盘坐标系，拟合成矩形：1.065 × 0.596 m，离地 0.763 m，四角残差 15 mm。绿幕取自源场景的 `backdrop`。桌腿、房间、灯光没观测到，是随手定的。只需要 pxr 和 numpy：`uv run --no-project --with usd-core --with numpy python real2sim/scenes/real_scene4_school/build_scene.py`。
  - 站位：桌沿在根节点前 0.19 m，桌面中心在机器人右侧 0.083 m，底盘左偏 1.63°（`env_cfg/robot/g1d_school.yml` 根节点 y=-0.79）。`build_asset.py` 里 `SCENES` 新增一行：Room 位置 (0.0828, -0.302)，物体平移 (0, -0.06)。
  - **和前三个场景不同**：升降柱保持 0，桌面相对躯干的高差按拟合值保留（0.082 m），不统一成办公室的 0.165 m。偏航来自"12 号电机就是 `torso_Joint`、方向与 URDF 相同"这个假设，没在真机上核对。细节见该目录的 `PLAN.md`。
- **物体** `scripts/objects/make_school_tableware.py`：复用 `make_plastic_cups.py` 的放样、杯壁分片碰撞体和 `save()`。
  - 纸杯：读 `bundle/cup_shapes.json` 的拟合轮廓（每只是两只纸杯套在一起）。
  - 纸盘：平底加锥形边，尺寸取自孪生的网格。
  - 质量取自孪生的密度假设，颜色是孪生的 sRGB 纯色换算成线性值（第一次没换算，黑杯渲染成灰色）。
- **验证**：
  - 本机：`scripts/check.sh` 通过（22 passed，1 skipped：worktree 里 XPolicyLab 是空的）；`test_scenes.py` 的期望值加上了 `g1d_school`。新文件已手动跑过 ruff（check.sh 只看已跟踪的文件）。
  - `5090-2` 隔离目录 `/data/robodojo-runs/school-scene-20261020-114517`：
    - `Assets/` 复制自 `office-beans-random-20261019` 的覆盖目录，再手动装上 `Room/real_scene4`。
    - `g1d_school` 的布局由运行目录里的 `school_layouts.py` 从该目录的 `g1d` 布局换算得到：Room 换掉，物体抬高 0.0828 m、平移 -0.06 m。**没有跑 `build_asset.py`**，因为 5090-2 没有 `g1d_v4`、`unitree_ros` 和完整资产库。
    - `check_object.py`：5 件物体全部 OK（倾角 0°，落地高度和半高相差 ≤ 0.1 mm）。
    - `scripted_demo.py --env_cfg_type g1d_school --task_name g1d_pour_beans_measuring_cups --seed 0`：成功，`_result.json` 的 success_rate 1.0、eval_time 1。头部画面和源孪生的渲染一致（深绿幕、木色桌板）。注意这个配置的步长是 1/300 s，颗粒任务正式评估仍然要用 `g1d_beans_collect`。
- **没做**：卷尺、叠图（真机画面在 sys01 上，这次 SSH 连不上）、灯光；杯盘没有任务和布局；没有装共享库。

### 采集、评估流水线规范化（2026-10-19，PR #1，2026-10-20 合并进 `main`）

- **起因**：用户要把"基于 Coding Agent 的 Real2Sim 采集 + 数据生成 + 仿真评估"做规范。在建议清单里，用户选了四项：数据契约、通用采集流水线、Agent 文档和 CI、场景标准化。
- **数据契约**（`scripts/datagen/CONTRACT.md`）：
  - 发现的问题：评估时给策略的夹爪 state 是上一步的指令值（`prev_control`），而旧数据集里存的是实测开度。
  - 对策：新增 env 配置项 `observation.robot.ee_state: measured`（`env/observation_manager/obs_manager.py`），G1-D 的四个配置都打开；不写这个键时行为不变，X5 等任务不受影响。
  - 录制直接写 XPolicyLab 标准轨迹 HDF5：state 取评估时的 obs，指令取评估时给策略的那句。LeRobot 只通过官方转换器生成。
  - 物理配置写进每个回合的 `additional_info/env_cfg` 和质检单。颗粒任务不要在 1/300 s 下评估。
  - 回放证据：
    - 两条录制回合在 `g1d_beans_collect` 下重放：state 偏差 ≤ 1e-4 rad；一条严格验收通过，另一条差 1 颗豆子；
    - 同样的动作放到 `g1d`（1/300 s）下：各洒出 57–59 颗，约 9%，连任务自身的判据都不通过。
- **评估入口**：
  - `src/eval_client/main.py` 新增三个参数：`--policy_module`（在进程内跑策略，不起服务器）、`--layout_ids`、`--allow_train_layouts`。
  - `env/seed_manager/seed_manager.py`：seed 目录里有 `split.json` 且标为 train 时，评估直接拒绝运行。
  - `EvalEnv` 新增 `end_on_success`（默认 True，任务判据达标即结束回合）和 `task_success`（任务判据是否触发过），取代采集时 monkey patch `get_reward` 的做法。
  - `scripted_demo.main()` 改用 `--policy_module`，不再往 `sys.modules` 里塞假的策略包。`cup_to_cup_actions` 新增参数 `pour_share`（不能叫 `pour`，会和函数内部的 `pour()` 重名；第一次 GPU 运行就是因此报的 TypeError）。旧采集器通过改模块常量 `C2C_POUR` 的方式仍然有效。
- **通用流水线** `scripts/datagen/`：
  - `collect.py`：录制器和 worker；
  - `run_batch.py`：分片（写 `split.json`）、多卡调度、续跑，记录源码和资产的 sha256；
  - `export.py`：调用官方转换器，再用 LeRobot 加载器逐回合核对长度、指令和首尾帧，生成数据集卡片；
  - `audit.py`：从原始数据独立复算验收，统计布局覆盖，写 `final_validation.json` 和 `sha256.json`；
  - `replay.py`：开环回放，检查数据环境和评估环境是否一致；
  - `upload.py`：上传 ModelScope；
  - `plugins/g1d_pour_beans_measuring_cups.py`：量杯任务插件，复用 `collect_office_beans` 里的计数和判据，验收版本 `all_beans_retained_v2`；
  - `registry.json`：已有的两批数据；
  - `SKILL.md`：操作手册，采集相关的常见坑已从本文件移到这里。
- **场景**：
  - `scripts/scenes/charuco.py`：打印标定板、检测、相机标定、求位姿；
  - `scripts/scenes/overlay.py`：用 URDF 正运动学加相机配置得到头部或腕部相机的位姿，把桌角和桌沿投影出来，和真实画面上的标注比较像素误差；
  - `scripts/scenes/acceptance.py`：按月度目标汇总三个场景的 `scene_spec.json`；
  - `real2sim/scenes/SKILL.md`：现场拍摄、卷尺测量和灯光记录规范。
- **Agent 文档和 CI**：
  - `CLAUDE.md` 新增本项目流水线的导航和规则（一个会话一个 worktree、提交前跑检查）；
  - 本文件瘦身；
  - README 加了数据生成入口、仓库结构和协作规则；
  - `scripts/check.sh` 和 `.github/workflows/checks.yml`：只对改动过的文件跑 ruff（旧文件还没格式化过），再跑单测和任务清单检查。为了让检查通过，顺手修了 `eval_env.py` 的一处 UP038 和 `main.py` 的一处格式问题。
- **验证**：
  - **本机**：`bash scripts/check.sh` 通过，共 23 个单测（采集 7、场景 5、量杯 6、平台 5）、ruff 0.11.7、任务清单检查和 `git diff --check`。
  - **CPU 单测上服务器**：在 `5090-2` 的仿真环境里跑了 17 个单测，用的是 OpenCV 4.11，包括 ChArUco 检测和"XPolicyLab 读取器读回我们写的 HDF5"。
  - **GPU 验证**：在隔离目录 `datagen-20261019-225252` 里跑（资产是 100 条那一轮的同一份，共享库没动）：
    - 脚本专家走新的 `--policy_module` 入口，`g1d` 下量杯 seed 0：`success_rate` 1.0，`eval_time` 1；
    - `run_batch.py` 用 2 张卡、分片 300 和 301 各 2 个布局：4 次尝试 4 次合格，每条 672 颗豆子全部进矮杯；夹持时夹爪 state（实测）是 0.75，action（指令）是 0；
    - 在 seed 300 上不加 `--allow_train_layouts` 跑评估，被拒绝；
    - 开环回放见上面"数据契约"一节和 CONTRACT（采集配置下偏差 ≤ 1e-4 rad，1/300 s 下洒出约 9%）。
  - **导出和审计**：在 `5090-2` 的隔离目录 `datagen-pipeline-20261019-223558` 里跑：官方转换器转了 2/2 个文件，每臂 8 维，AV1；`export.py` 逐回合核对通过（810 帧）；`audit.py` 输出 validated。
  - 原始数据已删除。本机留档在 `output/datagen_validation_20261019/`（报告、日志、质检单、数据集元数据、两帧图像）。
  - **合并 `a1b6db5` 之后复跑**：
    - 脚本专家跑量杯 seed 0：`success_rate` 1.0；
    - 采集探针（候选布局 7）：录制正常，但这一回合撒了 1 颗（671/672），没通过严格验收。同一个布局在 100 条那一轮里 392 帧就合格了；`a1b6db5` 改了倒豆循环的计数，会把杯角的豆子也倒完，这次倒了 452 帧。
  - **验证中发现并已修复**：`pour` 参数和函数内部的 `pour()` 重名；官方转换器在软链接目录下把根目录解析错了；只设 `CUDA_VISIBLE_DEVICES=1` 时渲染和计算不在同一张卡上，取相机数据失败（改成 `--device cuda:1 --device_id 1`）。
- **并行会话**：
  - 另一会话期间在 `main` 上的三个提交（`a1b6db5`、`353730a`、`4562fb4`，见下一节）都已合进来，每次都只有 `HANDOFF.md` 冲突，手工合并。
  - 物体建模 SKILL：`scripts/objects/SKILL.md`，以及 `CLAUDE.md`、`README.md`、平台 `index.html`、`export_office_beans.py` 的线程参数，仍在主工作区里，还没提交。

### 已完成：随机倒豆手 + 随机偏航 + 大范围位置的 100 条办公室数据（2026-10-19，`a1b6db5` 和 `353730a`，另一个会话）

- **用户要求**：左右手随机，两只杯子的偏航随机（用户说"很重要"），位置在能夹到的范围内尽量大；仍是办公室场景、100 条、严格全量判据、LeRobot 格式。
- **代码**：
  - `scripted_demo.flat_turn` 读取杯子的偏航，在 `C2C_FLAT_TURN` 窗口内转动手腕（相对原来的 45°，向外最多 35°、向内最多 15°），让指垫正好夹在一对平面上；超出窗口时取模 90° 的最近边界。56% 的朝向能正对，其余最多偏 20°。杯子在 45° 时转角为 0，旧行为不变。
  - `inside()` 对量杯改用 `measuring_cup_shape.in_cavity`。原来用的是杯口内切圆柱，角落里的豆子被算成已经倒出，于是只剩角落时杯子就回正，导致右手倒时有残留或撒豆。
  - `prepare_office_beans_layouts` 改为倒豆手一半对一半打乱，再加六维拉丁超立方采样：高杯 |x| 0.05–0.22 m，矮杯 |x| 0.05–0.18 m，y 从 −0.46 到 −0.28 m，偏航 0–90°。
- **探测**（运行目录 `5090-2:/data/robodojo-runs/office-beans-random-20261019` 的 `output/office_beans/exp{1,2,3}`，汇总脚本在本机 `.agent/remote-runs/office-beans-random/`）：
  - exp1（中心位置、极端偏航）：接杯手向内多转 25° 以上会把杯子碰倒，向外 44° 抓不起来；右手倒常剩下或撒出 1–5 颗。
  - exp2（修复后，全随机偏航）：18/24 合格，左手 11/12、右手 7/12，没有翻杯，也没有残留。
  - exp3（大范围，64 回合）：26/64 合格。失败主要是杯子离机器人太远（y > −0.29）或矮杯太靠外（|x| > 0.19）时掉杯，以及高杯太近（y < −0.46）够不到；落在选定范围内的是 16/23。
- **正式批次**：
  - 代码 `a1b6db5`，运行目录里关键文件的哈希和提交一致。
  - 400 个布局（左右手各 200），8 张卡，用旧采集器跑 collection_v3：176 个回合里 104 个合格（左 53、右 51），8 个进程退出码都是 0。
  - 协调脚本按手计数，左右各取最早的 50 条，硬链接到 `collection_v3/selected/` 后导出。
  - 数据集在 `5090-2:/data/robodojo-runs/office-beans-random-20261019/output/office_beans/collection_v3/lerobot`：517 MB，LeRobot v3.0，100 条，39827 帧，约 22.1 分钟。已登记进 `registry.json`（legacy 格式）。
- **验证**：
  - 导出器回读 100 条、39827 帧。
  - 原始审计逐条重算最后 30 帧：67200 颗全部转移，残留 0，散落 0；左右手各 50 条。
  - 覆盖：四个位置坐标按 20 档分箱全部覆盖，两只杯子的偏航（模 90°）18 档全部覆盖。
  - 最后 1 秒杯子的最大倾角 29.88°（阈值 30°）。
  - LeRobot 官方读取器解码了 600 张首尾帧，时间戳也通过。
- **ModelScope**（`4562fb4`）：经用户确认，上传为单独的私有数据集 `ChongZheng/g1d_office_pour_beans_random_hand_yaw_100`，和上一批分开。
  - 沿用服务器上的 `ms-venv` 和已保存的登录。服务器上的 `ms_upload.py`、`ms_verify.py` 改成由 `MS_DATA`、`MS_NAME`、`MS_MESSAGE` 三个环境变量指定目录和仓库，默认仍指向上一批；仓库里通用的上传脚本是 `scripts/datagen/upload.py`。
  - 核对结果：远端 17 个数据文件的路径、大小、SHA-256 全部一致；`private=True`，未登录访问返回 404；SDK 默认许可证 Apache-2.0；上传缓存已删除。

### 办公室量杯全量倒豆 100 条（2026-10-19，已发布）

- 实测量杯加 5.6 mm 红豆，600 Hz，118 次尝试 108 次合格，导出 100 条。共 40279 帧，3 路 640×480 视频。
- 独立复算：每条最后 30 帧里，672 颗豆子全部在目标杯内。官方加载器逐条解码了首尾帧。
- 已上传 ModelScope 私有仓库，逐文件核对 sha256 一致。
- 位置、格式和差异（legacy 格式：头部图像键是 `cam_head`，指令是自定义的一句话）见 `registry.json`。
- 用的令牌曾在聊天里出现过，已建议作废重建。

## 剩余事项与风险

- **X5 随机布局**：
  - 官方资产只有 1 张 HDR，"背景随机"实际只随机亮度；
  - 灯光亮度倍数 0.5–1.5 加 HDR 1400 加平行光时有几帧明显过曝（桌面发白），默认范围要调低或按画面亮度筛；
  - 写死位置的任务（`play_tic_tac_toe`、`cover_blocks`、`press_by_number`、`swap_blocks`、两个传送带任务等）位置随机不起作用，只能换实例、外观和干扰物；
  - 生成的布局还没有专家能解（仓库里没有 X5 脚本专家），现在只能当评测集用；
  - 页面没在浏览器里验证，平台没部署。
- **上海实验室 v3**：已装共享库、平台预览已换；部署机检出快进后，平台的场景抽屉和材质库才会显示 v3 的检查结果和墙面材质。
- **真机模型的仿真评测**：Motus eps222 在上海实验室 0/20，原因还没拆开（起始姿态、仿真画面与真机差异、杯位分布都可能）。要继续，先试双臂下垂的真机起始姿态（`remote_policy.py` 目前不支持设起始姿态），再在同一批布局上跑脚本专家做对照。服务端重启见「最近任务」第一节。
- **真机模型的仿真评测**：eps380 对齐后灰杯回合能倒进一部分（见「最近任务」第一节），桌高仍不确定（0.775–0.865 m），需要现场实测；判据要改成"倒半杯"；`5090-6` 的 `motus380-8111` 还占着 GPU 2（20 GB），不用时 `tmux kill-session -t motus380-8111`。夹爪联动改了 G1-D 默认行为，已有数据是两指独立时采的，用新代码重采或回放时注意差别。

- **榜单基线**：10-22 16:35 停止，54 个任务只完成 6 个（见「最近任务」）；是否续跑等用户决定。续跑时 pi0.5 策略 + Isaac 同卡约 25 GB，一卡一组；OpenWAM 策略卡和仿真卡要分开。`/data` 剩约 192 GB（91%），另一会话的 `g1d_green_table/` 占 1.4 TB。官方榜单页面是 JS 渲染，没抓到公开分数，要和自测基线对照需手动查 robodojo-benchmark.com/LeaderBoard。dsw-4 根分区 100% 满（19 GB），训练数据和产出要放 `/mnt/data`。

- **G1-D 移动版**：
  - 共享资产库还没有 `g1d_mobile_dex1.urdf` 和 `Eval_Layout/RoboDojo/g1d_mobile/`，要在隔离目录跑完整的 `build_asset.py` 验证后，经用户同意再装；
  - 录制（`collect.py` 的 `state_keys`）、LeRobot 导出和 XPolicyLab 的 `_robot_info.json` 都只认 16 维，要采带身体自由度的数据，先定契约再改录制器和转换器；
  - 第一个用到底盘的任务是 `g1d_deliver_cup_to_lounge`（见「最近任务」第一节）；底盘撞桌、撞椅只靠领先量限幅，没有专门测；轮子只是外观，不转；
  - 客区尺度只来自「圆桌高 0.75 m」，吧台左木柱宽 1.0 m 是下限估计，椅子尺寸和 7 kg 都是估计；下次去现场按 `real_scene2_counter/PLAN.md`「需要实测的尺寸」第 5、6 项量；
  - 脚本专家的绕行只挑椅子空档，不做路径规划：路线上有别的障碍时会撞上。10-28 加了岛台后，靠的是 `BACK_OFF` 0.5 m 让出靠墙侧吧凳（间隙约 9 cm）；吧凳位置是估计的（座高假设 0.75 m），实测后要按 `build_params.json` 的 `islands` 重算间隙；
  - 送杯任务 seed 0 布局 0、3 抓杯失败（10-28，新旧场景一样），疑似 10-27 的夹爪改动造成，要先修 `lift_cup` 的侧抓，再重跑送杯和倒水送杯；
  - 真机升降是速度接口，部署端要按官方 PD 示例把 `lift_joint_state` 换成 `HeightAdjust`；真机 `rt/hispeed_state` 高度的零点和仿真 0（最低）是否一致没核对；底盘加减速、腰和升降的动力学都没做辨识。

- **主工作区里未提交的改动**（10-20 晚核对）：`CLAUDE.md`、`HANDOFF.md`、`README.md`、`real2sim/scenes/SKILL.md`、`scripts/g1d/export_office_beans.py`，主要是「服务器切到 5090-2」那次的文档改动；主工作区的本地 `main` 还停在 `a694da5`，落后 `origin/main`。物体 SKILL 已随 PR #5 合并。
- **共享库缺 10-20 之后装的东西**：`/data/robodojo-assets/Assets` 是 10-20 的快照，之后装的内容没进来，缺 `Room/real_scene4` 和 `Eval_Layout/RoboDojo/` 下的 `g1d_school`、`g1d_mobile`（`real_scene5`、`g1d_shenzhen_lab` 已在 10-27 补装）。要用这几个场景，先在隔离目录跑 `build_asset.py` 验证，经同意后补装。worktree `.claude/worktrees/object-wizard` 里还有一份没提交的同类文档改动，已被这次提交取代，不要再合。
- **采集配置下的严格判据有方差**：同一组动作回放，2 条里有 1 条最后差 1 颗豆子。评估策略时，严格成功率要在同一布局上重复多次才有意义。
- **物体运行时随机还没有任务打开**：要用就在 `build_asset.py` 的 `RANDOMIZE` 加任务和标签，在隔离目录重新生成布局、跑脚本专家，先问用户（会改那个任务的评测布局）。打开前确认任务和脚本专家只从缩放后的包围盒取尺寸；`cup_grasp` 的 `contact_circle`、量杯腔体计数都没按缩放修正。
- **XPolicyLab 注册**：`XPolicyLab/utils/robot/_robot_info.json` 里还没有 `g1d*`，用 XPolicyLab 的 `train.sh` 会拿不到动作维度。要给 XPolicyLab 提 PR（子模块不在本仓库里改）。
- **legacy 数据集**：和新契约在头部图像键、指令两处不同；用它训练的策略，评估前要先对齐。
- **新流水线还没跑过整批**：collection_v3 是用旧采集器采的，下一批数据应该改用 `scripts/datagen/run_batch.py`。
- **学校场景**：桌面高度和底盘偏航都依赖假设（零升降、12 号电机即躯干偏航），要用卷尺和叠图核对；从 sys01 拷真机头部和腕部画面（`outputs/g1d-real2sim`）做叠图。杯盘要在数采平台登记实测后，按 `scripts/objects/SKILL.md` 重建。
- **共享库的倒豆布局过期**：`g1d`、`g1d_counter`、`g1d_lab` 的倒水、杯对杯布局是旧豆子参数（4 mm，`particle_mass` 6e-5），和当前 `build_asset.py`（5.6 mm）不一致；`g1d_shenzhen_lab` 已是新参数。要统一，就在隔离目录重跑 `build_asset.py`，经同意后替换（先备份）。
- **深圳实验室**：已合并、部署，10-27 装进共享库（整层房间版）；灯光没在 RTX 下按 IMG_4916 重新对色；站位卷尺（桌沿到根节点、横向偏移）和三视角叠图后，`acceptance.py` 才能通过。
- **场景**：除深圳实验室外没有卷尺实测，都没有叠图；`real_scene1` 和 09-21 那次真机运行时的摆放不一样，那批画面不能用来叠图。下次去现场按 `real2sim/scenes/SKILL.md` 一次采齐。
- **夹爪参数**：10-21 按下面的估算改成每指刚度 3.6e4 N/m、上限 55 N（之前 1e4、20 N）。**已有的两份 100 条量杯数据和 10-22 的深圳 500 条杯或马克杯数据（`g1d_shenzhen_pour_beans_cup_or_mug_500`）都是在旧参数下采的**，现在的 env 和它们不再完全一致；评估用这两份数据训练的策略时，要知道夹爪变了（夹持时的开度不变，力变大）。
  - **真机设定**（10-21 在 `ssh unitree` 上只读查看）：遥操作采集（`~/g1d_infra/teleop/robot_control/robot_arm.py`）、策略评测客户端（`~/client/jepa_g1d_client/g1d_jepa_client.py`、`motus_client`），以及 unitree_lerobot 官方的 `Dex1_1_Gripper_Controller`，用的都是电机层 PD：`kp=5.0`、`kd=0.05`、`tau=0`，读数 q 范围 0（闭合）到 5.36（张开）。每步指令都裁到当前位置附近：遥操作 ±0.18 rad（再加平滑滤波），评测客户端闭合时最多超前 0.3 rad（`GRIP_SQUEEZE`）。没有显式的力矩上限，真正决定夹持力的是 kp 乘以这个误差上限：遥操作最多约 0.9 N·m，评测最多约 1.5 N·m。
  - **换算成指尖力（估计，没实测）**：假设 q 和指尖行程成线性（每指 44.5 mm 对应 5.36 rad，约 8.3 mm/rad），一个电机同时推两指，那么每指的力约等于 60 × τ：遥操作约 55 N，评测约 90 N；每指的等效刚度约 3.6e4 N/m（仿真是 1e4）。没算传动效率和摩擦，实际会低一些。线性假设本身也没标定过（见下面的「真机对齐」）。
  - **要定准**：在真机上夹一个已知宽度的硬物，读电机 31、33 在 `rt/lowstate` 里的 `tau_est`，或者夹测力计直接量。
- **真机对齐**：
  - 动力学没做系统辨识；末端位姿动作还不支持（需要 G1-D 的 cuRobo 配置，并且左右臂各用一个 planner）；
  - 真机开环回放的关卡没有工具；
  - 真机夹爪读数 0–5.4 和指尖行程是否成线性没有标定；
  - 真机腕部相机的流名左右互换（部署端 `SWAP_WRISTS=True`）。
- **可达性**：杯对杯 seed 1、壶倒杯 seed 0 都是够不到的老问题。G1 手臂在桌面高度的可达距离约 0.45 m，自有任务的物体应放在 y≈[-0.55, -0.25]。
- **数采平台**：
  - 线上登记的场景（`registration.json` 和照片）和任务（`real2sim/tasks/*/registration.json`）要像物体登记那样拷回入库；
  - 权限只有两级；HTTP 明文传密码；
  - `5090-2` 上的资产库是手动刷新的元数据镜像；学校纸杯、纸盘的 `metadata.json`/`description.json` 是 10-20 从隔离目录 `school-scene-20261020-114517` 的 `ovl/Rigid/` 手动放进镜像的（共享库里没有），平台因此显示为「已入库」；下次整体重刷镜像会被删掉，要么先装共享库，要么重刷后再放一次；
  - 看不到 `scripts/datagen` 的产出（`quality.json`、数据集登记表）；
  - 场景的 `preview/scene.glb` 和 `preview/render/sim.jpg` 不入库，是手动同步到部署机的，场景重建后要重跑 `export_glb.py` / `render_scenes.py` 再同步；3D 预览要浏览器能访问 `registry.npmmirror.com`（three.js），访问不了时抽屉里显示加载失败，卡片和其他内容不受影响；
  - 公开仓库的 git 历史里仍然有运维细节，建议把仓库改为私有。
- **零碎**：
  - 旧批次 `office-beans-100-20261019` 的 final-audit 进程还在等一个不会完成的导出，需要手动结束；
  - 服务器的 `/usr/local/bin/ffmpeg` 不带 libx264，新建环境要在仿真环境的 `bin` 里链接系统的 `ffmpeg`；
  - 指定了 `robot_config`/`scene_config` 的 `_task.yml` 会覆盖 g1d 的机器人或场景。

## 下一会话

0. 物体指令名称（`chongzheng/objects-instruction-names`）：合并、部署数采平台后，管理员对已确认的登记重新点确认（prompt v3）；共享库已装的 38 件已写入，其余物体装库时跑 `apply`；`z/objects-*` 分支合并时检查名称表是否仍对得上这些分支的变体；实现 `g1d_pour_beans_any_cups` 时指令写成 `<source_cup>`、`<target_cup>`。
0. 物体 prompt v2（`chongzheng/platform-prompt-labels`）：合并后部署数采平台，管理员对东方树叶、农夫山泉等已确认的登记重新点确认；再按新 prompt 重做农夫山泉 00000 的照片标签（东方树叶等 `z/objects-plastic-bottle-00006` 合并后复核）。
0. RSI：先读 `rsi/AGENTS.md`。资产分池、ManiTwin 干扰物、Poly Haven HDR 已完成（见「最近任务」前两节），新资产在 `5090-2:/data/robodojo-runs/rsi-assets-20261024/`，用 `rsi_assets.py link` 合并进运行目录。下一步按优先级：材质（Poly Haven PBR 贴图转 MDL，官方材质被三套评测集用完了）→ 房间变体（新材质给 `Simple_Room` 换墙面）→ GSO 的鞋和玩具 → 任务物体的新实例 → 12 个 `_random` 任务基础版的 X5 脚本专家。每次加完新资产都要重跑 `rsi_assets.py scan` 并把 `pools.json` 入库。
0. 榜单：基线已停（16:35，用户要求），先问用户是否续跑；续跑后收两份基线（`smoke_results/baseline_*_20261022.md`，核对每个任务 `eval_time >= 1`），按任务对比 OpenWAM / pi0.5 / 有无官方数据；看失败视频做归类；然后开始把 `collect.py` 和插件接口推广到 dual X5，按优先级给无官方数据的任务写专家。
0. 实现平台登记的 `g1d_pour_beans_any_cups`：在平台任务抽屉复制「实现任务」文本交给 coding agent（输入 `real2sim/tasks/g1d_pour_beans_any_cups/registration.json`）。
0. 深圳实验室（已合并、部署、装共享库）：去现场量站位、拍头部 / 腕部画面做叠图（`overlay.py --env-cfg g1d_shenzhen_lab`）。
0. 部署机快进：别的会话在部署机上还有 28 个未提交文件，快进照 10-24 16:58 那次做（先 stash、快进、pop，新增 LFS 文件从本机补实际内容）。
1. 先看 `git log`、本文件和 `scripts/datagen/CONTRACT.md`。主工作区里还有物体建模 SKILL 会话没提交的改动（见"最近任务"里的"并行会话"）；拉取新的 `main` 前，先让那个会话提交，或者先 stash。
2. 下一批数据用新流水线采集。先跑单回合探针，确认 `joint_names`、夹爪 state 和图像都正常，再按 `scripts/datagen/SKILL.md` 批量采集、导出、审计、回放，最后登记。
3. 去现场按 `real2sim/scenes/SKILL.md` 采集（ChArUco 标定板照片、卷尺、灯光、机器人相机画面），在平台「登记场景」录入，回来跑 `overlay.py` 和 `acceptance.py`。
4. 给 XPolicyLab 提 PR，注册 `g1d*` 的动作维度；然后用一个 LeRobot 策略在 `--env-cfg g1d_beans_collect` 下走通"外部训练 → 本仓库评估"的整条链。
5. 学校场景：用户重建 sys01 连接后拷真机画面做叠图（`overlay.py --env-cfg g1d_school`）；要用学校场景做任务时，再写杯盘布局和任务。
6. G1-D 移动版：先修送杯任务布局 0、3 的抓杯（10-28 发现，见「最近任务」第一节）；在隔离目录跑完整 `build_asset.py` 和 `body_probe.py`（两个版本），征得同意再装共享库（连同 `lounge_chair`、带客区和岛台的 `real_scene2`、`g1d_counter_mobile` 布局）；送杯任务再跑 seed 1、2；要采身体自由度数据时，先改 CONTRACT 和录制器（`base_velocity`、`lift_joint_state` 还不进 16 维录制）。

## 最近历史

- 2026-10-29：物体指令名称：`description.json` 新增 `names`（中英文各至少 5 个，从简到详），给 54 件完备物体和 13 个关节瓶子写好（`scripts/objects/object_names.json`），指令模板的 `<标签>` 按语言取名称并跳过两个物体共有的名称；建模 prompt 升到 `real-object-v3`。
- 2026-10-28：物体建模 prompt 升到 `real-object-v2`：实物标签必须从照片做贴图，交付同视角照片对照图；v1 确认需重新确认。

- 2026-10-28：量杯新增 4 个泛化变体 `00002–00005`（同截面，改高度和颜色），`check_object` 全部 OK，已放上数采平台（待审核）。
- 2026-10-28：PR #31–#34 合进 `main`（泛化维度与综合等级、物体资产库分组与审核、塑料马克杯变体、吧台岛台），数采平台部署到合并后的 `main`。
- 2026-10-28：塑料马克杯 `plastic_mug` 补 `measurements_mm`（误差 0），新增 5 个泛化变体 `00001–00005`，`check_object` 全部 OK。
- 2026-10-28：吧台场景按 6 张新照片加过道里的高吧台和绿色岛台（含吧凳、线形吊灯），去掉现场已撤的微波炉，白色圆桌改为 4 把椅子，天花板灯槽改为垂直于墙，右侧木柜墙和饮水机按正面照片重排，垃圾桶加照片贴花；脚本专家送杯后退改为 0.5 m 让出吧凳。`5090-2` 上 RTX 渲染、倒水送杯 1/1、送杯 2/4（另 2 个是抓杯失败，旧场景下一样）。
- 2026-10-27：深圳实验室（整层房间版、574 个评测布局）装进 `5090-2` 共享库；本仓库 6 个开着的 PR（#24、#25、#26、#28、#29、#30）合进 `main`，G1-D 夹爪默认改为 55 N / 3.6e4 并限速 0.064 m/s。
- 2026-10-27：深圳实验室场景按 8 张全景照片补成整层房间（角柱、吊顶 + 穿孔铝板天花和灯条、工位、升降桌、货架、椅子、电梯厅栏杆闸机），`5090-2` 上 RTX 渲染和量杯脚本演示通过。
- 2026-10-24：RSI 导入 ManiTwin 干扰物 390 个（尺寸去重、地面和桌面两道稳定性筛查）、Poly Haven 室内 HDR 40 张；T0 改为统计三套官方评测集（seed 0/1/2），材质成为新缺口（504 种里 494 种被评测用掉）。
- 2026-10-24：RSI 资产分池 `rsi/assets/pools.json`（规则 1 严格口径，T0 813 项），X5 布局生成器和平台接入分池、干扰物池、新实例、换色缩放白名单；`5090-2` 上 `hang_mugs_random` 10 个布局验证正常。严格口径下现成的房间、干扰物、新实例几乎为零，要造新资产。
- 2026-10-25：办公室西墙（有门的墙面）按用户指出的改成磨砂玻璃，门加白框、执手、刷卡器；Isaac RTX 渲染确认后装进共享库并部署。
- 2026-10-24：办公室场景的墙面、柜子、椅子按照片返工（南墙腰线白板墙板、窗台高度、玻璃展示柜 + 木门高柜 + 活动柜、两款办公椅、木纹贴图），离线检查和 15 视角叠图核对通过；`5090-2` 上 Isaac 渲染和三个 `g1d` 脚本演示通过后，经用户同意装进共享库，平台预览已换。
- 2026-10-24：吧台场景补天花板和客区顶部；平台重建的渲染图和 3D 模型不再先显示旧缓存；数采平台部署到 `0d8c8c3`（部署机上别人未提交的改动先存补丁、快进后放回，冲突两处手工合并）。
- 2026-10-24：吧台场景加左侧客区（3 张照片拟合，墙纸凹龛、两张圆桌、可推动的扶手椅刚体）；新增移动版配置 `g1d_counter_mobile` 和第一个移动操作任务 `g1d_deliver_cup_to_lounge`（吧台拿杯送到客区圆桌），脚本专家 seed 0 前 4 个布局 4/4；修复 `is_all_gripper_open` 对 G1-D 夹爪方向判反。
- 2026-10-24：RSI 协议初版 `rsi/AGENTS.md` + `rsi/rsi.example.yaml`：官方评测资产保留、只许泛化不许押题、数据方和评测方对抗且只经裁判交换分档成绩；统计出房间和杂物几乎全是官方保留资产。
- 2026-10-23：上海实验室 `real_scene3_lab` 的墙面不再贴照片，改成纯色饰面加门、白板、电视；角落放了一台由 URDF 烘焙的静态 G1-D（`usd/g1d.usdc`）。离线检查通过；10-24 在 `5090-2` 跑 Isaac 渲染和三个脚本演示，全部通过，经用户同意装进共享库并更新平台预览。
- 2026-10-23：数采平台任务抽屉加「泛化随机方案」，新增 X5 布局生成器 `scripts/datagen/x5_layouts.py`（接手另一会话未提交的实现；相机扰动改为只转角度，平移会让部分环境出图全黑）；`5090-2` 端到端验证 10/10 正常。
- 2026-10-24：Motus eps380 按真机数采对齐（站位、桌高 0.83、杯位、起始姿态、灰 / 蓝杯、1740 颗豆、按回合指令）后在上海实验室仿真测 20 个布局：0/20 成功，灰杯 6/10 有豆子进杯；G1-D 夹爪改为两指联动（控制层）。

- 2026-10-23：G1-D 夹爪限速每指 0.064 m/s（按真机遥操作数据），专家采集通过率不变（6/8 对 6/8），合爪 2 帧 → 5 帧。

- 2026-10-23：农夫山泉塑料瓶和 5 个变体加竖纹螺旋盖、程序绘制的贴图标签；另出能拧盖的 `Articulation/plastic_bottle`（旋转 + mimic 棱柱关节，两圈拧开），新增 `bottle_label.py`、`check_bottle_cap.py`。
- 2026-10-23：真机 Motus 倒豆模型（ModelScope `Vincia/WanVlmMask-G1DPourBeansGenEps222-AbsQpos16-30000`）在上海实验室仿真量杯倒豆 0/20（夹爪总是夹空）；新增 `g1d_lab_beans_collect`，收入 `scripts/g1d/remote_policy.py`。
- 2026-10-22：启动榜单冲分（20 天）：5090-2 建基线运行目录，拉齐官方 Assets、OpenWAM / pi0.5 官方 checkpoint 和两套推理环境，冒烟通过后启动两个模型 54 个任务的全量基线评测；确认 20 个榜单任务没有官方训练数据。
- 2026-10-22：深圳实验室倒豆 500 条（矮量杯或马克杯、300–600 豆、2–5 干扰物、严格整回合验收）采完、审计、上传 ModelScope 私有仓库 `ChongZheng/g1d_shenzhen_pour_beans_cup_or_mug_500`。
- 2026-10-22：实验室场景 `real_scene3_lab` 的工位桌、网椅、活动柜、文件柜、边桌、圆桌建成立体模型（`/Background/Furniture`），从墙面照片里抠掉；隔离目录里三个脚本演示通过后，经用户同意换进共享库（见「最近任务」）。
- 2026-10-21：G1-D 夹爪力反馈：指垫 ContactSensor，输出 `obs["gripper_force"]`（N），不进数据集；5090-2 上量杯 seed 0 通过，夹持时每个指垫 15–22 N。
- 2026-10-21：查了真机 Dex1-1（kp=5、kd=0.05、tau=0，每步超前 0.18–0.3 rad），估算每指约 55 N；仿真夹爪改成刚度 3.6e4、上限 55 N，量杯 seed 0/100/101 回归通过。
- 2026-10-21：物体的颜色、尺寸泛化改为仿真里按布局随机，不建新资产；形状、材质等仍建变体资产，平台上收进本体卡片（用户：泛化放到一个卡片下；能不能在仿真里随机改尺寸和颜色而不是新建资产；用户选定两个都做、随机值写进布局）。
  - 仿真：`RigidObject` 读布局条目的 `visual.color`（线性 RGB）绑一个 PreviewSurface，覆盖资产自带材质；有 `visual.color_parts` 时只换这些网格（保留标签、透明件）。`scale` 本来就从布局读、每次复位重设；新改的是 `LayoutManager.get_instance_bbox_vertices` 对刚体按 `scale` 缩放 metadata 包围盒（判据和脚本专家都用它，原来恒为未缩放值），`RigidObject.get_bbox` 同样处理。现有布局里刚体全是 scale 1（在 5090-2 元数据镜像里全量查过，只有流体豆子不是 1，流体不受影响），所以现有任务行为不变。
  - 布局：`build_asset.py` 新增 `RANDOMIZE`（按任务、标签打开 `{"scale": (lo, hi), "color": True}`）和 `randomize_objects`：按 `任务/seed 目录/序号` 定种子，等比缩放并保持立在桌面上，质量按体积换算，颜色在 HSV 里取（饱和度 0.45–0.9、明度 0.35–0.9）再转线性。**现有任务都没开**：开了会改它们的评测布局；倒豆任务按量杯固定腔体数豆子，不能缩放；`cup_grasp` 读的 metadata `contact_circle` 也没按缩放修正。这些值随 `additional_info/layout` 进每个回合（CONTRACT 已写）。
  - 平台：物体扫描新增 `variant_of`（`metadata.json`，或装库前的 `preview/check.json`，`check_object.py` 会透传）；卡片视图里变体不单列，本体卡片带「+N 变体」角标，抽屉列出变体（点开是变体自己的抽屉，顶部注明本体）；列表视图仍列全部并注明本体。泛化设置加「运行时换色」「运行时缩放 ±%」（`gen.rt_color`、`gen.rt_scale`，写进 `spec.json` 的 `generalization`），卡片显示「随机 颜色 · 尺寸 ±15%」；登记向导第 3 步分成「运行时随机」和「另建变体资产」两组，快捷要求改成材质、把手、不等比这类必须建资产的；建模任务文本相应分两段（运行时随机怎么在 `RANDOMIZE` 打开，变体要写 `variant_of`）。`scripts/objects/SKILL.md`、CONTRACT 同步。
  - 验证：本机 `check.sh` 32 passed 1 skipped（新增 `scripts/g1d/test_randomize.py` 进 `check.sh`：缩放后贴桌、质量、换色可复现、`color_parts`、默认关闭；`test_server.py` 加 `variant_of` 两种来源和 `rt_*` 写进 spec）。`5090-2` 隔离目录 `/data/robodojo-runs/obj-random-20261021-225540`（GPU 5，资产软链接自 `shenzhen-lab-20261021-125246`）：拾取布局把目标换成量杯，分别为 1.0 原色 / 0.85 + 随机色 / 1.05 + 随机色，`scripted_demo` 3 回合画面里颜色、大小都对，杯子全程立在桌上；3 回合都没抓起来，原尺寸基线也一样（拾取脚本抓不了这只量杯，与缩放无关）。在仿真环境里给 isaacsim 打桩直接调用 `get_instance_bbox_vertices`：刚体 ×1.2、流体不变。回归：原始量杯倒豆布局 `g1d_shenzhen_lab` seed 0 `success_rate` 1.0。本机浏览器：造一个 `variant_of` 指向 `measuring_cup/00001` 的测试变体（已删），卡片归组、角标、两个抽屉、建模任务文本、向导第 3 步都正常。
  - 顺带修了上一条提交里 `server.py` 第 4 行超长（检查后才改的文档字符串，已进 `main`）和 `layout_manager.py` 一处老的 UP038。
  - `4f8cccf` 快进推 `main` 并部署：`5090-2` 从 `e4552a2` 快进（部署机 `HANDOFF.md` 的另一会话改动先存 `.cache/handoff-local-20261021-235903.patch` 再放回），重启后扫描正常，未登录 `/api/info` 401。线上还没有登记过 `variant_of` 变体或运行时随机，所以页面暂时看不出变化。
- 2026-10-21：场景卡片改用 Isaac RTX 渲染图做封面，场景抽屉顶部加可拖拽旋转的 3D 预览（用户：卡片放渲染的场景图，网页端 3D 预览）。新脚本 `scripts/scenes/export_glb.py`（只要 usd-core + numpy，本机可跑）把 `usd/scene.usda` 的可见网格（跳过 guide 碰撞体）连同 UsdPreviewSurface 材质和贴图转成 `preview/scene.glb`（Y 轴向上，节点名是 USD 路径）；`scripts/scenes/render_scenes.py`（GPU）只载场景、用场景自带灯光，从机器人一侧斜上方对准 `/World/Background`、`/Lights` 以外的部分（桌子和桌上物体），相机限制在地面范围内、天花板以下，写 `preview/render/sim.jpg`。平台扫描 `renders`、`model`，`/file` 放行 `.glb`；卡片封面优先渲染图，有模型的卡片带「3D」标记；抽屉用 three.js 0.169（浏览器按需从 `registry.npmmirror.com` 加载，importmap，不进仓库）+ OrbitControls：左键旋转、右键平移、滚轮缩放，不能转到地板以下，「隐藏房间」（墙、天花板、背景家具，地面保留）、「重置视角」、「全屏」；墙和天花板单面朝内，从房间外看自动剖开。抽屉每次重绘时把活的查看器节点放回原位，关抽屉或切走时释放 WebGL。`5090-2` 隔离目录 `/data/robodojo-runs/scene-render-20261021-222151`（GPU 5，只拷了场景目录和脚本）渲出 5 个场景，逐张看过；GLB 0.03–2.7 MB。验证：`check.sh`（29 passed 1 skipped，新增 `GlbTests` 和场景 `renders`/`model` 扫描、`.glb` 下载）、新脚本 ruff；本机浏览器 5 个场景都加载、贴图和透明材质正常、模拟拖拽能转、隐藏房间、切换场景后只剩一个画布、关抽屉后画布释放。`real2sim/scenes/SKILL.md`「接入和登记」和平台建模任务文本加了这两步。`e4552a2` 直接快进推 `main` 并部署：`5090-2` 从 `01e50fe` 快进（推临时分支再 `--ff-only`，部署机 `HANDOFF.md` 的另一会话改动先存 `.cache/handoff-local-20261021-222905.patch` 再放回），5 个场景的 `scene.glb` 和 `render/sim.jpg` 拷到部署机同一路径，重启后扫描到模型和渲染图，未登录 `/api/info` 401。没有登录线上页面看（没有账号密码）。
- 2026-10-21：登记向导（物体、场景、任务、本体、评测）误关不再丢内容（用户：点到左边区域填的都没了）。抽屉关闭或切走时（点遮罩、Esc、关闭按钮、点别的行、换页面），如果表单相对刚打开时有改动、选了照片或物体 / 场景 / 任务，就把抽屉节点和向导状态（当前步骤、照片、各选择框的列表）暂存在页面内存里，并提示已暂存；再点同一个登记按钮原样恢复，顶部提示「已恢复上次没提交的内容」和「清空重填」。提交成功后清掉暂存。只存在当前页面，刷新浏览器就没了。验证：ruff；本机浏览器：物体向导填两步后点遮罩、重开恢复（步骤、名称、质量、类别），换页面后仍能恢复，清空重填得到空表；没改动的表单关闭不暂存；任务向导选了物体后按 Esc、重开恢复，提交后暂存清空。
- 2026-10-21：数采平台新增「模型评测」选项卡（用户：提交评测任务，给定模型、部署代码和要测的任务，负责 Real2Sim 的同学交给 coding agent 部署和评测）。提交向导三步：① 模型名称、版本、框架、checkpoint 位置、部署代码（必填）、训练数据，接口（本体、动作空间、图像输入、指令语言）、推理环境、显存、动作块；提示仓库公开、不要填凭据；② 任务（带预览的搜索框，至少一个）、可选限定场景、泛化等级（默认 L0、L1）、每个任务 × env 配置的布局数（默认 50）和重复次数（默认 3）、是否加脚本专家对照；③ 截止、优先级、评测负责人（可空，等认领）。编号 = 模型名 slug + 日期，同时是 eval_result 里的策略名。写 `real2sim/evals/<编号>/request.json`；提交人由服务端按登录账号写，后续更新不能改，新申请状态为「待受理」。抽屉：部署与评测任务文本（隔离目录、凭据不落盘、XPolicyLab 服务或 `--policy_module` 适配器、按 CONTRACT 对齐、冒烟后全量、颗粒任务用 `g1d_beans_collect`、同步 `eval_result/`、写 `result.json` 的格式、更新状态），受理（状态、评测负责人、评测记录），结果（`result.json` 的总成功率、按任务 × env 配置 × 等级的成功率 / 区间 / 专家对照、问题；以及 eval_result 里同名策略的回合，可跳质检），申请内容。datagen SKILL 补了这条流程。验证：`test_server.py`（申请写入、提交人不可伪造、状态和负责人更新、`result.json` 扫描、坏 id、删除）、ruff；本机浏览器：成员账号提交（校验、编号、任务搜索和预览）、模拟 `result.json` 后结果表和列表成功率、改状态为已完成。
- 2026-10-21：本体卡片加仿真渲染图。新脚本 `scripts/embodiments/render_embodiments.py`：按 `env/robot_manager/robot_config/<robot_name>.py` 把每个本体单独放在地面上（最低网格点落地，固定底座所以和臂姿无关），初始化并保持默认关节角后，按连杆实际位置加边距取景（`set_world_pose`，初始化前的 USD 包围盒和稳定后的姿态对不上，Franka 第一版被裁掉），写 `real2sim/embodiments/<id>/preview/sim.png`（不入库）。平台扫描 `preview/`：卡片封面优先用渲染图，抽屉分「仿真渲染」「实拍照片」。`5090-2` 隔离目录 `/data/robodojo-runs/embodiment-render-20261021-144517`（GPU 4；G1-D 资产复制自 `g1d-mobile-20261021`，X5 / Franka 从共享库拷贝，共享库未改；要设 `OMNI_KIT_ACCEPT_EULA=YES`）渲出 g1d、g1d_mobile、x5、franka 四张，拷到部署机同一路径。两版 G1-D 外观相同（移动版只多底盘自由度），靠「变体」区分。验证：`test_server.py`（renders 扫描）、ruff；四张图逐张看过。
- 2026-10-21：经用户同意，本会话把 PR #17（G1-D 桌面版 / 移动版，另一会话）合并进 `main`（`33c7157`）：在 `1d110fb` 上合并当时的 `main`，冲突在 `HANDOFF.md`、`README.md`、`_robot_info.json`、`CONTRACT.md`，都是两边各加内容，全部保留（CONTRACT 夹爪那条改为七个 G1-D 配置，HANDOFF 里桌面版配置数改为六个）；`check.sh` 27 passed 1 skipped（含 `test_g1d_body.py`）。直接推 `main`，PR #17 自动标为已合并。部署到 `5090-2`：平台服务用系统 Python 导入新 `build_asset.py` 正常，本体页出现 `g1d_mobile`（共 4 个本体）；部署机上和 `main` 一致的 `real2sim/embodiments/g1d/registration.json` 先挪开、快进后核对一致。另一会话的 worktree `../RoboDojo-g1d-mobile` 还停在 `1d110fb`。
- 2026-10-21：G1-D 分为桌面版 `g1d`（腰、升降可选动作键）和移动版 `g1d_mobile`（加差速底盘 `base_velocity`，虚拟平面关节），接口按官方 unitree_sdk2；`5090-2` 上两版 `body_probe.py` 和量杯回归通过（分支 `chongzheng/robot-g1d-tabletop-mobile`）。
- 2026-10-21：在线上平台代填 G1-D（`g1d`）的本体信息，内容全部取自本文件和配置：Unitree G1-D 桌面版、双臂 7 + 夹爪 1、Dex1-1 带相机夹爪、头部左眼 + 两腕部相机、30 Hz、`real2sim/robot/g1d_v4`；标定只有相机标为已完成（fx≈279.5 / 314，误差 0.4–3.6 px），关节零位无记录，夹爪和动力学未做，记录里写明原因和标定工具包未接入。署名「种政（agent 代填）」，负责人 chongzheng；`real2sim/embodiments/g1d/registration.json` 拷回入库。从这次起，按用户要求不再开 PR：仍在 worktree 短分支上改、本机跑检查、变基到最新 `origin/main` 后快进推送（不强推）。
- 2026-10-21：数采平台新增「本体管理」选项卡（用户：G1-D 至少有桌面版和移动版两个本体了）。本体 = `env/robot_manager/robot_config/` 里的每个 robot_name（现在 g1d、x5、franka；PR #17 合并后 g1d_mobile 自动出现），加平台登记的本体；同一本体在各场景的站位配置（`env_cfg/robot/*.yml`）算一个，列出站位、臂 / 末端维度（`_robot_info.json`）、根节点位置、加载它的 env 配置和场景、有布局的任务和采集回合。抽屉可传照片，填名称、型号、变体、负责人（账号）、自由度、末端执行器、相机、控制频率、URDF 来源、真机、备注，以及相机 / 关节零位 / 夹爪 / 动力学四项标定状态和记录；流程 = 仿真接入、站位配置加四项标定。录入写 `real2sim/embodiments/<id>/registration.json`。「登记本体」两步向导给还没接入仿真的本体用，生成接入任务文本（URDF 转 USD、robot_config / robot_class / env_cfg/robot / `_robot_info.json`、对齐 CONTRACT、5090-2 关节探针）。顺带修复：抽屉里连续改两个字段时，第二次保存基于旧数据覆盖了第一次（`onFieldChange` 先在本地记录生效再保存）。验证：`test_server.py`（新增 `test_embodiments`、本体登记 / 照片 / 删除）、ruff；本机浏览器：三张本体卡片、G1-D 抽屉的 5 个站位、连续改三个字段都保存、登记新本体生成接入任务。PR #23 合并（`31ef96a`）并部署。
- 2026-10-21：按平台登记新建深圳实验室场景 `real_scene5_shenzhen_lab`（env 配置 `g1d_shenzhen_lab`），卷尺定尺度，5090-2 上 `build_asset.py` 和脚本演示通过，经用户同意装库（不含量杯任务布局；装进的是迁移前的旧库，5090-2 共享库待补）；修 `build_asset.py` 相对 `--assets` 时的软链接问题。PR #22 合并（`2975b4a`）并部署到 `5090-2`。
- 2026-10-21：登记任务的评测保留物体改为跟着角色走（用户：每个角色都要有候选和保留，干扰物也要有）：第 2 步每个角色分「训练候选」「评测保留」两个选择框，干扰物分「训练用」「评测保留」，去掉单独的「评测保留实例」栏；已是训练候选的物体不能再选为保留，反之亦然（保留的必须是训练里没见过的）。L3 只在选了保留物体时开启（多候选、干扰物只算训练多样性）。「目标布局数」改名「评测布局数」并注明只用于评测、训练数据量看合格回合目标。`registration.json`：角色和 `distractors` 各加 `holdout`，去掉 `holdout_objects`。顺带修复 PR #15 把 PR #14 的两处改回旧版的问题：实现任务文本里的杂物类别、抽屉里的「含杂物库：类别」（登记文件的 `clutter_categories` 一直正确）。验证：`test_server.py`、ruff；本机浏览器：两栏选择、双向互斥提示、L3 依据、评测布局数、实现任务文本和抽屉摘要。PR #21 合并（`d5db8aa`）并部署。
- 2026-10-21：PR #19 合并（`2944698`）并部署；部署前删掉了部署机上和 `main` 内容一致的未跟踪 `real2sim/tasks/g1d_pour_beans_any_cups/registration.json`（sha256 核对过）。线上第一个场景登记 `real_scene5_shenzhen_lab`（深圳实验室，B5-2L，种政 10-21 12:08 登记：6 张照片、卷尺桌面 75 × 120 × 60 cm、厚 2 cm、四条腿白色圆角桌、储物柜 75×120×45 cm 金属、顶部条形面光源）拷回入库，照片走 LFS。还没建模，没量机器人站位，也没有标定板；部署 PR #21 时已核对一致、挪开后快进并放回真实照片，部署机上现在是跟踪文件。
- 2026-10-21：登记任务的泛化等级不再手动勾选，按前面的选择自动开启（用户：干扰物、换物体、选场景本身就是泛化，最后一步重复）：L0 始终开；L1 = 第 4 步勾了位置或朝向；L2 = 第 3 步新加的「外观随机」（灯光、纹理、相机扰动）勾了任一项；L3 = 某角色候选物体多于 1 个、有干扰物或选了保留实例（保留实例挪到第 2 步）；L4 = 选了保留场景。第 4 步只显示各级开没开和依据，开启的级可改范围和目标布局数。`registration.json` 新增 `appearance`。另外三个登记向导对管理员不再强制「填完才能下一步」：跳过的项在底栏列出；提交前仍检查 ID（物体类别和编号、场景目录名、任务 ID），对管理员也生效；普通成员不变。验证：`test_server.py`、ruff；本机浏览器：管理员空表跳到最后一步、空后缀提交被拦、等级随选择开关、取消位置和朝向后 L1 关闭、L4 目标改 80 写进登记；场景向导管理员可跳；普通成员三个向导都被拦在第 1 步。
- 2026-10-21：按用户口述在线上平台代登记任务 `g1d_pour_beans_any_cups`（任意杯子倒红豆）：源杯 / 目标杯各从 8 个杯子资产里选（measuring_cup 0/1、plastic_cup 0、plastic_mug 0、paper_cup 0/1、water_cup 0、cup 7；不能同一实例），红豆 100–300 颗随机，倒完两只杯子直立放回桌面、两臂回初始姿态且夹爪张开。agent 补的部分：成功判据阈值（目标杯 ≥ 95%、杯子倾角 < 10°、关节误差 < 0.05 rad）、35 s、训练场景为 4 个已建模场景、L0 + L1（位置和朝向随机）、中英文各 5 条指令。直接在 `5090-2` 用 `server.Store` + `write_task_registration` 写入（署名「种政（agent 代登记）」，负责人 chongzheng），`registration.json` 拷回入库。还没实现；两只量杯和 plastic_mug 不在共享库，实现时要先补。部署机上这个文件是未跟踪的，下次快进部署前先删掉它（内容和入库的一致）。
- 2026-10-21：登记任务的位置随机移到第 4 步 L1（用户：位置泛化应属于泛化等级），第 2 步只选物体；L1 下勾「位置 / 朝向」，训练布局和 L1 评测集都按它们随机，范围仍由 coding agent 定，都不勾即固定摆放。`registration.json` 的随机字段从每个角色的 `randomize` 移到任务级 `randomize`。第 3 步场景不再每个场景一个下拉框（场景多了不好选），改为「训练场景」「L4 保留场景」两个搜索选择框：和物体选择共用组件（`PICK` 按键名区分物体和场景），带缩略图和悬停预览（桌面尺寸、env 配置、照片数），同一场景只能在一边，选进另一边自动移走；「加入全部已建模场景」一键加入。验证：`test_server.py`、ruff；本机浏览器：加入全部已建模场景、搜索、悬停预览、回车把实验室移到 L4 保留、L1 取消朝向后提交，登记和实现任务文本正确。PR #16 合并（`f717246`）并部署。
- 2026-10-21：登记任务的物体角色不再手填 x/y 范围和朝向角度（用户：不要人为定这么细），改为「每回合随机」两个勾选项：位置、朝向（默认都勾）。`registration.json` 的角色字段由 `xlim/ylim/yaw_deg` 换成 `randomize: ["position", "yaw"]`；实现任务文本让 coding agent 按可达范围（桌面物体 y≈[-0.55, -0.25]）、物体不重叠且留抓取空间、按抓取方式定朝向来设范围，L0 孪生仍固定摆放，范围写进任务配置和交付说明。线上没有登记过任务，不用迁移。验证：`test_server.py`、ruff；本机浏览器里勾选、取消、提交，看了实现任务文本和抽屉摘要。PR #15 合并（`e1b347c`）并部署。
- 2026-10-21：登记任务里的物体选择改为带预览的搜索列表（用户反馈：选的时候看不出是哪个）：输入名称、类别或 id 过滤，每行有缩略图，鼠标悬停（或方向键）在旁边浮出大图卡片（名称、id、尺寸、质量、入库状态；图取渲染图，没有就取实拍），回车或点击加入；已选标签带缩略图，悬停也能预览。干扰物的杂物库说明清楚了：它是共享资产库 `Object/RoboDojo/Clutter/`（95 类 193 个，`clutter.yml` 索引），任务配置的 `Clutter` 段引用它；勾选后列出各类别和数量，可只选部分类别（不选即全库），写进 `distractors.clutter_categories`。新接口 `GET /api/clutter` 从资产库镜像统计类别。杂物没有渲染图，只显示类别名。验证：`test_server.py`（`/api/clutter`）、ruff；本机用部署机的元数据镜像和主工作区的渲染图在浏览器里试了搜索、悬停预览、方向键 + 回车、标签预览、杂物类别写入登记。PR #14 合并（`99cf0b6`）并部署到 `5090-2`，线上镜像统计出杂物库 95 类 193 个。
- 2026-10-21：任务登记不再让用户写语言指令：第 1 步改为中文「任务定义」（必填）、指令转写数量 N（1–50，默认 5）和指令语言（中文 / 英文，可多选，每种语言 N 条）。`registration.json` 的 `instruction` 换成 `definition` 和 `instructions: {count, languages}`。实现任务文本要求 coding agent 从定义总结指令并转写（不加定义外的信息，第一条为规范说法），写进 `real2sim/tasks/<任务>/instructions.json`，由 `gen_instruction` 返回（`desc_manager` 每回合随机选一句），用户审过再批量采集。平台扫描该文件：任务流程新增「语言指令」阶段（条数够了才通过），抽屉显示已写的转写和条数。去掉「其他泛化」里的「语言指令改写」预设。线上还没有登记过任务，不用迁移。验证：`test_server.py`（新字段、扫描 instructions.json）、ruff；本机浏览器里校验（缺定义、N=0、不选语言都被拦）、提交、放一份 2 条的 instructions.json 后阶段显示不足、抽屉列出转写。PR #13 合并（`518ec9c`）并部署到 `5090-2`。
- 2026-10-21：任务负责人改为从平台账号里选（登记向导和任务抽屉都是下拉框，默认当前账号；存用户名、显示显示名）。新增所有登录用户可读的 `GET /api/members`（只有用户名和显示名，`/api/users` 仍只给管理员）；任务登记 JSON 的 `owner` 改为 `{username, name}`，取自任务记录顶层的 `owner`，不再放在 `reg` 里。线上还没有人填过负责人，不需要迁移。验证：`test_server.py`（成员接口对普通成员 200 且不含权限字段、登记 JSON 的负责人）、ruff；本机用非管理员账号在浏览器里看了两处下拉并保存。PR #12 合并（`fc0a742`）并部署到 `5090-2`，重启后未登录 `/api/members` 401、公网页面 200。
- 2026-10-20：数采平台新增「登记任务」四步向导（新建或给已有任务登记泛化方案：物体角色和候选物体、干扰物、训练 / L4 保留场景、L0–L4 等级和范围、其他泛化），写 `real2sim/tasks/<任务>/registration.json`（PR #11，已部署到 `5090-2`）。
- 2026-10-20：数采平台新增「登记场景」三步向导（≥ 5 张照片 + 卷尺实测 + 站位、标定板、环境、灯光等），写 `real2sim/scenes/<场景>/registration.json`，抽屉生成建模任务文本（PR #10，已部署到 `5090-2`）。
- 2026-10-20：按平台登记新建耳机充电盒 `airpods/00000`（`make_airpods.py`，PR #7），实测尺寸和质量，`5090-2` 落地检查通过；经用户同意装进共享库，平台镜像已补上。
- 2026-10-20：物体流程在建好之后能自动推进了（用户反馈 airpods 建好后流程不动、物理检验不应全靠人工、「训练 / 保留划分」没用）。原因：平台只看资产库镜像，agent 在隔离目录建好、未装共享库时平台看不到；`check_object.py` 只打印结果；尺寸写成「（高 × 宽 × 厚）：a × b × c」时解析成乱码，和 `measurements_mm` 对不上。改动：`check_object.py --image-dir` 在 `preview/` 里和 `sim.png` 一起写 `check.json`（落地结果、质量、尺寸、`measurements_mm`、资产路径），agent 可补 `checks`/`notes`（碰撞、抓取、盛液、渲染）；平台扫描它，资产库没有时用它的质量和尺寸做核对，「建模」与「入共享库」拆开，物理检验取 `check.json`、人工结论优先；尺寸解析支持括号名 + × 分隔值；删掉物体的训练 / 保留划分（没有代码读它）。建模任务文本和 `scripts/objects/SKILL.md` 同步。验证：`check.sh`、`test_server.py`（新增 check.json 回退）、解析 4 种写法、本机浏览器用 airpods 的真实 metadata 造 `check.json` 看流程 3/5 和物理检验显示；`check_object.py` 只做了编译检查，没在 GPU 上跑。airpods 要在合并部署后重跑一次 `check_object.py --image-dir` 并同步 `preview/`，流程才会在线上推进。
- 2026-10-20：数采平台「登记物体」改成三步向导（分支 `chongzheng/platform-object-wizard`）：① 中英文名称、类型（类别由英文名自动生成，可改）、编号，多角度照片可拖放、可不传；② 实物测量（材质、颜色、质量、静摩擦、尺寸、外观、建模要求，可留空，但没照片时必须填尺寸）；③ 泛化：变体数量、允许变化的属性（颜色/材质/尺寸/形状）、泛化要求（可只写数量和要求，让 coding agent 设计参数）。`spec.json` 新增 `name_en`、`generalization`；物体抽屉可改英文名和泛化，「建模任务」文本按泛化设置要求在同类别后续空号建变体（不实测、标为泛化变体）。界面按 UI UX Pro Max skill（装在 `~/.claude/skills/ui-ux-pro-max`）的表单规范做了第二版：圆点连线步骤条、上标签两列网格、带单位输入、资产 ID 标签、大拖放区和方形缩略图、属性可选标签、数量步进器、泛化要求快捷示例、吸底操作栏；校验错误标在对应字段（`aria-invalid` 并聚焦），换步骤时聚焦首个输入，提交中按钮显示进度。验证：`test_server.py`（spec 新字段）、ruff；本机起服务在浏览器里走完三步（含 1280 宽和手机宽截图核对；校验、英文名转类别、拖放过滤非图片、提交、抽屉里改泛化写回 spec.json）。PR #5 合并（`8da8d30`），19:13 部署到 `5090-2`：推到临时分支再快进（线上未入库的登记改动不受影响），重启服务后页面和未登录 401 正常，隧道在。线上登记的 `measuring_cup/00001`（删 `IMG_4871.jpg`、新增 3 张照片和 `spec.json`，其中 `tempImageK59SQD.jpg` 与 `tempImageKLstnv.jpg` 内容相同）和 `plastic_mug/00000/spec.json` 拷回入库（分支 `chongzheng/platform-registrations-1020`）；合并后部署机要先删掉这些未跟踪的同名文件再快进。部署机上还有不明来源的未入库目录 `real2sim/scenes/g1d_green_table/`，没动。
- 2026-10-20：新增物体建模 skill `scripts/objects/SKILL.md`（按平台登记建实物孪生刚体资产：各层放什么、生成脚本和 `metadata.json`/`measurements_mm` 约定、`5090-2` 隔离目录验证、登记与提交），`CLAUDE.md`、README 和平台「建模任务」文本指向它。用只读子 agent 按它规划重建 `plastic_mug/00000` 做了两轮演练，按发现的问题修正（现有 `metadata()` 还不写 `measurements_mm`、外径要排除把手、一个脚本生成整族物体、老脚本参数是类别目录等）；里面的服务器说明并入了另一会话「服务器切到 5090-2」对这份文件的改动。没有在 GPU 上按它实际建过物体。
- 2026-10-20：数采平台图片和视频可以被浏览器缓存了（用户反馈每次打开都要重新加载，很慢）。原因：`/file` 不带 `Cache-Control`、`ETag`、`Last-Modified`，浏览器每次都整张重下；经隧道实测约 400 KB/s，物体页 63 张渲染图约 9.8 MB。现在 `/file` 带 `ETag`（mtime + 大小）和 `Last-Modified`，`Cache-Control: private, max-age=60, stale-while-revalidate=604800`，`If-None-Match` 命中回 304；页面本身 `no-cache`，部署后立即生效。服务端改为 HTTP/1.1 长连接（空闲 60 s 断开；在读请求体之前就拒绝的回复带 `Connection: close`，416 带 `Content-Length: 0`，视频拖动断开时关连接）。验证：`test_server.py`（新增 ETag/304、页面 no-cache、同一连接连续 416/200/304、未登录上传关连接）、ruff 0.11.7；浏览器里第二次打开物体页，68 个文件全部来自缓存，传输 0 字节（第一次 9.8 MB）。未做：缩略图（服务器系统 Python 没有 Pillow，标准库解不了图片），首次打开仍要下原图。
- 2026-10-20：学校场景和杯盘上了数采平台：部署仓库已在 `aea8d4a`（另一个会话部署）；补了场景的 LFS 文件，场景预览图（两张源孪生渲染转 jpg、一张 Isaac 头部画面）放在部署机的 `real2sim/scenes/real_scene4_school/preview/`（不入库），物体渲染图放在 `real2sim/objects/Rigid/paper_*/*/preview/`，元数据放进资产库镜像；重启服务后用平台的扫描函数核对：场景关联 `g1d_school`、已构建、3 张预览；5 件物体都有生成脚本、质量、尺寸和渲染图。页面要登录，没有在浏览器里看。
- 2026-10-20：接入学校实验室绿幕桌（`real_scene4` / `g1d_school`）和纸杯、纸盘资产，来自同事的 MuJoCo 孪生（PR #2）。
- 2026-10-20：采集、评估流水线规范化合并进 `main`（PR #1）：数据契约、通用采集流水线、场景工具、Agent 文档和 CI。
- 2026-10-20：随机朝向那批 100 条数据私有上传 ModelScope（`4562fb4`）。
- 2026-10-19：随机倒豆手、杯子偏航和大范围杯位（`a1b6db5`），修复量杯在倒豆循环里的豆子计数；按这套设置采完 100 条，左右手各 50（`353730a`）。
- 2026-10-19：100 条量杯数据上传 ModelScope 私有仓库；平台的物体登记简化（取消角色字段，尺寸改为自由文本）。
- 2026-10-19：用实测量杯重采 100 条；夹持点从杯口下 32 mm 改为 45 mm（高杯在夹爪里会转动）。
- 2026-10-19：量杯、塑料马克杯、红豆按实测重建；红豆改用等体积的 5.6 mm PBD 颗粒，0.1 g。
- 2026-10-19：新增数采平台（场景、物体、任务、质检、Sim↔Real 指标），加了账号、管理员和物体照片，部署到 `5090-2`。
- 2026-10-19：旧量杯加 4 mm 豆子的 100 条批次被取代（导出停在 8 条）。
- 2026-10-17：新任务 `g1d_pour_beans_measuring_cups`；接入带相机的 Dex1-1 夹爪；新增三个塑料杯资产。
- 2026-10-16：三个场景各自成为 env 配置并装入共享库；自建资产移到 `real2sim/`；相机按棋盘格和真机画面重新标定；新任务 `g1d_pour_beans_cup_to_cup`。
- 2026-10-14：倒水任务改为倒红豆；修复拾取和叠块（TCP 雅可比、线性 PD 定心）；重建实验室场景。
- 2026-10-13：搭建服务器环境，接入 G1-D + Dex1；冷水壶资产，以及流体检查只能走 CPU 状态管线。
