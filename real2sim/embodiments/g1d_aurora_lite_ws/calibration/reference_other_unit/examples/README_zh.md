# 数据样例（只适用于原来那台 G1-D）

> **这里的所有数据都来自一台具体的 G1-D（2026-10-16 ~ 10-19）。** 相机内参、板位姿、姿态表、辨识参数在另一台机器人上都不能直接用；它们的用途是：看清文件格式、作为第一次运行的参照、核对你得到的数值是否在同一量级。

| 文件 | 格式 | 内容 | 用在哪 |
|---|---|---|---|
| `intrinsics/camera_intrinsics_head_left.json` 等 4 个 | `g1d_camera_intrinsics_v1` | 头部左/右目、左腕（ZMQ 55557）、右腕（55556）内参 | 拷到 `pc/calib_data/intrinsics/`（PC 脚本）和服务器 `real_data/<日期>/intrinsics/` |
| `joint_mapping.json` | `g1d_joint_mapping_v1` | 14 个手臂关节的电机序号、sign（全 +1）、方向检查余弦；相机端口 | 会话记录引用；新机器人方向检查后重写 |
| `dircheck_plan.json` | — | 方向检查用的准备位姿和每个关节的微动方向、离桌余量 | `pc/dircheck.py` 读 `pc/frames_test/dircheck/dircheck_plan.json` |
| `head_cam_T.json` | — | 头部左右目在躯干系的位姿（`dump_head_cam_T.py` 输出） | 和 `pc/locate_board.py` 里写死的值对应 |
| `poses_20261019/poses_left.json`、`poses_right.json` | `g1d_calib_pose_list_v1` | 10-19 下午布局的正式姿态表（左 47 / 右 43 个可用） | 机器人 `~/calib_sessions/poses_20261019/` |
| `poses_20261019/scene_header_B.json` | `g1d_calib_measurements_v1` 表头 | 真实板参数 + 真实内参 + 实测板位姿 | `export_meas.py --header`；机器人脚本读腕相机内参 |
| `poses_20261019/board_pose_measured.json` | `locate_board.py` 输出 | 头部双目测得的板位姿（左右目各一个 `T_torso_board`） | `gen_poses.py --board-json`、`touch_identify_auto.py` |
| `poses_20261019/summary.json` | — | 姿态表统计（预测精度、条件数、跳过数、相机到板距离） | 核对 |
| `params_1019/params_left_1019.json` | `g1d_calib_params_v1` | 左臂两个会话的辨识结果 | 触碰交叉验证 `--fixed` |
| `params_1019/params_dual_1019.json` | 同上 | 双臂 + 共享板 | 同上 |
| `params_1019/params_dual_trans_1019.json` | 同上 | 双臂 + 共享板 + 关节平移（3 mm 先验）；**触碰验证效果最好（5.3 → 1.6 mm）** | `apply_calibration.py`、`make_urdf_variants.py` |
| `touch_auto_left_20261019_1757/touches.jsonl` | `auto_touch.py` 输出 | 17:57 批次（姿态 013/033/034/036，±4 cm 偏移）的触碰记录 | `touch_identify_auto.py` 的输入样例 |
| `touch_auto_left_20261019_1757/log.txt` | 文本 | 同一批次的运行日志（逐步接近、接触判定、跳过原因） | 看懂日志 |

参数文件里 `measurements` 字段记录的是服务器上测量文件的绝对路径和哈希，只作溯源用。
