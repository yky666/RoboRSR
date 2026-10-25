# G1-D + 极光Lite / G1-WS v3（2026-10-24）

这是完整G1-D候选装配的纯仿真整合交付，未连接真机、DDS或真实相机；未提交、暂存或推送Git。

- 完整模型/源CAD/零件分组/碰撞与控制、开度与坐标对齐保留；77个URDF/STL与v2逐字节一致。
- 新完整双手重力任务：最终30mm20g和45mm50g都完成拾取、抬升、搬运、竖直放置、释放、撤离。另保留低摩擦失持、触觉越界、放置翻转等失败。不是统计成功率或实物精度。
- 厂家CPython3.10独立Simple/FEM四路输出；9351个资源SHA已复核。四路链路、原子发布、有限等待、HDF5、LeRobot v3、实际策略输入读回有证据。
- evidence/dataset_wide_45mm包含154帧完整无损HDF5和四触觉AV1的LeRobot样本。其它完整回合有全部数值、关键帧、原生/触觉日志和连续视频；旧夹具证据独立保留。

## 使用

先解压。APPLY_TO_REPO.py默认只校验；--apply才复制。它不操作Git，拒绝覆盖未知改动，支持基准HEAD或先前v1/v2。repo_overlay是提交候选文件，evidence是审阅材料，不要整包提交到Git。

python APPLY_TO_REPO.py --target /你的/RoboRSR
python APPLY_TO_REPO.py --target /你的/RoboRSR --apply

本体安装、完整任务、资源与数据链路命令见repo_overlay/scripts/g1d/xense_gripper/README.md及DATA_PIPELINE.md。完整任务实验的物理/录制频率240/10Hz；不冒充旧三RGB30Hz基准。虚拟链接空视觉引用有导入器告警，实际所需链接和22关节已运行验证。

本机审阅 http://127.0.0.1:8783/ ，本体管理 http://127.0.0.1:8782/#robots 。离线可在evidence/review启动支持Range的本地服务；不要直接file://打开期望视频任意跳转。

## 厂家私有资源

厂家原包已在本机 /mnt/data/yangky/g1d/vendor/xense/20261022/，不必重新下载。private_runtime_manifest.json列原ZIP、9351个运行文件、FEM/通用标定摘要；runtime_resources.py可复核或把原ZIP私有安装到新目录。原包含专有许可，未得到公开再分发授权，因此二进制/FEM原件不放repo_overlay或本ZIP。它们不是四个真实SN的标定文件；当前仅使用通用G1-WS仿真资源。

## 必须保留的实测边界

当前G1-D腕孔位公差/转接板、安装变换与TCP实测、当前机器零位和相机手眼、部件实测惯量、伺服/摩擦/胶体响应及真实SN标定原件仍未取得。旧标定包明确来自另一台机器，只作参考；未自动应用。厂家通用法兰未明确适配G1-D，未凭外观虚构可安装结论。

主PhysX胶面仍为刚体，厂家FEM是单向观测；没有双向软体耦合、布料实物对应、30Hz实时触觉闭环或已训练触觉策略验收。所有待确认参数仍pending。

逐项依据见本体calibration/README.md、validation/research_sources.json与task_campaign.json。包内摘要和应用检查结果便于独立核对。
