# 本地数据盘点

登录后，页面经已有的受认证 `/file` 接口读取仓库运行目录下的 `output/dataset_inventory.json`。
不需要重启服务。该文件不入 Git；不要把私人路径、连接信息或内部来源写进公开 HTML。
读取失败会明确提示当前只有缓存目录，不将缓存说成完整盘点。

数据集管理员在页面上的修改（名称、机构、链接、简介、口径、三个统计数、各分类）存为平台数据库里的 `dataset` 记录，
按组 `id` 叠加在盘点之上，只存和盘点不同的字段；盘点文件不变，「恢复盘点值」删掉这条记录。
管理员新增的条目是 `custom: true` 的整条记录，不在盘点里。重新生成盘点时组 `id` 要保持不变，否则旧修改不再生效。

JSON 顶层为 `schemaVersion: 1`、`date`（YYYY-MM-DD）、`recordCount`、`method` 和 `groups`。
每组包含唯一 `id`、`name`、可选 `templateId`（沿用公开目录分类）和非空 `versions`。
每条版本包含唯一 `id`、`name`、公开 `url`（或空串）、`source`（real/sim/ego/mixed/asset/unknown）、
`state`、`episodes`、`hours`、`gb`、`selectedEpisodes`、`selectedHours`、`note` 和 `selectionNote`。
数值只能是非负数或 `null`；所有版本数必须等于 `recordCount`，每条原记录恰好保留一次。

组可选 `declared`：发布方自己给的总量（数据集卡片、仓库 `meta/info.json`、论文），字段 `episodes`、`hours`（至少一个非空）、
`url`（必填，https）、`quote`（含数字的原文）、`unit`、`scope`、`qualifier`（约 / 超过 等）、`derived`、`note`、`checked`（查证日期）。
只在本地值未知时显示并标「官方」，不进入 `selected*`，也不计入顶部合计；不能推算，查不到就不写。

- `episodes/hours/gb` 保留原记录；原记录完整时长未知时，不能用部分实测覆盖它。
- `selectedEpisodes/selectedHours` 明确此次归组汇总选了哪些值。重复格式/分辨率/已知子集保留明细，选择值为空，并写明原因。
- 部分实测时长可以单列选入，但必须在 `selectionNote` 说明覆盖量与缺口。
- 视频片段不计入机器人 Episode 总数。未知值不当作零。
- 同源关系不确定时说明采用的主版本，不能声称已完成内容去重。
- 目录体量保留全部副本，可能嵌套重叠，不代表去重后的实际占用。
- 不同名称的合集之间仍可能交叉；顶部是按版本选择的已知部分，不能称为严格唯一总量。
- 页面显示时换算单位并保留三位有效数字：条数用万/亿，时长不足 1 h 用分钟，体量用 MB/GB/TB/PB（1 GB = 10^9 B）；JSON 里存原始精确值，精确值在悬停提示和详情里。

验证：`node src/real2sim_platform/test_dataset_catalog.cjs [output/dataset_inventory.json]`。
部署前备份现有 JSON 和 HTML，以临时文件校验后原子替换；页面与数据应一起检查。
