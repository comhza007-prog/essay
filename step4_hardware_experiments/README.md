# Step 4 实物实验数据与正式封档

本目录只保存双驱龙门实物台架实验材料，与 Step 1–3 数值仿真数据严格分离。原始日志不手工改数；清洗、事件对齐、统计和绘图均由封档内脚本生成。

## 已完成实验

| 实验 | 工况 | 重复次数 | 有效性 | 核心结果 |
| --- | --- | ---: | --- | --- |
| 实验一 | 空载、50 mm 低速对称往返 | 5 | 5/5 有效 | 保持位置 `50.0255 ± 0.0133 mm`；左/右跟踪 RMSE `0.5160 ± 0.0008 mm`、`0.4974 ± 0.0020 mm`；同步 RMS `0.1522 ± 0.0043 mm` |
| 实验二 | 2 kg 直接悬挂载荷突变，随后 50 mm 往返 | 5 | 5/5 有效 | 突变位置误差峰值 `0.0008 ± 0.0004 mm`；左/右完整轨迹 RMSE `0.6280 ± 0.0084 mm`、`0.6276 ± 0.0047 mm`；同步 RMS `0.2179 ± 0.0370 mm` |
| 实验三 | 2 kg、10 mm 往返，4 种控制器 × 2 种限流 | 40 | 40/40 有效 | 800 限流下，C2b-SyncAlloc 相对 C2b 的跟踪 RMSE、同步 RMS 和最大同步误差分别降低 `36.7%`、`30.8%`、`37.9%` |
| 实验四 | 2 kg、10 mm 往返，C2b/C2b-SyncAlloc × 3 种非对称限流 | 30 | 30/30 有效 | 1600/800 下同步 RMS 降低 `38.6%`、最大同步误差均值降低 `38.2%`，跟踪 RMSE 增加 `5.8%` |

四组实验均已完成正式封档。实验一、二无故障、无执行器饱和、无样本序号丢失；实验三、四所有正式运行均完整到达 `phase=6` 且故障码为 0。实验二反馈电流仍使用 `raw` 单位，不能在未标定时写成安培。

## 仓库内容

```text
step4_hardware_experiments/
├── archives/
│   ├── experiment1_full_archive_20261001.zip
│   ├── experiment2_full_archive_20261001.zip
│   ├── experiment3_full_archive_20261003.zip  # Git LFS
│   └── experiment4_full_archive_20261003.zip  # Git LFS
├── results/
│   ├── experiment1_noload/             # 实验一说明、统计表与论文图
│   ├── experiment2_loadstep_2kg/       # 实验二说明、统计表与论文图
│   ├── experiment3_current_limit_sweep/ # 实验三说明、统计表与论文图
│   └── experiment4_asymmetric_current_limits/ # 实验四说明、统计表与论文图
├── signal_dictionary.csv
└── trial_manifest_template.csv
```

完整 ZIP 包含原始 CSV、清洗数据、统计结果、论文图、固件/处理代码和 SHA-256 文件清单。仓库中的 `results` 目录仅展开论文写作常用的小体积材料，原始数据和完整代码以 ZIP 封档为准。

## 完整封档校验值

```text
b60681913d7f99099fb9da08d432ff42295143fbe69af8e3386fb0195359d2b7  archives/experiment1_full_archive_20261001.zip
e31c1f85b3bf790bd424d9a9435555b298587a46f671fe1371134614c2452ca6  archives/experiment2_full_archive_20261001.zip
362e8b6868a61c8b3148988a28435131b1fcaae768aadf38dcd4317670b6bdcd  archives/experiment3_full_archive_20261003.zip
1e4b012b22b0f064863f90cb77fe4afd8c1f0ab5b95684706fc39d43bb1c6dee  archives/experiment4_full_archive_20261003.zip
```

## 论文写作入口

- 实验一：[实验说明](results/experiment1_noload/实验1数据处理与论文说明.docx) · [统计工作簿](results/experiment1_noload/实验1统计汇总.xlsx)
- 实验二：[实验说明](results/experiment2_loadstep_2kg/实验2数据处理与论文说明.docx) · [统计工作簿](results/experiment2_loadstep_2kg/实验2统计汇总.xlsx)
- 实验三：[实验说明](results/experiment3_current_limit_sweep/实验3正式实验结果与论文说明.docx) · [统计工作簿](results/experiment3_current_limit_sweep/实验3统计汇总.xlsx)
- 实验四：[实验说明](results/experiment4_asymmetric_current_limits/实验4正式实验结果与论文说明.docx) · [统计工作簿](results/experiment4_asymmetric_current_limits/实验4统计汇总.xlsx)
- 四组实验的执行依据：[Step 4 实物实验与论文实验章节实施方案](../docs/STEP4_HARDWARE_EXPERIMENT_PLAN.md)

## 数据边界

- Step 1–3 的性能曲线仍属于数值仿真，不能与本目录实测结果混称。
- 实验一和实验二是 2026 年 10 月 1 日正式封档的实物数据；实验三和实验四是 2026 年 10 月 3 日正式封档的实物数据。
- 实验二载荷事件用 phase 2 内反馈电流相对 phase 1 中位数偏离至少 200 raw 的首点识别；人工挂载延迟不代表控制器动态延迟。
- 日志记录频率约为 76.923 Hz，不等于内部 1 ms 控制周期。
- 实验三、四实际采集采用按条件分块顺序，没有执行预定随机顺序；单次数据有效，但论文需披露时间/顺序混杂限制。

