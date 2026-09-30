# Step 2.6 实施计划：参数敏感性与工况拓展全面评测 (V1.0 正式版)

## 一、研究目标与定位

在 Step 2.5 中，推力空间同步优先约束分配器（SyncAlloc）与动态抗饱和回路（$\mathbf{z}_{\text{aw}}$）在标称强限流工况（$\Delta m = 3.0\text{ kg}, d = 0.28\text{ m}, \Delta f_{\text{fric}} = 30\%, I_{\max} = 4500\text{ counts}$）下，将最大同步误差从 $0.7734\text{ mm}$ 显著降至 $0.2931\text{ mm}$（降低 $62.1\%$），同时伴随着平动均方根误差 $\text{RMSE}_{yG}$ 从 $108.9\text{ mm}$ 增至 $148.8\text{ mm}$ 的平动让步代价。

为了探明该方案的适用边界，验证其在不同扰动强度、执行器退化和参数摄动下的表现趋势，避免单一工况的偶然性，Step 2.6 将在进入 RLS 在线辨识之前，开展**多维度参数敏感性与复杂工况拓展评测**。

> **学术边界与用词界定**：
> 本项评估属于离散数值仿真，目的在于**在多组等效仿真参数和工况下评估 SyncAlloc 的适用范围、性能变化和代价**，绝不宣称为“鲁棒性或稳定性已严格证明”。参数空间的数值有界性仅作为算法稳定运行的经验判据，理论证明需依托后续的严密李雅普诺夫/ISS分析。

---

## 二、评测控制器矩阵锁定 (2x2 完全消融)

为避免矩阵膨胀与冗余计算，敏感性扫描统一聚焦于 **$2 \times 2$ 完全因子消融控制器**：
1. **C2a**：独立截断 + 无抗饱和
2. **C2a-SyncAlloc**：推力空间同步优先分配 + 无抗饱和
3. **C2b**：独立截断 + 动态抗饱和
4. **C2b-SyncAlloc**：推力空间同步优先分配 + 动态抗饱和（复合方案）

（C0 级联 PID 与 C1 交叉耦合 CCC 仅在标称基准点输出单点参考线，不参与全网格扫描）。

---

## 三、评测六大维度设计与物理建模规范

### 维度 1: 偏载质量与偏心距扫描 (Eccentric Load Sensitivity)
- **物理机理**：偏载带来的偏转惯性扰动力矩可作简化量纲估计：$T_{\text{dist}} \approx \Delta m \cdot \ddot{y}_d \cdot d$（完整动力学中包含平动-偏转惯性质量耦合矩阵、摩擦差阻力矩与横梁扭转恢复阻尼）。
- **扫描网格**：
  - 偏载质量扫描：$\Delta m \in \{0.0, 1.5, 3.0, 4.5, 6.0\}\text{ kg}$（偏心距固定 $d = 0.28\text{ m}$）；
  - 偏心距扫描：$d \in \{0.0, 0.14, 0.28, 0.38\}\text{ m}$（偏载质量固定 $\Delta m = 3.0\text{ kg}$）。
- **考察指标**：$\text{Max\_Sync}$ 随偏载增长曲线、纠偏力矩缺额峰值、平动 $\text{RMSE}_{yG}$ 恶化代价。

### 维度 2: 左右导轨摩擦非对称性扫描 (Friction Asymmetry Sweep)
- **物理机理**：模拟导轨磨损、润滑不均导致的差动阻力。
- **扫描网格**：左右摩擦系数偏差比例 $\Delta f_{\text{fric}} \in \{0\%, 15\%, 30\%, 50\%, 70\%\}$。
- **考察指标**：换向过零时的同步冲击、全周期均方根同步误差 $\text{RMSE}_{\text{sync}}$。

### 维度 3: 执行器限流能力分级扫描 ($I_{\max}$ Escalation Sweep)
- **物理机理**：观察系统从充裕无饱和区、轻度饱和区过渡至极端饱和区的动态分水岭。
- **扫描网格**：$I_{\max} \in \{3500, 4500, 6000, 8000, 12000, 16000\}\text{ counts}$。
- **考察指标**：饱和临界电流阈值、SyncAlloc 相比独立截断的同步误差改善百分比随限流深度的变化趋势。

### 维度 4: 左右电机推力系数非对称扫描 (Thrust Gain Degradation)
- **物理机理**：模拟单侧电机退磁、温升衰退或齿轮齿条磨损。
- **严格固定总驱动基准**：为防止比值变化引起总驱动推力改变，令平均推力系数严格恒定（$K_{f,\text{mean}} = \text{mech.Kf}$）：
  $$\text{ratio} = \frac{K_{f,L}}{K_{f,R}} \in \{0.70, 0.85, 1.00, 1.15, 1.30\}$$
  $$K_{f,L} = \frac{2 K_{f,\text{mean}} \cdot \text{ratio}}{1 + \text{ratio}}, \quad K_{f,R} = \frac{2 K_{f,\text{mean}}}{1 + \text{ratio}}$$
  （满足 $\frac{K_{f,L} + K_{f,R}}{2} \equiv K_{f,\text{mean}}$）。
- **考察指标**：推力空间非对称边界解算的自洽性、力矩保真度与残差代数恒等式。

### 维度 5: 动态抗饱和参数空间扫描 ($K_{\text{aw}}$ & $\lambda_{\text{aw}}$ Tuning Grid)
- **物理机理**：评估不同参数组合下的数值有界性、跟踪性能和平动缺额抑制范围（非“稳定裕度证明”）。
- **扫描网格**：
  - $K_{\text{aw}} \in \{5, 10, 20, 40, 80\}$；
  - $\lambda_{\text{aw}} \in \{5, 10, 20, 40, 80\}$。
- **考察指标**：总不可实现控制残差积分 $\int \|\Delta\mathbf{i}_{\text{tot}}\|$、控制量总变差 $\text{TV}_{iR}$、状态 $\mathbf{z}_{\text{aw}}$ 范数峰值。

### 维度 6: 往返等效载荷突变工况 (Bidirectional Load Transfer Cycle)
- **物理机理**：模拟起重机典型“重载去、轻载回”等效载荷突变（非实车卸载实验）：
  ```matlab
  if tk < traj.t_half % 3.5 s (正向重载运送)
      delta_m = 4.5;   % kg
      d_load = 0.28;   % m
  else                % 反向空载返回
      delta_m = 0.5;   % kg
      d_load = 0.10;   % m
  end
  ```
- **考察指标**：工况突变点的过渡平滑性、正反向分阶段超调量（$\text{Overshoot}_{\text{fwd}}$, $\text{Overshoot}_{\text{rev}}$）及停靠调节时间。

---

## 四、脚本组织与数据交付清单

1. **执行脚本**：`output/step2_advanced_controllers/run_sensitivity_benchmark.m`
   - 实现 6 大维度的参数自动化循环遍历；
   - 包含实时动态断言（SyncAlloc 下游保护残差 $\|\Delta\mathbf{i}_{\text{external}}\| < 10^{-10}$，并按控制器架构检查残差代数恒等式）；
   - 输出数据文件 `sensitivity_results.mat` 与 `sensitivity_summary.csv`；按当前网格应生成 113 条数据记录（不含表头）。
2. **绘图输出**：
   - `sensitivity_eccentric_load.png`（维度 1 曲线）
   - `sensitivity_friction_asymmetry.png`（维度 2：导轨摩擦非对称曲线）
   - `sensitivity_thrust_asymmetry.png`（维度 4A/4B：推力增益非对称曲线，含 Oracle 对照）
   - `sensitivity_Imax_escalation.png`（维度 3 曲线）
   - `sensitivity_antiwindup_params.png`（维度 5：抗饱和参数敏感性曲线）
   - `sensitivity_load_transfer.png`（维度 6：往返分段载荷时域曲线）
3. **正式技术报告**：`output/step2_advanced_controllers/STEP2_6_SENSITIVITY_REPORT.md`
   - 全面收录数据表格与物理分析；
   - 严谨总结 SyncAlloc 的有效适用区间与平动代价规律。
