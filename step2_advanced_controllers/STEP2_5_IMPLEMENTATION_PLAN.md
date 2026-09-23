# Step 2.5 实施计划：推力空间同步优先约束分配器与往复换向基准测试 (V3.0 终版)

本方案旨在解决 Step 2 暴露的核心物理瓶颈：在强限流且大加速度需求下，独立执行器截断将左右电机钉死在正向驱动边界（左侧 $+4500\text{ counts}$，右侧 $-4500\text{ counts}$），导致防偏转力矩物理归零。方案将在二维物理推力空间中构建**同步优先约束分配器**，闭环连接动态抗饱和状态 $\mathbf{z}_{\text{aw}}$，并通过 **$2 \times 2$ 完全因子消融矩阵**与**往复换向工况**，客观评测同步优先与动态抗饱和的独立贡献与复合效能。

---

## 一、核心架构与数学规范

### 1. 执行器保护链条与抗饱和状态 $\mathbf{z}_{\text{aw}}$ 闭环反馈
分配器输出虽然理论上已在边界内，但为防止数值浮点溢出，仍串联最后一级物理保护截断：
$$\mathbf{i}_{\text{actual}} = \mathrm{sat}(\mathbf{i}_{\text{alloc}}, \mathbf{I}_{\text{eff}})$$
由于 $\mathbf{i}_{\text{actual}}$ 已处于有效限幅内，下游外部残差通常为零。因此，驱动动态抗饱和状态的残差**严格定义为实际执行器输出与未受限理想请求之间的缺额**：
$$\Delta\mathbf{i}_{\text{alloc}} = \mathbf{i}_{\text{actual}} - \mathbf{i}_{\text{request}}$$
$$\dot{\mathbf{z}}_{\text{aw}} = \mathbf{K}_{\text{aw}} \Delta\mathbf{i}_{\text{alloc}} - \lambda_{\text{aw}} \mathbf{z}_{\text{aw}}$$
$$\mathbf{z}_{\text{aw}}(k+1) = \mathbf{z}_{\text{aw}}(k) + T_s \dot{\mathbf{z}}_{\text{aw}}(k)$$

### 2. $2 \times 2$ 完全因子消融控制拓扑矩阵
明确四大控制器的输入输出映射拓扑，彻底解耦两个因子的作用：
- **C2a** (无抗饱和 + 独立截断):
  $$\mathbf{v}_\beta \xrightarrow{\mathbf{T}_{\text{act}}^{-1}} \mathbf{i}_{\text{request}} \xrightarrow{\mathrm{sat}(\mathbf{I}_{\text{eff}})} \mathbf{i}_{\text{actual}}$$
- **C2a-SyncAlloc** (无抗饱和 + 同步优先分配):
  $$\mathbf{v}_\beta \xrightarrow{\text{thrust\_allocator\_sync}} \mathbf{i}_{\text{alloc}} \xrightarrow{\mathrm{sat}(\mathbf{I}_{\text{eff}})} \mathbf{i}_{\text{actual}}$$
- **C2b** (动态抗饱和 + 独立截断):
  $$\mathbf{v}_{\text{cmd}} = \mathbf{v}_\beta + \mathbf{T}_{\text{act}}\mathbf{z}_{\text{aw}} \xrightarrow{\mathbf{T}_{\text{act}}^{-1}} \mathbf{i}_{\text{request}} \xrightarrow{\mathrm{sat}(\mathbf{I}_{\text{eff}})} \mathbf{i}_{\text{actual}}$$
- **C2b-SyncAlloc** (动态抗饱和 + 同步优先分配):
  $$\mathbf{v}_{\text{cmd}} = \mathbf{v}_\beta + \mathbf{T}_{\text{act}}\mathbf{z}_{\text{aw}} \xrightarrow{\text{thrust\_allocator\_sync}} \mathbf{i}_{\text{alloc}} \xrightarrow{\mathrm{sat}(\mathbf{I}_{\text{eff}})} \mathbf{i}_{\text{actual}}$$

同时以工程级联 PID（C0）与传统交叉耦合（C1）作为对照基线。

### 3. 纠偏力矩优先的理论保持边界与算法性质
- 在纠偏力矩请求位于执行器可行域内时，完整保持该力矩；
- 超出可行域时，优先提供最大可实现纠偏力矩；
- 算法为**闭式解析投影解**，无在线数值优化迭代循环。

### 4. 往复轨迹时序与静止保持设计
针对 $a_{\max} = 1.5\text{ m/s}^2, v_{\max} = 0.6\text{ m/s}, y_{\text{target}} = 1.0\text{ m}$：
- 加速段 $0.4\text{ s}$（位移 $0.12\text{ m}$），匀速段 $1.267\text{ s}$（位移 $0.76\text{ m}$），减速段 $0.4\text{ s}$（位移 $0.12\text{ m}$），单程运动总耗时 $t_{\text{motion}} \approx 2.07\text{ s}$；
- **阶段 1 ($0.0 \sim 3.5\text{ s}$)**: $0 \sim 2.07\text{ s}$ 运动至 $1.0\text{ m}$，并在 $1.0\text{ m}$ 保持静止 $1.43\text{ s}$（用以观察正向停靠超调量与调节稳定）；
- **阶段 2 ($3.5 \sim 7.0\text{ s}$)**: $3.5 \sim 5.57\text{ s}$ 反向运动返回原点 $0.0\text{ m}$，并在原点保持静止 $1.43\text{ s}$；
- 轨迹**位置和速度全局连续，加速度有界但在阶段切换点发生跳变**。

---

## 二、实施计划与推荐执行步骤

1. **Step 2.5.1: 编写推力空间约束分配器** `thrust_allocator_sync.m`
   - 采用有效限幅 $I_{\text{eff},L} = \min(I_{\text{fw},L}, I_{\max,L})$, $I_{\text{eff},R} = \min(I_{\text{fw},R}, I_{\max,R})$；
   - 在推力空间闭式解析解算力矩优先分配。
2. **Step 2.5.2: 编写并运行分配器专用测试** `verify_thrust_allocator.m`
   - 包含 5 项独立测试（无超限透传、纯平动削顶、纯力矩超限边界性、平动超限时力矩完整保留、非对称增益与有效限幅矩形闭合）；
   - 确保 5 项测试 100% PASS。
3. **Step 2.5.3: 实现 C2a-SyncAlloc 与 C2b-SyncAlloc 控制器**
   - 采用统一接口，输出分配电流与驱动 $\mathbf{z}_{\text{aw}}$ 的分配残差 $\Delta\mathbf{i}_{\text{alloc}}$。
4. **Step 2.5.4: 编写往复换向轨迹生成器** `trajectory_reciprocating.m`
   - 实现 $7.0\text{ s}$ 双程含停靠平滑运动。
5. **Step 2.5.5: 编写并运行往复换向基准测试** `run_reciprocating_benchmark.m`
   - 对比 6 大控制器，输出 `reciprocating_results_summary.csv` 与高分辨率对比图。
6. **Step 2.5.6: 撰写正式项目报告** `STEP2_5_BENCHMARK_REPORT.md`
   - 客观呈现仿真结果，恪守等效动力学仿真边界。
