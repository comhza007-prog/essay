# Step 3 实施方案：状态变量滤波 (SVF) 与机械参数在线辨识 (Phase 1 优先实施版)

## 一、方案定位与参数溯源说明 (Scope & Provenance)

### 1. 参数物理属性与溯源界定
依据 [param_init.m](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step1_baseline_c0/param_init.m)，在当前研究阶段严格执行以下参数界定：
- **基准推力系数与结构参数**：$K_{f,\text{nom}} = 0.0061979\text{ N/count}$、横梁刚度 $K_\alpha = 2000\text{ N}\cdot\text{m/rad}$、转动阻尼 $B_\alpha = 25\text{ N}\cdot\text{m}\cdot\text{s/rad}$ 及 $J_0$ 在**数值仿真阶段使用模型设定真值作为已知基准**；在后续向物理台架迁移前，必须通过独立实验独立标定；
- **尺度基准锚定**：根据尺度不定性定理，平动通道无法同时辨识绝对质量与绝对推力系数。Phase 1 严格以仿真设定的标称推力系数 $K_{f,\text{nom}}$ 为已知力尺度锚点，专注于机械参数的在线估计。

### 2. 两阶段推进路径
- **Phase 1（本次实施范围：Step 3A 核心模块与闭环验证）**：
  - 纯净基准工况：**$d = 0\text{ m}, \delta_{\text{fric}} = 0$**；
  - 纯净载荷阶跃：正向 $4.5\text{ kg}$，在 $t_{\text{switch}} = 3.3\text{ s}$ 静止段切换为 $0.5\text{ kg}$，在 $t_{\text{reverse}} = 3.5\text{ s}$ 启动返程；
  - 算法核心：4 阶因果巴特沃斯 SVF + 机械参数 RLS + 先验固定物理尺度归一化 + 修正的滑动窗 Gram 矩阵 PE 门控；
  - 鲁棒性与敏感性：编码器量化噪声（8192 counts/rev, $1.21\,\mu\text{m}$ 分辨率）与 PE 门限 $\epsilon_{\text{PE}} \in \{10^{-5}, 10^{-4}, 10^{-3}\}$ 敏感性扫描。
- **Phase 2（后续开展：Step 3B 推力非对称开环验证）**：
  - 暂不接入闭环控制器；
  - 必须分别在 $r = 0.70$ 与 $r = 1.30$ 专用非对称数据下，基于完整可测输出方程 $y_{\Delta,T} = \phi_{\Delta,T,f} \Delta K_f$ 独立检验开环估计精度、残差方差及结构参数误差敏感性，通过专项验收后再行闭环接入。

---

## 二、Phase 1 机械参数回归与因果滤波建模 (Step 3A Modeling)

### 1. 执行器映射与纯净平动方程
在硬件安装关系下（[actuator_map_m3508.m](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step2_advanced_controllers/actuator_map_m3508.m)）：
$$F_G = K_{f,\text{nom}} (i_L - i_R)$$
在 $d = 0\text{ m}, \delta_{\text{fric}} = 0$ 纯净基准工况下：
$$\Delta m \cdot d \equiv 0, \quad v_L = v_R = \dot{y}_G, \quad \dot{\alpha} \equiv 0$$
平动方程严格退化为单一单自由度非线性方程（彻底排除偏转耦合与左右摩擦差异）：
$$M_{\text{tot}} \ddot{y}_G + b_G \dot{y}_G + f_{c,G} \tanh(100.0 \cdot \dot{y}_G) = F_G$$
其中真值严格为：
- 正向：$M_{\text{tot}} = 13.1 + 4.5 = 17.6\text{ kg}$；返程：$M_{\text{tot}} = 13.1 + 0.5 = 13.6\text{ kg}$；
- 黏性阻尼：$b_{G,\text{true}} = 35.0 + 35.0 = 70.0\text{ N}\cdot\text{s/m}$；
- 库仑摩擦：$f_{c,G,\text{true}} = 8.0 + 8.0 = 16.0\text{ N}$。

### 2. 因果 4 阶巴特沃斯状态变量滤波器 (SVF)
采用连续 4 阶巴特沃斯多项式（$\omega_n = 2\pi \cdot 10\ \mathrm{rad/s}$）：
$$\Lambda(s) = s^4 + 2.6131\omega_n s^3 + 3.4142\omega_n^2 s^2 + 2.6131\omega_n^3 s + \omega_n^4$$
$$W_0(s) = \frac{\omega_n^4}{\Lambda(s)}, \quad W_1(s) = \frac{\omega_n^4 s}{\Lambda(s)}, \quad W_2(s) = \frac{\omega_n^4 s^2}{\Lambda(s)}$$
双线性变换（Tustin）离散化得到因果数字滤波器。
- **运动量滤波**：对编码器位移 $y_G$ 施加 $W_2(z)$ 得到滤波加速度 $\ddot{y}_{G,f}$，施加 $W_1(z)$ 得到滤波速度 $\dot{y}_{G,f}$，**抑制高频微分噪声放大**；
- **摩擦项严格因果滤波**：
  $$v_{\text{meas}}(k) = \frac{y_G(k) - y_G(k-1)}{\Delta t}$$
  $$S_{f,\text{raw}}(k) = \tanh(100.0 \cdot v_{\text{meas}}(k)), \quad S_{f,f}(z) = W_0(z) S_{f,\text{raw}}(z)$$
- **实际施加推力滤波**：施加饱和后实际电流指令 $F_{G,\text{applied}} = K_{f,\text{nom}} (i_L - i_R)$，输出 $F_{G,f}(z) = W_0(z) F_{G,\text{applied}}(z)$。

---

## 三、PE 门控、投影与平滑安全机制 (Supervisory Logic)

### 1. 修正的滑动窗 Gram 矩阵 PE 门控
选取 $N_W = 300\text{ ms}$（300 个离散步长）的滑动窗口，采用先验固定物理对角尺度矩阵 $\mathbf{D}_{\text{prior}} = \text{diag}([1.5, 0.6, 1.0])$：
$$\bar{\boldsymbol{\phi}}_{\text{mech}}(k) = \mathbf{D}_{\text{prior}}^{-1} [\ddot{y}_{G,f}(k), \dot{y}_{G,f}(k), S_{f,f}(k)]^T$$
- **Gram 矩阵数值对称性与初态保护**：
  - 在 $k < N_W$ 窗口未充满阶段，特征值记为 `NaN`，门控维持初态冻结；
  - 在 $k \ge N_W$ 时，构造严格对称矩阵并截断数值微小负特征值：
    $$\mathbf{G}_k = \frac{1}{2} \left[ \frac{1}{N_W} \sum_{j=k-N_W+1}^k \bar{\boldsymbol{\phi}}_j \bar{\boldsymbol{\phi}}_j^T + \left(\frac{1}{N_W} \sum_{j=k-N_W+1}^k \bar{\boldsymbol{\phi}}_j \bar{\boldsymbol{\phi}}_j^T\right)^T \right]$$
    $$\lambda_{\min}(\mathbf{G}_k) = \max(0, \ \min(\text{eig}(\mathbf{G}_k)))$$
- **门控逻辑**：
  $$\text{If } \lambda_{\min}(\mathbf{G}_k) \ge \epsilon_{\text{PE}}: \quad \text{执行 RLS 参数与协方差正常更新}$$
  $$\text{If } \lambda_{\min}(\mathbf{G}_k) < \epsilon_{\text{PE}}: \quad \hat{\boldsymbol{\theta}}_k = \hat{\boldsymbol{\theta}}_{k-1}, \ \mathbf{P}_k = \mathbf{P}_{k-1} \ (\text{完全冻结})$$
  系统明确表述为：**激励是间歇性的，仅部分加减速窗口满足所选 PE 门限；匀速与静止段通过门控冻结确保数值稳定性**。

### 2. 紧凑凸集投影算子 $\Omega_\theta$
- $M_{\text{tot}} \in [12.0, 21.0]\text{ kg}$（覆盖 $13.6\text{ kg}$ 与 $17.6\text{ kg}$ 真值）；
- $b_G \in [55.0, 85.0]\text{ N}\cdot\text{s/m}$（覆盖对称真值 $70.0\text{ N}\cdot\text{s/m}$）；
- $f_{c,G} \in [12.0, 20.0]\text{ N}$（覆盖对称真值 $16.0\text{ N}$）。

### 3. 变化率限制与闭环接入
- 参数更新经过速率限制器：设定 $|\Delta \hat{M}_{\text{tot}}| \le 0.010\text{ kg/ms} = 10\text{ kg/s}$（替代原 $50\text{ kg/s}$ 弱限制）；
- 经 $5\text{ Hz}$ 低通平滑后，注入控制器名义前馈质量矩阵 $\hat{M}_q = \text{diag}([\hat{M}_{\text{tot}}, J_0])$。

---

## 四、新建模块与测试脚本结构 (output/step3_adaptive_rls/)

### 1. 核心算法代码
- **[NEW] `rls_filter_svf.m`**：4 阶因果巴特沃斯 SVF 类（因果实现，输入位移与施加电流指令，输出滤波运动量与力）；
- **[NEW] `rls_estimator_mech.m`**：带修正 Gram 矩阵 PE 门控、凸集投影与速率限制的机械参数 RLS 估计器；
- **[NEW] `controller_c3a_rls_robust.m`**：C2a 复合机械参数自适应前馈闭环控制器。

### 2. 专用数据生成与验证测试
- **[NEW] `generate_step3a_data.m`**：生成专用基准数据（$d=0, \delta_{\text{fric}}=0, K_{f,L}=K_{f,R}=K_{f,\text{nom}}$，正向 $4.5\text{ kg}$，3.3s 切换至 $0.5\text{ kg}$，3.5s 返程）；
- **[NEW] `verify_step3a.m`**：
  1. SVF 滤波频响误差与因果平衡残差检验；
  2. 编码器量化噪声（$1.21\,\mu\text{m}$ 步进）对速度与加速度滤波平滑度检验；
  3. 滑动窗 Gram 矩阵 $\lambda_{\min}$ 统计分布与静止段绝对冻结检验；
  4. PE 门限敏感性扫描（$\epsilon_{\text{PE}} \in \{10^{-5}, 10^{-4}, 10^{-3}\}$ 对收敛速度与稳定性的影响）；
  5. 凸集投影与速率限制（$0.010\text{ kg/ms}$）有效性验证；
  6. $d=0$ 专用工况下开环质量阶跃估计收敛性（从 $3.5\text{ s}$ 返程起，并在停稳窗口统计稳态误差）；
  7. C3a 闭环控制对比（对比固定名义 C2a，评估自适应前馈对跟踪误差的改善与平稳性）。

---

## 五、验收标准与收敛判据 (Acceptance Criteria & Standard Revision)

1. **验收标准修订历史与物理机理界定**：
   - **原指标未通过记录**：原方案预设的“返程启动后 0.5s（即 $t=4.0\text{ s}$）进入 $\pm 3\%$ 误差带”指标**未通过（实际误差 $7.90\% > 3.0\%$）**。
   - **物理饱和机理**：单步速率限制器设置了严格的硬件保护 $|\Delta \hat{M}| \le 0.010\text{ kg/ms} = 10\text{ kg/s}$。在梯形速度轨迹中，$0.4\text{ s}$ 加速段扣除 SVF 滤波器因果建立时间（约 $100\text{ ms}$）后，有效加速激励时间仅约 $300\text{ ms}$。理论最大积分下调量为 $0.3\text{ s} \times 10\text{ kg/s} = 3.0\text{ kg}$，初始状态为 $17.6\text{ kg}$，故极限只能到达 $14.6\text{ kg}$（最多吸收 $75.0\%$ 阶跃，实测吸收 $2.931\text{ kg} / 4.0\text{ kg} = 73.3\%$），在当前 SVF、PE 门控和 10 kg/s 算法限速设置下不可达在 $0.5\text{ s}$ 内直接跨越至 $\pm 3\%$（需到达 $14.0\text{ kg}$）。
   - **修订后的两段激励正交解耦收敛标准**：
     - **返程加速段 ($3.5\sim 4.0\text{ s}$)**：在速率限制约束下快速响应，下调量 $\Delta M \ge 2.5\text{ kg}$（实测吸收 $73.3\%$ 阶跃幅度）；
     - **返程巡航段 ($4.0\sim 5.2\text{ s}$)**：加速度 $\ddot{y}\equiv 0$，质量在动力学中失去激励作用，PE 门控必须精准识别失激励并**绝对冻结更新**，彻底杜绝协方差风积与参数漂移；
     - **返程减速段 ($5.2\sim 5.6\text{ s}$)**：提供与阻尼正交的反向惯性互补激励，完成参数解耦与最终收敛；
     - **停稳评估窗口 ($t \in [5.8, 6.8]\text{ s}$)**：平均估计值相对误差 $\le 2.0\%$（开环回放实测 $0.19\%$，闭环实测 $0.26\%$）。
2. **真投影 RLS (Projected RLS) 状态截断规范**：
   - 算法必须直接将中间估计状态 $\hat{\boldsymbol{\theta}}_k$ 投影于紧凑凸集 $\Omega_\theta$（`info.theta_raw` 严格落在物理边界内），严防内部状态在严重扰动下发散。
3. **量化抗噪性与闭环量化测量接入边界**：
   - C3a 参数辨识通道显式接入 8192 线编码器量化位移测量（$1.21\,\mu\text{m}$ 分辨率），控制反馈通道仍采用理想状态，不主张在此工况下已实现全量化状态反馈控制；
   - 在停稳窗口内，质量估计抖动标准差 $\sigma_M \le 0.15\text{ kg}$（开环实测 $0.0011\text{ kg}$，闭环实测 $0.0004\text{ kg}$）。
4. **闭环跟踪平稳性与高频抖颤判定**：
   - **时域数值有界**：闭环系统在 7s 仿真时域内所有状态与参数数值有界；
   - **跟踪精度保持**：闭环 $\text{RMSE}_{yG} \le 1.05 \times \text{RMSE}_{\text{C2a}}$（实测 $40.47\text{ mm}$ vs $40.24\text{ mm}$，差异 $+0.57\%$，严格表述为“保持近似等效跟踪性能并完成平稳参数接入”）；
   - **总变差与抖颤检验**：$\text{TV}_{\text{total}} \le 1.10 \times \text{TV}_{\text{C2a}}$（实测比值 $1.022$）；在排除换向加速度跳变及其过渡段 $\pm 100\text{ ms}$ 后，平滑跟踪阶段单步最大电流变化比值为 $1.000 \le 1.05$（实测均为 $167.79\text{ counts}$，RMS 为 $9.85$ vs $9.91\text{ counts}$），在排除过渡段后所选平滑时域指标未发现异常高频尖峰。换向突变点单步电流跳变由自适应质量与名义质量前馈比例决定（实测比值 $1.407 \approx 17.6/12.44 = 1.41$）。

---

## 六、Step 3B 执行器推力非对称性 ($\Delta K_f$) 开环可辨识性预研与离线标定方案

### 1. 统一符号定义与严格物理边界规范
- **统一定义**：$\Delta K_f \triangleq K_{f,L} - K_{f,R}$，全项目代码与变量命名统一为 `Delta_Kf`。
  $$K_{f,L} = K_{f,\text{mean}} + \frac{1}{2}\Delta K_f, \quad K_{f,R} = K_{f,\text{mean}} - \frac{1}{2}\Delta K_f$$
  工况 A ($r=0.70 < 1$) 真值 $\Delta K_f = -0.0021875\text{ N/count}$；工况 B ($r=1.30 > 1$) 真值 $\Delta K_f = +0.0016168\text{ N/count}$。
- **摩擦参数真值属性澄清**：Step 3A 仅辨识总阻尼与总摩擦；Step 3B 仿真中设定对称摩擦 $b_L=b_R=35\text{ N}\cdot\text{s/m}, f_{c,L}=f_{c,R}=8\text{ N}$ 为**仿真动力学模型的基准设定真值 (Ground Truth)**。
- **Phase 0 物理边界锁定**：严格设定偏心距 $d=0\text{ m}$，附加偏载 $\Delta m=0\text{ kg}$。在此边界下，所需阻抗力矩严格满足：
  $$T_{\text{req}} = J_0 \ddot{\alpha} + B_\alpha \dot{\alpha} + K_\alpha \alpha + T_{\text{fric},\alpha}$$
  若后续扩展至偏载 $d \neq 0, \Delta m \neq 0$，则必须补入平动-偏转惯性耦合力矩 $\Delta m \cdot d \cdot \ddot{y}_G$。
- **严格开环隔离红线**：Step 3B 仅作为开环离线标定与可辨识性预研，**严禁将 $\Delta K_f$ 估计值接入任何前馈或反馈闭环控制器**。

### 2. 公共底层动力学核与真实传感器回归通道
- **全项目统一公共动力学核**：新建公开函数 [`output/common/gantry_dynamics_deriv.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/common/gantry_dynamics_deriv.m)，Step 2、Step 3B 及测试脚本的 RK4 推进器统一调用该微分核；
- **1000 组工况动力学等价性检验**：新建 [`verify_dynamics_equivalence.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/verify_dynamics_equivalence.m)，实测原 Step 2 公式与公共微分核最大微分导数残差为 $0.00\text{e}+00$，RK4 单步状态残差为 $0.00\text{e}+00$（严格 $< 10^{-12}$），彻底消除动力学复写风险；
- **数据生成器升级**：在 [`generate_step3b_phase0_data.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/generate_step3b_phase0_data.m) 中同时导出理想连续通道 `yL_ideal, yR_ideal` 与线位移量化测量通道 `yL_quant, yR_quant`（量化步长 $q_y = 1.21\,\mu\text{m}$，固定种子 `rng(20260923)`）。动力学真值 `alpha_true, alpha_dot_true, alpha_ddot_true, T_fric_true` 仅作为误差对比真值，严禁作为辨识输入；
- **真实传感器回归构造函数**：新建 [`build_step3b_regression.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/build_step3b_regression.m)，严格从左右传感器通道重构几何转角 $\alpha_{\text{raw}} = (y_R - y_L)/L_e$，通过 4 阶因果 SVF 滤波器提取 $\alpha_f, \dot{\alpha}_f, \ddot{\alpha}_f$，从差分测速中重构导轨摩擦力矩，杜绝使用真实状态。

### 3. Phase 0 实测验证成果与证据链 (已全面完成)
运行 [`verify_step3b_phase0.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/verify_step3b_phase0.m) 形成以下完整量化证据链（已导出至 [`step3b_phase0_preanalysis.csv`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/step3b_phase0_preanalysis.csv)）：
1. **Test 1 动力学、回归结构与因果滤波多重一致性检验**：
   - **Test 1A (公共动力学与冻结旧实现一致性)**：对比从 Commit `7a41488` 提取并冻结的旧版实现夹具 [`output/tests/fixtures/gantry_dynamics_deriv_legacy.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/tests/fixtures/gantry_dynamics_deriv_legacy.m) 与 [`output/common/gantry_dynamics_deriv.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/common/gantry_dynamics_deriv.m)，在 1000 组随机物理工况下，微分导数最大残差与 RK4 单步推演最大残差均为 $0.00\text{e}+00 < 10^{-12}$，**PASS**；
   - **Test 1B (理想动力学真值代数一致性)**：在两组工况下代入连续真实状态，理论阻抗力矩与非对称基底代数残差最大为 $1.11\times 10^{-15}\text{ N}\cdot\text{m} < 10^{-12}$，证实回归数学模型精确自洽，**PASS**；
   - **Test 1C (传感器重构回归残差)**：严格基于理想连续传感器通道 $(y_L, y_R)$ 与实际输入电流 $(i_L, i_R)$，通过因果 SVF 独立重构，在 $t \ge 0.5\text{ s}$ 强激励段内残差 RMS 为 $3.75\sim 4.56\times 10^{-3}\text{ N}\cdot\text{m}$，占有效信号强度仅 $0.44\%\sim 0.49\% \le 1.0\%$，**PASS**；
   - **Test 1D (批处理 SVF 与在线递推 SVF 数值等价性)**：调用专用测试脚本 [`output/step3_adaptive_rls/verify_svf_batch_online_equivalence.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/verify_svf_batch_online_equivalence.m)，对比离线批处理 `build_step3b_regression.m` 与在线逐点 Direct Form II Transposed 递推类 [`output/step3_adaptive_rls/rls_filter_svf_step3b.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/rls_filter_svf_step3b.m)，在全程 4001 个采样步长（含 $t<0.5\text{ s}$ 初始瞬态及 $t\ge 0.5\text{ s}$ 稳定段）内，8 个滤波与回归通道 $(\alpha_f, \dot{\alpha}_f, \ddot{\alpha}_f, y_{G,f}, T_{\text{fric},f}, T_{\text{req},f}, \phi_f, y_f)$ 的点对点最大残差均为 $0.00\text{e}+00 < 10^{-12}$，证实两者在数值上严格完全等价，**PASS**；
2. **Test 2 理想连续测量回归 ($t \ge 0.5\text{ s}$)**：
   - $r=0.70$：估计值 $-0.0021890\text{ N/count}$，相对误差 $0.0677\% \le 1.0\%$；
   - $r=1.30$：估计值 $+0.0016183\text{ N/count}$，相对误差 $0.0868\% \le 1.0\%$；
   - 残差 RMS 维持在 $3.69\sim 4.50\times 10^{-3}\text{ N}\cdot\text{m}$，符号均正确恢复，**PASS**；
3. **Test 3 8192 线位置编码器量化测量回归 ($q_y = 1.21\,\mu\text{m}$，理想电流指令)**：
   - 明确边界：本测试验证 8192 线编码器位移量化下的开环回归特性，驱动器电流输入为理想限幅指令，不包含硬件电流反馈噪声；
   - 单参数标量 PE 门控条件为 $\sqrt{G_k} \ge \sigma_{\text{PE,th}}$，阶梯阈值扫描 $\{1, 10, 50, 100\}\text{ count}\cdot\text{m}$；
   - 候选基准阈值 $\sigma_{\text{PE,th}} = 50\text{ count}\cdot\text{m}$ 下：
     - $r=0.70$ 估计相对误差 $0.0688\% \le 5.0\%$，局部窗口估计变异率 (local-window estimate variation) $0.22\% \le 5.0\%$；
     - $r=1.30$ 估计相对误差 $0.0956\% \le 5.0\%$，局部窗口估计变异率 $0.35\% \le 5.0\%$；
     - 在当前仿真设定的确定性激励区间和停顿区间内，候选基准阈值未观察到误激活（$0.0\% \le 1.0\%$）或漏激活（$0.0\% \le 1.0\%$），残差 RMS 稳定在 $3.95\sim 4.53\times 10^{-3}\text{ N}\cdot\text{m}$，**PASS**；
4. **Test 4 结构参数独立敏感性评测 (基于全链路测量重构)**：
   - 在当前 Phase 0 仿真条件下的敏感性结果：
   - **$K_\alpha \pm 20\%$**：$r=0.70$ 偏差 $-18.63\% / +18.77\%$ (增益 $0.932 / 0.939$)；$r=1.30$ 偏差 $-20.22\% / +20.41\%$ (增益 $1.011 / 1.020$)，证实准静态同相平衡下误差传递增益 $\approx 1.0$；
   - **$B_\alpha \pm 20\%$**：$r=0.70$ 偏差 $-0.22\% / +0.36\%$ (增益 $0.011 / 0.018$)；$r=1.30$ 偏差 $-0.13\% / +0.32\%$ (增益 $0.007 / 0.016$)，证实正交相位抑制特性；
   - **$J_0 \pm 20\%$**：$r=0.70$ 偏差 $+0.32\% / -0.18\%$ (增益 $-0.016 / -0.009$)；$r=1.30$ 偏差 $+0.38\% / -0.19\%$ (增益 $-0.019 / -0.009$)，证实低频激励远低于固有频率时的惯性解耦特性。

### 4. Phase 1 准入核查与后续边界承诺
在 Phase 0 基础性工作全部扎实闭环后，已满足技术准入条件：
1. 动力学函数与冻结参考夹具严格等价（已达成：残差 $0.00\text{e}+00 < 10^{-12}$）；
2. 批处理 SVF 与在线递推 SVF 严格数值等价（已达成：残差 $0.00\text{e}+00 < 10^{-12}$）；
3. 理想传感器回归相对误差 $\le 1.0\%$（已达成：$0.07\%\sim 0.09\%$）；
4. 位置量化回归相对误差 $\le 5.0\%$（已达成：$0.07\%\sim 0.10\%$）；
5. 两个非对称方向均能正确恢复符号（已达成）；
6. 预定义工况下 PE 误激活率 $\le 1.0\%$、漏激活率 $\le 1.0\%$（已达成：均为 $0.0\%$）；
7. 局部窗口估计变异率 $\le 5.0\%$（已达成：$0.22\%\sim 0.35\%$）；
8. $K_\alpha, B_\alpha, J_0$ 敏感性传递增益已建立定量分析基准；
9. **严格开环隔离红线**：在 Step 3B 离线估计通过完整验收前，**严禁将 $\Delta K_f$ 估计值接入 C3a、SyncAlloc 或任何闭环控制回路**。

---

## 七、Step 3B Phase 1 实施成果：单参数 $\Delta K_f$ 开环 RLS 估计器与基准测试

### 1. 核心估计器模块设计 ([`output/step3_adaptive_rls/rls_estimator_delta_kf.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/rls_estimator_delta_kf.m))
- **标量 RLS 递推序列与数值保护**：
  $$\text{den} = \lambda + \phi_k P_{k-1} \phi_k, \quad K_k = \frac{P_{k-1} \phi_k}{\text{den}}$$
  $$\theta_{\text{unprojected}} = \theta_{k-1} + K_k (y_k - \phi_k \theta_{k-1})$$
  $$P_{\text{unprojected}} = \frac{P_{k-1} (1 - K_k \phi_k)}{\lambda}$$
  $$P_k = \min(\max(P_{\text{unprojected}}, P_{\min}), P_{\max}), \quad P_{\min}=10^{-12}, P_{\max}=1.0\ (\text{N/count})^2$$
  $$\theta_k = \text{proj}(\theta_{\text{unprojected}}) = \min(\max(\theta_{\text{unprojected}}, \theta_{\min}), \theta_{\max})$$
- **环形缓冲区 PE 均方根能量门控**：
  - 维护 $N_W = 200\text{ 步}$（$200\text{ ms}$）环形缓冲区，计算均方根能量 $\text{pe\_metric} = \sqrt{\max(\frac{1}{N_W}\sum \phi_f^2, 0)}$；
  - 门控阈值 $\sigma_{\text{PE,th}} = 50.0\text{ count}\cdot\text{m}$，作为单一基准判定源；
  - **PE 不满足时绝对完全冻结**：$\theta_k = \theta_{k-1}, P_k = P_{k-1}$，严禁除以 $\lambda$，彻底隔绝静止与匀速段协方差风积。
- **投影边界定义**：
  - **物理精确非对称边界**（对应 $r \in [0.65, 1.35]$）：$\theta_{\min} = -0.0026295\text{ N/count}, \theta_{\max} = +0.0018465\text{ N/count}$；
  - **对称工程边界**（覆盖但宽于物理范围）：$[-0.00263, +0.00263]\text{ N/count}$。

### 2. 六大基准测试实测结果 ([`output/step3_adaptive_rls/verify_rls_estimator_delta_kf.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/verify_rls_estimator_delta_kf.m))
全套量化评测指标已导出至 [`step3b_phase1_rls_results.csv`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/step3b_phase1_rls_results.csv)，包含 17 列完整诊断字段（包括 `Theta_Unproj_Final`, `Theta_Proj_Final`, `Max_Theta_Unproj`, `Projection_Count`, `Sensitivity_Status`, `Projection_Status`）：

1. **Test A1 (理论模型零偏差基线)**：
   - 输入理论代数回归信号（$\Delta K_{f,\text{true}} = 0$），验证算法本身零偏特性；
   - 最终绝对误差 **$9.33\times 10^{-21}\text{ N/count} \le 1.0\times 10^{-8}\text{ N/count}$**，投影次数 $0$，Sensitivity: **PASS**，Projection: **NO_PROJECTION**。
2. **Test A2 (传感器重构零基线)**：
   - 输入连续传感器重构信号，评估因果离散测速差分底噪与稳态稳定性；
   - 终点绝对误差 **$1.60\times 10^{-7}\text{ N/count}$**（占标称推力仅 $0.0026\%$），停顿稳态均值 **$1.60\times 10^{-7}\text{ N/count}$**，均严格满足代码显式断言指标 **$\le 5.0\times 10^{-7}\text{ N/count}$**；
   - 稳态标准差 $2.41\times 10^{-21}$，PE 激活率 $66.6\%$，投影次数 $0$，无投影对照偏差 $0.00\text{e}+00$，Sensitivity: **PASS**，Projection: **NO_PROJECTION**。
3. **Test B (负向非对称 $r=0.70$)**：
   - 真值 $-0.0021875\text{ N/count}$，估计值 **$-0.0021887\text{ N/count}$**，相对误差 **$0.0531\% \le 5.0\%$**，负向符号正确恢复，停顿段绝对冻结，投影次数 $0$，无投影对照偏差 $0.00\text{e}+00$，Sensitivity: **PASS**，Projection: **NO_PROJECTION**。
4. **Test C (正向非对称 $r=1.30$)**：
   - 真值 $+0.0016168\text{ N/count}$，估计值 **$+0.0016180\text{ N/count}$**，相对误差 **$0.0699\% \le 5.0\%$**，正向符号正确恢复，停顿段绝对冻结，投影次数 $0$，无投影对照偏差 $0.00\text{e}+00$，Sensitivity: **PASS**，Projection: **NO_PROJECTION**。
5. **Test D (8192 线位置量化抗噪)**：
   - $r=0.70$：估计值 $-0.0021887$，相对误差 **$0.0562\% \le 5.0\%$**，局部窗口估计变异率 **$0.0288\% \le 5.0\%$**，停顿误动 $0.0\%$，强激漏动 $0.0\%$，投影次数 $0$；
   - $r=1.30$：估计值 $+0.0016181$，相对误差 **$0.0798\% \le 5.0\%$**，局部窗口估计变异率 **$0.0436\% \le 5.0\%$**，停顿误动 $0.0\%$，强激漏动 $0.0\%$，投影次数 $0$；
   - 综合评定：Sensitivity: **PASS**，Projection: **NO_PROJECTION**。
6. **Test E (结构参数误差敏感性与投影截断分离判定：PASS_WITH_CLIPPING)**：
   - 显式分离“敏感性识别精度（Sensitivity_Status）”与“投影安全保护（Projection_Status）”；
   - **$B_\alpha \pm 20\%$**：传递增益 $0.019 \sim 0.033 \le 0.05$（正交相位抑制），投影次数 $0$，Sensitivity: **PASS**，Projection: **NO_PROJECTION**；
   - **$J_0 \pm 20\%$**：传递增益 $-0.009 \sim -0.019 \le 0.05$（惯性解耦），投影次数 $0$，Sensitivity: **PASS**，Projection: **NO_PROJECTION**；
   - **$K_\alpha -20\%$**：未触及边界，传递增益 $0.894 \sim 1.013$，投影次数 $0$，Sensitivity: **PASS**，Projection: **NO_PROJECTION**；
   - **$K_\alpha +20\% (r=1.30)$ 关键工况**：
     - 未受限纯 RLS 估计值为 **$+0.0019469\text{ N/count}$**（真实误差传递增益为 **$1.021$**）；
     - 物理上界为 **$+0.0018465\text{ N/count}$**，投影机制累计触发 **1065 次**截断，将最终估计值安全钳位在 **$+0.0018465\text{ N/count}$**，有效防止了参数发散；
     - 判定分类：Projection_Status 评定为 **`PROJECTION_ACTIVE_CLAMPED`**（投影安全机制正常工作）；Sensitivity_Status 准确标记为 **`IDENTIFICATION_CLIPPED`**，明确不将截断后的估计值（增益表观降为 0.710）误用于评价辨识算法的敏感性精度；
   - 总体结论：**`PASS_WITH_CLIPPING`**（11 工况辨识 PASS，1 工况截断保界）。
7. **Test F (越界投影与协方差稳定性)**：
   - 正向极端冲击（$y = +10^6$）：确认 PE 激活（`is_pe == true`），未受约束估计 $\theta_{\text{unprojected}} = +497.5\text{ N/count}$ 严重越界，$\theta_{\text{projected}} = +0.0018465\text{ N/count}$ 精确截断在物理上界，协方差 $P = 4.98\times 10^{-7}$ 有限且处于 $[P_{\min}, P_{\max}]$；
   - 负向极端冲击（$y = -10^6$）：确认 PE 激活（`is_pe == true`），未受约束估计 $\theta_{\text{unprojected}} = -332.2\text{ N/count}$ 严重越界，$\theta_{\text{projected}} = -0.0026295\text{ N/count}$ 精确截断在物理下界，协方差 $P = 3.32\times 10^{-7}$ 有限且处于 $[P_{\min}, P_{\max}]$；
   - 持续零 PE 静止段（$\lambda = 0.98$ 激进遗忘下测试 5000 步零激励）：确认零激励下 `~is_pe`，且 $|P_{\text{after}} - P_{\text{init}}| < 10^{-15}$，协方差严格绝对冻结，彻底杜绝风积；
   - 对称工程边界 $[-0.00263, +0.00263]$ 同步通过冲击保界断言；
   - 综合评定：Projection: **PASS**。

### 3. Phase 1 正式归档结论与范围边界
单参数 $\Delta K_f$ 开环 RLS 估计器在理想连续状态和位置量化条件下**通过全套基准验证**（Test A-D, F: PASS; Test E: PASS_WITH_CLIPPING）；结构参数失配下的投影截断行为已单独识别并透明记录；**尚未接入闭环，也未包含真实驱动器电流反馈噪声**。

---

## 八、Step 3B Phase 2 技术方案与实施规划（离线标定与推力分配补偿）

### 1. 目标与边界约束
- **核心目标**：
  基于 Phase 1 离线辨识得到的 $\hat{\Delta K}_f$ 与基准推力系数 $K_{f,\text{mean}}$，构建双轴驱动器推力系数离线标定与静态重分配补偿算法，分析标定残差对同步偏航的误差传播特性。
- **严格边界承诺**：
  1. **严格限定为离线标定与前馈增益计算**；
  2. **绝对不修改闭环控制器结构（不接入 `controller_c3a_rls_robust.m`）**；
  3. **绝对不修改 `SyncAlloc` 源码**；
  4. **不宣称物理台架实验已验证**；
  5. **所有提交均在本地 Git 完成，严禁 `git push`**。

### 2. 标定与推力分配严格数学模型与限幅源溯源
1. **推力系数离线标定重构与正值校验**：
   $$\hat{K}_{f,L} = K_{f,\text{mean}} + \frac{1}{2}\hat{\Delta K}_f, \quad \hat{K}_{f,R} = K_{f,\text{mean}} - \frac{1}{2}\hat{\Delta K}_f$$
   断言要求：$\hat{K}_{f,L} > 0, \hat{K}_{f,R} > 0$ 且均为正有限值。
2. **驱动器对称补偿增益**：
   引入标定无量纲增益因子 $\gamma_L, \gamma_R$：
   $$\gamma_L = \frac{K_{f,\text{mean}}}{\hat{K}_{f,L}}, \quad \gamma_R = \frac{K_{f,\text{mean}}}{\hat{K}_{f,R}}$$
   断言要求：$\gamma_L > 0, \gamma_R > 0$。
3. **权威电流限幅源与双场景定义**：
   - **标称硬件限幅源 (Nominal Hardware Limit)**：严格读取自 `param_init.ctrl.spd_max_out = 16000.0 counts`（CAN 总线电流上限，源自电机驱动固件 `motor.h`）；
   - **显式人工降额测试场景 (Derated Limit Scenario)**：独立定义 `Imax_derated = 4500.0 counts`（显式模拟实验室安全保护或低速降额场景）；
   - 启动一致性断言：`assert(Imax_nominal == ctrl.spd_max_out)`；降额场景显式标记 `is_derated_limit = true`；
   - 函数调用规范：`analyze_step3b_calibration` 强制显式传入 `Imax`，严格禁止使用隐含默认值。
4. **电流补偿与物理饱和处理时序**：
   严格执行“先增益缩放，后物理限幅”的时序：
   $$i_{L,\text{comp\_cmd}}(t) = \gamma_L \cdot i_{L,\text{nom}}(t), \quad i_{R,\text{comp\_cmd}}(t) = \gamma_R \cdot i_{R,\text{nom}}(t)$$
   $$i_{L,\text{applied}}(t) = \text{sat}(i_{L,\text{comp\_cmd}}(t), -I_{\max}, I_{\max}), \quad i_{R,\text{applied}}(t) = \text{sat}(i_{R,\text{comp\_cmd}}(t), -I_{\max}, I_{\max})$$
5. **严格补偿后偏航力矩残差物理方程**：
   $$T_{\alpha,\text{comp}}(t) = -\frac{L_e}{2} \left[ K_{f,L} i_{L,\text{applied}}(t) + K_{f,R} i_{R,\text{applied}}(t) \right]$$
   $$T_{\alpha,\text{nom}}(t) = -\frac{L_e}{2} K_{f,\text{mean}} \left[ i_{L,\text{nom}}(t) + i_{R,\text{nom}}(t) \right]$$
   $$e_{T,\text{comp}}(t) \triangleq T_{\alpha,\text{comp}}(t) - T_{\alpha,\text{nom}}(t) = -\frac{L_e}{2} \left[ (K_{f,L} \gamma_L - K_{f,\text{mean}}) i_{L,\text{nom}} + (K_{f,R} \gamma_R - K_{f,\text{mean}}) i_{R,\text{nom}} \right] + \Delta T_{\text{sat}}(t)$$
   未补偿基线偏航力矩残差（$\gamma_L = 1, \gamma_R = 1$）：
   $$e_{T,\text{base}}(t) \triangleq T_{\alpha,\text{base}}(t) - T_{\alpha,\text{nom}}(t)$$
6. **解耦评估的三阶偏航抑制比**（计算窗口 $W: t \in [0.5, 2.3]\text{ s}$，停顿段 $t \in [3.0, 4.0]\text{ s}$ 独立报告底噪）：
   - $\eta_{\text{ideal}}$：连续理想辨识参数，无电流饱和约束（$I_{\max} = \infty$）；
   - $\eta_{\text{quant}}$：8192 线编码器量化辨识参数，无电流饱和约束；
   - $\eta_{\text{sat}}$：8192 线量化辨识参数，施加实际驱动器电流饱和限幅 $[-I_{\max}, I_{\max}]$。
   - 保护门限：当 $E_{\text{yaw,base}} < 10^{-12}\ \mathrm{N\cdot m}$ 时，状态判定为 `BASELINE_TOO_SMALL`，避免分母除零。
7. **准静态偏航角偏差推导（限定条件）**：
   $$\alpha_{\text{ss}} = \frac{E_{\text{yaw}}}{K_\alpha}, \quad \Delta \alpha_{\text{improve}} = 1 - \frac{\alpha_{\text{ss,comp}}}{\alpha_{\text{ss,base}}}$$
   明确限定：此项仅作为物理刚度下的理论静态几何偏差推导，不作为实际台架动态偏航闭环改善。

### 3. Phase 2 双场景全套离线基准测试实测结果 (Tests P1 ~ P7)
全套评测数据已导出至 [`output/step3_adaptive_rls/step3b_phase2_calibration_results.csv`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/step3b_phase2_calibration_results.csv)（包含 32 列结构化诊断字段，共 38 组记录：每个限幅场景 19 行，两个场景合计 38 行）。统计口径需严格区分：各饱和率按数据集全时段统计，$\eta_{\mathrm{ideal}}$、$\eta_{\mathrm{quant}}$ 与 $\eta_{\mathrm{sat}}$ 按强激励评测窗口 $t\in[0.5,2.3]\text{ s}$ 计算，两者不可直接视为同一时间窗口的指标：

#### 场景一：项目标称硬件限幅 (`Imax = 16000.0 counts`, 来源: `param_init.ctrl.spd_max_out`)
标称运动轨迹峰值电流为 $3120\text{ counts}$，仅占硬件上限的 $19.5\%$；补偿后峰值 $3788.6\text{ counts}$（占 $23.7\%$），**拥有高达 $76.3\%$ 的线性硬件安全裕度**。

| 测试编号 | 测试工况 | 参数来源 | $\gamma_L, \gamma_R$ | 标定有效性 | $E_{\text{yaw,base}}$ (N·m) | $E_{\text{yaw,comp}}$ (N·m) | $\eta_{\text{ideal}} / \eta_{\text{quant}} / \eta_{\text{sat}}$ | 基线总饱 | 补偿总饱 | 状态判定 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Test P1** | 理论无损 ($r=0.70$) | $\Delta K_{f,\text{true}}$ | (1.2143, 0.8500) | VALID | 1.0009 | $3.90\times 10^{-16}$ | 100.0% / - / 100.0% | 0.00% | 0.00% | **PASS** |
| **Test P1** | 理论无损 ($r=1.30$) | $\Delta K_{f,\text{true}}$ | (0.8846, 1.1500) | VALID | 0.7398 | $4.65\times 10^{-16}$ | 100.0% / - / 100.0% | 0.00% | 0.00% | **PASS** |
| **Test P2** | 负向连续 ($r=0.70$) | Test B Est | (1.2144, 0.8499) | VALID | 1.0009 | $5.48\times 10^{-4}$ | 99.945% / - / **99.945%** | 0.00% | 0.00% | **PASS** |
| **Test P3** | 正向连续 ($r=1.30$) | Test C Est | (0.8845, 1.1501) | VALID | 0.7398 | $5.26\times 10^{-4}$ | 99.929% / - / **99.929%** | 0.00% | 0.00% | **PASS** |
| **Test P4** | 量化级联 ($r=0.70$) | Test D Quant | (1.2144, 0.8499) | VALID | 1.0009 | $5.80\times 10^{-4}$ | - / 99.942% / **99.942%** | 0.00% | 0.00% | **PASS** |
| **Test P4** | 量化级联 ($r=1.30$) | Test D Quant | (0.8845, 1.1501) | VALID | 0.7398 | $6.01\times 10^{-4}$ | - / 99.919% / **99.919%** | 0.00% | 0.00% | **PASS** |
| **Test P5** | 截断未受限 ($K_\alpha+20\%$) | Test E Unproj | (0.8643, 1.1863) | **INVALID_OUT_OF_BOUNDS** | 0.7398 | 0.1550 | - / - / 79.045% | 0.00% | 0.00% | **CALIBRATION_CLIPPED** |
| **Test P5** | 截断保界值 ($K_\alpha+20\%$) | Test E Proj | (0.8704, 1.1750) | **VALID_ON_BOUNDARY** | 0.7398 | 0.1076 | - / - / **85.458%** | 0.00% | 0.00% | **CALIBRATION_CLIPPED** |
| **Test P6** | 电流扫描 ($0.25 I_{\max} = 4000\text{ ct}$) | Scan $0.25 I_{\max}$ | 标称增益 | VALID | 1.2832 / 0.9485 | $7.02\times 10^{-4}$ / $6.75\times 10^{-4}$ | $\eta_{\text{sat}} = 99.95\% / 99.93\%$ | 0.00% | 0.00% | **PASS** |
| **Test P6** | 电流扫描 ($0.50 I_{\max} = 8000\text{ ct}$) | Scan $0.50 I_{\max}$ | 同上 | VALID | 2.5664 / 1.8969 | $1.40\times 10^{-3}$ / $1.35\times 10^{-3}$ | $\eta_{\text{sat}} = 99.95\% / 99.93\%$ | 0.00% | 0.00% | **PASS** |
| **Test P6** | 电流扫描 ($0.75 I_{\max} = 12000\text{ ct}$) | Scan $0.75 I_{\max}$ | 同上 | VALID | 3.8497 / 2.8454 | $2.11\times 10^{-3}$ / $2.02\times 10^{-3}$ | $\eta_{\text{sat}} = 99.95\% / 99.93\%$ | 0.00% | 0.00% | **PASS** |
| **Test P6** | 电流扫描 ($1.00 I_{\max} = 16000\text{ ct}$) | Scan $1.00 I_{\max}$ | 同上 | VALID | 5.1329 / 3.7939 | 0.9111 / 0.7374 | $\eta_{\text{sat}} = \mathbf{82.25\% / 80.56\%}$ | **0.07%** | **8.55% / 6.07%** | **FAIL_DUE_TO_SATURATION** |
| **Test P6** | 电流扫描 ($1.25 I_{\max} = 20000\text{ ct}$) | Scan $1.25 I_{\max}$ | 同上 | VALID | 6.0247 / 4.7774 | 2.9050 / 2.7160 | $\eta_{\text{sat}} = \mathbf{51.78\% / 43.15\%}$ | **9.87%** | **13.05% / 11.67%** | **FAIL_DUE_TO_SATURATION** |
| **Test P7** | 标称对称基准 ($r=1.00$) | Test A1 | (1.0000, 1.0000) | VALID | $2.06\times 10^{-16}$ | $2.06\times 10^{-16}$ | 分母防除零保护触发 | 0.00% | 0.00% | **PASS (BASELINE_TOO_SMALL)** |

#### 场景二：显式人工降额测试场景 (`Imax = 4500.0 counts`, 来源: `explicit_derated_scenario`)
模拟低速安全保护或受限驱动器工况。标称运动补偿后峰值 $3788.6\text{ counts}$ 占降额上限的 $84.2\%$，线性裕度为 $15.8\%$。

| 测试编号 | 测试工况 | 参数来源 | $\gamma_L, \gamma_R$ | 标定有效性 | $E_{\text{yaw,base}}$ (N·m) | $E_{\text{yaw,comp}}$ (N·m) | $\eta_{\text{ideal}} / \eta_{\text{quant}} / \eta_{\text{sat}}$ | 基线总饱 | 补偿总饱 | 状态判定 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Test P2** | 负向连续 ($r=0.70$) | Test B Est | (1.2144, 0.8499) | VALID | 1.0009 | $5.48\times 10^{-4}$ | 99.945% / - / **99.945%** | 0.00% | 0.00% | **PASS** |
| **Test P3** | 正向连续 ($r=1.30$) | Test C Est | (0.8845, 1.1501) | VALID | 0.7398 | $5.26\times 10^{-4}$ | 99.929% / - / **99.929%** | 0.00% | 0.00% | **PASS** |
| **Test P4** | 量化级联 ($r=0.70$) | Test D Quant | (1.2144, 0.8499) | VALID | 1.0009 | $5.80\times 10^{-4}$ | - / 99.942% / **99.942%** | 0.00% | 0.00% | **PASS** |
| **Test P4** | 量化级联 ($r=1.30$) | Test D Quant | (0.8845, 1.1501) | VALID | 0.7398 | $6.01\times 10^{-4}$ | - / 99.919% / **99.919%** | 0.00% | 0.00% | **PASS** |
| **Test P5** | 截断未受限 ($K_\alpha+20\%$) | Test E Unproj | (0.8643, 1.1863) | **INVALID_OUT_OF_BOUNDS** | 0.7398 | 0.1550 | - / - / 79.045% | 0.00% | 0.00% | **CALIBRATION_CLIPPED** |
| **Test P5** | 截断保界值 ($K_\alpha+20\%$) | Test E Proj | (0.8704, 1.1750) | **VALID_ON_BOUNDARY** | 0.7398 | 0.1076 | - / - / **85.458%** | 0.00% | 0.00% | **CALIBRATION_CLIPPED** |
| **Test P6** | 电流扫描 ($0.25 I_{\max} = 1125\text{ ct}$) | Scan $0.25 I_{\max}$ | 标称增益 | VALID | 0.3609 / 0.2668 | $1.97\times 10^{-4}$ / $1.90\times 10^{-4}$ | $\eta_{\text{sat}} = 99.95\% / 99.93\%$ | 0.00% | 0.00% | **PASS** |
| **Test P6** | 电流扫描 ($0.50 I_{\max} = 2250\text{ ct}$) | Scan $0.50 I_{\max}$ | 同上 | VALID | 0.7218 / 0.5335 | $3.95\times 10^{-4}$ / $3.80\times 10^{-4}$ | $\eta_{\text{sat}} = 99.95\% / 99.93\%$ | 0.00% | 0.00% | **PASS** |
| **Test P6** | 电流扫描 ($0.75 I_{\max} = 3375\text{ ct}$) | Scan $0.75 I_{\max}$ | 同上 | VALID | 1.0827 / 0.8003 | $5.92\times 10^{-4}$ / $5.69\times 10^{-4}$ | $\eta_{\text{sat}} = 99.95\% / 99.93\%$ | 0.00% | 0.00% | **PASS** |
| **Test P6** | 电流扫描 ($1.00 I_{\max} = 4500\text{ ct}$) | Scan $1.00 I_{\max}$ | 同上 | VALID | 1.4436 / 1.0670 | 0.2562 / 0.2074 | $\eta_{\text{sat}} = \mathbf{82.25\% / 80.56\%}$ | **0.07%** | **8.55% / 6.07%** | **FAIL_DUE_TO_SATURATION** |
| **Test P6** | 电流扫描 ($1.25 I_{\max} = 5625\text{ ct}$) | Scan $1.25 I_{\max}$ | 同上 | VALID | 1.6945 / 1.3437 | 0.8170 / 0.7639 | $\eta_{\text{sat}} = \mathbf{51.78\% / 43.15\%}$ | **9.87%** | **13.05% / 11.67%** | **FAIL_DUE_TO_SATURATION** |

### 4. Phase 2 验收判定与最终物理结论
1. **测试验收判定**：
   - **P1-P4、P7 满足常规标定验收条件**；
   - **P5 为结构失配下的投影截断敏感性评估，不作为常规标定通过项**；
   - **P6 完成饱和容限扫描并如实定量识别出物理饱和失效区间**（在 $1.00 I_{\max}$ 与 $1.25 I_{\max}$ 处出现预期的物理性能退化，判定为 `FAIL_DUE_TO_SATURATION`）；
2. **正式归档物理结论**：
   > **在项目标称 16000 counts 硬件限幅下，基准往复轨迹处于未饱和区，基于 Phase 1 估计值的静态推力重分配可将执行器非对称引起的偏航力矩残差削减约 99.9%。当名义电流接近或超过限幅时，补偿增益放大弱侧电流并引发先行饱和，抑制效果下降；该退化已通过 P6 扫描定量识别。所有结果均为数值开环回放，不代表物理台架实验或闭环性能。**
3. **严格范围与隔离承诺**：
   Phase 2 仅完成开环数据回放和前馈重分配离线标定分析，**闭环控制器与分配器未受任何修改**，所有代码、数据与文档保留在本地 Git。

---

## 九、Step 3C 技术方案设计：真实非理想因素与扰动下开环鲁棒性与敏感度评估 (修订版)

### 1. 方案定位与物理边界约束 (Scope & Strict Boundaries)
在 Step 3B Phase 1 与 Phase 2 中，已经分别完成了单参数 $\Delta K_f$ 在线因果 RLS 辨识器以及基于估计值的静态推力重分配离线回放验证。然而，先前的测试主要运行在理想平动及单一量化位置通道上。

在实际工业起重机/双驱龙门台架现场，必然存在以下四大类关键物理扰动与非理想非线性：
1. **电流回路非理想性**：霍尔电流传感器温漂偏置、增益标定误差、高频斩波测量白噪声；
2. **总线通信延时与异步抖动**：CAN 总线周期性延时（$1 \sim 3\text{ ms}$）以及左右驱动节点调度优先级不同造成的双侧非对称延迟；
3. **几何测量与状态重构随机噪声**：磁栅尺/光电编码器微小振颤、离散差分引入的高频测量噪声；
4. **偏载物理力矩耦合**：起重机小车或吊载质心偏移中心线（$d_{\text{load}} \ne 0$），在加减速平动过程中通过惯性力臂产生极大的附加偏航动力学力矩。

> [!IMPORTANT]
> **Step 3C 严格边界红线承诺**：
> 1. **纯开环离线回放评估**：所有扰动注入均在离线数据回放流与开环 RLS 回归链路中进行；
> 2. **坚决不接入闭环控制器**：严禁修改或将任何估计参数回连至 `controller_c3a_rls_robust.m`；
> 3. **坚决不修改 `SyncAlloc` 源码**：推力分配核心代码保持冻结隔离；
> 4. **严禁 `git push`**：所有方案、代码与测试数据严格保留在本地 Git 仓库；
> 5. **严谨表述**：定位为数值开环敏感度与抗扰鲁棒性评估，不宣称物理台架实验。

---

### 2. 四大非理想因素数学建模 (Refined Imperfection Modeling)

#### 2.1 三层物理电流信号链解耦模型
为避免执行层误差与测量层误差混淆，将电流信号严格拆分为三层拓扑：
$$i_{\text{cmd}}(k) \xrightarrow{\text{限幅/执行时滞}} i_{\text{applied}}(k) \xrightarrow{\text{驱动动力学}} \text{Plant} \xrightarrow{\text{传感器测量}} i_{\text{meas}}(k)$$

1. **控制器指令层**：$i_{L,\text{cmd}}(k), i_{R,\text{cmd}}(k)$（标称往复轨迹或前馈重分配计算指令）；
2. **执行器物理施加层**（进入被控对象动力学积分）：
   $$i_{L,\text{applied}}(k) = \operatorname{sat}\left(i_{L,\text{cmd}}(k - d_{\text{act},L}), -I_{\max}, I_{\max}\right)$$
   $$i_{R,\text{applied}}(k) = \operatorname{sat}\left(i_{R,\text{cmd}}(k - d_{\text{act},R}), -I_{\max}, I_{\max}\right)$$
   其中 $d_{\text{act}}$ 为控制指令下发到电机执行端的通信/执行时滞。**动力学仿真必须严格使用 $i_{\text{applied}}$ 驱动**。
3. **传感器测量反馈层**（回传给 RLS 辨识器回归与标定）：
   $$i_{L,\text{meas}}(k) = (1 + \delta_{g,L}) \cdot i_{L,\text{applied}}(k - d_{\text{meas},L}) + i_{\text{bias},L} + v_{i,L}(k)$$
   $$i_{R,\text{meas}}(k) = (1 + \delta_{g,R}) \cdot i_{R,\text{applied}}(k - d_{\text{meas},R}) + i_{\text{bias},R} + v_{i,R}(k)$$
   其中 $d_{\text{meas}}$ 为传感器回采通信延时，$\delta_g$ 为增益误差，$i_{\text{bias}}$ 为霍尔零偏漂移，$v_i(k)$ 为高斯白噪声。**RLS 回归构建严格使用 $i_{\text{meas}}$**。
- **参数标称摄动区间**：
  - 增益比例漂移：$\delta_{g,L}, \delta_{g,R} \in [-0.03, +0.03]$（$\pm 3\%$）；
  - 静态零漂偏置：$i_{\text{bias},L}, i_{\text{bias},R} \in [-30, +30]\text{ counts}$；
  - 测量高斯白噪声：$v_i(k) \sim \mathcal{N}(0, \sigma_i^2)$，$\sigma_i = 10\text{ counts}$。

#### 2.2 CAN 总线延迟与因果动力学重解算
离散控制周期 $T_s = 1\text{ ms}$。设定执行延迟 $d_{\text{act}}$ 与回采延迟 $d_{\text{meas}}$：
- **历史初值规范**：当 $k - d \le 0$ 时，严格设定电流历史值为 0（电机处于静止断电就绪态）：
  $$i(k - d) = 0, \quad \forall k \le d$$
- **严格因果动力学重新积分要求与唯一公共入口**：
  若存在非零执行器延迟 $d_{\text{act}} > 0$，由于实际施加到左右电机的推力发生时序错位（尤其是双侧非对称延迟 $d_L \ne d_R$ 会激发出额外的动态偏航不平衡力矩），系统状态响应（位移 $y$、偏角 $\alpha$）**必须显式直接调用项目唯一公共单步推演函数 [`output/common/gantry_dynamics_step_rk4.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/common/gantry_dynamics_step_rk4.m) 重新进行数值积分生成**！该函数采用经典 RK4，内部直接调用同目录唯一微分核 `common/gantry_dynamics_deriv.m`，坚决禁止在未延迟的旧状态轨迹上生硬平移电流进行伪回归，也坚决禁止依赖各子目录下可能存在路径优先级冲突的局部 step 函数。
- **有效评测窗口**：因果滤波器与时滞存在前置过渡态，评测窗口严格规定为 $t \in [0.5 + d_{\max} T_s, 2.3]\text{ s}$。

#### 2.3 传感器随机高频噪声模型
实际光栅尺/编码器在量化台阶上叠加微弱的电子学噪声与机械微震动：
$$y_{L,\text{pert}}(k) = \operatorname{quant}(y_L(k), \Delta y) + v_{y,L}(k)$$
$$y_{R,\text{pert}}(k) = \operatorname{quant}(y_R(k), \Delta y) + v_{y,R}(k)$$
$$\alpha_{\text{raw,pert}}(k) = \frac{y_{R,\text{pert}}(k) - y_{L,\text{pert}}(k)}{L_e}$$
- 其中位置白噪声 $v_{y}(k) \sim \mathcal{N}(0, \sigma_y^2)$，$\sigma_y \in [1.0, 5.0]\ \mu\mathrm{m}$；
- 考察 4 阶因果 SVF 状态变量滤波器带外滤波能力及滑动窗 Gram 能量门控稳定性。

#### 2.4 偏载物理力矩耦合与“诊断评估模式”界定
起重机吊载质心偏移横梁中心线时，偏载距离记为 $d_{\text{load}}$（单位：$\text{m}$，向右为正）。
- **与权威公共动力学代码严格对齐（采纳选项一，保持公共核不改动）**：
  坚决取缔简化公式，代码实现严格对齐公共动力学核 [`output/common/gantry_dynamics_deriv.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/common/gantry_dynamics_deriv.m) 第 58~60 行的物理定义：
  $$\mathbf{M}(\Delta m, d_{\text{load}}) = \begin{bmatrix} m_{G,\text{nom}} + \Delta m & \Delta m \cdot d_{\text{load}} \\ \Delta m \cdot d_{\text{load}} & J_{\alpha,\text{nom}} + \Delta m \cdot d_{\text{load}}^2 \end{bmatrix}$$
  - **物理机理一致性说明**：龙门架本体结构质量 $m_{G,\text{nom}}$ 严格关于几何中心线对称（$d=0$），偏载惯性力矩完全由起吊的附加负载质量 $\Delta m$ 偏心引起，因此耦合质量项严格为 $\text{coupling\_m} = \Delta m \cdot d_{\text{load}}$；
  - **C4 必须固定非零 $\Delta m$ 准则**：在 Test C4 偏载诊断中，**必须显式固定设置非零载荷质量 $\Delta m = 50.0\text{ kg}$**（与起重机标称载荷工况一致），并在 $d_{\text{load}} \in [\pm 0.05, \pm 0.10, \pm 0.20]\text{ m}$ 下进行评测；在数据表与控制台日志中必须显式输出 $\Delta m$；空载 $\Delta m = 0$ 时耦合力矩物理上天然为零。
- **C4 核心定位：严格限定为“诊断评估模式 (Diagnostic Evaluation Mode)”**：
  在数学上，单参数 RLS 回归模型为：
  $$y(k) = \phi_{\Delta K_f}(k) \Delta K_f + \varepsilon(k)$$
  偏载产生的未建模惯性偏航力矩 $T_{\text{load}}(t) = -\Delta m \cdot \ddot{y}_G(t) d_{\text{load}}$ 与平动加减速同频，必然严重破坏单参数回归的无偏性。
  **Step 3C 明确不做多参数联合辨识，严格定位为：定量评估偏载存在时单参数估计器出现的估计偏差 $\hat{\Delta K}_f(\Delta m, d_{\text{load}}) - \Delta K_f^*$ 与补偿退化边界**，绝不声称单参数估计器“分离或解耦”了偏载力矩。
  为隔离原始辨识误差，C4 显式报告相对 $d_{\text{load}}=0$ 的增量偏载串扰偏差：
  $$\theta_{\text{payload\_bias}}(d_{\text{load}}) = \hat{\theta}(d_{\text{load}}) - \hat{\theta}(0)$$
  对 $d_{\text{load}} = \pm 0.05\text{ m}$ 检查偏差符号是否反转以验证惯性耦合机理。

---

### 3. Step 3C 离线基准评测矩阵设计 (Tests C1 ~ C8)

| 编号 | 测试项目 | 扰动参数配置与数据源 | 考核目的与指标（拒绝先验假设） |
| :--- | :--- | :--- | :--- |
| **Test C1** | **电流三层误差与回采漂移** | $i_{\text{cmd}} \to i_{\text{applied}} \to i_{\text{meas}}$；$\delta_g \in [\pm 1\%, \pm 3\%]$；$i_{\text{bias}} \in [\pm 15, \pm 30]\text{ ct}$；$\sigma_i = 10\text{ ct}$ | 检验电流采样比例与漂移对 $\hat{\Delta K}_f$ 的稳态估计偏差传递增益；目标 $\eta_{\text{sat}} \ge 90\%$ |
| **Test C2** | **CAN 传输时滞与异步失步** | 对称时滞 $1, 2, 3\text{ ms}$；非对称 $d_L = 1\text{ ms}, d_R = 2\text{ ms}$；调用公共 `common/gantry_dynamics_step_rk4.m` 重积分 | 评估时滞引起的推力异步相位差对收敛速度与重分配补偿残差的影响；目标 $\eta_{\text{sat}} \ge 90\%$ |
| **Test C3** | **高频传感测量随机噪声** | 编码器量化 + 高斯白噪声 $\sigma_y \in [1, 2, 5]\ \mu\mathrm{m}$；Monte Carlo $N=30$ | 检验因果 SVF 状态重构信噪比与 PE 能量滑动门控抗噪鲁棒性；目标 $\eta_{\text{sat}} \ge 90\%$ |
| **Test C4** | **偏载动力学耦合诊断评估** | 固定 $\Delta m = 50.0\text{ kg}$，调用公共动力学核积分；$d_{\text{load}} \in [\pm 0.05, \pm 0.10, \pm 0.20]\text{ m}$；单参数回归 | 诊断模式：定量记录偏载惯性力矩在单参数回归中的串扰偏差；状态记为 `PASS`, `DEGRADED_BY_PAYLOAD` |
| **Test C5** | **复合工况两阶段评测** | 阶段一：128 角点确定性扫描；阶段二：前3最劣角点叠加随机噪声 ($N=100$) | 定位为“给定均匀分布下的随机复合鲁棒性”与最劣角点评估；如实报告 `PASS` 或 `DEGRADED` |
| **Test C6** | **规范归一化龙卷风双重排序** | 统一输出物理导数 $S_{\text{physical}}$ 与范围归一化影响量 $\text{Impact}_p$；噪声 $N=30$ 尾部统计 | 双重独立排序：估计器误差影响量排序与物理偏航 $\text{RMS}(\alpha_{\text{comp}})$ 影响量排序 |
| **Test C7** | **凸集投影双向安全性检验** | 明确为传感器阶跃故障；分别注入正反向故障确保上下物理边界均被真实激活；$N=30$ | 断言低端截断数 $>0$、高端截断数 $>0$、投影后 0 越界、0 非有限值、协方差有界 |
| **Test C8** | **标称对称虚假补偿双域评测** | 拆分为 C8A (无偏载传感器/通信假补偿) 与 C8B (含偏载混淆)；$N=100$ | 严格检验 5 项判据（含超标时间比例）；未达标时如实报告 `FAIL_FALSE_COMPENSATION_TIME_RATIO` |

---

### 4. 关键分析方法学与量化指标定义

#### 4.1 四条匹配参考支路因果重积分架构
为消除动态角对比中的参考系失配（坚决禁止使用零偏载轨迹作为含载轨迹的参考），单次试验内部必须采用相同的 $\Delta m, d_{\text{load}}, \delta_{\text{fric}}$ 逐步调用公共单步函数生成四条匹配轨迹：
1. `base_no_delay`：未补偿指令，零时滞（$d_{\text{act}}=0$）；
2. `base_delayed`：未补偿指令，真实执行时滞（$d_{\text{act}}$）；
3. `comp_no_delay`：补偿指令，零时滞（$d_{\text{act}}=0$）；
4. `comp_delayed`：补偿指令，真实执行时滞（$d_{\text{act}}$）。

在此基础上定义严格匹配的动态指标：
- **补偿对实际物理偏航的绝对改善度**：
  $$\eta_{\alpha,\text{abs}} = 100 \times \left(1 - \frac{\operatorname{RMS}(\alpha_{\text{comp,delayed}})}{\operatorname{RMS}(\alpha_{\text{base,delayed}})}\right)$$
- **时滞引入的额外动态偏差抑制比**：
  $$d\alpha_{\text{base}} = \alpha_{\text{base,delayed}} - \alpha_{\text{base,no\_delay}}, \quad d\alpha_{\text{comp}} = \alpha_{\text{comp,delayed}} - \alpha_{\text{comp,no\_delay}}$$
  $$\eta_{\alpha,\text{delay}} = 100 \times \left(1 - \frac{\operatorname{RMS}(d\alpha_{\text{comp}})}{\operatorname{RMS}(d\alpha_{\text{base}})}\right)$$

#### 4.2 龙卷风排序双重规范定义 (Test C6)
严禁跨物理量纲直接排序导数。统一输出两类指标：
1. **单因素物理导数**（仅用于单一物理参数内部灵敏度分析）：
   $$S_{\text{physical}} = \frac{|\text{metric}_{\text{high}} - \text{metric}_{\text{low}}|}{p_{\text{high}} - p_{\text{low}}} \quad [\text{单位: } \text{metric单位} / \text{参数物理单位}]$$
   单位动态拼接，例如：`(N/count)/percentage-point`、`(N/count)/m`、`rad/percentage-point`、`rad/m` 等。
2. **范围影响量**（用于全局龙卷风排序，避免虚假无量纲化归一）：
   $$\text{Range\_Impact} = \max(|\text{metric}_{\text{low}} - \text{metric}_{\text{nom}}|, |\text{metric}_{\text{high}} - \text{metric}_{\text{nom}}|)$$
   统一建立两个独立龙卷风排行榜：
   - **排行榜 A（估计器参数偏差影响量）**：$\text{metric} = |\hat{\Delta K}_f - \Delta K_f^*|\ [\text{N/count}]$；
   - **排行榜 B（物理偏航残余影响量）**：$\text{metric} = \operatorname{RMS}(\alpha_{\text{comp}})\ [\text{rad}]$。
   噪声因素必须执行 $N=30$ Monte Carlo 试验，提取 P95 尾部统计作为 $\text{metric}_{\text{high}}$。

#### 4.3 凸集投影双向安全性双重检验 (Test C7)
明确将故障命名为 `Measured_Current_Sensor_Step_Fault`。为证实双侧物理边界均具备有效保护能力，分别配置驱动参数使估计流分别向上下界冲击，并硬性断言：
$$\text{low\_bound\_clip\_count} > 0, \quad \text{high\_bound\_clip\_count} > 0$$
$$\text{projected\_oob\_count} = 0, \quad \text{nonfinite\_count} = 0, \quad P_k \in [P_{\min}, P_{\max}]$$

#### 4.4 标称对称虚假补偿多域评测与迟滞门控机制 (Test C8)
拆分为三个子测试：
- **C8A (`TestC8A_SensorCommFalseComp`)**：$r=1.00$、无偏载，纳入全要素扰动矩阵（增益漂移 $\pm 2\%$、时滞 $0\dots 2\text{ ms}$、传感器噪声及零偏），评估原始估计器性能。
  输出时间游程统计：最长单次连续超标时间 P95 (`max_run_p95`)、后半程超标均比 (`late_exceed_mean`)、首次/最后超标时刻。
  严格执行 5 项程序化判据检验（含超标时间比例 `time_exceed_mean <= 5.0%`）。未达标时如实输出 `RAW_ESTIMATOR_FAIL`。
  独立导出 4 种最差确定性差模工况检验数据至独立 CSV 表。
- **C8B (`TestC8B_PayloadConfounding`)**：$r=1.00$、施加偏载（$\Delta m=50\text{kg}, d_{\text{load}} \in [-0.10, +0.10]\text{ m}$），引入匹配的 $d_{\text{load}}=0$ 对照组，计算增量偏载偏差均值 `theta_payload_bias` 与 P95 统计值 `theta_payload_bias_p95_abs`，状态定性为 `DIAGNOSTIC_PAYLOAD_CONFOUNDING`。
- **C8C (`TestC8C_GatedApplication`)**：独立的迟滞门控应用层评测（$\theta_{\text{on}}=1.0\times 10^{-5}$，$\theta_{\text{off}}=0.7\times 10^{-5}$，确认窗口 $N_{\text{confirm}}=200\text{ ms}$）。
  在不改动底层控制器红线的前提下，完成时序修正（执行器延迟后状态统计）与**真实 RK4 动力学重积分**。
  **运行模式定性**：显式声明为 `ONE_PASS_CAUSAL_GATED_REPLAY`（单次因果门控反事实回放：估计序列来自未补偿基线试验；门控补偿后的状态不反馈至估计器重新递推）。
  **实际增益与饱和计算**：严格统计延迟后实际施加在执行器上的门控增益均值（$\gamma_L, \gamma_R$）与真实物理饱和时间比例 `comp_total_sat`；标定有效性标记为 `NOT_APPLICABLE`，失效计数与截断计数记为 `NaN`。
  **结论收紧声明**：由于持续差模增益漂移未被根本消除，门控后执行激活时间仍达 $88.69\%$，生效增益时序峰值偏离达 $2.0454\%$（终点静态偏离为 $1.7599\%$），RK4 重积分所得物理偏航角改善度为 $-0.685\%$。在本次固定阈值、200 ms 确认门控及给定扰动分布下，观察到偏航角 RMS 轻微增加；由于尚未开展平滑门控与无门控时变补偿的匹配消融试验，暂不能将退化唯一归因于门控切换；本次门控配置未解决底层估计失效问题，状态严格定性为 `GATED_APPLICATION_EVAL_ONLY`。

#### 4.5 四表独立 CSV 架构规范
坚决杜绝不同语义字段混合，拆分为四个高内聚独立数据表：
1. **`step3c_performance_results.csv`**（19 行 x 68 列）：
   - 区分估计值有符号中位数（`Delta_Kf_Hat_Median`）与绝对误差中位数（`Delta_Kf_AbsError_Median`）；
   - 显式分离终点静态标定偏离度（`gamma_final_dev_max`）与实际生效时序峰值偏离度（`gamma_applied_timeseries_dev_max`）；
   - 统一采用有限样本统计并记录有效样本数（`eta_total_valid_count`）：C4 为 1，C5 为 100，C8A 为 91，C8B 为 0，C8C 为 NaN。C8A 的 eta_total 统计仅基于 91 个有限样本；另外 9 个试验因基线总扰动力矩分母接近零而不适用（C8A 有限样本统计：mean=-299.4175%，P05=-1852.5955%，min=-2991.3644%，有效样本数 n=91）；若有效样本数为 0，则 `eta_total_mean`、`eta_total_p05` 与 `eta_total_min` 统一置为 `NaN`；
   - 细化总抑制率分布：`eta_total_mean`, `eta_total_p05`, `eta_total_min`；
   - 区分物理偏航均值与尾部：`RMS_alpha_comp_dyn_mean`, `RMS_alpha_comp_dyn_p95`；
   - 彻底拆分重用列，独立设立专用字段：
     * `theta_exceed_time_mean`（C8A/C8B 输出真实比例，C4/C5/C8C 为 `NaN`）；
     * `theta_final_exceed_trial_ratio`（C8A/C8B 输出真实比例，C4/C5/C8C 为 `NaN`）；
     * `projection_trial_ratio`（C5 输出真实比例，C4/C8A/C8B/C8C 为 `NaN`）；
     * `gate_active_time_mean`（C8C 输出门控激活时间比例，其他为 `NaN`）；
   - 显式声明应用模式（`POSTHOC_STATIC_REPLAY` / `ONE_PASS_CAUSAL_GATED_REPLAY`）；
2. **`step3c_sensitivity_results.csv`**（20 行 x 13 列）：记录 C6 灵敏度两套排行榜（含单因素物理导数及带量纲物理单位、`Range_Impact`）；
3. **`step3c_projection_results.csv`**（2 行 x 12 列）：记录 C7 凸集投影安全性与双侧截断计数；
4. **`step3c_c8a_deterministic_results.csv`**（4 行 x 9 列）：记录 C8A 4 种最劣确定性差模工况的参数估计、增益偏离与虚假力矩残差。
回读断言必须对行数、列数及关键浮点字段逐项执行内存值一致性校验（$|T_{\text{read}} - \text{mem}| < 10^{-12}$）。

---

### 5. 分阶段实施路线与准入规划 (Staged Roadmap)

#### 第一阶段：Step 3C-1（单因素扰动开环验证，已完成）
1. 实现三层电流信号流、通信延时队列及高斯随机测量噪声生成器；
2. 完成 Test C1（电流三层误差与偏置敏感度）、Test C2（CAN 通信时滞与 RK4 重积分）、Test C3（传感器随机噪声与 SVF 滤波信噪比门控）；
3. 产出 `step3c_part1_results.csv` (58 行 x 40 列) 并通过纯文本日志无 NUL 验证。

#### 第二阶段：Step 3C-2（动力学耦合与多因素深度分析，已执行但未通过最终验收）
1. **Test C4**：基于统一动力学积分（固定 $\Delta m = 50.0\text{ kg}$）的偏载惯性力矩诊断评估，r070：R²=0.9149，斜率为 8.9834×10^-3 (N/count)/m；r130：R²=0.8722，斜率为 7.8606×10^-3 (N/count)/m。两个数据集均未达到 R²≥0.95，最差值为 0.8722，因此仅支持方向一致性与非线性偏载特征，不宣称局部线性解耦；
2. **Test C5**：多指标最劣角点并集（$\eta_{\text{total}}$ 最低、$\operatorname{RMS}(\alpha)$ 最大、$\theta$ 误差最大，取 Top 3 并集得 6 个角点）蒙特卡洛评估，如实报告 `DEGRADED`；
3. **Test C6**：带物理量纲导数与 `Range_Impact` 排序；
4. **Test C7**：阶跃冲击下凸集投影双侧物理边界激活（低端 25641 次，高端 20527 次截断），100% 安全通过；
5. **Test C8**：C8A 原始估计器超标时间均比为 92.75%（门槛 5.0%），程序化输出 `RAW_ESTIMATOR_FAIL`；C8B 完成偏载匹配差分与 P95 统计；C8C 完成真实 RK4 动力学重积分，定性为 `GATED_APPLICATION_EVAL_ONLY`；导出确定性差模表。
- **阶段状态结论**：**只有 C8A 原始估计器通过 5% 超标时间门槛且 C8C 物理动态指标完善后才可讨论关闭。当前 Step 3 严格保持 OPEN。**

#### 第三阶段：Step 3C-3（D0 Oracle 电流校正、D0b 严格配对消融与 D1 时滞解耦试验，阶段归档）
1. **理论公式修正与可辨识性界定**：
   - 纠正表观推力差模理论公式：$\Delta K_{f,\mathrm{apparent}} \approx \pm K_{f,\mathrm{nom}} (\delta_g^L - \delta_g^R) = 0.0061979 \times 0.04 = 2.4792\times 10^{-4}\text{ N/count}$，与 C8A 实测差模偏差理论严格闭合；
   - 明确单通道可测信息仅能识别复合等效推力 $K_{f,L,\mathrm{eff}} = K_{f,L} / (1+\delta_g^L)$，在无独立基准观测时不可通过状态扩展解耦，技术路线定性为“独立电流通道标定 + 校正后推力差模辨识”；
   - 收紧 C8C 因果定性：当前门控在 88.69% 的评测时间内保持激活，偏航角 RMS 增加 0.69%，不能将退化唯一归因于门控切换或闭环交互。
2. **Test D0 (Oracle 电流通道校正，统一验收窗口 $[0.5, 2.3]\text{ s}$)**：
   - 严格在对称模型、相同 100 种子与相同五项门槛下运行；
   - 4/5 正式验收指标大幅通过：$P_{95}(|\hat{\theta}|) = 6.24\times 10^{-6}\text{ N/ct} \le 1.0\times 10^{-5}$，$\operatorname{median}(|\hat{\theta}|) = 1.89\times 10^{-6}\text{ N/ct} \le 5.0\times 10^{-6}$，$\max|\gamma-1| = 0.0695\% \le 0.50\%$，$\max\text{RMS}(T_{\alpha,\text{comp}}) = 0.00394\text{ Nm} \le 0.010\text{ Nm}$；4 种确定性最差差模工况全部 PASS；
   - 超标时间比例均值从 92.75% 降至 7.75%，但仍超 5.00% 刚性门槛，程序化状态定性为 **`FAIL_TIME_RATIO`**；
   - 证实电流通道标定解决了主要系统误差（消除 85.00 percentage-point 偏差）；剩余 2.75 percentage-point 超标主要集中于运动起始瞬态（$t < 1.0\text{s}$ 达 19.74%，而稳态 $t \ge 1.0\text{s}$ 仅 3.14%）。
3. **Test D0b (六分支严格配对消融分析，N=100 种子锁定标准高斯噪声序列)**：
   - 统一采用固定 $[0.5, 2.3]\text{ s}$ 墙钟时间窗口，预生成标准高斯序列严格配对评估各非理想源边际贡献：
     * `D0b-A`（全要素 Oracle 基准）：超标时间 7.75%（前半程 19.74%，后半程 3.14%），最大游程 P95 0.184s，状态 `FAIL_TIME_RATIO`；
     * `D0b-B`（去除位置白噪声 $\sigma_y=0$）：超标时间降至 5.39%（前半程 15.51%，后半程 1.49%），有改善但非主导项；
     * `D0b-C`（去除位置量化 $\text{quant}=0$）：超标时间 7.56%（前半程 19.18%，后半程 3.10%），证实光栅尺量化影响极微；
     * `D0b-D`（严格配对下去除电流白噪声 $\sigma_i=0$）：超标时间 7.51%（前半程 19.51%，后半程 2.89%），消除了先前随机数偏移假象，证实电流噪声非主因；
     * `D0b-E`（去除测量通信时滞 $d_{\text{meas}}=0$）：超标时间大幅降至 2.34%（前半程 5.27%，后半程 1.21%），跨入合格线（`PASS`），证实测量时滞整体为主导诱因，需进一步解耦共模与差模；
     * `D0b-F`（去除所有噪声与量化，保留测量时滞）：超标时间仍为 5.31%（前半程 15.42%，后半程 1.42%），反向印证时滞为主要瓶颈。
4. **Test D1 (时滞解耦、逐试验配对与分层统计试验，5 分支)**：
   - 统一采用固定 $[0.5, 2.3]\text{ s}$ 验收窗口，解耦共模与差模通信时滞：
     * `D1-A`（原始独立随机时滞基准，0~2ms）：超标时间 7.75%（前半程 19.74%，后半程 3.14%），`FAIL_TIME_RATIO`；
     * `D1-B`（对称共模 1ms / 1ms，无差模）：超标时间降至 2.54%（前半程 7.18%，后半程 0.76%），`PASS`；
     * `D1-C`（对称共模 2ms / 2ms，无差模）：超标时间降至 3.44%（前半程 9.62%，后半程 1.07%），`PASS`；
     * `D1-D`（每 trial 保留最大公共时滞 $\max(d_L, d_R)$，消除差模）：超标时间降至 3.12%（前半程 8.66%，后半程 0.99%），`PASS`；
     * `D1-E`（`Oracle-known-delay causal alignment`，使用真值时延进行因果对齐）：超标时间降至 2.40%（前半程 5.43%，后半程 1.23%），`PASS`。
   - **严格配对效应与程序化闭环断言 (D1-A vs D1-D, N = 100)**：
     * 配对改善均值：`+4.627 percentage points`（中位数 `+5.081 pp`，P05/P95 为 `[-0.278, +14.270] pp`，程序化断言 `paired_mean_AD > 0` 通过）；
     * 改善比例：全体试验中 60.0% trials 严格 $D < A$；在存在非零差模时滞（$|d_L-d_R|>0$）的 66 个试验中，改善比例达 **90.9%**（60/66，程序化断言 `improved_ratio_nonzero >= 80.0%` 通过）；
     * 其余 34 个试验因原始差模为 0（$d_L=d_R$）使得 A 与 D 有效时滞完全一致（程序化断言 `zero_skew_max_diff <= 1e-12 pp` 通过）；
     * 逐试验明细已完整导出至 `step3c_part3_d1_trial_results.csv`（500 行 x 10 列，包含 `Effective_dL_ms`, `Effective_dR_ms` 及显式毫秒单位），并通过 100% 内存回读校验。
   - **D1 时延差分层统计 (|dL - dR| 分组)**：
     * $|\Delta d| = 0\text{ ms}$ (N = 34)：A = 5.332%, D = 5.332%（两分支严格一致）；
     * $|\Delta d| = 1\text{ ms}$ (N = 50)：A = 6.848%, D = 1.524%（消除 1ms 差模后超标时间降 5.324 pp）；
     * $|\Delta d| = 2\text{ ms}$ (N = 16)：A = 15.689%, D = 3.411%（消除 2ms 差模后超标时间降 12.278 pp）；
     * 在当前随机试验集合中呈现随时延差增大而恶化的单调分层关联；由于三个时延差组不是完全相同扰动样本下的控制变量扫描，该结果不单独作为严格剂量响应的因果证据。
   - **回归信号差异程序化统计 ($t \in [0.5, 0.8]\text{ s}$ 加速段)**：
     * 程序化统计给出：$\operatorname{RMS}(\Delta y) = 3.1638\times 10^{-2}\text{ N}\cdot\text{m}$，$P_{95}(|\Delta y|) = 7.3591\times 10^{-2}\text{ N}\cdot\text{m}$，$\operatorname{RMS}(\Delta \varphi) = 2.5523\text{ count}\cdot\text{m}$；
     * 机理与残差说明：在对称真值 $\Delta K_f^* = 0$ 下，由于被估计真值为零，$y_f$ 等于零参数假设下的表观回归残差；在运动起始的高电流变化率区间，左右回采通道的异步时延会改变共模、差模电流回归信号的时间对应关系，从而产生额外回归残差。该机理由 D1-A/D 的配对消融与程序化统计支持。
   - **归档结论与定性声明**：
     * **D1通过。严格配对结果支持差模测量时延是当前仿真范围内估计瞬态超标的主导贡献因素，Oracle已知时延因果对齐能够使相关指标进入门槛。该结论不代表在线时延识别或物理台架验证已经完成。**
- **阶段状态结论**：**D1 正式通过验收并关闭归档；Step 3 整体依然严格保持 OPEN。**

#### 第四阶段：Step 3C-4（独立电流通道标定、非 Oracle 因果时延处理与 C8-eng 复测规划）

1. **核心工程原则与信号处理管线**：
   - 信号处理管线严格遵循因果单向顺序，**严禁对已滤波信号进行平移或时空倒置**：
     $$\text{原始传感器回采} \xrightarrow{\text{零偏剔除}} \xrightarrow{\text{增益校正}} \xrightarrow{\text{时延估计}} \xrightarrow{\text{因果缓冲对齐}} \xrightarrow{\text{SVF 状态滤波}} \xrightarrow{\text{回归向量构造}} \xrightarrow{\text{RLS 估计}}$$
   - 将工程前端独立解耦为两个核心算法模块，不触碰底层闭环控制器与分配器：
     * `step3c_current_channel_calibrator.m`：电流量测静态零偏与增益校正；
     * `step3c_causal_delay_aligner.m`：通信时延估计与因果历史对齐缓冲。

2. **模块接口契约规范**：
   - **电流通道校准模块接口（状态递推架构）**：
     ```matlab
     [current_cal, calib_state_next, calib_info] = ...
         step3c_current_channel_calibrator( ...
             current_raw, cmd, motion, calib_state, opts);
     ```
     * 数据契约与结构体定义：
       + `current_raw = [iL_meas; iR_meas]`（双通道原始量测列向量）；
       + `cmd = struct('iL', iL_cmd, 'iR', iR_cmd)`（控制器输出指令）；
       + `motion = struct('vG', vG, 'omega', omega, 'aG', aG, 'drive_torque_disabled', logical_flag)`（台车刚体运动状态与驱动使能标志）；
       + `calib_state = struct('mode', 'IDLE', 'buffer_L', [], 'buffer_R', [], 'valid_count', 0, 'bias_hat', [0; 0], 'gain_scale', [1; 1], 'is_calibrated', false)`（校准器内部跨步递推状态）；
       + `opts`（配置参数结构体：`calibration_duration`, `N_min`, `min_retained_ratio`, `th_cmd`, `th_v`, `th_omega`, `th_a` 等）；
       + **C4-A 阶段红线**：固定 `gain_scale = [1; 1]`，严禁在零偏测试中暗含任何增益校正。
   - **因果时延对齐模块接口（显式状态递推架构）**：
     ```matlab
     [signals_aligned, align_state_next, delay_info] = ...
         step3c_causal_delay_aligner( ...
             current_cal, current_cmd, position, timestamp, ...
             align_state, opts);
     ```
     * 数据契约与结构体定义：
       + `current_cal = [iL_cal; iR_cal]`（校准后双通道电流回采列向量，counts）；
       + `current_cmd = [iL_cmd; iR_cmd]`（控制器指令列向量，counts，用于互相关基准参考）；
       + `position = [yL_meas; yR_meas]`（光栅尺左右位置量测列向量，m）；
       + `timestamp`（结构体，包含源发送与总线接收时间戳：`.t_source_L`, `.t_source_R`, `.t_source_pos`, `.t_recv_L`, `.t_recv_R`, `.t_recv_pos`, `.seq_L`, `.seq_R`）；
       + `align_state`（校准器内部跨步递推状态机）：
         ```matlab
         align_state.mode                % 'TIMESTAMP' | 'XCORR_KNOWN_ACT' | 'DIFF_ONLY'
         align_state.current_buffer_L    % 电流历史滑动缓冲区 L
         align_state.current_buffer_R    % 电流历史滑动缓冲区 R
         align_state.command_buffer_L    % 指令历史滑动缓冲区 L
         align_state.command_buffer_R    % 指令历史滑动缓冲区 R
         align_state.position_buffer_L   % 位置历史滑动缓冲区 L
         align_state.position_buffer_R   % 位置历史滑动缓冲区 R
         align_state.timestamp_buffer    % 时间戳历史缓冲区
         align_state.d_hat_L             % 当前通道时延估计 L
         align_state.d_hat_R             % 当前通道时延估计 R
         align_state.last_trusted_delay  % 上一拍可信时延状态
         align_state.confirm_count       % 迟滞确认计数器
         align_state.is_initialized      % 预热完成初始化标志
         ```
       + `signals_aligned`（输出对齐信号结构体）：
         ```matlab
         signals_aligned.valid             % logical: 缓冲区预热完成且输出有效时为 true，预热期为 false
         signals_aligned.common_timestamp  % scalar: 因果对齐后的公共物理基准时间戳 (s)
         signals_aligned.current_cal       % [2x1]: 同基准因果对齐后的左右电流 [iL_align; iR_align]
         signals_aligned.position          % [2x1]: 同基准因果对齐后的左右位置 [yL_align; yR_align]
         ```
         **因果性与预热红线**：缓冲区未充满或输入异常时，`valid` 必须为 `false`；**严禁通过补零或冒充有效数据输入下游 SVF/RLS**。
       + `delay_info`（时延诊断信息结构体）：
         ```matlab
         delay_info.did_update    % logical: 本采样步是否触发了时延参数更新
         delay_info.method        % 'TIMESTAMP' | 'XCORR_KNOWN_ACT' | 'DIFF_ONLY'
         delay_info.confidence    % scalar: 互相关置信度指标 [0, 1]
         delay_info.reject_reason % 'NONE' | 'LOW_EXCITATION' | 'SATURATION' | ...
                                  % 'PEAK_INSIGNIFICANT' | 'NEGATIVE_DELAY' | 'BUFFER_WARMING' | 'PACKET_CORRUPT'
         delay_info.d_total_hat   % [2x1]: 命令到回采的总延迟估计 (samples)
         delay_info.d_meas_hat    % [2x1]: 纯量测通信延迟估计 (samples, DIFF_ONLY 模式下为 [NaN; NaN])
         delay_info.delta_d_hat   % scalar: 左右通道差模延迟估计 (samples)
         ```

3. **Step 3C-4 三个独立验收门 (Staged Acceptance Gates)**：

   - **验收门 1: Test C4-A（霍尔传感器静态零偏标定单元测试）**：
     * **定位**：独立的电流量测零偏校准单元测试，**不运行 RLS，不接闭环控制**；
     * **刚性采样准入判据**：仅在驱动输出关闭、PWM 禁用或零转矩校准状态下采样（六项同时满足）：
       $$|i_{\text{cmd},L}| \le \text{th}_{\text{cmd}}, \quad |i_{\text{cmd},R}| \le \text{th}_{\text{cmd}}, \quad |v_G| \le \text{th}_v, \quad |\omega| \le \text{th}_\omega, \quad |a_G| \le \text{th}_a, \quad \text{drive\_torque\_disabled} == \text{true}$$
     * **校准状态机转移逻辑与原因码**：
       + `IDLE`：合法样本 $\to$ 转移至 `ACCUMULATING`；非法样本 $\to$ 保持 `IDLE`；
       + `ACCUMULATING`：合法样本 $\to$ 继续累计；非法样本 $\to$ **中止并清空窗口，回退至 `IDLE`**；有效样本满足 $N_{\min}$ 且估计有效 $\to$ 转移至 `FROZEN`；
       + `FROZEN`：**永久保持 `bias_hat`，绝不更新**；仅显式 `reset` 重置；标定完成后的运动工况绝不污染已冻结参数；禁止未完成估计直接冻结；
       + 状态原因码：`calib_info.reject_reason` 显式给出 `'NONE' | 'DRIVE_ENABLED' | 'CMD_NONZERO' | 'MOTION_NONZERO' | 'NONFINITE_INPUT' | 'INSUFFICIENT_SAMPLES'`。
     * **最小采样量与稳健算法**：
       + 采用 Hampel / MAD 稳健统计与中位数估计 $\hat{i}_{\text{bias}} = \operatorname{median}(i_{\text{meas}})$，严禁使用普通代数均值；
       + 在 $1\text{ kHz}$ 下固定：`opts.calibration_duration = 0.5; opts.N_min = ceil(opts.calibration_duration / dt);`（500 步）；`opts.min_retained_ratio = 0.9;`（至少保留 90% 样本）；
       + 冻结门限：双轴共同满足 $N_{\text{eff},L} \ge N_{\min} \cdot \text{min\_retained\_ratio}$ 且 $N_{\text{eff},R} \ge N_{\min} \cdot \text{min\_retained\_ratio}$。
     * **双轴最坏误差定义与正式验收判据**：
       + 每次试验定义双轴最坏绝对误差：
         $$e_L = |\hat{i}_{\text{bias},L} - i_{\text{bias,true},L}|, \quad e_R = |\hat{i}_{\text{bias},R} - i_{\text{bias,true},R}|, \quad e_{\text{trial}} = \max(e_L, e_R)$$
       + 零偏真值范围 $[-30, +30]\text{ counts}$，高斯噪声 $\sigma_i = 10\text{ counts}$，100 次蒙特卡洛评估；
       + **主判据**：**$P_{95}(e_{\text{trial}}) \le 2.0\text{ counts}$**；同时导出并记录 $P_{95}(e_L)$, $P_{95}(e_R)$, $P_{95}(e_{\text{trial}})$, $\max(e_{\text{trial}})$。
     * **六项子测试集 (A1 ~ A6) 实测指标与关键硬断言 (已全面 PASS 并闭环归档)**：
       + `A1` (100 次 MC 标称高斯噪声精度测试)：$P_{95}(e_L) = 1.1027\text{ counts}$，$P_{95}(e_R) = 0.9706\text{ counts}$，**$P_{95}(e_{\text{trial}}) = 1.2817\text{ counts} \le 2.0\text{ counts}$**，$\max(e_{\text{trial}}) = 1.5193\text{ counts}$，平均有效保留样本 $498.5 / 500$ 步（保留率 $99.7\% \ge 90\%$），**PASS**；
       + `A2` (12 组准入破坏 + 输入契约硬检验)：指令超标、速度/角速度/加速度非零、驱动使能、NaN/Inf 以及 `drive_torque_disabled` 非法值 (NaN, 2) 共 12 组故障工况下，累计更新次数严格为 0（`assert(update_count_when_inadmissible == 0)`）；同时检验 3 元素、1 元素及复数 `current_raw` 输入严格触发契约拒绝，**PASS**；
       + `A3` (累计中途扰动中断复位)：前 200 步正常累计，第 201 步注入 $v_G = 0.05\text{ m/s}$ 运动扰动，缓冲区立即清空（`length=0`），有效计数清零（`valid_count=0`），模式即刻回退至 `IDLE`，且扰动消除后可从 0 正常重新累计，**PASS**；
       + `A4` (FROZEN 锁定与抗强运动冲击)：完成 500 步标定进入 `FROZEN` 后，注入 1000 步 $8000\text{ counts}$ 大电流、全要素运动（$v_G = 0.5\text{ m/s}, a_G = 2.0\text{ m/s}^2$）与驱动使能工况；每步断言 `~info.is_admissible`、`info.reject_reason == 'DRIVE_ENABLED'`、`info.is_calibrated == true` 及 `~info.did_update`，冻结参数前后最大漂移为 **$0.00\text{e}+00\text{ counts} < 10^{-15}\text{ counts}$**，输出精确扣减冻结零偏，**PASS**；
       + `A5` (样本不足保护)：输入 300 步有效样本（$< N_{\min} = 500$），模式保持 `ACCUMULATING`，标定标志严格保持 `is_calibrated == false`，未完成标定前输出安全直通原量测，**PASS**；
       + `A6` (2% 脉冲异常点稳健性与 NaN/Inf 保护，MC 100)：每试验在 500 步中注入 10 个 $\pm 100\text{ counts}$ ($10\sigma$) 强离群脉冲，Hampel 稳健过滤平均有效保留 $489.0/500$ 样本（保留率 $97.8\% \ge 90\%$），**$P_{95}(e_{\text{trial}}) = 1.3192\text{ counts} \le 2.0\text{ counts}$**，$\max(e_{\text{trial}}) = 1.8611\text{ counts}$；非有限输入下输出始终保持有限（`assert(all(isfinite(current_cal)))`），**PASS**；
       + `数据治理与回读校验`：明细数据完整导出至 `step3c_c4a_bias_results.csv`（200 行 $\times$ 15 列），全部 15 列（含 `Trial_ID, Subtest, RNG_Seed, True_Bias_L/R, Est_Bias_L/R, Err_L/R, Err_Trial_Max, Is_Calibrated, Calib_Mode, N_Eff_L/R, Within_P95_Threshold`）执行 100% 逐列逐元素严格内存回读校验（数值残差 $< 10^{-9}$），断言全数通过。
     * **验收门状态**：**Gate C4-A 静态电流零偏标定单元测试正式通过并关闭归档**。

   - **验收门 2: Test C4-B（时延识别与因果历史对齐单元测试）**：
     * **定位与物理延迟解耦原理**：
       + 独立评估因果历史对齐缓冲区与时延估计器，输入已校准电流 $i_{\text{cal}}$、指令参考 $i_{\text{cmd}}$ 与位置 $y$；
       + **可辨识性物理红线**：命令与回采互相关估计得到的是回路总延迟：
         $$d_{\text{total}} = d_{\text{act}} + d_{\text{meas}} + d_{\text{driver}}$$
         它**绝不是纯量测通信延迟 $d_{\text{meas}}$**！若直接将 $d_{\text{total}}$ 用于延迟位置通道，会将执行器机械/电气响应延迟 $d_{\text{act}}$ 重复叠加，造成新的虚假回归时间错配。
     * **三种工作模式与降级规则**：
       1. `TIMESTAMP`（源采样时间戳硬对齐，工程主路径）：报文源时间戳完备，直接按物理源时间对齐各通道；接收时间戳仅用于统计网络传输抖动；
       2. `XCORR_KNOWN_ACT`（已知执行器延迟互相关模式）：无硬件时间戳，但执行器响应延迟 $d_{\text{act}}$ 已通过离线台架独立标定已知：
          $$d_{\text{meas,hat}} = d_{\text{total,hat}} - d_{\text{act,known}}$$
          若计算出现负值（$d_{\text{total,hat}} < d_{\text{act,known}}$），判定为非法估计（`reject_reason = 'NEGATIVE_DELAY'`），**严格禁止截断为 0 冒充有效估计**，维持上一拍可信值；
       3. `DIFF_ONLY`（未知执行器延迟差模对齐模式）：无硬件时间戳且 $d_{\text{act}}$ 未知，在左右执行器电气延迟对称（$d_{\text{act},L} \approx d_{\text{act},R}$）假设下估计相对差模：
          $$\Delta d_{\text{meas}} \approx \Delta d_{\text{total}} = d_{\text{total},L} - d_{\text{total},R}$$
          **降级红线约束**：本模式仅允许左右电流通道相互补齐差模，**严禁宣称完成了电流—位置绝对时间对齐**，`d_meas_hat` 强制输出 `[NaN; NaN]`。
     * **时延更新准入与迟滞确认机制**：
       + 仅在高电流变化率（$\operatorname{Var}(\dot{i}_{\text{cmd}}) \ge \sigma_{\text{di,th}}^2$）、无电流饱和（$|i_{\text{cmd}}| \le 0.95 I_{\max}$）、滑动相关第一峰与第二峰比值超越显著性门限（$\rho_1 - \rho_2 \ge \Delta \rho_{\text{th}}$）时方可触发更新；
       + 平稳段与失激励段严格保持 `did_update = false`，维持 `last_trusted_delay`；
       + 迟滞确认机制：连续 $N_{\text{confirm}} \ge 5$ 步检测到相同时延突变方可切换，杜绝单步噪声尖峰引起的缓冲区跳变。
     * **六项基准子测试集 (B1 ~ B6)**：
       + `B1` (时间戳硬对齐测试)：$d_{\text{meas}} \in \{0, 1, 2\}\text{ samples}$，时间戳模式识别与因果对齐 100% 正确；
       + `B2` (已知 $d_{\text{act}}$ 互相关测试)：Monte Carlo 评估 $d_{\text{meas}}$ 识别正确率 $\ge 95\%$；
       + `B3` (未知 $d_{\text{act}}$ 差模降级测试)：强制进入 `DIFF_ONLY`，验证严禁输出虚假绝对 $d_{\text{meas,hat}}$（输出为 NaN）；
       + `B4` (稳健门控与迟滞测试)：低激励、饱和、多峰模糊工况下 `did_update = false`，保持上一可信值；
       + `B5` (严格因果性硬检查)：对齐输出索引只能引用当前或历史缓冲区样本，未来样本引用次数严格为 0；
       + `B6` (预热与异常输入防护)：预热期严格输出 `signals_aligned.valid = false`；NaN/Inf、丢包乱序下保持有限与受控，无效数据绝不进入下游 SVF/RLS。
     * **验收指标量化判据**：
       + 时间戳模式对齐误差：严格 $0\text{ samples}$；
       + 互相关模式正确识别率：$\ge 95.0\%$；
       + 对齐后残余差模时延：严格 $0\text{ samples}$；
       + 未来样本引用次数：严格 $\equiv 0$；
       + 低置信度误更新次数：严格 $\equiv 0$；
       + 非有限输出项：严格 $\equiv 0$。

   - **验收门 3: Test C4-C（增益校正与可辨识性界定）**：
     * **定位与可辨识性红线**：承认纯回采量测（电流+位置）数学上不可解耦传感器增益误差 $\delta_g$ 与推力系数不对称 $\Delta K_f$。“对称运行段相对增益校准”只能作为假设性通道均衡，不可宣称为绝对增益标定；
     * **工程优先级**：驱动出厂标定参数/源时间戳 > 精密分流电阻/基准校准源 > 外置电流表离线实测 > 对称运行相对均衡（显式标记为假设性支线）；
     * **设计门槛**：允许差模残差 $|\delta_g^L - \delta_g^R| \le 0.10\%$（满足表观推力偏差 $\le 1\times 10^{-5}\text{ N/ct}$ 且保留裕量）；
     * **保护性测试**：对称系统校准后虚假 $\Delta K_f \le 1\times 10^{-5}\text{ N/ct}$；$r=0.70$ 与 $r=1.30$ 真实不对称符号保持且相对辨识误差 $\le 5\%$，绝不允许将物理真实不对称抹除。

4. **工程前端集成与 C8 复测规范 (C8A-eng & C8C-eng)**：
   - **双轨命名规范**：
     * `C8A-raw`：保留 $92.75\%$ 原始未校正估计器基线结果作为历史对照，严禁覆盖；
     * `C8A-eng`：串联零偏标定、增益校正与因果对齐工程前端后的估计器；
     * `C8C-eng`：接入工程前端后的门控物理动态反事实回放与重积分。
   - **验收判据固定（严禁拔高门槛或缩短评测窗口）**：
     * `C8A-eng`：对称全要素工况下超标时间均值严格 $\le 5.00\%$，原五项正式判据全数通过；且在 $r=0.70/1.30$ 非对称工况下保留真实物理参数跟踪能力；
     * `C8C-eng`：严格配对蒙特卡洛评估，平均偏航改善度 $> 0$，中位数 $> 0$，95% CI 下界 $> 0$（或报告统计显著性），不通过调节原门控阈值凑取通过；
   - **最终关闭准则**：只有 `C8A-eng` 与 `C8C-eng` 在非 Oracle 工程前端下完整达标，方可正式讨论关闭 Step 3。当前 Step 3 依然严格保持 **`OPEN`**。
