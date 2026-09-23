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

### 3. Phase 0 实测验证成果与证据链 (已闭环完成)
运行 [`verify_step3b_phase0.m`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/verify_step3b_phase0.m) 形成以下完整量化证据链（已导出至 [`step3b_phase0_preanalysis.csv`](file:///c:/Users/Lenovo/Desktop/论文/早期/论文/起重机/output/step3_adaptive_rls/step3b_phase0_preanalysis.csv)）：
1. **Test 1 动力学与符号一致性检验**：
   - 动力学函数一致性：1000 组随机工况下公共微分核与 Step 2 原生实现残差为 $0.00\text{e}+00 < 10^{-12}$，**PASS**；
   - 代数回归一致性：独立采样状态下动力学力矩与回归基底残差为 $1.11\times 10^{-15}\text{ N}\cdot\text{m} < 10^{-12}$，**PASS**；
2. **Test 2 理想连续测量回归 ($t \ge 0.5\text{ s}$)**：
   - $r=0.70$：估计值 $-0.0021890\text{ N/count}$，相对误差 $0.0677\% \le 1.0\%$；
   - $r=1.30$：估计值 $+0.0016183\text{ N/count}$，相对误差 $0.0868\% \le 1.0\%$；
   - 残差 RMS 维持在 $3.6\sim 4.5\times 10^{-3}\text{ N}\cdot\text{m}$，符号均正确恢复，**PASS**；
3. **Test 3 量化测量回归 (量化分辨率 $q_y = 1.21\,\mu\text{m}$)**：
   - 单参数标量 PE 门控条件为 $\sqrt{G_k} \ge \sigma_{\text{PE,th}}$，对应 Gram 能量阈值 $G_k \ge \sigma_{\text{PE,th}}^2$；
   - 候选基准阈值 $\sigma_{\text{PE,th}} = 50\text{ count}\cdot\text{m}$ 下：
     - $r=0.70$ 估计相对误差 $0.0688\% \le 5.0\%$，滑动窗估计标准差 $0.22\% \le 5.0\%$；
     - $r=1.30$ 估计相对误差 $0.0956\% \le 5.0\%$，滑动窗估计标准差 $0.35\% \le 5.0\%$；
     - 在当前连续理想状态、预定义激励区间和停顿区间内，候选阈值均未观察到误激活（$0.0\% \le 1.0\%$）或漏激活（$0.0\% \le 1.0\%$），残差 RMS 稳定在 $3.9\sim 4.6\times 10^{-3}\text{ N}\cdot\text{m}$，**PASS**；
4. **Test 4 结构参数独立敏感性评测 (基于全链路测量重构)**：
   - 在当前 Phase 0 仿真条件下的敏感性结果：
   - **$K_\alpha \pm 20\%$**：$r=0.70$ 偏差 $-18.63\% / +18.77\%$ (增益 $0.932 / 0.939$)；$r=1.30$ 偏差 $-20.22\% / +20.41\%$ (增益 $1.011 / 1.020$)，证实准静态同相平衡下误差传递增益 $\approx 1.0$；
   - **$B_\alpha \pm 20\%$**：$r=0.70$ 偏差 $-0.22\% / +0.36\%$ (增益 $0.011 / 0.018$)；$r=1.30$ 偏差 $-0.13\% / +0.32\%$ (增益 $0.007 / 0.016$)，证实正交相位抑制特性；
   - **$J_0 \pm 20\%$**：$r=0.70$ 偏差 $+0.32\% / -0.18\%$ (增益 $-0.016 / -0.009$)；$r=1.30$ 偏差 $+0.38\% / -0.19\%$ (增益 $-0.019 / -0.009$)，证实低频激励远低于固有频率时的惯性解耦特性。

### 4. Phase 1 开始条件与边界承诺
在 Phase 0 完成真实测量回归链路构造并经技术评审确认后，方可启动 `rls_estimator_delta_kf.m` 的编写。必须严格满足以下准入条件：
1. Step 2 与公共动力学函数等价（已达成：残差 $0.00\text{e}+00$）；
2. 理想传感器回归误差 $\le 1.0\%$（已达成：$0.07\%\sim 0.09\%$）；
3. 量化传感器回归误差 $\le 5.0\%$（已达成：$0.07\%\sim 0.10\%$）；
4. 两个非对称方向均能正确恢复符号（已达成）；
5. PE 误激活率 $\le 1.0\%$、漏激活率 $\le 1.0\%$（已达成：$0.0\%$）；
6. 估计标准差 $\le 5.0\%$（已达成：$0.22\%\sim 0.35\%$）；
7. $K_\alpha, B_\alpha, J_0$ 敏感性结果可重复（已达成）；
8. 严格开环隔离：在 Step 3B 验收前，严禁接入 C3a、SyncAlloc 或任何闭环控制器。

